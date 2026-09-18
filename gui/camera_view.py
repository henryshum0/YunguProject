"""Dependency-free ROS image decoding and threaded camera subscription helpers."""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite
from queue import Empty, Full, Queue
from threading import Event, Lock, Thread
from time import monotonic
from typing import Any

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


CAMERA_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
)


class ImageDecodeError(ValueError):
    """Raised when an incoming ROS image cannot be rendered by Tk."""


@dataclass(frozen=True, slots=True)
class CameraFrame:
    """RGB image data suitable for conversion into a PPM Tk PhotoImage."""

    width: int
    height: int
    rgb: bytes

    def ppm_bytes(self) -> bytes:
        return f"P6\n{self.width} {self.height}\n255\n".encode("ascii") + self.rgb

    def resized_to_fit(self, maximum_width: int, maximum_height: int) -> "CameraFrame":
        """Return an aspect-preserving nearest-neighbour preview frame."""
        if maximum_width <= 0 or maximum_height <= 0:
            raise ValueError("preview dimensions must be positive")
        scale = min(1.0, maximum_width / self.width, maximum_height / self.height)
        width = max(1, int(self.width * scale))
        height = max(1, int(self.height * scale))
        if (width, height) == (self.width, self.height):
            return self
        output = bytearray(width * height * 3)
        destination = 0
        for y in range(height):
            source_y = min(self.height - 1, int(y * self.height / height))
            for x in range(width):
                source_x = min(self.width - 1, int(x * self.width / width))
                source = (source_y * self.width + source_x) * 3
                output[destination:destination + 3] = self.rgb[source:source + 3]
                destination += 3
        return CameraFrame(width=width, height=height, rgb=bytes(output))


def decode_ros_image(message: Any) -> CameraFrame:
    """Decode the common uncompressed ROS image encodings into packed RGB."""
    width = int(message.width)
    height = int(message.height)
    encoding = str(message.encoding).lower()
    if width <= 0 or height <= 0:
        raise ImageDecodeError("image dimensions must be positive")

    layouts = {
        "rgb8": (3, (0, 1, 2)),
        "bgr8": (3, (2, 1, 0)),
        "rgba8": (4, (0, 1, 2)),
        "bgra8": (4, (2, 1, 0)),
        "mono8": (1, (0, 0, 0)),
    }
    if encoding not in layouts:
        supported = ", ".join(layouts)
        raise ImageDecodeError(f"unsupported image encoding {encoding!r}; supported: {supported}")
    channels, rgb_indices = layouts[encoding]
    minimum_step = width * channels
    step = int(message.step)
    if step < minimum_step:
        raise ImageDecodeError(
            f"image step {step} is too small for {width}px {encoding} rows")
    source = bytes(message.data)
    expected_size = step * height
    if len(source) < expected_size:
        raise ImageDecodeError(
            f"image data has {len(source)} bytes; expected at least {expected_size}")

    # Gazebo's RGB cameras normally publish tightly-packed rgb8 images.  Keep
    # that common path in C-level byte copying instead of visiting 921,600
    # individual channel values in Python for every 640x480 frame.
    if encoding == "rgb8":
        if step == minimum_step:
            return CameraFrame(width=width, height=height, rgb=source[:expected_size])
        return CameraFrame(
            width=width,
            height=height,
            rgb=b"".join(
                source[row * step:row * step + minimum_step]
                for row in range(height)),
        )

    output = bytearray(width * height * 3)
    destination = 0
    for row in range(height):
        row_start = row * step
        for column in range(width):
            pixel_start = row_start + column * channels
            output[destination] = source[pixel_start + rgb_indices[0]]
            output[destination + 1] = source[pixel_start + rgb_indices[1]]
            output[destination + 2] = source[pixel_start + rgb_indices[2]]
            destination += 3
    return CameraFrame(width=width, height=height, rgb=bytes(output))


class CameraPreview:
    """Latest-frame ROS preview that never lets camera work backlog the GUI."""

    def __init__(
        self,
        topic: str,
        *,
        node_name: str = "skills_test_gui_camera",
        maximum_fps: float = 15.0,
        maximum_width: int | None = None,
        maximum_height: int | None = None,
    ) -> None:
        if not topic.strip():
            raise ValueError("camera image topic must not be empty")
        if not node_name.strip():
            raise ValueError("camera preview node name must not be empty")
        if not isfinite(maximum_fps) or maximum_fps <= 0.0:
            raise ValueError("camera preview maximum_fps must be greater than zero")
        if (maximum_width is None) != (maximum_height is None):
            raise ValueError("camera preview maximum_width and maximum_height must be set together")
        if maximum_width is not None and (maximum_width <= 0 or maximum_height <= 0):
            raise ValueError("camera preview dimensions must be positive")
        self.topic = topic.strip()
        self.maximum_fps = float(maximum_fps)
        self._maximum_size = (
            (int(maximum_width), int(maximum_height))
            if maximum_width is not None and maximum_height is not None
            else None
        )
        self._frames: Queue[CameraFrame] = Queue(maxsize=1)
        self._errors: Queue[str] = Queue(maxsize=1)
        self._stop = Event()
        self._image_ready = Event()
        self._image_lock = Lock()
        self._latest_image: Image | None = None
        self._received_frames = 0
        self._dropped_frames = 0
        self._decoded_frames = 0
        self._node: Node = rclpy.create_node(node_name.strip())
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._subscription = self._node.create_subscription(
            Image, self.topic, self._on_image, CAMERA_QOS)
        self._thread = Thread(target=self._spin, name="skills-camera-preview", daemon=True)
        self._decode_thread = Thread(
            target=self._decode_latest_images,
            name="skills-camera-decode",
            daemon=True,
        )
        self._thread.start()
        self._decode_thread.start()

    def _spin(self) -> None:
        while not self._stop.is_set():
            self._executor.spin_once(timeout_sec=0.1)

    @staticmethod
    def _put_latest(queue: Queue, value: object) -> None:
        try:
            queue.put_nowait(value)
            return
        except Full:
            pass
        try:
            queue.get_nowait()
        except Empty:
            pass
        queue.put_nowait(value)

    def _on_image(self, message: Image) -> None:
        # ROS callbacks must be short. Keep exactly one image and replace it
        # when a newer frame arrives; decoding happens outside the executor.
        with self._image_lock:
            if self._latest_image is not None:
                self._dropped_frames += 1
            self._latest_image = message
            self._received_frames += 1
        self._image_ready.set()

    def _decode_latest_images(self) -> None:
        minimum_period = 1.0 / self.maximum_fps
        next_decode_time = 0.0
        while not self._stop.is_set():
            self._image_ready.wait(timeout=0.1)
            if self._stop.is_set():
                return
            now = monotonic()
            if now < next_decode_time:
                self._stop.wait(next_decode_time - now)
                if self._stop.is_set():
                    return
            self._image_ready.clear()
            with self._image_lock:
                message = self._latest_image
                self._latest_image = None
            if message is None:
                continue
            try:
                frame = decode_ros_image(message)
                # Downsampling is pure byte work, so do it here rather than
                # blocking Tk's event loop with a large image resize.
                if self._maximum_size is not None:
                    frame = frame.resized_to_fit(*self._maximum_size)
                self._put_latest(self._frames, frame)
                with self._image_lock:
                    self._decoded_frames += 1
            except ImageDecodeError as error:
                self._put_latest(self._errors, str(error))
            next_decode_time = monotonic() + minimum_period

    def latest_frame(self) -> CameraFrame | None:
        frame: CameraFrame | None = None
        while True:
            try:
                frame = self._frames.get_nowait()
            except Empty:
                return frame

    def latest_error(self) -> str | None:
        error: str | None = None
        while True:
            try:
                error = self._errors.get_nowait()
            except Empty:
                return error

    def statistics(self) -> tuple[int, int, int]:
        """Return received, discarded-stale, and decoded frame counts."""
        with self._image_lock:
            return self._received_frames, self._dropped_frames, self._decoded_frames

    def close(self) -> None:
        if self._stop.is_set():
            return
        self._stop.set()
        self._image_ready.set()
        self._executor.shutdown(timeout_sec=1.0)
        self._thread.join(timeout=1.0)
        self._decode_thread.join(timeout=1.0)
        self._node.destroy_node()
