"""Dependency-free ROS image decoding and threaded camera subscription helpers."""

from __future__ import annotations

from dataclasses import dataclass
from queue import Empty, Full, Queue
from threading import Event, Thread
from typing import Any

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


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
    """Own an image-only ROS node/executor so it never races GUI service calls."""

    def __init__(self, topic: str) -> None:
        if not topic.strip():
            raise ValueError("camera image topic must not be empty")
        self.topic = topic.strip()
        self._frames: Queue[CameraFrame] = Queue(maxsize=1)
        self._errors: Queue[str] = Queue(maxsize=1)
        self._stop = Event()
        self._node: Node = rclpy.create_node("skills_test_gui_camera")
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._subscription = self._node.create_subscription(
            Image, self.topic, self._on_image, qos_profile_sensor_data)
        self._thread = Thread(target=self._spin, name="skills-camera-preview", daemon=True)
        self._thread.start()

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
        try:
            self._put_latest(self._frames, decode_ros_image(message))
        except ImageDecodeError as error:
            self._put_latest(self._errors, str(error))

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

    def close(self) -> None:
        if self._stop.is_set():
            return
        self._stop.set()
        self._executor.shutdown(timeout_sec=1.0)
        self._thread.join(timeout=1.0)
        self._node.destroy_node()
