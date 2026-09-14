"""Draw detection boxes onto a camera frame, for a look-and-see debug view.

Purely a debug aid, and deliberately dependency-free (no OpenCV, no cv_bridge):
it works directly on the raw ROS image buffer. Nothing in the detection or
search path consumes the overlay.

One caveat worth knowing while reading it: the bridged Gazebo image carries the
simulation clock while detections carry the wall clock the navigation stack runs
on, so boxes are drawn on the most recent frame rather than on the frame they
were computed from. At normal rates the two are within a frame of each other.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


class ImageOverlayError(ValueError):
    """Raised when an image buffer cannot be decoded for overlay drawing."""


#: Channel order of each supported encoding, as indices into an RGB triple.
_ENCODINGS = {
    "rgb8": (0, 1, 2),
    "bgr8": (2, 1, 0),
}

#: Box colour per class family, as RGB. Anything unlisted falls back to grey.
_CLASS_COLORS = {
    "pedestrian": (255, 96, 32),
    "people": (255, 150, 40),
    "car": (40, 200, 255),
    "van": (60, 170, 255),
    "truck": (90, 140, 255),
    "bus": (120, 120, 255),
    "motor": (255, 220, 60),
    "bicycle": (255, 240, 120),
}
_DEFAULT_COLOR = (200, 200, 200)

#: Minimal 3x5 bitmap font, enough for upper-case labels and scores.
_FONT = {
    "0": ("###", "#.#", "#.#", "#.#", "###"), "1": (".#.", "##.", ".#.", ".#.", "###"),
    "2": ("###", "..#", "###", "#..", "###"), "3": ("###", "..#", "###", "..#", "###"),
    "4": ("#.#", "#.#", "###", "..#", "..#"), "5": ("###", "#..", "###", "..#", "###"),
    "6": ("###", "#..", "###", "#.#", "###"), "7": ("###", "..#", "..#", "..#", "..#"),
    "8": ("###", "#.#", "###", "#.#", "###"), "9": ("###", "#.#", "###", "..#", "###"),
    ".": ("...", "...", "...", "...", ".#."), "-": ("...", "...", "###", "...", "..."),
    " ": ("...", "...", "...", "...", "..."),
    "A": ("###", "#.#", "###", "#.#", "#.#"), "B": ("##.", "#.#", "##.", "#.#", "##."),
    "C": ("###", "#..", "#..", "#..", "###"), "D": ("##.", "#.#", "#.#", "#.#", "##."),
    "E": ("###", "#..", "###", "#..", "###"), "F": ("###", "#..", "###", "#..", "#.."),
    "G": ("###", "#..", "#.#", "#.#", "###"), "H": ("#.#", "#.#", "###", "#.#", "#.#"),
    "I": ("###", ".#.", ".#.", ".#.", "###"), "J": ("..#", "..#", "..#", "#.#", "###"),
    "K": ("#.#", "#.#", "##.", "#.#", "#.#"), "L": ("#..", "#..", "#..", "#..", "###"),
    "M": ("#.#", "###", "###", "#.#", "#.#"), "N": ("##.", "#.#", "#.#", "#.#", "#.#"),
    "O": ("###", "#.#", "#.#", "#.#", "###"), "P": ("###", "#.#", "###", "#..", "#.."),
    "Q": ("###", "#.#", "#.#", "###", "..#"), "R": ("###", "#.#", "##.", "#.#", "#.#"),
    "S": ("###", "#..", "###", "..#", "###"), "T": ("###", ".#.", ".#.", ".#.", ".#."),
    "U": ("#.#", "#.#", "#.#", "#.#", "###"), "V": ("#.#", "#.#", "#.#", "#.#", ".#."),
    "W": ("#.#", "#.#", "###", "###", "#.#"), "X": ("#.#", "#.#", ".#.", "#.#", "#.#"),
    "Y": ("#.#", "#.#", "###", ".#.", ".#."), "Z": ("###", "..#", ".#.", "#..", "###"),
}
_GLYPH_WIDTH = 3
_GLYPH_HEIGHT = 5


@dataclass(frozen=True, slots=True)
class OverlayBox:
    """One box to draw: pixel bounds plus the label to write above it."""

    min_x: float
    min_y: float
    max_x: float
    max_y: float
    class_id: str
    score: float


def decode(width: int, height: int, step: int, encoding: str, data) -> np.ndarray:
    """Copy a raw ROS image buffer into a writable ``(H, W, 3)`` RGB array.

    ``data`` is any object supporting the buffer protocol, so the caller can pass
    the message field directly instead of materializing it as ``bytes`` first —
    at 640x480x30 fps that copy alone is worth avoiding.
    """
    order = _ENCODINGS.get(encoding.lower())
    if order is None:
        raise ImageOverlayError(
            f"unsupported image encoding '{encoding}'; supported: {', '.join(_ENCODINGS)}")
    if width <= 0 or height <= 0:
        raise ImageOverlayError("image dimensions must be positive")
    buffer = np.frombuffer(data, dtype=np.uint8)
    if step < width * 3 or buffer.size < step * height:
        raise ImageOverlayError("image buffer is smaller than its declared geometry")
    pixels = buffer[: step * height].reshape(height, step)[:, : width * 3].reshape(
        height, width, 3)
    # A view into the incoming message would be read-only and shared, so copy;
    # for RGB the channel order is already right and a plain copy is cheaper.
    return pixels.copy() if order == (0, 1, 2) else pixels[:, :, order].copy()


def encode(image: np.ndarray, encoding: str) -> bytes:
    """Serialize an ``(H, W, 3)`` RGB array back into a raw ROS image buffer."""
    order = _ENCODINGS.get(encoding.lower())
    if order is None:
        raise ImageOverlayError(f"unsupported image encoding '{encoding}'")
    if order != (0, 1, 2):
        image = image[:, :, order]
    return np.ascontiguousarray(image, dtype=np.uint8).tobytes()


def draw(image: np.ndarray, boxes: tuple[OverlayBox, ...], *, thickness: int = 2) -> np.ndarray:
    """Draw every box and its label onto an RGB image, in place."""
    for box in boxes:
        color = _CLASS_COLORS.get(box.class_id, _DEFAULT_COLOR)
        _rectangle(image, box, color, thickness)
        label = f"{box.class_id.upper().replace('-', ' ')} {box.score:.2f}"
        _text(image, label, int(round(box.min_x)), int(round(box.min_y)) - _GLYPH_HEIGHT * 2 - 2, color)
    return image


def _rectangle(image: np.ndarray, box: OverlayBox, color, thickness: int) -> None:
    height, width, _ = image.shape
    min_x = int(np.clip(round(box.min_x), 0, width - 1))
    max_x = int(np.clip(round(box.max_x), 0, width - 1))
    min_y = int(np.clip(round(box.min_y), 0, height - 1))
    max_y = int(np.clip(round(box.max_y), 0, height - 1))
    if max_x <= min_x or max_y <= min_y:
        return
    edge = max(1, thickness)
    image[min_y:min(min_y + edge, height), min_x:max_x + 1] = color
    image[max(max_y - edge + 1, 0):max_y + 1, min_x:max_x + 1] = color
    image[min_y:max_y + 1, min_x:min(min_x + edge, width)] = color
    image[min_y:max_y + 1, max(max_x - edge + 1, 0):max_x + 1] = color


def _text(image: np.ndarray, text: str, x: int, y: int, color, *, scale: int = 2) -> None:
    """Draw upper-case text with the built-in 3x5 font, clipped to the image."""
    height, width, _ = image.shape
    cursor = x
    top = max(y, 0)
    for character in text.upper():
        glyph = _FONT.get(character)
        if glyph is None:
            cursor += (_GLYPH_WIDTH + 1) * scale
            continue
        for row_index, row in enumerate(glyph):
            for column_index, pixel in enumerate(row):
                if pixel != "#":
                    continue
                pixel_x = cursor + column_index * scale
                pixel_y = top + row_index * scale
                if 0 <= pixel_x < width and 0 <= pixel_y < height:
                    image[pixel_y:min(pixel_y + scale, height),
                          pixel_x:min(pixel_x + scale, width)] = color
        cursor += (_GLYPH_WIDTH + 1) * scale
        if cursor >= width:
            return
