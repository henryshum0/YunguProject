from __future__ import annotations

from types import SimpleNamespace

import pytest

from gui.camera_view import ImageDecodeError, decode_ros_image


def image(*, width: int, height: int, encoding: str, step: int, data: bytes):
    return SimpleNamespace(width=width, height=height, encoding=encoding, step=step, data=data)


def test_decode_ros_image_converts_bgr_and_row_padding_to_rgb() -> None:
    frame = decode_ros_image(image(
        width=2, height=1, encoding="bgr8", step=8,
        data=bytes((3, 2, 1, 30, 20, 10, 99, 99)),
    ))
    assert frame.width == 2
    assert frame.height == 1
    assert frame.rgb == bytes((1, 2, 3, 10, 20, 30))
    assert frame.ppm_bytes().startswith(b"P6\n2 1\n255\n")


@pytest.mark.parametrize(
    ("encoding", "data", "expected"),
    [
        ("rgb8", bytes((1, 2, 3)), bytes((1, 2, 3))),
        ("rgba8", bytes((1, 2, 3, 99)), bytes((1, 2, 3))),
        ("bgra8", bytes((3, 2, 1, 99)), bytes((1, 2, 3))),
        ("mono8", bytes((17,)), bytes((17, 17, 17))),
    ],
)
def test_decode_ros_image_supports_standard_encodings(encoding: str, data: bytes, expected: bytes) -> None:
    assert decode_ros_image(image(width=1, height=1, encoding=encoding, step=len(data), data=data)).rgb == expected


@pytest.mark.parametrize(
    "message, error",
    [
        (image(width=0, height=1, encoding="rgb8", step=3, data=b"123"), "dimensions"),
        (image(width=1, height=1, encoding="16UC1", step=2, data=b"12"), "unsupported"),
        (image(width=2, height=1, encoding="rgb8", step=5, data=b"12345"), "step"),
        (image(width=1, height=2, encoding="mono8", step=1, data=b"1"), "expected"),
    ],
)
def test_decode_ros_image_reports_invalid_input(message, error: str) -> None:
    with pytest.raises(ImageDecodeError, match=error):
        decode_ros_image(message)
