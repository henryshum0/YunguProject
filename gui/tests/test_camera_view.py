from __future__ import annotations

from types import SimpleNamespace

import pytest

from gui.camera_view import CameraFrame, ImageDecodeError, decode_ros_image


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


def test_camera_frame_resizes_to_fit_without_distorting_aspect_ratio() -> None:
    frame = CameraFrame(width=4, height=2, rgb=bytes((10, 20, 30)) * 8)
    preview = frame.resized_to_fit(3, 3)

    assert (preview.width, preview.height) == (3, 1)
    assert len(preview.rgb) == 3 * 1 * 3

    native = frame.resized_to_fit(8, 8)
    assert native is frame


def test_subsample_factor_fits_the_frame_into_the_preview_box() -> None:
    """The factor handed to Tk must always land inside the box, never over it."""
    frame = CameraFrame(width=640, height=480, rgb=b"\x00" * (640 * 480 * 3))
    assert frame.subsample_factor(320, 240) == 2
    # A feed rendered at the preview size costs no reduction at all.
    assert frame.subsample_factor(640, 480) == 1
    assert frame.subsample_factor(1280, 960) == 1
    # Non-integer ratios round up rather than overflowing the box.
    factor = frame.subsample_factor(300, 240)
    assert factor == 3
    assert -(-640 // factor) <= 300 and -(-480 // factor) <= 240


def test_subsample_factor_rejects_an_empty_preview_box() -> None:
    frame = CameraFrame(width=4, height=2, rgb=bytes((0, 0, 0)) * 8)
    with pytest.raises(ValueError, match="positive"):
        frame.subsample_factor(0, 10)


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
