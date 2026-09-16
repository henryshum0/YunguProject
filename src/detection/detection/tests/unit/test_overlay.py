"""The debug overlay drawing, on raw ROS image buffers."""

from __future__ import annotations

import numpy as np
import pytest

from detection.overlay import ImageOverlayError, OverlayBox, decode, draw, encode


def blank(width: int = 64, height: int = 48, value: int = 30) -> np.ndarray:
    return np.full((height, width, 3), value, dtype=np.uint8)


def test_decode_and_encode_round_trip_both_channel_orders() -> None:
    image = blank()
    image[0, 0] = (10, 20, 30)
    for encoding in ("rgb8", "bgr8"):
        raw = encode(image, encoding)
        assert np.array_equal(decode(64, 48, 64 * 3, encoding, raw), image)


def test_decode_honours_row_padding() -> None:
    padded_step = 64 * 3 + 7
    raw = bytes(bytearray(padded_step * 48))
    assert decode(64, 48, padded_step, "rgb8", raw).shape == (48, 64, 3)


def test_decode_rejects_unusable_buffers() -> None:
    with pytest.raises(ImageOverlayError, match="unsupported image encoding"):
        decode(64, 48, 192, "mono8", bytes(192 * 48))
    with pytest.raises(ImageOverlayError, match="dimensions must be positive"):
        decode(0, 48, 192, "rgb8", bytes(192 * 48))
    with pytest.raises(ImageOverlayError, match="smaller than its declared geometry"):
        decode(64, 48, 192, "rgb8", bytes(100))


def test_draw_marks_the_box_edges_and_leaves_the_interior_alone() -> None:
    image = blank()
    draw(image, (OverlayBox(10.0, 20.0, 40.0, 40.0, "car", 0.84),), thickness=2)
    assert (image[20, 10:40] != 30).any()   # top edge
    assert (image[39, 10:40] != 30).any()   # bottom edge
    assert (image[25:35, 25] == 30).all()   # interior untouched


def test_classes_are_drawn_in_different_colours() -> None:
    car, pedestrian = blank(), blank()
    box = OverlayBox(10.0, 20.0, 40.0, 40.0, "car", 0.5)
    draw(car, (box,))
    draw(pedestrian, (OverlayBox(10.0, 20.0, 40.0, 40.0, "pedestrian", 0.5),))
    assert not np.array_equal(car, pedestrian)


def test_boxes_outside_the_image_do_not_raise() -> None:
    image = blank()
    draw(image, (
        OverlayBox(-100.0, -100.0, -50.0, -50.0, "car", 0.5),
        OverlayBox(500.0, 500.0, 900.0, 900.0, "van", 0.5),
        OverlayBox(60.0, 44.0, 200.0, 200.0, "pedestrian", 0.5),
    ))
    assert image.shape == (48, 64, 3)


def test_the_label_is_drawn_above_the_box() -> None:
    labelled, unlabelled = blank(width=200, height=100), blank(width=200, height=100)
    draw(labelled, (OverlayBox(20.0, 40.0, 120.0, 80.0, "car", 0.84),))
    draw(unlabelled, (OverlayBox(20.0, 40.0, 120.0, 80.0, "car", 0.84),), thickness=2)
    # Rows above the box carry the label, so they differ from the empty image.
    assert (labelled[26:39, 20:120] != 30).any()
