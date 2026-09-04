from __future__ import annotations

import pytest

from gui.input_parser import parse_corners, parse_frame, parse_timeout, parse_waypoints


def test_parse_waypoints_supports_comma_or_space_separated_rows() -> None:
    assert parse_waypoints("1, 2, 3, 90\n4 5 6 180") == (
        (1.0, 2.0, 3.0, 90.0),
        (4.0, 5.0, 6.0, 180.0),
    )


@pytest.mark.parametrize("text", ("", "1, 2, 3", "1, 2, nope, 0", "1, 2, nan, 0"))
def test_parse_waypoints_rejects_invalid_rows(text: str) -> None:
    with pytest.raises(ValueError):
        parse_waypoints(text)


def test_parse_corners_requires_four_distinct_finite_values() -> None:
    assert parse_corners((("0", "0"), ("10", "0"), ("10", "5"), ("0", "5"))) == (
        (0.0, 0.0), (10.0, 0.0), (10.0, 5.0), (0.0, 5.0))
    with pytest.raises(ValueError, match="distinct"):
        parse_corners((("0", "0"), ("10", "0"), ("10", "5"), ("0", "0")))
    with pytest.raises(ValueError, match="finite"):
        parse_corners((("0", "0"), ("10", "0"), ("nan", "5"), ("0", "5")))


def test_parse_frame_and_timeout() -> None:
    assert parse_frame("NED") == "ned"
    assert parse_timeout("2.5") == 2.5
    with pytest.raises(ValueError):
        parse_frame("map")
    with pytest.raises(ValueError):
        parse_timeout("0")
