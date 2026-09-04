"""Validation helpers shared by the skills test GUI and its tests."""

from __future__ import annotations

from math import isfinite
from typing import Sequence


Waypoint = tuple[float, float, float, float]
Corner = tuple[float, float]


def parse_waypoints(text: str) -> tuple[Waypoint, ...]:
    """Parse non-empty ``x, y, z, heading_deg`` rows."""
    rows = [line.strip() for line in text.splitlines() if line.strip()]
    if not rows:
        raise ValueError("enter at least one waypoint row")
    parsed: list[Waypoint] = []
    for row_number, row in enumerate(rows, start=1):
        fields = row.replace(",", " ").split()
        if len(fields) != 4:
            raise ValueError(f"waypoint row {row_number} must contain x, y, z, heading_deg")
        parsed.append(tuple(_finite(value, f"waypoint row {row_number}") for value in fields))
    return tuple(parsed)  # type: ignore[return-value]


def parse_corners(rows: Sequence[tuple[str, str]]) -> tuple[Corner, ...]:
    """Parse exactly four distinct ENU ``(x, y)`` search corners."""
    if len(rows) != 4:
        raise ValueError("enter exactly four search-area corners")
    corners = tuple(
        (_finite(x, f"corner {index} x"), _finite(y, f"corner {index} y"))
        for index, (x, y) in enumerate(rows, start=1)
    )
    if len(set(corners)) != 4:
        raise ValueError("search-area corners must be distinct")
    return corners


def parse_frame(frame: str) -> str:
    """Normalize the supported navigation-frame selector."""
    normalized = frame.strip().lower()
    if normalized not in {"enu", "ned"}:
        raise ValueError("navigation frame must be ENU or NED")
    return normalized


def parse_timeout(value: str) -> float:
    """Parse a strictly positive service timeout in seconds."""
    timeout = _finite(value, "timeout")
    if timeout <= 0.0:
        raise ValueError("timeout must be greater than zero")
    return timeout


def _finite(value: str, label: str) -> float:
    try:
        number = float(value)
    except ValueError as exc:
        raise ValueError(f"{label} must be a number") from exc
    if not isfinite(number):
        raise ValueError(f"{label} must be finite")
    return number
