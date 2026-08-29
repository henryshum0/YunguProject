"""Coordinate transforms between planner-local ENU and world ENU.

Both frames are ENU meters with the same axis orientation; the relation is a
rotation about z followed by a translation:

    p_world = Rz(yaw_deg) * p_map + T
    p_map   = Rz(-yaw_deg) * (p_world - T)

Calibration of T is described in docs/coverage-search-integration.md section 3
(method A/B); yaw_deg is 0.0 when the map axes align with world.
"""

import math


def to_world(x, y, z, yaw_deg, tx, ty, tz):
    """Map a planner-local ENU point into world ENU."""
    a = math.radians(yaw_deg)
    ca, sa = math.cos(a), math.sin(a)
    return (
        ca * x - sa * y + tx,
        sa * x + ca * y + ty,
        z + tz,
    )


def to_map(x, y, z, yaw_deg, tx, ty, tz):
    """Map a world ENU point back into planner-local ENU."""
    a = math.radians(yaw_deg)
    ca, sa = math.cos(a), math.sin(a)
    dx, dy, dz = x - tx, y - ty, z - tz
    return (
        ca * dx + sa * dy,
        -sa * dx + ca * dy,
        dz,
    )


def yaw_from_quat(qx, qy, qz, qw):
    """Extract the ENU yaw of a z-axis rotation (assumes no roll/pitch)."""
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def quat_from_yaw(yaw):
    """ENU yaw as a z-axis quaternion (qx=qy=0)."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))
