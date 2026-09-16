"""Back-projecting detections onto the ground plane, and pose lookup by stamp."""

from __future__ import annotations

from math import cos, isclose, radians, sin

import pytest

from detection.camera import Extrinsics, PinholeCamera, PixelBox
from detection.localization import (
    PoseBuffer,
    TimedPose,
    anchor_pixel,
    ground_footprint,
    ground_intersection,
    ground_position,
)


CAMERA = PinholeCamera.from_horizontal_fov(
    width=640, height=480, horizontal_fov_deg=100.0, max_range_m=50.0)
EXTRINSICS = Extrinsics.from_rpy((0.30, 0.0, -0.13), 0.0, 60.0, 0.0)
GROUND_Z = -1.654


def pose(x=0.0, y=0.0, z=15.0, yaw_deg=0.0, stamp=100.0) -> TimedPose:
    half = radians(yaw_deg) / 2.0
    return TimedPose(stamp_sec=stamp, position=(x, y, z),
                     orientation=(cos(half), 0.0, 0.0, sin(half)))


def test_the_image_centre_lands_on_the_optical_axis_ground_point() -> None:
    placed = ground_intersection(CAMERA, EXTRINSICS, pose(), (320.0, 240.0),
                                 ground_z_m=GROUND_Z, max_range_m=80.0)
    assert placed is not None
    (x, y, z), distance = placed
    height = 15.0 - 0.13 - GROUND_Z
    # The optical axis is pitched 60 degrees down, so it meets the ground at
    # height / tan(60) ahead of the camera.
    assert isclose(x, 0.30 + height / (3 ** 0.5), abs_tol=1e-6)
    assert isclose(y, 0.0, abs_tol=1e-9)
    assert isclose(z, GROUND_Z, abs_tol=1e-9)
    assert isclose(distance, height / sin(radians(60.0)), rel_tol=1e-6)


def test_yaw_rotates_the_ground_point_with_the_vehicle() -> None:
    east = ground_intersection(CAMERA, EXTRINSICS, pose(yaw_deg=0.0), (320.0, 240.0),
                               ground_z_m=GROUND_Z, max_range_m=80.0)
    north = ground_intersection(CAMERA, EXTRINSICS, pose(yaw_deg=90.0), (320.0, 240.0),
                                ground_z_m=GROUND_Z, max_range_m=80.0)
    assert east is not None and north is not None
    assert isclose(east[0][0], north[0][1], abs_tol=1e-6)
    assert isclose(north[0][0], 0.0, abs_tol=1e-6)


def test_rays_at_or_above_the_horizon_have_no_ground_point() -> None:
    # The top image rows look up when the camera is level.
    level = Extrinsics.from_rpy((0.0, 0.0, 0.0), 0.0, 0.0, 0.0)
    assert ground_intersection(CAMERA, level, pose(), (320.0, 0.0),
                               ground_z_m=GROUND_Z, max_range_m=80.0) is None


def test_a_ground_point_beyond_the_range_limit_is_rejected() -> None:
    assert ground_intersection(CAMERA, EXTRINSICS, pose(z=60.0), (320.0, 240.0),
                               ground_z_m=GROUND_Z, max_range_m=20.0) is None


def test_the_bottom_anchor_is_the_ground_contact_point() -> None:
    box = PixelBox.from_center(320.0, 200.0, 40.0, 20.0)
    assert anchor_pixel(box, "bottom_center") == (320.0, 210.0)
    assert anchor_pixel(box, "center") == (320.0, 200.0)
    with pytest.raises(ValueError, match="unsupported localizer anchor"):
        anchor_pixel(box, "top")

    lower = ground_position(CAMERA, EXTRINSICS, pose(), box, anchor="bottom_center",
                            ground_z_m=GROUND_Z, max_range_m=80.0)
    centre = ground_position(CAMERA, EXTRINSICS, pose(), box, anchor="center",
                             ground_z_m=GROUND_Z, max_range_m=80.0)
    assert lower is not None and centre is not None
    # Lower in the image means closer to the vehicle on the ground.
    assert lower[0][0] < centre[0][0]


def test_ground_footprint_grows_with_the_box_width() -> None:
    narrow = ground_footprint(CAMERA, EXTRINSICS, pose(), PixelBox.from_center(320.0, 240.0, 20.0, 20.0),
                              ground_z_m=GROUND_Z, max_range_m=80.0)
    wide = ground_footprint(CAMERA, EXTRINSICS, pose(), PixelBox.from_center(320.0, 240.0, 80.0, 20.0),
                            ground_z_m=GROUND_Z, max_range_m=80.0)
    assert narrow is not None and wide is not None
    assert wide[0] > narrow[0] > 0.0


def test_projection_and_back_projection_agree_on_a_ground_point() -> None:
    """A point on the ground projects to a pixel that back-projects to itself."""
    import numpy as np

    vehicle = pose(x=2.0, y=-1.0, yaw_deg=35.0)
    ground_point = np.array([[9.0, 4.0, GROUND_Z]])
    camera_in_world = vehicle.body_in_world().compose(EXTRINSICS)
    pixel = CAMERA.project(camera_in_world.to_child(ground_point))[0]
    placed = ground_intersection(CAMERA, EXTRINSICS, vehicle, tuple(pixel),
                                 ground_z_m=GROUND_Z, max_range_m=80.0)
    assert placed is not None
    assert np.allclose(placed[0], ground_point[0], atol=1e-6)


def test_pose_buffer_returns_the_nearest_sample_and_drops_old_ones() -> None:
    buffer = PoseBuffer(history_sec=2.0)
    for index in range(40):
        buffer.add(pose(x=float(index), stamp=100.0 + index * 0.1))
    assert len(buffer) < 40  # older than the 2 s window were dropped

    nearest = buffer.nearest(103.42, max_age_sec=0.3)
    assert nearest is not None
    assert isclose(nearest.stamp_sec, 103.4, abs_tol=1e-6)
    assert buffer.nearest(90.0, max_age_sec=0.3) is None
    assert PoseBuffer(history_sec=1.0).nearest(100.0, max_age_sec=1.0) is None


def test_pose_buffer_rejects_an_invalid_window() -> None:
    with pytest.raises(ValueError, match="positive finite"):
        PoseBuffer(history_sec=0.0)
