"""Camera geometry: projection direction, framing, and box handling."""

from __future__ import annotations

from math import isclose, radians, tan

import numpy as np
import pytest

from detection.camera import (
    CameraGeometryError,
    Extrinsics,
    PinholeCamera,
    PixelBox,
    box_corners,
    rotation_from_quaternion,
    rotation_from_rpy,
)


def camera() -> PinholeCamera:
    return PinholeCamera.from_horizontal_fov(
        width=640, height=480, horizontal_fov_deg=100.0, max_range_m=50.0)


def test_horizontal_fov_gives_the_gazebo_intrinsics() -> None:
    model = camera()
    assert isclose(model.fx, 320.0 / tan(radians(50.0)), rel_tol=1e-12)
    assert model.fx == model.fy  # Gazebo cameras have square pixels
    assert (model.cx, model.cy) == (320.0, 240.0)
    assert isclose(model.vertical_fov_deg, 83.58, abs_tol=0.01)


def test_projection_follows_the_gazebo_optical_convention() -> None:
    model = camera()
    # +X is the optical axis, +Y is image-left and +Z is image-up.
    ahead, left, above = model.project(
        np.array([[10.0, 0.0, 0.0], [10.0, 1.0, 0.0], [10.0, 0.0, 1.0]]))
    assert (ahead[0], ahead[1]) == (model.cx, model.cy)
    assert left[0] < model.cx and isclose(left[1], model.cy)
    assert above[1] < model.cy and isclose(above[0], model.cx)


def test_points_behind_the_camera_have_no_projection() -> None:
    pixels = camera().project(np.array([[-1.0, 0.0, 0.0], [0.0, 1.0, 1.0]]))
    assert np.isnan(pixels).all()


def test_ray_direction_inverts_the_projection() -> None:
    model = camera()
    point = np.array([[12.0, -3.0, 2.0]])
    pixel = model.project(point)[0]
    direction = model.ray_direction(*pixel)
    recovered = direction * (point[0][0] / direction[0])
    assert np.allclose(recovered, point[0], atol=1e-9)


def test_image_overlap_and_clipping() -> None:
    model = camera()
    inside = PixelBox(100.0, 100.0, 200.0, 160.0)
    assert model.image_overlap_fraction(inside) == 1.0
    assert model.clip(inside) == inside

    half_out = PixelBox(-50.0, 100.0, 50.0, 200.0)
    assert isclose(model.image_overlap_fraction(half_out), 0.5)
    assert model.clip(half_out) == PixelBox(0.0, 100.0, 50.0, 200.0)

    outside = PixelBox(700.0, 100.0, 800.0, 200.0)
    assert model.image_overlap_fraction(outside) == 0.0


def test_pixel_box_geometry() -> None:
    box = PixelBox.from_center(100.0, 50.0, 40.0, 10.0)
    assert (box.min_x, box.max_x, box.min_y, box.max_y) == (80.0, 120.0, 45.0, 55.0)
    assert (box.size_x, box.size_y, box.short_side) == (40.0, 10.0, 10.0)
    assert box.area == 400.0
    assert PixelBox.bounding(np.array([[1.0, 2.0], [5.0, 0.0]])) == PixelBox(1.0, 0.0, 5.0, 2.0)
    with pytest.raises(CameraGeometryError):
        PixelBox.bounding(np.array([[np.nan, 0.0]]))


def test_extrinsics_round_trip_and_composition() -> None:
    body = Extrinsics.from_rpy((1.0, 2.0, 3.0), 0.0, 0.0, 90.0)
    point_world = np.array([[4.0, 2.0, 3.0]])
    point_body = body.to_child(point_world)
    assert np.allclose(body.to_parent(point_body), point_world, atol=1e-12)

    camera_in_body = Extrinsics.from_rpy((0.30, 0.0, -0.13), 0.0, 60.0, 0.0)
    chained = body.compose(camera_in_body)
    direct = camera_in_body.to_child(body.to_child(point_world))
    assert np.allclose(chained.to_child(point_world), direct, atol=1e-12)


def test_sixty_degree_pitch_points_the_optical_axis_down_and_forward() -> None:
    rotation = rotation_from_rpy(0.0, radians(60.0), 0.0)
    axis = rotation @ np.array([1.0, 0.0, 0.0])
    assert isclose(axis[0], 0.5, abs_tol=1e-9)
    assert isclose(axis[2], -0.8660254, abs_tol=1e-6)


def test_rotation_from_quaternion_matches_yaw() -> None:
    yaw = radians(30.0)
    quaternion = (np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2))
    assert np.allclose(rotation_from_quaternion(quaternion), rotation_from_rpy(0.0, 0.0, yaw),
                       atol=1e-12)
    with pytest.raises(CameraGeometryError):
        rotation_from_quaternion((0.0, 0.0, 0.0, 0.0))


def test_box_corners_rotate_about_the_centre() -> None:
    corners = box_corners((0.0, 0.0, 1.0), (4.0, 2.0, 2.0), yaw_deg=90.0)
    assert corners.shape == (8, 3)
    # A 90-degree yaw swaps the box's length and width extents.
    assert isclose(corners[:, 0].max() - corners[:, 0].min(), 2.0, abs_tol=1e-9)
    assert isclose(corners[:, 1].max() - corners[:, 1].min(), 4.0, abs_tol=1e-9)
    assert isclose(corners[:, 2].min(), 0.0, abs_tol=1e-9)
    with pytest.raises(CameraGeometryError):
        box_corners((0.0, 0.0, 0.0), (1.0, 0.0, 1.0), yaw_deg=0.0)
