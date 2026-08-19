from math import sqrt

import pytest
from pydantic import ValidationError

from coverage_planner.camera import (
    CameraGeometryError,
    ground_footprint_dimensions,
    ground_footprint_polygon,
)
from coverage_planner.models.camera import CameraConfig


def camera_config(**overrides: object) -> CameraConfig:
    values: dict[str, object] = {
        "image_width_px": 1920,
        "image_height_px": 1080,
        "horizontal_fov_deg": 60.0,
        "vertical_fov_deg": 90.0,
        "pitch_deg": -90.0,
        "yaw_mode": "follow_path",
        "forward_overlap": 0.25,
        "side_overlap": 0.30,
        "minimum_ground_sampling_distance_cm_per_px": None,
    }
    values.update(overrides)
    return CameraConfig.model_validate(values)


def test_calculates_nadir_footprint_and_overlap_spacing() -> None:
    dimensions = ground_footprint_dimensions(
        camera_config(), flight_altitude_m=35.0, ground_elevation_m=5.0
    )
    assert dimensions.height_above_ground_m == 30.0
    assert dimensions.width_m == pytest.approx(20.0 * sqrt(3.0))
    assert dimensions.length_m == pytest.approx(60.0)
    assert dimensions.capture_spacing_m == pytest.approx(45.0)
    assert dimensions.scan_line_spacing_m == pytest.approx(14.0 * sqrt(3.0))


def test_uses_height_above_ground_not_absolute_altitude() -> None:
    low_ground = ground_footprint_dimensions(
        camera_config(), flight_altitude_m=30.0, ground_elevation_m=0.0
    )
    high_ground = ground_footprint_dimensions(
        camera_config(), flight_altitude_m=30.0, ground_elevation_m=15.0
    )
    assert high_ground.width_m == pytest.approx(low_ground.width_m / 2.0)
    assert high_ground.length_m == pytest.approx(low_ground.length_m / 2.0)


def test_yaw_zero_points_length_axis_north() -> None:
    footprint = ground_footprint_polygon(
        camera_config(),
        center_enu_m=(100.0, 200.0),
        flight_altitude_m=30.0,
        ground_elevation_m=0.0,
        yaw_deg=0.0,
    )
    min_x, min_y, max_x, max_y = footprint.bounds
    assert (min_x, max_x) == pytest.approx((100.0 - 10.0 * sqrt(3.0), 100.0 + 10.0 * sqrt(3.0)))
    assert (min_y, max_y) == pytest.approx((170.0, 230.0))


def test_yaw_ninety_points_length_axis_east() -> None:
    footprint = ground_footprint_polygon(
        camera_config(),
        center_enu_m=(0.0, 0.0),
        flight_altitude_m=30.0,
        ground_elevation_m=0.0,
        yaw_deg=90.0,
    )
    min_x, min_y, max_x, max_y = footprint.bounds
    assert (min_x, max_x) == pytest.approx((-30.0, 30.0))
    assert (min_y, max_y) == pytest.approx((-10.0 * sqrt(3.0), 10.0 * sqrt(3.0)))
    assert footprint.area == pytest.approx(1200.0 * sqrt(3.0))


@pytest.mark.parametrize("ground_elevation_m", [30.0, 31.0])
def test_rejects_camera_at_or_below_ground(ground_elevation_m: float) -> None:
    with pytest.raises(CameraGeometryError, match="greater than ground_elevation_m"):
        ground_footprint_dimensions(
            camera_config(), flight_altitude_m=30.0, ground_elevation_m=ground_elevation_m
        )


def test_rejects_oblique_camera_projection() -> None:
    with pytest.raises(ValidationError, match="oblique camera projection not implemented"):
        camera_config(pitch_deg=-45.0)


@pytest.mark.parametrize("field,value", [("forward_overlap", 1.0), ("side_overlap", -0.1)])
def test_rejects_invalid_overlap(field: str, value: float) -> None:
    with pytest.raises(ValidationError):
        camera_config(**{field: value})


def test_validates_fixed_yaw_configuration() -> None:
    camera = camera_config(yaw_mode="fixed", fixed_yaw_deg=270.0)
    assert camera.fixed_yaw_deg == 270.0
    with pytest.raises(ValidationError, match="fixed_yaw_deg is required"):
        camera_config(yaw_mode="fixed")
    with pytest.raises(ValidationError, match="only valid"):
        camera_config(fixed_yaw_deg=10.0)
