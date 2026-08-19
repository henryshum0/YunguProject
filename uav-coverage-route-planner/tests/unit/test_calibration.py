import pytest
from pydantic import TypeAdapter

from coverage_planner.geometry.calibration import (
    CalibrationConfig,
    CalibrationError,
    MapCalibration,
)

ADAPTER = TypeAdapter(CalibrationConfig)


def test_origin_scale_accounts_for_downward_image_y() -> None:
    config = ADAPTER.validate_python({
        "mode": "origin_scale", "pixel_origin": [100, 200], "enu_origin_m": [10, 20],
        "east_direction_image": [1, 0], "north_direction_image": [0, -1],
        "meters_per_pixel": 0.5,
    })
    calibration = MapCalibration(config)
    assert calibration.pixel_to_enu((104, 194)) == pytest.approx((12, 23))
    assert calibration.enu_to_pixel((12, 23)) == pytest.approx((104, 194))


def test_bounds_maps_image_corners_to_enu_bounds() -> None:
    config = ADAPTER.validate_python({
        "mode": "bounds", "image_width_px": 101, "image_height_px": 51,
        "min_x": -5, "max_x": 15, "min_y": 10, "max_y": 20,
    })
    calibration = MapCalibration(config)
    assert calibration.pixel_to_enu((0, 0)) == pytest.approx((-5, 20))
    assert calibration.pixel_to_enu((100, 50)) == pytest.approx((15, 10))
    calibration.validate_image_size(101, 51)


def test_bounds_rejects_mismatched_image_dimensions() -> None:
    config = ADAPTER.validate_python({
        "mode": "bounds", "image_width_px": 101, "image_height_px": 51,
        "min_x": -5, "max_x": 15, "min_y": 10, "max_y": 20,
    })
    calibration = MapCalibration(config)
    with pytest.raises(ValueError, match="do not match"):
        calibration.validate_image_size(100, 51)


def test_affine_control_points_report_residual_and_round_trip() -> None:
    config = ADAPTER.validate_python({
        "mode": "affine", "maximum_residual_m": 1e-8,
        "control_points": [
            {"pixel": [0, 0], "enu_m": [10, 20]},
            {"pixel": [100, 0], "enu_m": [60, 20]},
            {"pixel": [0, 100], "enu_m": [10, -30]},
            {"pixel": [100, 100], "enu_m": [60, -30]},
        ],
    })
    calibration = MapCalibration(config)
    assert calibration.maximum_residual_m < 1e-10
    assert calibration.pixel_to_enu((40, 30)) == pytest.approx((30, 5))
    assert calibration.enu_to_pixel((30, 5)) == pytest.approx((40, 30))


def test_affine_rejects_collinear_control_points() -> None:
    config = ADAPTER.validate_python({
        "mode": "affine",
        "control_points": [
            {"pixel": [0, 0], "enu_m": [0, 0]},
            {"pixel": [1, 1], "enu_m": [1, 1]},
            {"pixel": [2, 2], "enu_m": [2, 2]},
        ],
    })
    with pytest.raises(ValueError, match="collinear"):
        MapCalibration(config)


def test_load_wraps_invalid_calibration_with_source_path(tmp_path) -> None:
    source = tmp_path / "calibration.json"
    source.write_text('{"mode": "unknown"}', encoding="utf-8")
    with pytest.raises(CalibrationError, match=str(source)):
        MapCalibration.load(source)


def test_exposes_defensive_copy_of_affine_matrix() -> None:
    config = ADAPTER.validate_python({
        "mode": "origin_scale", "pixel_origin": [0, 0], "meters_per_pixel": 2,
    })
    calibration = MapCalibration(config)
    matrix = calibration.pixel_to_enu_matrix
    matrix[0, 0] = 99
    assert calibration.pixel_to_enu((1, 0)) == pytest.approx((2, 0))
