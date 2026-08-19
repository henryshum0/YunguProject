from shapely.geometry import Point

from coverage_planner.models import CameraConfig, SemanticMap
from coverage_planner.visibility import visible_detection_ground


def test_building_wall_removes_ground_behind_it_from_visible_area() -> None:
    semantic = SemanticMap.model_validate({
        "schema_version": "1.0", "world_name": "occlusion", "coordinate_frame": "ENU",
        "units": "meters", "search_area": {"kind": "rectangle", "coords": [
            [-20, -20], [20, -20], [20, 20], [-20, 20]]},
        "nodes": [{"id": "building", "properties": {"category": "building",
            "type": "office", "label": "building", "passability": "restricted",
            "visibility": "public", "elevation_min_m": 0, "elevation_max_m": 8},
            "shape": {"type": "rectangle", "min_corner": [-2, 2],
                      "max_corner": [2, 4]}}],
        "metadata": {"ground_truth_excluded": True, "source": "test"},
    })
    camera = CameraConfig(image_width_px=100, image_height_px=100,
        horizontal_fov_deg=120, vertical_fov_deg=120, pitch_deg=-90,
        yaw_mode="follow_path", forward_overlap=0.2, side_overlap=0.2)
    visible = visible_detection_ground(
        camera=camera, center_enu_m=(0, 0), flight_altitude_m=10,
        ground_elevation_m=0, yaw_deg=0, semantic_map=semantic)
    assert visible.covers(Point(8, 0))
    assert not visible.covers(Point(0, 8))
    assert not visible.covers(Point(0, 3))


def test_full_target_and_image_margin_reduce_valid_ground_footprint() -> None:
    semantic = SemanticMap.model_validate({
        "schema_version": "1.0", "world_name": "empty", "coordinate_frame": "ENU",
        "units": "meters", "search_area": {"kind": "rectangle", "coords": [
            [-20, -20], [20, -20], [20, 20], [-20, 20]]}, "nodes": [],
        "metadata": {"ground_truth_excluded": True, "source": "test"},
    })
    base = CameraConfig(image_width_px=100, image_height_px=100,
        horizontal_fov_deg=90, vertical_fov_deg=90, pitch_deg=-90,
        yaw_mode="follow_path", forward_overlap=0.2, side_overlap=0.2)
    conservative = base.model_copy(update={"target_width_m": 2, "target_length_m": 4,
                                           "target_height_m": 2,
                                           "image_boundary_margin_ratio": 0.1})
    raw = visible_detection_ground(camera=base, center_enu_m=(0, 0),
        flight_altitude_m=10, ground_elevation_m=0, yaw_deg=0, semantic_map=semantic)
    valid = visible_detection_ground(camera=conservative, center_enu_m=(0, 0),
        flight_altitude_m=10, ground_elevation_m=0, yaw_deg=0, semantic_map=semantic)
    assert valid.area < raw.area

