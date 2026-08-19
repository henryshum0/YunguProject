import json

import pytest
from shapely.geometry import box

from coverage_planner.models import CameraConfig, SemanticMap
from coverage_planner.multi_planner import DroneAssignment, TwoDroneCoveragePlanner
from coverage_planner.reporting import export_multi_plan


def _semantic_map() -> SemanticMap:
    return SemanticMap.model_validate({"schema_version": "1.0", "world_name": "dual",
        "coordinate_frame": "ENU", "units": "meters",
        "search_area": {"kind": "rectangle", "coords": [[0, 0], [40, 0],
            [40, 20], [0, 20]]}, "nodes": [],
        "metadata": {"ground_truth_excluded": True, "source": "test"}})


def _camera() -> CameraConfig:
    return CameraConfig(image_width_px=100, image_height_px=100,
        horizontal_fov_deg=90, vertical_fov_deg=90, pitch_deg=-90,
        yaw_mode="follow_path", forward_overlap=0.5, side_overlap=0.5)


def _options() -> dict[str, object]:
    return {"flight_altitude_m": 10, "horizontal_clearance_m": 1,
            "vertical_clearance_m": 2, "scan_direction_deg": 90}


def test_two_disjoint_assignments_produce_independent_exports(tmp_path) -> None:
    plan = TwoDroneCoveragePlanner().plan(assignments=(
        DroneAssignment("drone_1", box(0, 0, 20, 20), (0, 0, 10)),
        DroneAssignment("drone_2", box(20, 0, 40, 20), (40, 0, 10))),
        semantic_map=_semantic_map(), camera=_camera(), planner_options=_options())
    assert len(plan.drones) == 2
    assert plan.drones[0].result.visible_detection_geometry.intersection(
        plan.drones[1].result.visible_detection_geometry).area == pytest.approx(0)
    export_multi_plan(plan, tmp_path)
    manifest = json.loads((tmp_path / "mission_manifest.json").read_text())
    assert manifest["simultaneous_start"] is True
    assert manifest["temporal_collision_avoidance"] is False
    assert manifest["drones"][0]["responsibility_geometry"]["type"] == "Polygon"
    assert (tmp_path / "drone_1" / "flight_plan.json").is_file()
    assert (tmp_path / "drone_2" / "flight_plan.json").is_file()


def test_overlapping_responsibility_areas_are_rejected() -> None:
    with pytest.raises(ValueError, match="overlap"):
        TwoDroneCoveragePlanner().plan(assignments=(
            DroneAssignment("drone_1", box(0, 0, 25, 20), (0, 0, 10)),
            DroneAssignment("drone_2", box(20, 0, 40, 20), (40, 0, 10))),
            semantic_map=_semantic_map(), camera=_camera(), planner_options=_options())
