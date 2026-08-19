from shapely.geometry import box

from coverage_planner.coverage.continuous import build_continuous_flight_plan
from coverage_planner.models import CameraConfig, Waypoint


def camera() -> CameraConfig:
    return CameraConfig.model_validate({
        "image_width_px": 1920, "image_height_px": 1080,
        "horizontal_fov_deg": 60, "vertical_fov_deg": 45,
        "pitch_deg": -90, "yaw_mode": "follow_path",
        "forward_overlap": 0.3, "side_overlap": 0.3})


def test_continuous_sampling_uses_speed_and_frequency() -> None:
    route = (
        Waypoint("a", 1, "capture", 0, 0, 25, 0, -90, True, 0, 0),
        Waypoint("b", 2, "capture", 0, 20, 25, 0, -90, True, 0, 0),
    )
    plan, footprints = build_continuous_flight_plan(
        route, camera=camera(), flight_altitude_m=25, ground_elevation_m=0,
        video_analysis_rate_hz=2, coverage_speed_mps=5, connector_speed_mps=4,
        obstacle_speed_mps=2.5, return_speed_mps=4)
    assert len(footprints) == 9
    assert plan.route_segments[0].kind == "coverage_lane"
    assert plan.route_segments[0].heading_deg == 0
    assert plan.route_segments[0].speed_mps == 5
    assert plan.lanes[0].length_m == 20
    assert all(footprint.intersects(box(-1, 0, 1, 20)) for footprint in footprints.values())


def test_heading_follows_flight_direction_and_connector_speed() -> None:
    route = (
        Waypoint("a", 1, "capture", 0, 0, 25, 0, -90, True, 0, 0),
        Waypoint("b", 2, "capture", 10, 0, 25, 0, -90, True, 1, 0),
    )
    plan, _ = build_continuous_flight_plan(
        route, camera=camera(), flight_altitude_m=25, ground_elevation_m=0,
        video_analysis_rate_hz=2, coverage_speed_mps=5, connector_speed_mps=4,
        obstacle_speed_mps=2.5, return_speed_mps=4)
    assert plan.route_segments[0].kind == "connector"
    assert plan.route_segments[0].heading_deg == 90
    assert plan.route_segments[0].speed_mps == 4


def test_independent_completion_points_are_connectors() -> None:
    route = (
        Waypoint("a", 1, "capture", 0, 0, 25, 0, -90, True, is_completion=True),
        Waypoint("b", 2, "capture", 1, 1, 25, 0, -90, True, is_completion=True),
    )
    plan, _ = build_continuous_flight_plan(
        route, camera=camera(), flight_altitude_m=25, ground_elevation_m=0,
        video_analysis_rate_hz=2, coverage_speed_mps=5, connector_speed_mps=4,
        obstacle_speed_mps=2.5, return_speed_mps=4)
    assert plan.route_segments[0].kind == "connector"
    assert not plan.lanes


def test_dense_samples_are_replaced_by_uniform_control_points() -> None:
    route = tuple(Waypoint(
        str(index), index, "capture", 0, y, 25, 0, -90, True, 0, 0)
        for index, y in enumerate((0, 5, 10, 15, 20), 1))
    plan, _ = build_continuous_flight_plan(
        route, camera=camera(), flight_altitude_m=25, ground_elevation_m=0,
        video_analysis_rate_hz=2, coverage_speed_mps=5, connector_speed_mps=4,
        obstacle_speed_mps=2.5, return_speed_mps=4)
    assert [(waypoint.x, waypoint.y) for waypoint in plan.waypoints] == [
        (0, 0), (0, 10), (0, 20)]
    assert len(plan.route_segments) == 2
    assert all(segment.length_m == 10 for segment in plan.route_segments)
    assert plan.lanes[0].length_m == 20


def test_control_points_respect_configured_maximum_spacing() -> None:
    route = (
        Waypoint("a", 1, "capture", 0, 0, 25, 0, -90, True, 0, 0),
        Waypoint("b", 2, "capture", 0, 25, 25, 0, -90, True, 0, 0),
    )
    plan, _ = build_continuous_flight_plan(
        route, camera=camera(), flight_altitude_m=25, ground_elevation_m=0,
        video_analysis_rate_hz=2, coverage_speed_mps=5, connector_speed_mps=4,
        obstacle_speed_mps=2.5, return_speed_mps=4, control_point_spacing_m=6)
    assert len(plan.waypoints) == 6
    assert all(segment.length_m == 5 for segment in plan.route_segments)
    assert plan.control_point_spacing_m == 6


def test_home_departure_is_a_connector() -> None:
    route = (
        Waypoint("wp_start", 1, "transit", 0, 0, 25, 0, -90, False),
        Waypoint("lane", 2, "capture", 10, 0, 25, 0, -90, True, 0, 0),
    )
    plan, _ = build_continuous_flight_plan(
        route, camera=camera(), flight_altitude_m=25, ground_elevation_m=0,
        video_analysis_rate_hz=2, coverage_speed_mps=5, connector_speed_mps=4,
        obstacle_speed_mps=2.5, return_speed_mps=4)
    assert plan.route_segments[0].kind == "connector"


def test_connector_keeps_continuous_video_detection_enabled() -> None:
    route = (
        Waypoint("a", 1, "capture", 0, 0, 25, 0, -90, True, 0, 0),
        Waypoint("b", 2, "capture", 20, 0, 25, 0, -90, True, 1, 0),
    )
    plan, footprints = build_continuous_flight_plan(
        route, camera=camera(), flight_altitude_m=25, ground_elevation_m=0,
        video_analysis_rate_hz=2, coverage_speed_mps=5, connector_speed_mps=4,
        obstacle_speed_mps=2.5, return_speed_mps=4)
    assert plan.route_segments[0].detection_enabled
    assert len(footprints) > 2


def test_completion_point_adds_its_planned_stationary_view() -> None:
    route = (
        Waypoint("start", 0, "transit", 0, 0, 25, 0, -90, False),
        Waypoint(
            "completion", 1, "capture", 10, 0, 25, 90, -90, True,
            is_completion=True),
    )
    _, footprints = build_continuous_flight_plan(
        route, camera=camera(), flight_altitude_m=25, ground_elevation_m=0,
        video_analysis_rate_hz=2, coverage_speed_mps=5, connector_speed_mps=4,
        obstacle_speed_mps=2.5, return_speed_mps=4)
    assert "segment_point_0002_image_0000" in footprints


def test_coverage_lane_explicitly_enables_capture() -> None:
    route = (
        Waypoint("a", 1, "capture", 0, 0, 25, 0, -90, True, 0, 0),
        Waypoint("b", 2, "capture", 0, 20, 25, 0, -90, True, 0, 0),
    )
    plan, _ = build_continuous_flight_plan(
        route, camera=camera(), flight_altitude_m=25, ground_elevation_m=0,
        video_analysis_rate_hz=2, coverage_speed_mps=5, connector_speed_mps=4,
        obstacle_speed_mps=2.5, return_speed_mps=4)
    assert plan.route_segments[0].detection_enabled


def test_removes_consecutive_duplicate_route_points() -> None:
    route = (
        Waypoint("start", 0, "transit", 0, 0, 25, 0, -90, False),
        Waypoint("a", 1, "capture", 10, 0, 25, 0, -90, True, 0, 0),
        Waypoint("duplicate", 2, "capture", 10, 0, 25, 0, -90, True, 0, 0),
        Waypoint("b", 3, "capture", 20, 0, 25, 0, -90, True, 0, 0),
    )
    plan, _ = build_continuous_flight_plan(
        route, camera=camera(), flight_altitude_m=25, ground_elevation_m=0,
        video_analysis_rate_hz=2, coverage_speed_mps=5, connector_speed_mps=4,
        obstacle_speed_mps=2.5, return_speed_mps=4)
    assert all(segment.length_m > 0 for segment in plan.route_segments)


def test_marks_a_foldback_as_turn_in_place() -> None:
    route = (
        Waypoint("start", 0, "transit", 0, 0, 25, 0, -90, False),
        Waypoint("tip", 1, "capture", 10, 0, 25, 0, -90, True),
        Waypoint("back", 2, "transit", 1, 0, 25, 0, -90, False),
    )
    plan, _ = build_continuous_flight_plan(
        route, camera=camera(), flight_altitude_m=25, ground_elevation_m=0,
        video_analysis_rate_hz=2, coverage_speed_mps=5, connector_speed_mps=4,
        obstacle_speed_mps=2.5, return_speed_mps=4,
        control_point_spacing_m=20)
    assert plan.waypoints[1].turn_in_place
    assert plan.waypoints[1].hold_time_s == 0.5


def test_connector_visibility_is_sampled_across_requested_region() -> None:
    route = (
        Waypoint("a", 1, "capture", 0, 0, 25, 0, -90, True, 0, 0),
        Waypoint("b", 2, "capture", 40, 0, 25, 0, -90, True, 1, 0),
    )
    plan, footprints = build_continuous_flight_plan(
        route, camera=camera(), flight_altitude_m=25, ground_elevation_m=0,
        video_analysis_rate_hz=2, coverage_speed_mps=5, connector_speed_mps=4,
        obstacle_speed_mps=2.5, return_speed_mps=4,
        capture_region=box(18, -1, 22, 1))
    assert plan.route_segments[0].detection_enabled
    assert any("segment_0001_image" in identifier for identifier in footprints)
