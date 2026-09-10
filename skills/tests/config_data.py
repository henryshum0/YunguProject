"""Shared immutable configuration fixture data for skills unit tests."""

from pathlib import Path

from skills.config import CoveragePlannerSkillConfig, OffboardSkillConfig, SkillRuntimeConfig


TEST_CONFIG = SkillRuntimeConfig(
    offboard=OffboardSkillConfig(
        frame_id="map",
        queue_service="/waypoint_buffer",
        clear_service="/waypoint_buffer/clear",
        takeoff_topic="/takeoff_cmd",
        land_topic="/land_cmd",
        queue_status_topic="/waypoint_buffer/status",
    ),
    coverage_planner=CoveragePlannerSkillConfig(
        frame_id="map",
        plan_service="/coverage_planner/plan_coverage",
        planner_config_file=Path("/tmp/test-planner.json"),
    ),
)
