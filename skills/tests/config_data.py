"""Shared immutable configuration fixture data for skills unit tests."""

from pathlib import Path

from skills.config import (
    CoveragePlannerSkillConfig,
    DetectionSkillConfig,
    OffboardSkillConfig,
    SkillRuntimeConfig,
)


DETECTION_CONFIG = DetectionSkillConfig(
    frame_id="map",
    detections_topic="/detection/detections",
    detections_world_topic="/detection/detections_world",
    image_overlay_topic="/detection/image_overlay",
    vehicle_odom_topic="/gz/odom_super",
    config_file=Path("/tmp/test-detection.yaml"),
)


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

#: The same workspace configuration, with detection available.
TEST_CONFIG_WITH_DETECTION = SkillRuntimeConfig(
    offboard=TEST_CONFIG.offboard,
    coverage_planner=TEST_CONFIG.coverage_planner,
    detection=DETECTION_CONFIG,
)
