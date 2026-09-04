"""Coverage generation and evaluation."""

from coverage_planner.coverage.continuous import build_continuous_flight_plan
from coverage_planner.coverage.evaluation import (
    CoverageEvaluationError,
    evaluate_patch_coverage,
)
from coverage_planner.coverage.optimization import (
    DirectionScore,
    optimize_scan_direction,
    prepare_lane_route,
    supplement_uncovered_patches,
)
from coverage_planner.coverage.scanlines import (
    CapturePlan,
    ScanlinePlanningError,
    generate_capture_plan,
)

__all__ = [
    "CapturePlan",
    "CoverageEvaluationError",
    "DirectionScore",
    "ScanlinePlanningError",
    "build_continuous_flight_plan",
    "evaluate_patch_coverage",
    "generate_capture_plan",
    "optimize_scan_direction",
    "prepare_lane_route",
    "supplement_uncovered_patches",
]
