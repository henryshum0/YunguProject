"""Solver-independent lane-routing problem and optimization methods."""

from coverage_planner.optimization.cost_matrix import (
    LaneTransitionCostProvider,
    LaneTransitionCosts,
    OrientedLaneState,
    build_transition_costs,
)
from coverage_planner.optimization.greedy import GreedyLaneRouter
from coverage_planner.optimization.problem import (
    CoverageLane,
    LaneJob,
    LaneRoutingProblem,
    LaneRoutingSolution,
    OptimizedRoute,
    RouteOptimizationProblem,
    build_lane_routing_problem,
    build_route_optimization_problem,
)
from coverage_planner.optimization.solvers import (
    EXACT_LANE_LIMIT,
    RouteOptimizationMethod,
    optimize_route,
)

__all__ = [
    "EXACT_LANE_LIMIT",
    "CoverageLane",
    "GreedyLaneRouter",
    "LaneJob",
    "LaneRoutingProblem",
    "LaneRoutingSolution",
    "LaneTransitionCostProvider",
    "LaneTransitionCosts",
    "OptimizedRoute",
    "OrientedLaneState",
    "RouteOptimizationMethod",
    "RouteOptimizationProblem",
    "build_lane_routing_problem",
    "build_route_optimization_problem",
    "build_transition_costs",
    "optimize_route",
]
