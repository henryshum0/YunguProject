from math import sqrt

import pytest
from shapely.geometry import Polygon

from coverage_planner.coverage.optimization import prepare_lane_route
from coverage_planner.coverage.scanlines import CapturePlan
from coverage_planner.models import Waypoint
from coverage_planner.optimization import (
    GreedyLaneRouter,
    build_lane_routing_problem,
    build_transition_costs,
    optimize_route,
)


def point(identifier: str, sequence: int, x: float, y: float,
          line: int, segment: int = 0) -> Waypoint:
    return Waypoint(
        identifier, sequence, "capture", x, y, 25, 0, -90, True,
        line, segment,
    )


def test_lane_route_keeps_only_endpoints_and_chooses_near_orientation() -> None:
    plan = CapturePlan(90, (), (
        point("a", 1, 0, 0, 0),
        point("b", 2, 5, 0, 0),
        point("c", 3, 10, 0, 0),
        point("d", 4, 0, 10, 1),
        point("e", 5, 5, 10, 1),
        point("f", 6, 10, 10, 1),
    ))
    route, skipped = prepare_lane_route(
        plan, start_enu_m=(11, 0), obstacles=Polygon())
    assert skipped == ()
    assert [(item.x, item.y) for item in route] == [
        (10, 0), (0, 0), (0, 10), (10, 10)]
    assert [item.id for item in route] == [
        "wp_0001", "wp_0002", "wp_0003", "wp_0004"]


def test_canonical_problem_separates_jobs_from_solver() -> None:
    plan = CapturePlan(90, (), (
        point("a", 1, 0, 0, 0), point("b", 2, 10, 0, 0),
        point("c", 3, 0, 10, 1), point("d", 4, 10, 10, 1),
    ))
    problem, skipped = build_lane_routing_problem(
        plan, start_enu_m=(11, 0), obstacles=Polygon())
    assert skipped == ()
    assert [lane.id for lane in problem.coverage_lanes] == [
        "coverage_lane_0001", "coverage_lane_0002"]
    assert all(len(job.orientations) == 2 for job in problem.jobs)
    solution = GreedyLaneRouter().solve(problem)
    assert solution.method == "greedy_obstacle_distance"
    assert solution.job_order == ("coverage_lane_0001", "coverage_lane_0002")
    assert solution.orientation_indices == (1, 0)
    assert solution.transition_cost_m == 11
    assert solution.return_cost_m == pytest.approx(sqrt(101))
    assert [(waypoint.x, waypoint.y) for waypoint in solution.ordered_waypoints] == [
        (10, 0), (0, 0), (0, 10), (10, 10)]


def test_transition_matrix_uses_same_oriented_lane_states_for_all_solvers() -> None:
    plan = CapturePlan(90, (), (
        point("a", 1, 0, 0, 0), point("b", 2, 10, 0, 0),
        point("c", 3, 0, 10, 1), point("d", 4, 10, 10, 1),
    ))
    problem, _ = build_lane_routing_problem(
        plan, start_enu_m=(0, 0), obstacles=Polygon())
    costs = build_transition_costs(problem)
    assert len(costs.states) == 4
    first_forward = costs.state_index(0, 0)
    first_reverse = costs.state_index(0, 1)
    second_forward = costs.state_index(1, 0)
    assert costs.depot_to_state_m[first_forward] == 0
    assert costs.depot_to_state_m[first_reverse] == 10
    assert costs.state_to_state_m[first_forward][first_reverse] == float("inf")
    assert costs.state_to_state_m[first_forward][second_forward] == pytest.approx(sqrt(200))
    assert costs.state_to_depot_m[second_forward] == pytest.approx(sqrt(200))


@pytest.mark.parametrize("method", ["two_opt", "or_opt", "heuristic", "exact", "auto"])
def test_route_optimizers_share_generated_lanes_and_costs(method: str) -> None:
    plan = CapturePlan(90, (), (
        point("a", 1, 0, 0, 0), point("b", 2, 10, 0, 0),
        point("c", 3, 20, 10, 1), point("d", 4, 30, 10, 1),
        point("e", 5, 0, 10, 2), point("f", 6, 10, 10, 2),
    ))
    problem, _ = build_lane_routing_problem(
        plan, start_enu_m=(0, 0), obstacles=Polygon())
    selected, candidates = optimize_route(problem, method=method)  # type: ignore[arg-type]
    assert set(selected.job_order) == {lane.id for lane in problem.coverage_lanes}
    assert len(selected.orientation_indices) == len(problem.coverage_lanes)
    assert selected.transition_cost_m + selected.return_cost_m <= (
        candidates[0].transition_cost_m + candidates[0].return_cost_m + 1e-9)


@pytest.mark.parametrize("lane_count, expected", [(12, "auto:exact"), (13, "auto:heuristic")])
def test_auto_uses_fixed_size_based_strategy(lane_count: int, expected: str) -> None:
    waypoints = tuple(
        point(f"p{index}", index + 1, index * 2.0, 0, index)
        for index in range(lane_count)
    )
    problem, _ = build_lane_routing_problem(
        CapturePlan(90, (), waypoints), start_enu_m=(0, 0), obstacles=Polygon())
    selected, _ = optimize_route(problem, method="auto")
    assert selected.method == expected
