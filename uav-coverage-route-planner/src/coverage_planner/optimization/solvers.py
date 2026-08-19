"""Route optimizers sharing one obstacle-aware oriented-lane cost matrix."""

from __future__ import annotations

from collections.abc import Sequence
from itertools import pairwise
from math import inf
from typing import Literal

from coverage_planner.optimization.cost_matrix import LaneTransitionCosts, build_transition_costs
from coverage_planner.optimization.greedy import GreedyLaneRouter
from coverage_planner.optimization.problem import (
    OptimizedRoute,
    RouteOptimizationProblem,
    renumber_waypoints,
)

RouteOptimizationMethod = Literal["greedy", "two_opt", "or_opt", "heuristic", "exact", "auto"]
EXACT_LANE_LIMIT = 12
LOCAL_SEARCH_MAX_PASSES = 4
TWO_OPT_MAX_SPAN = 12


def optimize_route(
    problem: RouteOptimizationProblem, *, method: RouteOptimizationMethod,
) -> tuple[OptimizedRoute, tuple[OptimizedRoute, ...]]:
    """Solve one problem and return the selected route plus compared candidates."""
    greedy = GreedyLaneRouter().solve(problem)
    if method == "greedy":
        return greedy, (greedy,)
    costs = build_transition_costs(problem)
    order = tuple(_job_indices(problem, greedy.job_order))
    candidates = [greedy]
    if method == "auto" and len(problem.coverage_lanes) <= EXACT_LANE_LIMIT:
        exact = _with_method(_exact_solution(problem, costs), "auto:exact")
        return exact, (greedy, exact)
    if method in {"two_opt", "heuristic", "auto"}:
        order = _two_opt_order(problem, costs, order)
        candidates.append(_solution(problem, costs, order, "two_opt"))
    if method in {"or_opt", "heuristic", "auto"}:
        order = _or_opt_order(problem, costs, order)
        candidates.append(_solution(problem, costs, order, "or_opt"))
    if method == "exact":
        if len(problem.coverage_lanes) > EXACT_LANE_LIMIT:
            raise ValueError(
                f"exact route optimization supports at most {EXACT_LANE_LIMIT} lanes")
        candidates.append(_exact_solution(problem, costs))
    selected = min(candidates, key=_objective)
    if method == "heuristic":
        selected = _with_method(selected, "heuristic")
    elif method == "auto":
        selected = _with_method(selected, "auto:heuristic")
    return selected, tuple(candidates)


def _two_opt_order(
    problem: RouteOptimizationProblem, costs: LaneTransitionCosts, order: tuple[int, ...],
) -> tuple[int, ...]:
    best, best_cost = order, _order_cost(problem, costs, order)[0]
    improved = True
    passes = 0
    while improved and passes < LOCAL_SEARCH_MAX_PASSES:
        passes += 1
        improved = False
        for left in range(len(best) - 1):
            for right in range(left + 2, min(len(best), left + TWO_OPT_MAX_SPAN) + 1):
                candidate = best[:left] + tuple(reversed(best[left:right])) + best[right:]
                candidate_cost = _order_cost(problem, costs, candidate)[0]
                if candidate_cost + 1e-9 < best_cost:
                    best, best_cost, improved = candidate, candidate_cost, True
    return best


def _or_opt_order(
    problem: RouteOptimizationProblem, costs: LaneTransitionCosts, order: tuple[int, ...],
) -> tuple[int, ...]:
    best, best_cost = order, _order_cost(problem, costs, order)[0]
    improved = True
    passes = 0
    while improved and passes < LOCAL_SEARCH_MAX_PASSES:
        passes += 1
        improved = False
        for source in range(len(best)):
            item = best[source]
            remainder = best[:source] + best[source + 1:]
            for target in range(len(remainder) + 1):
                candidate = remainder[:target] + (item,) + remainder[target:]
                candidate_cost = _order_cost(problem, costs, candidate)[0]
                if candidate_cost + 1e-9 < best_cost:
                    best, best_cost, improved = candidate, candidate_cost, True
    return best


def _order_cost(
    problem: RouteOptimizationProblem, costs: LaneTransitionCosts, order: Sequence[int],
) -> tuple[float, tuple[int, ...]]:
    if not order:
        return 0.0, ()
    previous: dict[int, tuple[float, tuple[int, ...]]] = {}
    first = order[0]
    for orientation in range(len(problem.coverage_lanes[first].orientations)):
        state = costs.state_index(first, orientation)
        previous[state] = (costs.depot_to_state_m[state], (orientation,))
    for job_index in order[1:]:
        current: dict[int, tuple[float, tuple[int, ...]]] = {}
        for orientation in range(len(problem.coverage_lanes[job_index].orientations)):
            target = costs.state_index(job_index, orientation)
            value, history = min(
                (cost + costs.state_to_state_m[source][target], choices + (orientation,))
                for source, (cost, choices) in previous.items()
            )
            current[target] = (value, history)
        previous = current
    return min(
        (cost + costs.state_to_depot_m[state], choices)
        for state, (cost, choices) in previous.items()
    )


def _exact_solution(
    problem: RouteOptimizationProblem, costs: LaneTransitionCosts,
) -> OptimizedRoute:
    count = len(problem.coverage_lanes)
    if count == 0:
        return _solution(problem, costs, (), "exact")
    table: dict[tuple[int, int], tuple[float, tuple[int, ...], tuple[int, ...]]] = {}
    for job_index, lane in enumerate(problem.coverage_lanes):
        for orientation in range(len(lane.orientations)):
            state = costs.state_index(job_index, orientation)
            table[(1 << job_index, state)] = (
                costs.depot_to_state_m[state], (job_index,), (orientation,))
    for mask in range(1, 1 << count):
        entries = [(state, value) for (entry_mask, state), value in table.items()
                   if entry_mask == mask]
        for source, (cost, order, orientations) in entries:
            for job_index, lane in enumerate(problem.coverage_lanes):
                if mask & (1 << job_index):
                    continue
                target_mask = mask | (1 << job_index)
                for orientation in range(len(lane.orientations)):
                    target = costs.state_index(job_index, orientation)
                    candidate = cost + costs.state_to_state_m[source][target]
                    key = (target_mask, target)
                    if candidate < table.get(key, (inf, (), ()))[0]:
                        table[key] = (
                            candidate, order + (job_index,), orientations + (orientation,))
    full = (1 << count) - 1
    _, order, orientations = min(
        (cost + costs.state_to_depot_m[state], order, orientations)
        for (mask, state), (cost, order, orientations) in table.items() if mask == full
    )
    return _solution(problem, costs, order, "exact", orientations)


def _solution(
    problem: RouteOptimizationProblem,
    costs: LaneTransitionCosts,
    order: Sequence[int],
    method: str,
    orientations: tuple[int, ...] | None = None,
) -> OptimizedRoute:
    _, selected_orientations = _order_cost(problem, costs, order)
    if orientations is not None:
        selected_orientations = orientations
    states = tuple(costs.state_index(job, orientation)
                   for job, orientation in zip(order, selected_orientations, strict=True))
    transition = 0.0 if not states else costs.depot_to_state_m[states[0]] + sum(
        costs.state_to_state_m[left][right] for left, right in pairwise(states))
    return_cost = 0.0 if not states else costs.state_to_depot_m[states[-1]]
    waypoints = tuple(
        waypoint
        for job, orientation in zip(order, selected_orientations, strict=True)
        for waypoint in problem.coverage_lanes[job].orientations[orientation]
    )
    return OptimizedRoute(
        method=method,
        ordered_waypoints=renumber_waypoints(waypoints),
        job_order=tuple(problem.coverage_lanes[index].id for index in order),
        orientation_indices=selected_orientations,
        skipped_point_ids=(),
        transition_cost_m=transition,
        return_cost_m=return_cost,
    )


def _objective(solution: OptimizedRoute) -> tuple[float, str]:
    return solution.transition_cost_m + solution.return_cost_m, solution.method


def _job_indices(problem: RouteOptimizationProblem, ids: Sequence[str]) -> list[int]:
    by_id = {lane.id: index for index, lane in enumerate(problem.coverage_lanes)}
    return [by_id[item] for item in ids]


def _with_method(solution: OptimizedRoute, method: str) -> OptimizedRoute:
    return OptimizedRoute(
        method, solution.ordered_waypoints, solution.job_order,
        solution.orientation_indices, solution.skipped_point_ids,
        solution.transition_cost_m, solution.return_cost_m)
