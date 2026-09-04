"""Deterministic nearest-neighbor lane ordering baseline."""

from __future__ import annotations

from itertools import pairwise

from coverage_planner.models.waypoint import Waypoint
from coverage_planner.optimization.cost_matrix import LaneTransitionCostProvider
from coverage_planner.optimization.problem import (
    OptimizedRoute,
    RouteOptimizationProblem,
    renumber_waypoints,
)


class GreedyLaneRouter:
    """Choose the nearest reachable lane entry and its best orientation."""

    method = "greedy_obstacle_distance"

    def solve(self, problem: RouteOptimizationProblem) -> OptimizedRoute:
        remaining = list(enumerate(problem.jobs))
        costs = LaneTransitionCostProvider(problem)
        ordered: list[Waypoint] = []
        job_order: list[str] = []
        orientation_indices: list[int] = []
        selected_state_indices: list[int] = []
        while remaining:
            choices: list[tuple[float, int, int]] = []
            for remaining_index, (job_index, job) in enumerate(remaining):
                for orientation_index, orientation in enumerate(job.orientations):
                    state_index = costs.state_index(job_index, orientation_index)
                    cost = (costs.depot_to_state_m(state_index)
                            if not selected_state_indices else
                            costs.state_to_state_m(selected_state_indices[-1], state_index))
                    choices.append((cost, remaining_index, orientation_index))
            _, remaining_index, orientation_index = min(
                choices, key=lambda item: (item[0], item[1], item[2]))
            job_index, job = remaining.pop(remaining_index)
            state_index = costs.state_index(job_index, orientation_index)
            selected = job.orientations[orientation_index]
            ordered.extend(selected)
            job_order.append(job.id)
            orientation_indices.append(orientation_index)
            selected_state_indices.append(state_index)
        transition_cost = 0.0
        if selected_state_indices:
            transition_cost = costs.depot_to_state_m(selected_state_indices[0])
            transition_cost += sum(
                costs.state_to_state_m(left, right)
                for left, right in pairwise(selected_state_indices))
        return_cost = (costs.state_to_depot_m(selected_state_indices[-1])
                       if selected_state_indices else 0.0)
        return OptimizedRoute(
            method=self.method,
            ordered_waypoints=renumber_waypoints(tuple(ordered)),
            job_order=tuple(job_order),
            orientation_indices=tuple(orientation_indices),
            skipped_point_ids=(),
            transition_cost_m=transition_cost,
            return_cost_m=return_cost,
        )
