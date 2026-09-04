"""Obstacle-aware transition costs shared by every lane-routing solver."""

from __future__ import annotations

from dataclasses import dataclass
from itertools import pairwise
from math import hypot

from coverage_planner.optimization.problem import RouteOptimizationProblem
from coverage_planner.routing.visibility import RoutingError, VisibilityRouter


@dataclass(frozen=True, slots=True)
class OrientedLaneState:
    job_index: int
    orientation_index: int
    entry_enu_m: tuple[float, float]
    exit_enu_m: tuple[float, float]


@dataclass(frozen=True, slots=True)
class LaneTransitionCosts:
    """Complete depot/state cost matrix using collision-free shortest paths."""

    states: tuple[OrientedLaneState, ...]
    depot_to_state_m: tuple[float, ...]
    state_to_state_m: tuple[tuple[float, ...], ...]
    state_to_depot_m: tuple[float, ...]

    def state_index(self, job_index: int, orientation_index: int) -> int:
        return next(index for index, state in enumerate(self.states)
                    if state.job_index == job_index
                    and state.orientation_index == orientation_index)


class LaneTransitionCostProvider:
    """Lazily cache the same obstacle costs used to materialize an exact matrix."""

    def __init__(self, problem: RouteOptimizationProblem) -> None:
        self.problem = problem
        self.router = VisibilityRouter(problem.obstacles)
        self.states = _oriented_states(problem)
        self._state_indices = {
            (state.job_index, state.orientation_index): index
            for index, state in enumerate(self.states)
        }
        self._depot_to: dict[int, float] = {}
        self._between: dict[tuple[int, int], float] = {}
        self._to_depot: dict[int, float] = {}

    def state_index(self, job_index: int, orientation_index: int) -> int:
        return self._state_indices[(job_index, orientation_index)]

    def depot_to_state_m(self, state_index: int) -> float:
        if state_index not in self._depot_to:
            self._depot_to[state_index] = _path_cost(
                self.router, self.problem.start_enu_m, self.states[state_index].entry_enu_m)
        return self._depot_to[state_index]

    def state_to_state_m(self, source_index: int, target_index: int) -> float:
        key = (source_index, target_index)
        if key not in self._between:
            source, target = self.states[source_index], self.states[target_index]
            self._between[key] = (
                float("inf") if source.job_index == target.job_index else
                _path_cost(self.router, source.exit_enu_m, target.entry_enu_m))
        return self._between[key]

    def state_to_depot_m(self, state_index: int) -> float:
        if state_index not in self._to_depot:
            self._to_depot[state_index] = _path_cost(
                self.router, self.states[state_index].exit_enu_m, self.problem.start_enu_m)
        return self._to_depot[state_index]


def build_transition_costs(problem: RouteOptimizationProblem) -> LaneTransitionCosts:
    provider = LaneTransitionCostProvider(problem)
    states = provider.states
    depot_to = tuple(provider.depot_to_state_m(index) for index in range(len(states)))
    between = tuple(tuple(provider.state_to_state_m(source, target)
                            for target in range(len(states)))
                    for source in range(len(states)))
    to_depot = tuple(provider.state_to_depot_m(index) for index in range(len(states)))
    return LaneTransitionCosts(states, depot_to, between, to_depot)


def _oriented_states(problem: RouteOptimizationProblem) -> tuple[OrientedLaneState, ...]:
    return tuple(
        OrientedLaneState(
            job_index=job_index,
            orientation_index=orientation_index,
            entry_enu_m=(orientation[0].x, orientation[0].y),
            exit_enu_m=(orientation[-1].x, orientation[-1].y),
        )
        for job_index, job in enumerate(problem.jobs)
        for orientation_index, orientation in enumerate(job.orientations)
    )


def _path_cost(
    router: VisibilityRouter, start: tuple[float, float], end: tuple[float, float],
) -> float:
    try:
        path = router.shortest_path(start, end)
    except RoutingError:
        return float("inf")
    return sum(hypot(b[0] - a[0], b[1] - a[1]) for a, b in pairwise(path))
