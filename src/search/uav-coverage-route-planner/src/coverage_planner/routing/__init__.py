"""Altitude-aware obstacle selection and waypoint routing."""

from coverage_planner.routing.obstacles import FlightObstacles, select_flight_obstacles
from coverage_planner.routing.visibility import (
    RoutingError,
    route_reachable_waypoints,
    shortest_collision_free_path,
)

__all__ = [
    "FlightObstacles", "RoutingError", "route_reachable_waypoints", "select_flight_obstacles",
    "shortest_collision_free_path",
]
