"""Visibility-graph routing around polygonal flight obstacles."""

from __future__ import annotations

from dataclasses import replace
from heapq import heappop, heappush
from math import atan2, degrees, hypot

from shapely.geometry import LineString, MultiPolygon, Point

from coverage_planner.models.search_area import Polygonal
from coverage_planner.models.waypoint import Waypoint


class RoutingError(ValueError):
    """Raised when no collision-free route can be found."""


class VisibilityRouter:
    """Reusable visibility graph for many routes around the same obstacles."""

    def __init__(self, obstacles: Polygonal) -> None:
        self.obstacles = obstacles
        self.vertices = _vertices(obstacles)
        self.graph: dict[int, dict[int, float]] = {
            index: {} for index in range(2, len(self.vertices) + 2)
        }
        for left in range(len(self.vertices)):
            for right in range(left + 1, len(self.vertices)):
                a, b = self.vertices[left], self.vertices[right]
                if _visible(a, b, obstacles):
                    weight = hypot(b[0] - a[0], b[1] - a[1])
                    self.graph[left + 2][right + 2] = weight
                    self.graph[right + 2][left + 2] = weight

    def shortest_path(
        self, start: tuple[float, float], end: tuple[float, float],
    ) -> tuple[tuple[float, float], ...]:
        if self.obstacles.covers(Point(start)) or self.obstacles.covers(Point(end)):
            raise RoutingError("route endpoint lies inside a flight-obstacle safety buffer")
        if _visible(start, end, self.obstacles):
            return (start, end)
        graph = {index: neighbors.copy() for index, neighbors in self.graph.items()}
        graph[0] = {}
        graph[1] = {}
        for endpoint_index, endpoint in ((0, start), (1, end)):
            for vertex_index, vertex in enumerate(self.vertices, 2):
                if _visible(endpoint, vertex, self.obstacles):
                    weight = hypot(vertex[0] - endpoint[0], vertex[1] - endpoint[1])
                    graph[endpoint_index][vertex_index] = weight
                    graph[vertex_index][endpoint_index] = weight
        indices = _dijkstra(graph, 0, 1)
        points = {0: start, 1: end}
        points.update({index: point for index, point in enumerate(self.vertices, 2)})
        return tuple(points[index] for index in indices)


def route_reachable_waypoints(
    start: Waypoint,
    capture_waypoints: tuple[Waypoint, ...],
    obstacles: Polygonal,
    *,
    return_to_start: bool = False,
) -> tuple[tuple[Waypoint, ...], tuple[str, ...]]:
    """Route each capture in order while reporting unreachable destinations."""
    routed = [start]
    router = VisibilityRouter(obstacles)
    skipped = [
        destination.id for destination in capture_waypoints
        if obstacles.covers(Point(destination.x, destination.y))
    ]
    reachable = [
        destination for destination in capture_waypoints
        if not obstacles.covers(Point(destination.x, destination.y))
    ]
    for destination in reachable:
        try:
            points = router.shortest_path(
                (routed[-1].x, routed[-1].y), (destination.x, destination.y))
        except RoutingError:
            skipped.append(destination.id)
            continue
        for point in points[1:-1]:
            routed.append(Waypoint(
                id="", sequence=0, kind="transit", x=point[0], y=point[1], z=destination.z,
                yaw_deg=0.0, camera_pitch_deg=destination.camera_pitch_deg, capture=False,
            ))
        routed.append(destination)
    if return_to_start and routed[-1] is not start:
        try:
            points = router.shortest_path(
                (routed[-1].x, routed[-1].y), (start.x, start.y))
            for point in points[1:-1]:
                routed.append(Waypoint(
                    id="", sequence=0, kind="transit", x=point[0], y=point[1], z=start.z,
                    yaw_deg=0.0, camera_pitch_deg=start.camera_pitch_deg, capture=False,
                ))
            routed.append(replace(start, id="wp_home_return"))
        except RoutingError:
            skipped.append("wp_home_return")
    result = []
    for index, waypoint in enumerate(routed, 1):
        next_point = routed[index] if index < len(routed) else waypoint
        identifier = (
            waypoint.id if waypoint.capture or waypoint.id == "wp_home_return"
            else f"wp_{index:04d}_transit"
        )
        result.append(replace(
            waypoint, id=identifier, sequence=index,
            yaw_deg=(waypoint.yaw_deg if waypoint.is_completion else
                     _yaw((waypoint.x, waypoint.y), (next_point.x, next_point.y))),
        ))
    return tuple(result), tuple(skipped)


def shortest_collision_free_path(
    start: tuple[float, float], end: tuple[float, float], obstacles: Polygonal,
) -> tuple[tuple[float, float], ...]:
    return VisibilityRouter(obstacles).shortest_path(start, end)


def _visible(a: tuple[float, float], b: tuple[float, float], obstacles: Polygonal) -> bool:
    line = LineString([a, b])
    return line.relate(obstacles)[0] == "F"


def _vertices(obstacles: Polygonal) -> list[tuple[float, float]]:
    polygons = obstacles.geoms if isinstance(obstacles, MultiPolygon) else [obstacles]
    points: list[tuple[float, float]] = []
    for polygon in polygons:
        points.extend((float(x), float(y)) for x, y in list(polygon.exterior.coords)[:-1])
    return points


def _dijkstra(
    graph: dict[int, dict[int, float]], start: int, end: int,
) -> tuple[int, ...]:
    """Return a deterministic shortest path through an adjacency-list graph."""
    distances = {start: 0.0}
    previous: dict[int, int] = {}
    queue: list[tuple[float, int]] = [(0.0, start)]
    visited: set[int] = set()
    while queue:
        distance, node = heappop(queue)
        if node in visited:
            continue
        visited.add(node)
        if node == end:
            break
        for neighbor in sorted(graph[node]):
            candidate = distance + graph[node][neighbor]
            known = distances.get(neighbor)
            if known is None or candidate < known - 1e-12:
                distances[neighbor] = candidate
                previous[neighbor] = node
                heappush(queue, (candidate, neighbor))
            elif known is not None and abs(candidate - known) <= 1e-12:
                if node < previous.get(neighbor, node + 1):
                    previous[neighbor] = node
                    heappush(queue, (candidate, neighbor))
    if end not in distances:
        raise RoutingError("no collision-free route exists between waypoints")
    path = [end]
    while path[-1] != start:
        path.append(previous[path[-1]])
    path.reverse()
    return tuple(path)


def _yaw(a: tuple[float, float], b: tuple[float, float]) -> float:
    if a == b:
        return 0.0
    return degrees(atan2(b[0] - a[0], b[1] - a[1])) % 360.0
