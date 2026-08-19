"""Local FastAPI planning application backed by the pure-Python planner."""

from __future__ import annotations

from pathlib import Path
from threading import Lock
from time import perf_counter
from typing import Any, Literal

from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field, field_validator
from shapely.geometry import LineString, box, mapping, shape

from coverage_planner.coverage.generators import build_boustrophedon_planning_cells
from coverage_planner.geometry.calibration import MapCalibration
from coverage_planner.io import load_semantic_map
from coverage_planner.io.semantic_map import building_safety_elevations, building_safety_geometry
from coverage_planner.models import CameraConfig
from coverage_planner.multi_planner import DroneAssignment, TwoDroneCoveragePlanner
from coverage_planner.planner import CoveragePlanner, PlanResult
from coverage_planner.reporting import export_multi_plan, export_plan

ROOT = Path(__file__).resolve().parents[2]
EXAMPLE = ROOT / "examples/yungu2030"
WEB = ROOT / "web"
RESULTS = ROOT / "results/web_latest"
HOME_ENU_M = (153.4, 67.2)
PLANNING_LOCK = Lock()


class PlanRequest(BaseModel):
    search_geometry: dict[str, Any]
    home_x_m: float = HOME_ENU_M[0]
    home_y_m: float = HOME_ENU_M[1]
    flight_altitude_m: float = Field(gt=0)
    horizontal_clearance_m: float = Field(ge=0)
    vertical_clearance_m: float = Field(ge=0)
    scan_direction_deg: float | None = None
    camera: CameraConfig
    coverage_generation_method: Literal["global_scanline", "bcd"] = "global_scanline"
    scan_pattern: Literal["scanline_clipped", "bcd"] | None = None
    video_analysis_rate_hz: float = Field(default=2.0, gt=0)
    control_point_spacing_m: float = Field(default=10.0, gt=0)
    coverage_speed_mps: float = Field(default=5.0, gt=0)
    connector_speed_mps: float = Field(default=4.0, gt=0)
    obstacle_speed_mps: float = Field(default=2.5, gt=0)
    return_speed_mps: float = Field(default=4.0, gt=0)
    route_optimization_method: Literal["auto"] = "auto"
    completion_strategy: Literal["full_greedy", "local_insertion"] = "local_insertion"

    @field_validator("scan_direction_deg")
    @classmethod
    def validate_scan_direction(cls, value: float | None) -> float | None:
        if value not in {None, 0.0, 90.0}:
            raise ValueError("scan_direction_deg must be 0, 90, or null")
        return value


class DroneRequest(BaseModel):
    drone_id: str = Field(min_length=1)
    search_geometry: dict[str, Any]
    home_x_m: float
    home_y_m: float


class DualPlanRequest(BaseModel):
    drones: tuple[DroneRequest, DroneRequest]
    flight_altitude_m: float = Field(gt=0)
    horizontal_clearance_m: float = Field(ge=0)
    vertical_clearance_m: float = Field(ge=0)
    scan_direction_deg: float | None = None
    camera: CameraConfig
    coverage_generation_method: Literal["global_scanline", "bcd"] = "global_scanline"
    scan_pattern: Literal["scanline_clipped", "bcd"] | None = None
    video_analysis_rate_hz: float = Field(default=2.0, gt=0)
    control_point_spacing_m: float = Field(default=10.0, gt=0)
    coverage_speed_mps: float = Field(default=5.0, gt=0)
    connector_speed_mps: float = Field(default=4.0, gt=0)
    obstacle_speed_mps: float = Field(default=2.5, gt=0)
    return_speed_mps: float = Field(default=4.0, gt=0)
    route_optimization_method: Literal["auto"] = "auto"
    completion_strategy: Literal["full_greedy", "local_insertion"] = "local_insertion"

    @field_validator("scan_direction_deg")
    @classmethod
    def validate_scan_direction(cls, value: float | None) -> float | None:
        if value not in {None, 0.0, 90.0}:
            raise ValueError("scan_direction_deg must be 0, 90, or null")
        return value


def _request_generation_method(
    request: PlanRequest | DualPlanRequest,
) -> Literal["global_scanline", "bcd"]:
    """Resolve the canonical field while accepting the legacy Web API key."""
    if request.scan_pattern is not None:
        return "bcd" if request.scan_pattern == "bcd" else "global_scanline"
    return request.coverage_generation_method


app = FastAPI(title="UAV Coverage Generation and Route Optimization Planner")


@app.get("/api/map")
def map_data() -> dict[str, Any]:
    semantic = load_semantic_map(EXAMPLE / "semantic_map.json")
    calibration = MapCalibration.load(EXAMPLE / "map_calibration.json")
    image_corners = [
        calibration.pixel_to_enu(point)
        for point in ((0.0, 0.0), (1919.0, 0.0), (0.0, 1079.0), (1919.0, 1079.0))
    ]
    image_size = (1920, 1080)
    return {
        "world_name": semantic.world_name,
        "background": {
            "url": "/api/background",
            "bounds": [
                min(point[0] for point in image_corners),
                min(point[1] for point in image_corners),
                max(point[0] for point in image_corners),
                max(point[1] for point in image_corners),
            ],
            "content_bounds": list(calibration.content_bounds_enu(image_size)),
        },
        "search_area": mapping(shape({"type":"Polygon","coordinates":[[*semantic.search_area.coords, semantic.search_area.coords[0]]]})),
        "buildings": [{
            "id": node.id,
            "height_m": building_safety_elevations(semantic, node)[1],
            "bounds": list(building_safety_geometry(semantic, node).bounds),
            "ground_contact": node.properties.ground_contact,
        } for node in semantic.building_nodes],
        "excluded_search_regions": [{
            "id": region.id,
            "label": region.label,
            "reason": region.reason,
            "geometry": mapping(box(*region.shape.min_corner, *region.shape.max_corner)),
        } for region in semantic.excluded_search_regions],
    }


@app.get("/api/background")
def background() -> FileResponse:
    return FileResponse(EXAMPLE / "overhead_map_rotated_180.jpg", media_type="image/jpeg")


@app.post("/api/plan")
def plan(request: PlanRequest) -> dict[str, Any]:
    try:
        semantic = load_semantic_map(EXAMPLE / "semantic_map.json")
        with PLANNING_LOCK:
            planning_started_at = perf_counter()
            result = CoveragePlanner().plan(
                semantic_map=semantic, search_geometry=shape(request.search_geometry),
                camera=request.camera, flight_altitude_m=request.flight_altitude_m,
                start=(request.home_x_m, request.home_y_m, request.flight_altitude_m),
                horizontal_clearance_m=request.horizontal_clearance_m,
                vertical_clearance_m=request.vertical_clearance_m,
                scan_direction_deg=request.scan_direction_deg,
                coverage_generation_method=_request_generation_method(request),
                video_analysis_rate_hz=request.video_analysis_rate_hz,
                control_point_spacing_m=request.control_point_spacing_m,
                coverage_speed_mps=request.coverage_speed_mps,
                connector_speed_mps=request.connector_speed_mps,
                obstacle_speed_mps=request.obstacle_speed_mps,
                return_speed_mps=request.return_speed_mps,
                route_optimization_method=request.route_optimization_method,
                completion_strategy=request.completion_strategy,
            )
            planning_time_s = perf_counter() - planning_started_at
            export_plan(result, RESULTS)
        return {"planning_time_s": planning_time_s, "summary": _web_result(
            result,
            [request.home_x_m, request.home_y_m, request.flight_altitude_m],
            camera=request.camera,
            flight_altitude_m=request.flight_altitude_m,
        )}
    except (ValueError, TypeError) as exc:
        raise HTTPException(status_code=422, detail=str(exc)) from exc


@app.post("/api/plan-dual")
def plan_dual(request: DualPlanRequest) -> dict[str, Any]:
    try:
        semantic = load_semantic_map(EXAMPLE / "semantic_map.json")
        assignments = (
            DroneAssignment(
                request.drones[0].drone_id, shape(request.drones[0].search_geometry),
                (request.drones[0].home_x_m, request.drones[0].home_y_m,
                 request.flight_altitude_m)),
            DroneAssignment(
                request.drones[1].drone_id, shape(request.drones[1].search_geometry),
                (request.drones[1].home_x_m, request.drones[1].home_y_m,
                 request.flight_altitude_m)),
        )
        options = {
            "flight_altitude_m": request.flight_altitude_m,
            "horizontal_clearance_m": request.horizontal_clearance_m,
            "vertical_clearance_m": request.vertical_clearance_m,
            "scan_direction_deg": request.scan_direction_deg,
            "coverage_generation_method": _request_generation_method(request),
            "video_analysis_rate_hz": request.video_analysis_rate_hz,
            "control_point_spacing_m": request.control_point_spacing_m,
            "coverage_speed_mps": request.coverage_speed_mps,
            "connector_speed_mps": request.connector_speed_mps,
            "obstacle_speed_mps": request.obstacle_speed_mps,
            "return_speed_mps": request.return_speed_mps,
            "route_optimization_method": request.route_optimization_method,
            "completion_strategy": request.completion_strategy,
        }
        with PLANNING_LOCK:
            planning_started_at = perf_counter()
            result = TwoDroneCoveragePlanner().plan(
                assignments=assignments, semantic_map=semantic,
                camera=request.camera, planner_options=options)
            planning_time_s = perf_counter() - planning_started_at
            export_multi_plan(result, RESULTS)
        response = []
        for drone in result.drones:
            first = drone.result.continuous_flight.waypoints[0]
            response.append({
                "drone_id": drone.drone_id,
                "responsibility_area": mapping(drone.assigned_geometry),
                "summary": _web_result(
                    drone.result,
                    [first.x, first.y, first.z],
                    camera=request.camera,
                    flight_altitude_m=request.flight_altitude_m,
                ),
            })
        return {"planning_time_s": planning_time_s, "drones": response}
    except (ValueError, TypeError) as exc:
        raise HTTPException(status_code=422, detail=str(exc)) from exc


def _web_result(
    result: PlanResult,
    home: list[float],
    *,
    camera: CameraConfig,
    flight_altitude_m: float,
) -> dict[str, Any]:
    coverage_ratio = (
        sum(patch.area_m2 * patch.coverage_ratio for patch in result.patches)
        / result.effective_area.geometry.area
    )
    worst_patch = min(result.patches, key=lambda patch: patch.coverage_ratio, default=None)
    uncovered_area_m2 = sum(
        patch.area_m2 * (1.0 - patch.coverage_ratio) for patch in result.patches)
    route_coordinates = [(waypoint.x, waypoint.y)
                         for waypoint in result.continuous_flight.waypoints]
    route = LineString(route_coordinates) if len(route_coordinates) > 1 else None
    route_length = route.length if route is not None else 0.0
    visibility_samples = sorted(({
        "id": sample_id,
        "geometry": mapping(geometry),
        "route_progress": (route.project(geometry.centroid) / route_length
                           if route is not None and route_length else 0.0),
    } for sample_id, geometry in result.visibility_samples),
        key=lambda sample: (sample["route_progress"], sample["id"]))
    coverage_cells = (
        build_boustrophedon_planning_cells(
            result.effective_area.geometry,
            camera=camera,
            flight_altitude_m=flight_altitude_m,
            ground_elevation_m=0.0,
            scan_direction_deg=result.scan_direction_deg,
        ) if result.coverage_generation_method == "bcd" else ())
    return {
            "coverage_ratio": coverage_ratio,
            "minimum_required_coverage_ratio": result.minimum_required_coverage_ratio,
            "coverage_requirement_met": result.coverage_requirement_met,
            "worst_patch_id": worst_patch.id if worst_patch is not None else None,
            "worst_patch_coverage_ratio": (
                worst_patch.coverage_ratio if worst_patch is not None else None),
            "uncovered_area_m2": uncovered_area_m2,
            "unreachable": list(result.unreachable_patch_ids),
            "unreachable_candidate_point_count": len(
                result.unreachable_candidate_point_ids),
            "uncovered_patch_count": len(result.unreachable_patch_ids),
            "warnings": list(result.warnings),
            "scan_pattern": result.scan_pattern,
            "coverage_generation_method": result.coverage_generation_method,
            "route_optimization_method": result.route_optimization_method,
            "completion_strategy": result.completion_strategy,
            "route_optimization_candidates": [{
                "method": candidate.method,
                "transition_distance_m": candidate.transition_cost_m,
                "return_distance_m": candidate.return_cost_m,
                "connection_distance_m": (
                    candidate.transition_cost_m + candidate.return_cost_m),
            } for candidate in result.route_optimization_candidates],
            "scan_direction_deg": result.scan_direction_deg,
            "path_length_m": result.path_length_m,
            "lane_count": len(result.continuous_flight.lanes),
            "flight_waypoint_count": len(result.continuous_flight.waypoints),
            "visibility_sample_count": result.continuous_flight.visibility_sample_count,
            "initial_candidate_metrics": [{
                "pattern": item.pattern, "coverage_ratio": item.coverage_ratio,
                "planning_point_count": item.planning_point_count,
                "path_length_m": item.path_length_m,
                "unreachable_count": item.unreachable_patch_count,
            } for item in result.strategy_comparison],
            "final_solution_metrics": {
                "coverage_ratio": coverage_ratio,
                "path_length_m": result.path_length_m,
                "uncovered_patch_count": len(result.unreachable_patch_ids),
            },
            "home": home,
            "effective_area": mapping(result.effective_area.geometry),
            "visible_detection_area": mapping(result.visible_detection_geometry),
            "visibility_samples": visibility_samples,
            "obstacles": mapping(result.obstacles.geometry),
            "coverage_cells": [{
                "id": f"cell_{index + 1:03d}",
                "index": index,
                "geometry": mapping(cell),
                "area_m2": cell.area,
                "label_point": [
                    cell.representative_point().x,
                    cell.representative_point().y,
                ],
            } for index, cell in enumerate(coverage_cells)],
            "completion_points": [{
                "id": waypoint.id,
                "x": waypoint.x,
                "y": waypoint.y,
                "z": waypoint.z,
            } for waypoint in result.planning_route
                if waypoint.is_completion],
            "patches": [{"id":p.id,"geometry":mapping(p.geometry),"covered":p.covered,"ratio":p.coverage_ratio} for p in result.patches],
            "flight_waypoints": [{
                "id": w.id, "x": w.x, "y": w.y, "z": w.z,
                "heading_deg": w.heading_deg, "speed_mps": w.speed_mps,
                "turn_in_place": w.turn_in_place, "hold_time_s": w.hold_time_s,
            } for w in result.continuous_flight.waypoints],
            "route_segments": [{
                "id": segment.id, "kind": segment.kind,
                "start_waypoint_id": segment.start_waypoint_id,
                "end_waypoint_id": segment.end_waypoint_id,
                "heading_deg": segment.heading_deg, "speed_mps": segment.speed_mps,
                "detection_enabled": segment.detection_enabled,
                "source_scan_line_index": segment.source_scan_line_index,
                "source_scan_segment_index": segment.source_scan_segment_index,
                "source_coverage_cell_index": segment.source_coverage_cell_index,
            } for segment in result.continuous_flight.route_segments],
        }


@app.get("/api/export/{filename}")
def download(filename: str) -> FileResponse:
    allowed={"flight_plan.json","flight_plan.yaml","mission_manifest.json",
             "patches.geojson","route.geojson","coverage_report.json","visualization.png"}
    if filename not in allowed or not (RESULTS/filename).is_file():
        raise HTTPException(status_code=404, detail="export not found")
    return FileResponse(RESULTS/filename, filename=filename)


@app.get("/api/export/{drone_id}/{filename}")
def download_drone(drone_id: str, filename: str) -> FileResponse:
    allowed_drones = {"drone_1", "drone_2"}
    allowed = {"flight_plan.json", "flight_plan.yaml", "patches.geojson", "route.geojson",
               "coverage_report.json", "visualization.png"}
    path = RESULTS / drone_id / filename
    if drone_id not in allowed_drones or filename not in allowed or not path.is_file():
        raise HTTPException(status_code=404, detail="export not found")
    return FileResponse(path, filename=filename)


app.mount("/", StaticFiles(directory=WEB, html=True), name="web")
