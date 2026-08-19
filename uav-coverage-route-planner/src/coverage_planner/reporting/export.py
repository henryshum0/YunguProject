"""Deterministic mission artifact export."""

from __future__ import annotations

import json
from pathlib import Path

import matplotlib
import yaml

matplotlib.use("Agg")
from matplotlib import pyplot as plt
from shapely.geometry import LineString, mapping

from coverage_planner.multi_planner import MultiDronePlan
from coverage_planner.planner import PlanResult


def export_plan(result: PlanResult, output_dir: str | Path) -> Path:
    output = Path(output_dir); output.mkdir(parents=True, exist_ok=True)
    summary = _summary(result)
    flight_mission = _flight_mission(result)
    _json(output/"flight_plan.json", flight_mission)
    (output/"flight_plan.yaml").write_text(
        yaml.safe_dump(flight_mission, sort_keys=False, allow_unicode=True), encoding="utf-8")
    _json(output/"patches.geojson", {"type":"FeatureCollection","features":[
        {"type":"Feature","geometry":mapping(p.geometry),"properties":{"id":p.id,"area_m2":p.area_m2,
         "covered":p.covered,"coverage_ratio":p.coverage_ratio}} for p in result.patches]})
    flight_points = result.continuous_flight.waypoints
    route = LineString([(w.x,w.y) for w in flight_points]) if len(flight_points)>1 else LineString()
    _json(output/"route.geojson", {"type":"Feature","geometry":mapping(route),"properties":summary})
    _json(output/"coverage_report.json", summary | {"warnings":list(result.warnings)})
    _visualization(result, output/"visualization.png")
    return output


def export_multi_plan(plan: MultiDronePlan, output_dir: str | Path) -> Path:
    """Write two independent flight plans plus one coordination-free manifest."""
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    drones = []
    for drone in plan.drones:
        drone_dir = export_plan(drone.result, output / drone.drone_id)
        drones.append({
            "drone_id": drone.drone_id,
            "responsibility_geometry": mapping(drone.assigned_geometry),
            "responsibility_area_m2": drone.result.effective_area.geometry.area,
            "path_length_m": drone.result.path_length_m,
            "coverage_ratio": _summary(drone.result)["coverage_ratio"],
            "flight_plan": str((drone_dir / "flight_plan.json").relative_to(output)),
            "coverage_report": str((drone_dir / "coverage_report.json").relative_to(output)),
        })
    _json(output / "mission_manifest.json", {
        "schema_version": "1.0", "mission_type": "independent_two_drone_detection",
        "temporal_collision_avoidance": False, "simultaneous_start": True,
        "mission_status": ("ready" if all(
            drone.result.coverage_requirement_met for drone in plan.drones)
            else "infeasible_coverage"),
        "drones": drones,
    })
    return output

def _summary(result: PlanResult) -> dict[str, object]:
    covered=sum(p.area_m2*p.coverage_ratio for p in result.patches); effective=result.effective_area.geometry.area
    segment_lengths: dict[str, float] = {}
    for segment in result.continuous_flight.route_segments:
        segment_lengths[segment.kind] = segment_lengths.get(segment.kind, 0.0) + segment.length_m
    transition_distance = sum(
        segment.length_m for segment in result.continuous_flight.route_segments
        if segment.kind != "coverage_lane")
    exact = next((candidate for candidate in result.route_optimization_candidates
                  if candidate.method == "exact"), None)
    selected_connection = min(
        (candidate.transition_cost_m + candidate.return_cost_m
         for candidate in result.route_optimization_candidates), default=0.0)
    exact_connection = (
        exact.transition_cost_m + exact.return_cost_m if exact is not None else None)
    return {"flight_waypoint_count":len(result.continuous_flight.waypoints),
      "route_segment_count":len(result.continuous_flight.route_segments),
      "coverage_lane_count":len(result.continuous_flight.lanes),"path_length_m":result.path_length_m,
      "total_requested_area_m2":result.effective_area.metrics.requested_area_m2,
      "effective_search_area_m2":effective,"building_excluded_area_m2":result.effective_area.metrics.building_excluded_area_m2,
      "covered_area_m2":covered,"uncovered_area_m2":max(0,effective-covered),
      "coverage_ratio":covered/effective if effective else 0,"scan_direction_deg":result.scan_direction_deg,
      "minimum_required_coverage_ratio":result.minimum_required_coverage_ratio,
      "coverage_requirement_met":result.coverage_requirement_met,
      "scan_pattern":result.scan_pattern,
      "coverage_generation_method":result.coverage_generation_method,
      "route_optimization_method":result.route_optimization_method,
      "completion_strategy":result.completion_strategy,
      "optimization_method":"coverage_generation_plus_route_optimization",
      "route_optimization_candidates":[{
          "method": candidate.method,
          "transition_distance_m": candidate.transition_cost_m,
          "return_distance_m": candidate.return_cost_m,
          "connection_distance_m": candidate.transition_cost_m + candidate.return_cost_m,
      } for candidate in result.route_optimization_candidates],
      "optimality_gap": (
          (selected_connection - exact_connection) / exact_connection
          if exact_connection not in {None, 0.0} else None),
      "initial_candidate_metrics":[{
          "pattern": item.pattern, "coverage_ratio": item.coverage_ratio,
          "planning_point_count": item.planning_point_count,
          "path_length_m": item.path_length_m,
          "unreachable_patch_count": item.unreachable_patch_count,
      } for item in result.strategy_comparison],
      "final_solution_metrics": {
          "coverage_ratio": covered/effective if effective else 0,
          "path_length_m": result.path_length_m,
          "flight_waypoint_count": len(result.continuous_flight.waypoints),
          "uncovered_patch_count": len(result.unreachable_patch_ids),
      },
      "transition_distance_m":transition_distance,
      "turn_count":max(0, len(result.continuous_flight.lanes)-1),
      "minimum_obstacle_clearance_m":result.minimum_obstacle_clearance_m,
      "coverage_lane_length_m":segment_lengths.get("coverage_lane", 0.0),
      "connector_length_m":segment_lengths.get("connector", 0.0),
      "obstacle_avoidance_length_m":segment_lengths.get("obstacle_avoidance", 0.0),
      "return_home_length_m":segment_lengths.get("return_home", 0.0),
      "transition_distance_ratio":(
          transition_distance/result.path_length_m if result.path_length_m else 0.0),
      "visibility_sample_count":result.continuous_flight.visibility_sample_count,
      "unreachable_candidate_point_count":len(result.unreachable_candidate_point_ids),
      "unreachable_candidate_point_ids":list(result.unreachable_candidate_point_ids),
      "uncovered_patch_count":len(result.unreachable_patch_ids),
      "unreachable_patch_ids":list(result.unreachable_patch_ids),
      "unreachable_ground":[{
          "geometry": mapping(item.geometry),
          "area_m2": item.area_m2,
          "patch_ids": list(item.patch_ids),
          "reason": item.reason,
      } for item in result.unreachable_ground]}


def _flight_mission(result: PlanResult) -> dict[str, object]:
    flight = result.continuous_flight
    return {
        "schema_version": "3.0", "coordinate_frame": "ENU", "units": "meters",
        "mission_status": ("ready" if result.coverage_requirement_met
                           else "infeasible_coverage"),
        "map_id": result.semantic_map.world_name,
        "video_detection": {
            "mode": "continuous_video_stream",
            "analysis_rate_hz": flight.video_analysis_rate_hz,
            "control_point_spacing_m": flight.control_point_spacing_m,
            "forward_overlap": flight.forward_overlap,
            "lane_overlap": flight.lane_overlap,
            "target_envelope": {"width_m": flight.target_width_m,
                                "length_m": flight.target_length_m,
                                "height_m": flight.target_height_m},
            "image_boundary_margin_ratio": flight.image_boundary_margin_ratio,
            "building_wall_occlusion": True,
        },
        "lanes": [{
            "id": lane.id, "sequence": lane.sequence, "heading_deg": lane.heading_deg,
            "speed_mps": lane.speed_mps, "route_segment_ids": list(lane.route_segment_ids),
            "length_m": lane.length_m,
        } for lane in flight.lanes],
        "route_segments": [{
            "id": segment.id, "sequence": segment.sequence, "kind": segment.kind,
            "start_waypoint_id": segment.start_waypoint_id,
            "end_waypoint_id": segment.end_waypoint_id,
            "heading_deg": segment.heading_deg, "speed_mps": segment.speed_mps,
            "length_m": segment.length_m, "detection_enabled": segment.detection_enabled,
            "source_scan_line_index": segment.source_scan_line_index,
            "source_scan_segment_index": segment.source_scan_segment_index,
            "source_coverage_cell_index": segment.source_coverage_cell_index,
        } for segment in flight.route_segments],
        "waypoints": [{
            "id": waypoint.id, "sequence": waypoint.sequence,
            "x": waypoint.x, "y": waypoint.y, "z": waypoint.z,
            "heading_deg": waypoint.heading_deg, "speed_mps": waypoint.speed_mps,
            "turn_in_place": waypoint.turn_in_place,
            "hold_time_s": waypoint.hold_time_s,
        } for waypoint in flight.waypoints],
        "summary": _summary(result),
    }


def _json(path: Path, payload: object) -> None:
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")


def _visualization(result: PlanResult, path: Path) -> None:
    figure, axes = plt.subplots(figsize=(12, 8))
    for patch in result.patches:
        geometries = patch.geometry.geoms if patch.geometry.geom_type == "MultiPolygon" else [patch.geometry]
        for geometry in geometries:
            x, y = geometry.exterior.xy
            axes.fill(x, y, color="#4f9d69" if patch.covered else "#d95d5d", alpha=0.35)
    flight_points = result.continuous_flight.waypoints
    if flight_points:
        axes.plot([w.x for w in flight_points], [w.y for w in flight_points], color="#1864ab", linewidth=.8)
        axes.scatter([w.x for w in flight_points], [w.y for w in flight_points],
                     s=5, color="#111111")
    axes.set_aspect("equal"); axes.set_xlabel("East x (m)"); axes.set_ylabel("North y (m)")
    axes.set_title(f"{result.semantic_map.world_name} coverage plan"); figure.tight_layout()
    figure.savefig(path, dpi=160); plt.close(figure)
