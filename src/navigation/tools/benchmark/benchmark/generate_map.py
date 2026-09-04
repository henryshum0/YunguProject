#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Generate a flat Gazebo world with random gate/pillar obstacles.

The map is discretized into an NxN grid (config ``grid.n_x`` x ``grid.n_y``)
and every cell receives ``objects_per_cell`` random obstacles (gates and/or
pillars) at random positions inside the cell. A clearing distance keeps a
minimum center-to-center gap between obstacles (and the generator refuses
positions that would make geometries overlap). The result is a Gazebo world
``.sdf`` (ground plane + sun + static obstacle models) which the PX4 sim can
load via ``world: <world_name>`` in ``src/navigation/config/simulation.yaml``.

Usage:
    python3 src/navigation/tools/benchmark/benchmark/generate_map.py [--config PATH]
    ros2 run benchmark generate_map [--config PATH] [--output DIR] [--seed N]
"""

import argparse
import math
import random
import sys
from pathlib import Path

try:
    import yaml
except ImportError:  # pragma: no cover
    sys.exit("ERROR: PyYAML is required. Install with: pip install PyYAML")


def find_project_root(start: Path) -> Path | None:
    """Closest workspace ancestor containing navigation configuration."""
    for parent in (start, *start.parents):
        if (parent / "src" / "navigation" / "config" / "simulation.yaml").is_file():
            return parent
    return None


def load_config(path: Path) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    if not isinstance(cfg, dict) or "benchmark" not in cfg:
        sys.exit(f"ERROR: {path} must contain a top-level 'benchmark' mapping")
    return cfg["benchmark"]


def make_obstacle(rng: random.Random, cfg: dict):
    """Return a random obstacle descriptor, or None if no type is enabled."""
    gates_on = bool(cfg.get("gates", {}).get("enabled", True))
    pillars_on = bool(cfg.get("pillars", {}).get("enabled", True))
    choices = []
    if gates_on:
        choices.append("gate")
    if pillars_on:
        choices.append("pillar")
    if not choices:
        return None

    kind = rng.choice(choices)
    if kind == "gate":
        gate_cfg = cfg.get("gates", {})
        width = rng.uniform(float(gate_cfg["width_min"]),
                            float(gate_cfg["width_max"]))
        center_height = rng.uniform(float(gate_cfg["center_height_min"]),
                                    float(gate_cfg["center_height_max"]))
        thickness = float(gate_cfg["slab_thickness"])
        depth = float(gate_cfg["slab_depth"])
        # A circumscribed radius keeps randomly yawed slab footprints separate.
        half = math.hypot(width, depth) / 2.0
        return {"kind": "gate", "width": width,
                "center_height": center_height, "thickness": thickness,
                "depth": depth, "half": half,
                "yaw": rng.uniform(-math.pi, math.pi)}
    radius = rng.uniform(float(cfg["pillars"]["radius_min"]),
                         float(cfg["pillars"]["radius_max"]))
    height = rng.uniform(float(cfg["pillars"]["height_min"]),
                         float(cfg["pillars"]["height_max"]))
    return {"kind": "pillar", "radius": radius, "height": height,
            "half": radius, "yaw": 0.0}


def place_obstacles(cfg: dict):
    """Place obstacles cell by cell; returns (placed, skipped, map size).

    A grid cell is the *sampling region for an obstacle's center*: the
    footprint may extend into neighbouring cells (only the whole map bounds
    must contain it). ``objects_per_cell`` objects are attempted per cell, so
    raising it directly increases density up to what the obstacle sizes and the
    clearing distance physically allow.
    """
    map_x = float(cfg["map"]["x"])
    map_y = float(cfg["map"]["y"])
    nx = int(cfg["grid"]["n_x"])
    ny = int(cfg["grid"]["n_y"])
    per_cell = int(cfg["objects_per_cell"])
    clearing = float(cfg["clearing_distance"])
    spawn_clear = float(cfg.get("spawn_clearance", 0.0))
    max_attempts = int(cfg.get("max_attempts", 50))
    rng = random.Random(cfg.get("seed"))

    cell_w = map_x / nx
    cell_h = map_y / ny

    # Packing: to fit `per_cell` centers in one cell they need to be ~cell/per_line
    # apart, where per_line is how many fit along the short cell axis. Cap the
    # effective clearing at that spacing so objects_per_cell actually raises
    # density. Geometric overlap is still always prevented via the half-extents.
    per_line = max(1, math.ceil(math.sqrt(per_cell)))
    cell_spacing = min(cell_w, cell_h) / per_line
    effective_clearing = min(clearing, cell_spacing)

    placed = []   # list of placed obstacle dicts (with x, y)
    skipped = 0

    def too_close(x: float, y: float, half: float) -> bool:
        # Keep the drone spawn area (world origin) clear.
        if math.hypot(x, y) < spawn_clear:
            return True
        for o in placed:
            # Minimum center distance: effective clearing, but never less than
            # the two half-extents so geometries cannot overlap.
            min_dist = max(effective_clearing, half + o["half"])
            if math.hypot(x - o["x"], y - o["y"]) < min_dist:
                return True
        return False

    for ci in range(nx):
        x0, x1 = ci * cell_w, (ci + 1) * cell_w
        for cj in range(ny):
            y0, y1 = cj * cell_h, (cj + 1) * cell_h
            for _ in range(per_cell):
                obj = make_obstacle(rng, cfg)
                if obj is None:
                    return placed, skipped, map_x, map_y
                half = obj["half"]
                # Sample the CENTER inside the cell; the whole footprint only
                # has to stay on the map (it may cross cell borders).
                lo_x, hi_x = max(x0, half), min(x1, map_x - half)
                lo_y, hi_y = max(y0, half), min(y1, map_y - half)
                if lo_x >= hi_x or lo_y >= hi_y:
                    skipped += 1   # cannot fit this footprint on the map here
                    continue
                ok = False
                for _ in range(max_attempts):
                    x = rng.uniform(lo_x, hi_x)
                    y = rng.uniform(lo_y, hi_y)
                    if too_close(x, y, half):
                        continue
                    obj["x"], obj["y"] = x, y
                    placed.append(obj)
                    ok = True
                    break
                if not ok:
                    skipped += 1

    return placed, skipped, map_x, map_y


def build_sdf(cfg: dict, placed, map_x: float, map_y: float) -> str:
    world_name = cfg.get("world_name", "benchmark")
    gsize = max(map_x, map_y) + 20.0
    L = []
    A = L.append

    A('<?xml version="1.0"?>')
    A('<sdf version="1.9">')
    A(f'  <world name="{world_name}">')
    A('    <physics type="ode">')
    A('      <max_step_size>0.004</max_step_size>')
    A('      <real_time_factor>1.0</real_time_factor>')
    A('      <real_time_update_rate>250</real_time_update_rate>')
    A('    </physics>')
    A('    <gravity>0 0 -9.8</gravity>')
    A('    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>')
    A('    <atmosphere type="adiabatic" />')
    A('    <scene>')
    A('      <grid>false</grid>')
    A('      <ambient>0.58 0.58 0.56 1</ambient>')
    A('      <background>0.78 0.80 0.82 1</background>')
    A('      <shadows>true</shadows>')
    A('    </scene>')
    # Spherical coordinates are REQUIRED: the gz NavSat (GPS) sensor computes
    # lat/lon/alt from the world origin. Without this block it publishes zeros,
    # PX4 fuses that invalid GPS, and the EKF altitude estimate breaks (the
    # vehicle physically climbs in gz while PX4 thinks it is stationary).
    A('    <spherical_coordinates>')
    A('      <surface_model>EARTH_WGS84</surface_model>')
    A('      <world_frame_orientation>ENU</world_frame_orientation>')
    A('      <latitude_deg>30.334034</latitude_deg>')
    A('      <longitude_deg>120.036295</longitude_deg>')
    A('      <elevation>0</elevation>')
    A('    </spherical_coordinates>')

    # --- Flat ground plane ---
    A('    <model name="ground_plane">')
    A('      <static>true</static>')
    A('      <link name="link">')
    A('        <collision name="collision">')
    A(f'          <geometry><plane><normal>0 0 1</normal><size>{gsize:.1f} {gsize:.1f}</size></plane></geometry>')
    A('          <surface><friction><ode><mu>100</mu><mu2>50</mu2></ode></friction>'
      '<contact><ode /></contact><bounce /></surface>')
    A('        </collision>')
    A('        <visual name="visual">')
    A(f'          <geometry><plane><normal>0 0 1</normal><size>{gsize:.1f} {gsize:.1f}</size></plane></geometry>')
    A('          <material><ambient>0.74 0.74 0.70 1</ambient>'
      '<diffuse>0.74 0.74 0.70 1</diffuse><specular>0.05 0.05 0.05 1</specular></material>')
    A('        </visual>')
    A('      </link>')
    A('    </model>')

    # --- Sun ---
    A('    <light name="sunUTC" type="directional">')
    A('      <pose>0 0 500 0 0 0</pose>')
    A('      <cast_shadows>true</cast_shadows>')
    A('      <intensity>1.15</intensity>')
    A('      <direction>-0.25 0.45 -0.86</direction>')
    A('      <diffuse>0.98 0.94 0.86 1</diffuse>')
    A('      <specular>0.28 0.28 0.25 1</specular>')
    A('    </light>')

    # --- Obstacles ---
    for i, o in enumerate(placed):
        if o["kind"] == "gate":
            # A gate is a finite horizontal slab: traverse below or above it.
            w = o["width"]
            h = o["center_height"]
            t = o["thickness"]
            d = o["depth"]
            color = "0.85 0.45 0.10 1"      # orange (gates)
            A(f'    <model name="obstacle_{i}">')
            A('      <static>true</static>')
            A(f'      <pose>{o["x"]:.3f} {o["y"]:.3f} 0 0 0 {o["yaw"]:.3f}</pose>')
            A('      <link name="link">')
            geom = f'<box><size>{w:.3f} {d:.3f} {t:.3f}</size></box>'
            A('        <collision name="slab">'
              f'<pose>0 0 {h:.3f} 0 0 0</pose><geometry>{geom}</geometry>'
              '</collision>')
            A('        <visual name="slab_vis">'
              f'<pose>0 0 {h:.3f} 0 0 0</pose><geometry>{geom}</geometry>'
              f'<material><ambient>{color}</ambient><diffuse>{color}</diffuse>'
              '</material></visual>')
            A('      </link>')
            A('    </model>')
            continue

        # Pillar
        r, h = o["radius"], o["height"]
        z = h / 2.0
        geom = f'<cylinder><radius>{r:.3f}</radius><length>{h:.3f}</length></cylinder>'
        color = "0.55 0.55 0.62 1"   # grey
        A(f'    <model name="obstacle_{i}">')
        A('      <static>true</static>')
        A(f'      <pose>{o["x"]:.3f} {o["y"]:.3f} {z:.3f} 0 0 {o["yaw"]:.3f}</pose>')
        A('      <link name="link">')
        A(f'        <collision name="collision"><geometry>{geom}</geometry></collision>')
        A(f'        <visual name="visual"><geometry>{geom}</geometry>')
        A(f'          <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>')
        A('        </visual>')
        A('      </link>')
        A('    </model>')

    A('  </world>')
    A('</sdf>')
    return "\n".join(L) + "\n"


def main():
    parser = argparse.ArgumentParser(
        description="Generate a flat Gazebo benchmark world with random gate/pillar obstacles.")
    parser.add_argument("--config", default=None,
                        help="Path to benchmark.yaml "
                             "(default: <project>/src/navigation/tools/benchmark/config/benchmark.yaml)")
    parser.add_argument("--output", default=None,
                        help="Override the output directory (default from config)")
    parser.add_argument("--seed", type=int, default=None,
                        help="Override the RNG seed for reproducibility")
    args = parser.parse_args()

    script = Path(__file__).resolve()
    root = find_project_root(script)
    if root is None:
        root = script.parents[5]
    config_path = Path(args.config) if args.config else \
        root / "src" / "navigation" / "tools" / "benchmark" / "config" / "benchmark.yaml"
    if not config_path.is_file():
        sys.exit(f"ERROR: config not found: {config_path}")

    cfg = load_config(config_path)
    if args.seed is not None:
        cfg["seed"] = args.seed

    placed, skipped, map_x, map_y = place_obstacles(cfg)

    out_dir = Path(args.output) if args.output else Path(cfg["output_dir"])
    if not out_dir.is_absolute():
        out_dir = root / out_dir
    out_dir.mkdir(parents=True, exist_ok=True)
    world_name = cfg.get("world_name", "benchmark")
    out_path = out_dir / f"{world_name}.sdf"
    out_path.write_text(build_sdf(cfg, placed, map_x, map_y), encoding="utf-8")

    n_gates = sum(1 for o in placed if o["kind"] == "gate")
    n_pillars = len(placed) - n_gates
    expected = (int(cfg["grid"]["n_x"]) * int(cfg["grid"]["n_y"])
                * int(cfg["objects_per_cell"]))
    print(f"Benchmark world written: {out_path}")
    print(f"  map: {map_x:.1f} x {map_y:.1f} m, grid "
          f"{int(cfg['grid']['n_x'])}x{int(cfg['grid']['n_y'])}, "
          f"{int(cfg['objects_per_cell'])} obj/cell (target {expected})")
    print(f"  obstacles placed: {len(placed)} (gates={n_gates}, pillars={n_pillars}), "
          f"skipped={skipped}")
    print(f"  clearing distance: {cfg['clearing_distance']} m, seed: {cfg.get('seed')}")
    print(f"  to load in the sim: set `world: {world_name}` in "
          "src/navigation/config/simulation.yaml")


if __name__ == "__main__":
    main()
