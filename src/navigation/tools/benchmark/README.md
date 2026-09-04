# benchmark

Randomly generates a **flat Gazebo map with gate/pillar obstacles** as a world
`.sdf` for benchmarking the SUPER planner under obstacle-dense environments.

## What it does

`benchmark/generate_map.py` reads [`config/benchmark.yaml`](config/benchmark.yaml),
discretizes the map into an **N×N grid** (default 10×10), and places
**`objects_per_cell`** random gates/pillars in each cell. A **clearing
distance** enforces a minimum center-to-center gap between every pair of
obstacles (and refuses positions that would overlap geometries). The result is
written as a Gazebo world with a flat ground plane, sun, and static obstacle
models.

Gates are finite, horizontal **suspended slabs** meant to test vertical obstacle
avoidance: the drone must pass beneath the slab or fly above it when crossing
its span. They remain bypassable around their finite ends. Pillars remain solid
cylinders. Tune the slab span, centre-height range, thickness, and depth in
`config/benchmark.yaml`.

## Density control

- `grid.n_x` / `grid.n_y` — the map is split into an N×N grid.
- `objects_per_cell` — how many obstacles spawn in each cell (the density
  parameter).
- `clearing_distance` — minimum distance between obstacle centers; larger
  values make the map sparser and prevent overlap.

## Usage

```bash
# Build (once)
colcon build --packages-select benchmark --symlink-install

# Generate a world (writes to the PX4 worlds dir by default)
source install/setup.bash
ros2 run benchmark generate_map

# Options
ros2 run benchmark generate_map --config path/to/benchmark.yaml \
                                --output /custom/output/dir \
                                --seed 42
```

Then load it in the sim by setting the world in
[`src/navigation/config/simulation.yaml`](../../config/simulation.yaml):

```yaml
world: benchmark
```

and launch as usual:

```bash
./src/utils/start_sim.sh        # Terminal 1
ros2 launch offboard offboard.launch.py
```

## Populating the offboard waypoint buffer (`waypoint_populator`)

A standalone node that publishes a **serpentine (lawnmower) path** over the
benchmark map — top to bottom, alternating left-to-right / right-to-left
between rows — so the offboard node's waypoint buffer gets populated and flown
automatically. Run it separately from the offboard stack (it does not modify
the offboard package):

```bash
# Terminal 2 (after offboard is up)
ros2 run benchmark waypoint_populator

# Options
ros2 run benchmark waypoint_populator \
    --spacing 2.5 --margin 1.0 --z 5.0 \
    --start-delay 5.0
```

By default the route is submitted atomically to the offboard waypoint-buffer service
(`/waypoint_buffer`). Override it with `--queue-service NAME`. RViz's `2D Goal Pose`
tool still sends individual manual goals through `/waypoint_pose`.
Map size is read from `config/benchmark.yaml`; override with `--map-x/--map-y`.

## Config keys (`config/benchmark.yaml`)

| Key | Description |
|-----|-------------|
| `map.x`, `map.y` | Map size [m] (flat ground plane) |
| `grid.n_x`, `grid.n_y` | Grid discretization (default 10×10) |
| `objects_per_cell` | Number of obstacles per grid cell (density) |
| `gates.enabled`, `gates.width_min/max`, `gates.center_height_min/max`, `gates.slab_thickness`, `gates.slab_depth` | Random slab span and centre-height range; fixed vertical thickness and local-Y depth [m] |
| `pillars.enabled`, `pillars.radius_min/max`, `pillars.height_min/max` | Pillar radius/height ranges [m] |
| `clearing_distance` | Min center-to-center distance between obstacles [m] |
| `spawn_clearance` | Clear radius around the world origin (drone spawn point) [m] |
| `max_attempts` | Placement retries per object before it is skipped |
| `seed` | RNG seed (`null` = random) |
| `world_name` | Output `<world_name>.sdf` |
| `output_dir` | Output directory (relative to project root; default = PX4 worlds dir) |
