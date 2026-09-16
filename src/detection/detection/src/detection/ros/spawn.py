"""Put the ground-truth targets into a running Gazebo world.

The targets are spawned at runtime rather than written into the world file for
two reasons: ``VisionFlow-PX4`` is a submodule that should stay clean, and the
target set then has a single owner — ``src/detection/config/targets.yaml``, the
same file the mock detector projects from. What the camera sees and what the
mock reports therefore cannot drift apart.

Positions in that file are ENU metres in the navigation frame; this script adds
``world_origin_in_gz`` to convert them to Gazebo world coordinates.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import shutil
import subprocess
import sys
import time
from math import radians

import yaml

from detection.config import DetectionConfig, DetectionConfigError, load_targets, resolve_workspace_path


SPAWN_TIMEOUT_SEC = 20.0
REMOVE_TIMEOUT_MS = 3000

#: How often the world is polled while waiting for the simulator to come up.
WORLD_POLL_PERIOD_SEC = 2.0


class SpawnError(RuntimeError):
    """Raised when targets cannot be placed into the running world."""


def _create_executable() -> list[str]:
    """Locate the ros_gz_sim entity spawner, preferring a direct invocation."""
    try:
        from ament_index_python.packages import get_package_prefix

        candidate = Path(get_package_prefix("ros_gz_sim")) / "lib" / "ros_gz_sim" / "create"
        if candidate.is_file():
            return [str(candidate)]
    except Exception:  # noqa: BLE001 - fall back to the ros2 CLI below
        pass
    if shutil.which("ros2") is None:
        raise SpawnError(
            "neither the ros_gz_sim 'create' executable nor the 'ros2' CLI is available; "
            "source the ROS 2 environment first")
    return ["ros2", "run", "ros_gz_sim", "create"]


def wait_for_world(world: str, *, wait_sec: float) -> bool:
    """Wait until the Gazebo world offers its entity-spawn service.

    Lets the spawner be started together with the rest of the stack instead of
    only after the simulator has finished coming up.
    """
    if wait_sec <= 0.0:
        return True
    if shutil.which("gz") is None:
        return True   # nothing to poll with; let the spawn attempt report the failure
    service = f"/world/{world}/create"
    deadline = time.monotonic() + wait_sec
    announced = False
    while True:
        result = subprocess.run(["gz", "service", "-l"], capture_output=True, text=True,
                                timeout=WORLD_POLL_PERIOD_SEC * 3, check=False)
        if service in result.stdout:
            return True
        if time.monotonic() >= deadline:
            return False
        if not announced:
            announced = True
            print(f"  waiting up to {wait_sec:.0f}s for Gazebo world '{world}' ...")
        time.sleep(WORLD_POLL_PERIOD_SEC)


def _world_name(explicit: str, reference: Path) -> str:
    """Return the Gazebo world name, defaulting to the simulation configuration."""
    if explicit:
        return explicit
    simulation_config = resolve_workspace_path(
        "src/simulation/config/simulation.yaml", reference=reference)
    try:
        payload = yaml.safe_load(simulation_config.read_text(encoding="utf-8")) or {}
    except OSError as error:
        raise SpawnError(
            f"cannot read the simulation config '{simulation_config}' to find the world "
            f"name; pass --world explicitly: {error}") from error
    world = str(payload.get("world", "")).strip()
    if not world:
        raise SpawnError(f"'{simulation_config}' declares no world; pass --world explicitly")
    return world


def spawn_targets(
    *,
    detection_config: Path,
    targets_config: Path,
    models_dir: Path,
    world: str,
    dry_run: bool = False,
) -> int:
    """Spawn every configured target; return how many entered the world."""
    detection = DetectionConfig.load(detection_config)
    field = load_targets(targets_config, ground_z_m=detection.world.ground_z_m)
    command_prefix = _create_executable() if not dry_run else ["<create>"]

    spawned = 0
    for target in field.targets:
        model_file = models_dir / target.model / "model.sdf"
        if not model_file.is_file():
            print(f"  {target.target_id}: no model at '{model_file}', skipped", file=sys.stderr)
            continue
        x, y, z = target.gazebo_position(field.world_origin_in_gz)
        command = [
            *command_prefix,
            "-world", world,
            "-file", str(model_file),
            "-name", target.target_id,
            "-x", f"{x:.4f}", "-y", f"{y:.4f}", "-z", f"{z:.4f}",
            "-Y", f"{radians(target.yaw_deg):.6f}",
        ]
        if dry_run:
            print("  " + " ".join(command))
            spawned += 1
            continue
        result = subprocess.run(
            command, capture_output=True, text=True, timeout=SPAWN_TIMEOUT_SEC, check=False)
        if result.returncode != 0:
            message = (result.stderr or result.stdout or "").strip().splitlines()
            detail = message[-1] if message else f"exit code {result.returncode}"
            print(f"  {target.target_id}: not spawned ({detail})", file=sys.stderr)
            continue
        print(f"  {target.target_id}: {target.class_id} at gz ({x:.2f}, {y:.2f}, {z:.2f}), "
              f"yaw {target.yaw_deg:.0f} deg")
        spawned += 1
    return spawned


def remove_targets(*, detection_config: Path, targets_config: Path, world: str) -> int:
    """Remove every configured target from the world; return how many were removed."""
    if shutil.which("gz") is None:
        raise SpawnError("the 'gz' command is required to remove entities")
    detection = DetectionConfig.load(detection_config)
    field = load_targets(targets_config, ground_z_m=detection.world.ground_z_m)
    removed = 0
    for target in field.targets:
        result = subprocess.run(
            ["gz", "service", "-s", f"/world/{world}/remove",
             "--reqtype", "gz.msgs.Entity", "--reptype", "gz.msgs.Boolean",
             "--timeout", str(REMOVE_TIMEOUT_MS),
             "--req", f'name: "{target.target_id}", type: MODEL'],
            capture_output=True, text=True, check=False)
        if result.returncode == 0 and "true" in result.stdout.lower():
            print(f"  {target.target_id}: removed")
            removed += 1
        else:
            print(f"  {target.target_id}: not removed (not in the world?)", file=sys.stderr)
    return removed


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="Spawn the configured ground-truth detection targets into a running "
                    "Gazebo world, or remove them again.")
    parser.add_argument("--detection-config", required=True, type=Path,
                        help="path to src/detection/config/detection.yaml")
    parser.add_argument("--targets-config", required=True, type=Path,
                        help="path to src/detection/config/targets.yaml")
    parser.add_argument("--models-dir", required=True, type=Path,
                        help="directory holding the target Gazebo models")
    parser.add_argument("--world", default="",
                        help="Gazebo world name (default: the world in simulation.yaml)")
    parser.add_argument("--remove", action="store_true",
                        help="remove the configured targets instead of spawning them")
    parser.add_argument("--dry-run", action="store_true",
                        help="print the spawn commands without contacting Gazebo")
    parser.add_argument("--wait-sec", type=float, default=0.0,
                        help="wait this long for the Gazebo world before spawning")
    arguments = parser.parse_args(argv)

    try:
        world = _world_name(arguments.world, arguments.targets_config)
        if arguments.remove:
            print(f"Removing detection targets from Gazebo world '{world}':")
            count = remove_targets(
                detection_config=arguments.detection_config,
                targets_config=arguments.targets_config,
                world=world)
            print(f"{count} target(s) removed.")
            return 0
        if not arguments.dry_run and not wait_for_world(world, wait_sec=arguments.wait_sec):
            print(f"spawn_targets: Gazebo world '{world}' did not appear within "
                  f"{arguments.wait_sec:.0f}s; is the simulation running?", file=sys.stderr)
            return 1
        print(f"Spawning detection targets into Gazebo world '{world}':")
        count = spawn_targets(
            detection_config=arguments.detection_config,
            targets_config=arguments.targets_config,
            models_dir=arguments.models_dir,
            world=world,
            dry_run=arguments.dry_run)
        print(f"{count} target(s) spawned.")
        return 0 if count else 1
    except (DetectionConfigError, SpawnError) as error:
        print(f"spawn_targets: {error}", file=sys.stderr)
        return 1
    except subprocess.TimeoutExpired:
        print("spawn_targets: Gazebo did not answer; is the simulation running?", file=sys.stderr)
        return 1
