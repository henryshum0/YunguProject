#!/usr/bin/env bash
#
# One-shot launcher for visualizing the X500 LiDAR in RViz2:
#   1. GZ -> ROS bridge (lidar scan + point cloud + odometry) via ros_gz_bridge
#   2. TF bridge (world -> base_link from /odom, static base_link -> lidar_link)
#   3. RViz2 with a ready-made config (fixed frame = world)
#
# Usage:
#   ./bridge_all.sh            # bridge + TF + RViz
#   ./bridge_all.sh --no-rviz  # bridge + TF only (you open RViz yourself)
#
# Prerequisites: ROS 2 (Humble) sourced or installed at /opt/ros/humble,
# ros_gz_bridge installed, and the gz-sim server running with x500_lidar
# spawned (e.g. `make px4_sitl gz_x500_lidar_yungu`).

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_FILE="${SCRIPT_DIR}/x500_lidar_bridge.yaml"
TF_NODE="${SCRIPT_DIR}/tf_bridge.py"
RVIZ_CONFIG="${SCRIPT_DIR}/x500.rviz"

LAUNCH_RVIZ=1
for arg in "$@"; do
  case "$arg" in
    --no-rviz) LAUNCH_RVIZ=0 ;;
    *) echo "Unknown argument: $arg (supported: --no-rviz)" >&2; exit 1 ;;
  esac
done

# This system runs gz-sim 8 (Harmonic), i.e. gz-transport13 / gz-msgs10.
export GZ_VERSION="${GZ_VERSION:-harmonic}"

if ! command -v ros2 >/dev/null 2>&1; then
  if [[ -f /opt/ros/humble/setup.bash ]]; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
  else
    echo "ERROR: 'ros2' not found and /opt/ros/humble/setup.bash is missing." >&2
    exit 1
  fi
fi

if [[ ! -f "${CONFIG_FILE}" || ! -f "${TF_NODE}" ]]; then
  echo "ERROR: missing bridge files in ${SCRIPT_DIR}" >&2
  exit 1
fi

pids=()
cleanup() {
  for p in "${pids[@]:-}"; do
    kill "$p" 2>/dev/null || true
  done
}
trap cleanup EXIT INT TERM

echo "Starting GZ -> ROS bridge using ${CONFIG_FILE} ..."
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:="${CONFIG_FILE}" &
pids+=("$!")

echo "Starting TF bridge (${TF_NODE}) ..."
python3 "${TF_NODE}" &
pids+=("$!")

if [[ "${LAUNCH_RVIZ}" == "1" ]]; then
  sleep 3   # let the bridge + TF come up first
  echo "Launching RViz2 with ${RVIZ_CONFIG} (fixed frame: world) ..."
  ros2 run rviz2 rviz2 -d "${RVIZ_CONFIG}" &
  pids+=("$!")
fi

wait
