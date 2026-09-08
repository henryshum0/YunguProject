#!/usr/bin/env bash
#
# Launch the GZ <-> ROS bridge + TF bridge for the Yungu sim stack.
# The topics to bridge, and whether to launch the parameter_bridge / image bridge / TF bridge,
# are read from src/simulation/config/simulation.yaml (the `bridge` section).
#
# Usage:
#   ./bridge.sh            # bridge + TF only
#
# Prerequisites: ROS 2 (Humble) sourced or installed at /opt/ros/humble,
# ros_gz_bridge + ros_gz_image installed, and the gz-sim server running (e.g. via start_sim.sh).

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "${SCRIPT_DIR}/.." && pwd)"
SIMULATION_CONFIG_DIR="${WORKSPACE}/src/simulation/config"
SIM_CONFIG="${YUNGU_SIM_CONFIG:-${SIMULATION_CONFIG_DIR}/simulation.yaml}"
SIM_CONFIG_HELPER="${SIMULATION_CONFIG_DIR}/sim_config.py"
BRIDGE_YAML=""

if [[ ! -f "${SIM_CONFIG}" || ! -f "${SIM_CONFIG_HELPER}" ]]; then
  echo "ERROR: missing simulation config or config helper." >&2
  exit 1
fi

# This system runs gz-sim 8 (Harmonic), i.e. gz-transport13 / gz-msgs10.
export GZ_VERSION="${GZ_VERSION:-$(python3 "${SIM_CONFIG_HELPER}" --config "${SIM_CONFIG}" get gz_version)}"

if ! command -v ros2 >/dev/null 2>&1; then
  if [[ -f /opt/ros/humble/setup.bash ]]; then
    # shellcheck disable=SC1091
    # ROS 2's generated setup scripts probe variables that may be unset, so
    # temporarily disable nounset while sourcing them.
    set +u
    source /opt/ros/humble/setup.bash
    set -u
  else
    echo "ERROR: 'ros2' not found and /opt/ros/humble/setup.bash is missing." >&2
    exit 1
  fi
fi

# Also source the workspace overlay so `ros2 run lidar_bridge tf_bridge`
# resolves. Without it (start_sim.sh doesn't source the overlay either), the
# TF bridge dies with "Package 'lidar_bridge' not found" and the
# world -> base_link TF goes missing in RViz.
if [[ -f "${WORKSPACE}/install/setup.bash" ]]; then
  set +u
  # shellcheck disable=SC1091
  source "${WORKSPACE}/install/setup.bash"
  set -u
fi

bridge_enabled="$(python3 "${SIM_CONFIG_HELPER}" --config "${SIM_CONFIG}" get bridge.enabled)"
tf_enabled="$(python3 "${SIM_CONFIG_HELPER}" --config "${SIM_CONFIG}" get bridge.tf_enabled)"

pids=()
cleanup() {
  for p in "${pids[@]:-}"; do
    kill "$p" 2>/dev/null || true
  done
  if [[ -n "${BRIDGE_YAML}" ]]; then
    rm -f "${BRIDGE_YAML}"
  fi
}
trap cleanup EXIT INT TERM HUP

if [[ "${bridge_enabled}" == "true" ]]; then
  # Generate the ros_gz_bridge parameter_bridge config from the configured topics.
  BRIDGE_YAML="$(mktemp /tmp/yungu_bridge_XXXXXX.yaml)"
  python3 "${SIM_CONFIG_HELPER}" --config "${SIM_CONFIG}" bridge-config >"${BRIDGE_YAML}"
  echo "Starting GZ -> ROS bridge using ${BRIDGE_YAML} (topics from ${SIM_CONFIG}) ..."
  ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:="${BRIDGE_YAML}" &
  pids+=("$!")

  mapfile -t image_topics < <(python3 "${SIM_CONFIG_HELPER}" --config "${SIM_CONFIG}" image-bridge-topics)
  if [[ "${#image_topics[@]}" -gt 0 ]]; then
    if ! ros2 pkg executables ros_gz_image 2>/dev/null | grep -qx 'ros_gz_image image_bridge'; then
      echo "ERROR: ros_gz_image/image_bridge is required for configured camera topics. Install ros-humble-ros-gz-image." >&2
      exit 1
    fi
    echo "Starting GZ -> ROS image bridge for: ${image_topics[*]}"
    ros2 run ros_gz_image image_bridge "${image_topics[@]}" &
    pids+=("$!")
  fi
else
  echo "GZ<->ROS bridge disabled (bridge.enabled=false in ${SIM_CONFIG}); skipping parameter_bridge."
fi

if [[ "${tf_enabled}" == "true" ]]; then
  echo "Starting TF bridge (ros2 run lidar_bridge tf_bridge) ..."
  ros2 run lidar_bridge tf_bridge &
  pids+=("$!")
else
  echo "TF bridge disabled (bridge.tf_enabled=false in ${SIM_CONFIG}); skipping."
fi

echo "Bridge + TF running."
wait
