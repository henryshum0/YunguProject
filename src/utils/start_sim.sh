#!/usr/bin/env bash
#
# One-shot launcher for the Yungu simulation stack:
#   1. PX4 SITL + Gazebo (gz-sim)       make px4_sitl gz_x500_lidar_yungu
#   2. MicroXRCEAgent                   uXRCE-DDS agent on UDP 8888
#   3. GZ <-> ROS bridge + TF           src/utils/gz_bridges/bridge.sh
#
# PX4 SITL runs in its own xterm window (so you can watch its output and
# interact with it). When this script is interrupted (Ctrl+C), that window and
# the whole PX4/Gazebo process tree are terminated as well.
#
# Usage:
#   ./start_sim.sh                          # all three components
#   PX4_MODEL=gz_x500_lidar_yungu ./start_sim.sh
#   XRCE_PORT=8888 ./start_sim.sh
#   GZ_VERSION=harmonic ./start_sim.sh
#   HEADLESS=1 ./start_sim.sh               # run Gazebo without its GUI (server only)
#
# Press Ctrl+C to stop everything.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "${SCRIPT_DIR}/../.." && pwd)"
PX4_DIR="${WORKSPACE}/VisionFlow-PX4"
BRIDGE_SCRIPT="${SCRIPT_DIR}/gz_bridges/bridge.sh"

PX4_MODEL="${PX4_MODEL:-gz_x500_lidar_yungu}"
XRCE_PORT="${XRCE_PORT:-8888}"
GZ_VERSION="${GZ_VERSION:-harmonic}"

# Set HEADLESS to any non-empty value (e.g. HEADLESS=1) to start Gazebo without
# its GUI. PX4's gz-sim init script only launches the GUI when HEADLESS is empty,
# so with this set it runs just the physics server (useful for CI or machines
# without a display).
HEADLESS="${HEADLESS:-}"
export HEADLESS

LOG_DIR="/tmp/yungu_sim"
mkdir -p "${LOG_DIR}"

# ---------------------------------------------------------------------------
#  Sanity checks
# ---------------------------------------------------------------------------
[[ -d "${PX4_DIR}" ]] || { echo "ERROR: PX4 directory not found: ${PX4_DIR}" >&2; exit 1; }
[[ -f "${BRIDGE_SCRIPT}" ]] || { echo "ERROR: bridge script not found: ${BRIDGE_SCRIPT}" >&2; exit 1; }
command -v MicroXRCEAgent >/dev/null 2>&1 || { echo "ERROR: MicroXRCEAgent not found on PATH" >&2; exit 1; }

# Terminal emulator for the PX4 SITL window. If none is available, fall back
# to running it in the background (with its output still logged).
if command -v xterm >/dev/null 2>&1; then
  TERMINAL="xterm"
else
  TERMINAL=""
  echo "WARNING: 'xterm' not found - PX4 SITL will run in the background." >&2
fi

pids=()
cleaned=0

# Kill every process we started, including children that detached into their
# own process groups/sessions (e.g. the gz server, px4, MicroXRCEAgent).
cleanup() {
  [[ "${cleaned}" -eq 1 ]] && return
  cleaned=1
  echo
  echo "Stopping simulation stack ..."

  # 1. Graceful SIGTERM to the process groups we spawned with setsid.
  for p in "${pids[@]:-}"; do
    kill -- "-${p}" 2>/dev/null || kill "${p}" 2>/dev/null || true
  done

  # 2. Give them a moment to shut down, then SIGKILL anything still alive.
  sleep 3
  for p in "${pids[@]:-}"; do
    kill -9 -- "-${p}" 2>/dev/null || kill -9 "${p}" 2>/dev/null || true
  done

  # 3. Safety net: remove leftover simulation processes by name (covers
  #    children that forked into a new session, which group-kill would miss).
  pkill -9 -x px4                  2>/dev/null || true
  pkill -9 -f "gz sim"             2>/dev/null || true
  pkill -9 -x gz-server            2>/dev/null || true
  pkill -9 -x MicroXRCEAgent       2>/dev/null || true
  pkill -9 -f "parameter_bridge"   2>/dev/null || true
  pkill -9 -f "tf_bridge.py"       2>/dev/null || true

  echo "Simulation stack stopped."
}
trap cleanup EXIT INT TERM

# ---------------------------------------------------------------------------
#  1. PX4 SITL + Gazebo
# ---------------------------------------------------------------------------
if [[ -n "${HEADLESS}" ]]; then
  echo "Starting PX4 SITL + Gazebo (model: ${PX4_MODEL}, HEADLESS: Gazebo GUI disabled) ..."
else
  echo "Starting PX4 SITL + Gazebo (model: ${PX4_MODEL}) ..."
fi
if [[ -n "${TERMINAL}" ]]; then
  export PX4_DIR PX4_MODEL LOG_DIR HEADLESS
  setsid xterm -T "PX4 SITL (${PX4_MODEL})" -hold \
      -e bash -c 'cd "$PX4_DIR" && make px4_sitl "$PX4_MODEL" 2>&1 | tee "$LOG_DIR/px4_sitl.log"' &
  pids+=("$!")
else
  (cd "${PX4_DIR}" && make px4_sitl "${PX4_MODEL}") \
      >"${LOG_DIR}/px4_sitl.log" 2>&1 &
  pids+=("$!")
fi

echo "Waiting for PX4 SITL to come up ..."
for _ in $(seq 1 90); do
  if grep -qE "Ready for takeoff|INFO *\[commander\]" "${LOG_DIR}/px4_sitl.log" 2>/dev/null; then
    echo "PX4 SITL is up."
    break
  fi
  sleep 1
done
sleep 2

# ---------------------------------------------------------------------------
#  2. MicroXRCEAgent
# ---------------------------------------------------------------------------
echo "Starting MicroXRCEAgent (udp4, port ${XRCE_PORT}) ..."
setsid MicroXRCEAgent udp4 -p "${XRCE_PORT}" \
    >"${LOG_DIR}/xrce_agent.log" 2>&1 &
pids+=("$!")

sleep 2

# ---------------------------------------------------------------------------
#  3. GZ <-> ROS bridge + TF
# ---------------------------------------------------------------------------
echo "Starting GZ <-> ROS bridge ..."
GZ_VERSION="${GZ_VERSION}" setsid "${BRIDGE_SCRIPT}" \
    >"${LOG_DIR}/bridge.log" 2>&1 &
pids+=("$!")

echo
echo "Simulation stack running. Logs in ${LOG_DIR}:"
echo "  - PX4 SITL:    ${LOG_DIR}/px4_sitl.log  (in its own xterm window)"
echo "  - uXRCE agent: ${LOG_DIR}/xrce_agent.log"
echo "  - gz bridge:   ${LOG_DIR}/bridge.log"
echo
echo "Next: ros2 launch offboard offboard.launch.py"
echo "Press Ctrl+C to stop everything (also terminates the PX4 SITL window)."
wait || true
