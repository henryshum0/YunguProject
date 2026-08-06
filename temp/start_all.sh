#!/usr/bin/env bash
#
# start_all.sh — GPU-forced Yungu simulation + offboard + planner + RViz.
#
# Equivalent to running:
#   1. ./temp/start_sim_gpu.sh
#   2. ros2 launch offboard offboard.launch.py
#
# …but in one command with GPU enforcement and unified cleanup.
#
# Usage:
#   ./temp/start_all.sh
#   PX4_MODEL=gz_x500_lidar_yungu ./temp/start_all.sh
#   NO_RVIZ=1 ./temp/start_all.sh                     # skip RViz
#   NO_XTERM=1 ./temp/start_all.sh                    # PX4 in background (no popup)
#
# Press Ctrl+C to stop everything.
#

# NOTE: no 'set -e' or 'set -u' here — ROS 2 setup.bash and colcon scripts
# often reference unset variables and would trigger spurious exits.
set -o pipefail

# ---------------------------------------------------------------------------
#  Paths
# ---------------------------------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "${SCRIPT_DIR}/.." && pwd)"
PX4_DIR="${WORKSPACE}/VisionFlow-PX4"
BRIDGE_SCRIPT="${WORKSPACE}/src/utils/gz_bridges/bridge.sh"
OFFBOARD_LAUNCH="${WORKSPACE}/src/offboard/launch/offboard.launch.py"

PX4_MODEL="${PX4_MODEL:-gz_x500_lidar_yungu}"
XRCE_PORT="${XRCE_PORT:-8888}"
GZ_VERSION="${GZ_VERSION:-harmonic}"

LOG_DIR="/tmp/yungu_sim"
mkdir -p "${LOG_DIR}"

# ---------------------------------------------------------------------------
#  GPU enforcement
# ---------------------------------------------------------------------------
unset LIBGL_ALWAYS_SOFTWARE
unset GZ_SIM_RENDER_ENGINE
unset MESA_D3D12_DEFAULT_ADAPTER_NAME
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA

echo "=== GPU Info ==="
if command -v glxinfo &>/dev/null; then
    glxinfo -B | grep -E 'OpenGL vendor|OpenGL renderer|OpenGL version' || true
else
    echo "(glxinfo not available)"
fi
echo "MESA_D3D12_DEFAULT_ADAPTER_NAME=${MESA_D3D12_DEFAULT_ADAPTER_NAME}"
echo "================="

# ---------------------------------------------------------------------------
#  Sanity checks
# ---------------------------------------------------------------------------
[[ -d "${PX4_DIR}" ]] || {
    echo "ERROR: PX4 directory not found: ${PX4_DIR}" >&2; exit 1
}
[[ -f "${BRIDGE_SCRIPT}" ]] || {
    echo "ERROR: bridge script not found: ${BRIDGE_SCRIPT}" >&2; exit 1
}
command -v MicroXRCEAgent >/dev/null 2>&1 || {
    echo "ERROR: MicroXRCEAgent not found on PATH" >&2; exit 1
}

# Terminal emulator for PX4 SITL
if [[ "${NO_XTERM:-}" == "1" ]]; then
    TERMINAL=""
elif command -v xterm >/dev/null 2>&1; then
    TERMINAL="xterm"
else
    TERMINAL=""
    echo "WARNING: 'xterm' not found — PX4 runs in background" >&2
fi

# ---------------------------------------------------------------------------
#  Cleanup — kill everything on exit
# ---------------------------------------------------------------------------
PIDS=()
cleaned=0

cleanup() {
    [[ "${cleaned}" -eq 1 ]] && return
    cleaned=1
    echo ""
    echo "=== Stopping everything ... ==="

    # 1. Graceful SIGTERM to process groups
    for p in "${PIDS[@]:-}"; do
        kill -- "-${p}" 2>/dev/null || kill "${p}" 2>/dev/null || true
    done

    sleep 2

    # 2. SIGKILL stragglers
    for p in "${PIDS[@]:-}"; do
        kill -9 -- "-${p}" 2>/dev/null || kill -9 "${p}" 2>/dev/null || true
    done

    # 3. Safety net — kill by name
    pkill -9 -x px4 2>/dev/null || true
    pkill -9 -f "gz sim" 2>/dev/null || true
    pkill -9 -x gz-server 2>/dev/null || true
    pkill -9 -x MicroXRCEAgent 2>/dev/null || true
    pkill -9 -f "parameter_bridge" 2>/dev/null || true
    pkill -9 -f "tf_bridge.py" 2>/dev/null || true
    pkill -9 -f "offboard_node" 2>/dev/null || true
    pkill -9 -f "fsm_node" 2>/dev/null || true
    pkill -9 -f "rviz2" 2>/dev/null || true

    echo "All processes stopped."
}
trap cleanup EXIT INT TERM

# ===========================================================================
#  Phase 1 — PX4 SITL + Gazebo
# ===========================================================================
echo ""
echo "══════════════════════════════════════════════"
echo " Phase 1/2 — PX4 SITL + Gazebo"
echo "══════════════════════════════════════════════"
echo ""

# Source ROS 2 — MUST source system underlay FIRST, then workspace overlay.
# Otherwise bridge.sh sees 'ros2' from the workspace but can't find
# system-installed packages like ros_gz_bridge.
ROS2_SOURCED=0
if [[ -f /opt/ros/humble/setup.bash ]]; then
    source /opt/ros/humble/setup.bash && ROS2_SOURCED=1
fi
if [[ -f "${WORKSPACE}/install/setup.bash" ]]; then
    source "${WORKSPACE}/install/setup.bash" && ROS2_SOURCED=1
fi
if [[ "${ROS2_SOURCED}" -eq 0 ]]; then
    echo "ERROR: Could not source ROS 2. Check /opt/ros/humble/setup.bash" >&2
    exit 1
fi
echo "ROS 2 sourced OK ($(which ros2))"

echo "Starting PX4 SITL + Gazebo (model: ${PX4_MODEL}) ..."
if [[ -n "${TERMINAL}" ]]; then
    export PX4_DIR PX4_MODEL LOG_DIR
    setsid xterm -T "PX4 SITL (${PX4_MODEL})" -hold \
        -e bash -c 'cd "$PX4_DIR" && make px4_sitl "$PX4_MODEL" 2>&1 | tee "$LOG_DIR/px4_sitl.log"' &
    PIDS+=("$!")
else
    (cd "${PX4_DIR}" && make px4_sitl "${PX4_MODEL}") \
        >"${LOG_DIR}/px4_sitl.log" 2>&1 &
    PIDS+=("$!")
fi

echo "Waiting for PX4 SITL to come up ..."
for _ in $(seq 1 120); do
    if grep -qE "Ready for takeoff|INFO *\[commander\]" "${LOG_DIR}/px4_sitl.log" 2>/dev/null; then
        echo "PX4 SITL is up."
        break
    fi
    sleep 2
done
sleep 2

# ===========================================================================
#  Phase 1b — MicroXRCEAgent
# ===========================================================================
echo "Starting MicroXRCEAgent (udp4, port ${XRCE_PORT}) ..."
setsid MicroXRCEAgent udp4 -p "${XRCE_PORT}" \
    >"${LOG_DIR}/xrce_agent.log" 2>&1 &
PIDS+=("$!")
sleep 2

# ===========================================================================
#  Phase 1c — GZ <-> ROS bridge + TF
# ===========================================================================
echo "Starting GZ <-> ROS bridge ..."
GZ_VERSION="${GZ_VERSION}" setsid "${BRIDGE_SCRIPT}" \
    >"${LOG_DIR}/bridge.log" 2>&1 &
PIDS+=("$!")
sleep 3

# Wait for key topics to appear
echo "Waiting for ROS topics ..."
for _ in $(seq 1 30); do
    if ros2 topic list 2>/dev/null | grep -q "/x500_lidar/scan/points"; then
        echo "Gazebo topics available."
        break
    fi
    sleep 1
done

# ===========================================================================
#  Phase 1d — Point cloud → world transform (for ROG-Map)
# ===========================================================================
echo "Starting cloud_to_world (lidar_link → world) ..."
python3 "${SCRIPT_DIR}/cloud_to_world.py" \
    >"${LOG_DIR}/cloud_to_world.log" 2>&1 &
PIDS+=("$!")
sleep 2

# ===========================================================================
#  Phase 2 — Offboard + Planner + RViz
# ===========================================================================
echo ""
echo "══════════════════════════════════════════════"
echo " Phase 2/2 — Offboard + Planner + RViz"
echo "══════════════════════════════════════════════"
echo ""

if [[ "${NO_RVIZ:-}" == "1" ]]; then
    echo "Launching offboard + planner (no RViz) ..."
    ros2 launch "${OFFBOARD_LAUNCH}" rviz:=false &
    PIDS+=("$!")
else
    echo "Launching offboard + planner + RViz ..."
    ros2 launch "${OFFBOARD_LAUNCH}" &
    PIDS+=("$!")
fi

echo ""
echo "══════════════════════════════════════════════"
echo " All systems running."
echo ""
echo " Logs: ${LOG_DIR}/"
echo "  - px4_sitl.log   — PX4 (also in xterm)"
echo "  - xrce_agent.log — uXRCE-DDS"
echo "  - bridge.log     — GZ→ROS bridge + TF"
echo ""
echo " In PX4 xterm:  commander takeoff"
echo " In RViz:       use '2D Goal Pose' to send goals"
echo ""
echo " Press Ctrl+C to stop everything."
echo "══════════════════════════════════════════════"
echo ""

# Wait forever — cleanup runs on Ctrl+C
wait || true
