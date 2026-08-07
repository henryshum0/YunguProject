#!/usr/bin/env bash
#
# start_sim_gpu.sh — GPU-forced sim + control chain + RViz (one-shot).
#
# Wraps src/utils/start_sim.sh with:
#   - NVIDIA GPU enforcement (WSL MESA_D3D12)
#   - after sim is up: offboard + SUPER planner + super_bridge + RViz
#
# Usage:
#   ./temp/start_sim_gpu.sh                          # full stack with GUI
#   HEADLESS=1 ./temp/start_sim_gpu.sh               # Gazebo server-only (RViz still opens)
#   NO_RVIZ=1 ./temp/start_sim_gpu.sh                # skip RViz
#   PLANNER_CONFIG=fastlio_live.yaml ./temp/start_sim_gpu.sh  # custom planner config
#
# Press Ctrl+C to stop everything.
#

set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "${SCRIPT_DIR}/.." && pwd)"
OFFBOARD_LAUNCH="${WORKSPACE}/src/offboard/launch/offboard.launch.py"

# Default: main's gazebo.yaml (super_bridge → /lidar_slam/odom + /cloud_registered)
PLANNER_CONFIG="${PLANNER_CONFIG:-gazebo.yaml}"

# --- GPU enforcement (WSL: force NVIDIA via MESA_D3D12) ---------------------
unset LIBGL_ALWAYS_SOFTWARE
unset GZ_SIM_RENDER_ENGINE
unset MESA_D3D12_DEFAULT_ADAPTER_NAME
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA

echo "=== GPU Info ==="
command -v glxinfo &>/dev/null && glxinfo -B | grep -E 'OpenGL vendor|OpenGL renderer|OpenGL version' || true
echo "MESA_D3D12_DEFAULT_ADAPTER_NAME=${MESA_D3D12_DEFAULT_ADAPTER_NAME}"
echo "================="

# ---------------------------------------------------------------------------
#  Source ROS (system underlay first, then workspace overlay)
# ---------------------------------------------------------------------------
if [[ -f /opt/ros/humble/setup.bash ]]; then
    source /opt/ros/humble/setup.bash
fi
if [[ -f "${WORKSPACE}/install/setup.bash" ]]; then
    source "${WORKSPACE}/install/setup.bash"
fi

# ---------------------------------------------------------------------------
#  Phase 1 — Simulator (PX4 + Gazebo + uXRCE + bridge), GPU + HEADLESS aware
# ---------------------------------------------------------------------------
echo ""
echo "Phase 1/2 — Simulator (PX4 + Gazebo + uXRCE + bridge)"
echo ""

"${WORKSPACE}/src/utils/start_sim.sh" &
SIM_PID=$!

# Wait for the sim stack to be up (start_sim.sh blocks until Ctrl+C, so we
# poll for the key topic instead).
echo "Waiting for PX4 + Gazebo topics (up to 90s)..."
for _ in $(seq 1 90); do
    if ros2 topic list 2>/dev/null | grep -q "/x500_lidar/scan/points"; then
        echo "Simulator topics available."
        break
    fi
    sleep 1
done
sleep 5

# ---------------------------------------------------------------------------
#  Phase 2 — Offboard + planner (super_bridge) + RViz
# ---------------------------------------------------------------------------
echo ""
echo "Phase 2/2 — Offboard + planner + RViz"
echo ""

if [[ "${NO_RVIZ:-}" == "1" ]]; then
    ros2 launch "${OFFBOARD_LAUNCH}" planner_config:="${PLANNER_CONFIG}" rviz:=false &
else
    ros2 launch "${OFFBOARD_LAUNCH}" planner_config:="${PLANNER_CONFIG}" &
fi
CTRL_PID=$!

echo ""
echo "======================================================"
echo " All systems running."
echo "  planner config: ${PLANNER_CONFIG}"
echo "  (gazebo.yaml → PX4 fused odom via super_bridge)"
echo ""
echo " In PX4 xterm: commander takeoff"
echo "                commander mode offboard"
echo " In RViz:      use '2D Goal Pose' to send goals"
echo " Ctrl+C to stop."
echo "======================================================"

# Wait for either process to exit; then clean up both
wait "${SIM_PID}" 2>/dev/null
kill "${CTRL_PID}" 2>/dev/null
