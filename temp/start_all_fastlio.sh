#!/usr/bin/env bash
#
# start_all_fastlio.sh — GPU sim + FAST-LIO + PX4-EKF2 fusion + planner + offboard + RViz
#
# One-shot launcher aligned with src/utils/start_sim.sh (HEADLESS support etc.)
# plus the FAST-LIO chain:
#   - FAST-LIO odometry → PX4 EKF2 (fastlio_px4_bridge → /fmu/in/vehicle_visual_odometry)
#   - PX4 fused odometry → super_bridge → /lidar_slam/odom + /cloud_registered → planner
#
# Usage:
#   ./temp/start_all_fastlio.sh                 # full stack with GUI
#   HEADLESS=1 ./temp/start_all_fastlio.sh      # no Gazebo GUI
#   NO_RVIZ=1 ./temp/start_all_fastlio.sh       # skip RViz
#
# Press Ctrl+C to stop everything.
#

set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "${SCRIPT_DIR}/.." && pwd)"
PX4_DIR="${WORKSPACE}/VisionFlow-PX4"
BRIDGE_SCRIPT="${WORKSPACE}/src/utils/gz_bridges/bridge.sh"
OFFBOARD_LAUNCH="${WORKSPACE}/src/offboard/launch/offboard.launch.py"

PX4_MODEL="${PX4_MODEL:-gz_x500_lidar_yungu}"
XRCE_PORT="${XRCE_PORT:-8888}"
GZ_VERSION="${GZ_VERSION:-harmonic}"

# HEADLESS: non-empty → Gazebo runs server-only (no GUI). Mirrors start_sim.sh.
HEADLESS="${HEADLESS:-}"
export HEADLESS

LOG_DIR="/tmp/yungu_sim"
mkdir -p "${LOG_DIR}"

# ---------------------------------------------------------------------------
#  GPU enforcement (WSL: force NVIDIA via MESA_D3D12)
# ---------------------------------------------------------------------------
unset LIBGL_ALWAYS_SOFTWARE
unset GZ_SIM_RENDER_ENGINE
unset MESA_D3D12_DEFAULT_ADAPTER_NAME
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA

echo "=== GPU Info ==="
command -v glxinfo &>/dev/null && glxinfo -B | grep -E 'OpenGL vendor|OpenGL renderer|OpenGL version' || true
echo "MESA_D3D12_DEFAULT_ADAPTER_NAME=${MESA_D3D12_DEFAULT_ADAPTER_NAME}"
echo "================="

# ---------------------------------------------------------------------------
#  Sanity checks
# ---------------------------------------------------------------------------
[[ -d "${PX4_DIR}" ]] || { echo "ERROR: PX4 directory not found: ${PX4_DIR}" >&2; exit 1; }
[[ -f "${BRIDGE_SCRIPT}" ]] || { echo "ERROR: bridge script not found: ${BRIDGE_SCRIPT}" >&2; exit 1; }
command -v MicroXRCEAgent >/dev/null 2>&1 || { echo "ERROR: MicroXRCEAgent not found on PATH" >&2; exit 1; }

# Terminal emulator for PX4 SITL window
if [[ "${NO_XTERM:-}" == "1" ]]; then
    TERMINAL=""
elif command -v xterm >/dev/null 2>&1; then
    TERMINAL="xterm"
else
    TERMINAL=""
    echo "WARNING: 'xterm' not found — PX4 runs in background" >&2
fi

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
#  Cleanup — kill everything on exit
# ---------------------------------------------------------------------------
PIDS=()
cleaned=0
cleanup() {
    [[ "${cleaned}" -eq 1 ]] && return
    cleaned=1
    echo ""; echo "=== Stopping everything ... ==="

    # 1. Graceful SIGTERM to process groups
    for p in "${PIDS[@]:-}"; do kill -- "-${p}" 2>/dev/null || kill "${p}" 2>/dev/null || true; done
    sleep 2

    # 2. SIGKILL stragglers
    for p in "${PIDS[@]:-}"; do kill -9 -- "-${p}" 2>/dev/null || kill -9 "${p}" 2>/dev/null || true; done

    # 3. Safety net — kill by name
    pkill -9 -x px4 2>/dev/null || true
    pkill -9 -f "gz sim" 2>/dev/null || true
    pkill -9 -x gz-server 2>/dev/null || true
    pkill -9 -x MicroXRCEAgent 2>/dev/null || true
    pkill -9 -f "parameter_bridge" 2>/dev/null || true
    pkill -9 -f "tf_bridge.py" 2>/dev/null || true
    pkill -9 -f "offboard_node" 2>/dev/null || true
    pkill -9 -f "super_bridge" 2>/dev/null || true
    pkill -9 -f "fsm_node" 2>/dev/null || true
    pkill -9 -f "rviz2" 2>/dev/null || true
    pkill -9 -f "fastlio_mapping" 2>/dev/null || true
    pkill -9 -f "fastlio_px4_bridge" 2>/dev/null || true
    pkill -9 -f "gazebo_imu_bridge" 2>/dev/null || true
    pkill -9 -f "add_time_field" 2>/dev/null || true
    pkill -9 -f "cloud_to_world" 2>/dev/null || true
    pkill -9 -f "static_transform_publisher" 2>/dev/null || true
    echo "All stopped."
}
trap cleanup EXIT INT TERM

# ===========================================================================
#  Phase 1 — PX4 SITL + Gazebo (GPU + HEADLESS-aware, mirrors start_sim.sh)
# ===========================================================================
echo ""
echo "Phase 1/3 — PX4 SITL + Gazebo"

if [[ -n "${HEADLESS}" ]]; then
    echo "Starting PX4 SITL + Gazebo (model: ${PX4_MODEL}, HEADLESS: no GUI) ..."
else
    echo "Starting PX4 SITL + Gazebo (model: ${PX4_MODEL}) ..."
fi

if [[ -n "${TERMINAL}" ]]; then
    export PX4_DIR PX4_MODEL LOG_DIR HEADLESS
    setsid xterm -T "PX4 SITL (${PX4_MODEL})" -hold \
        -e bash -c 'cd "$PX4_DIR" && make px4_sitl "$PX4_MODEL" 2>&1 | tee "$LOG_DIR/px4_sitl.log"' &
    PIDS+=("$!")
else
    (cd "${PX4_DIR}" && make px4_sitl "${PX4_MODEL}") >"${LOG_DIR}/px4_sitl.log" 2>&1 &
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
setsid MicroXRCEAgent udp4 -p "${XRCE_PORT}" >"${LOG_DIR}/xrce_agent.log" 2>&1 &
PIDS+=("$!")
sleep 2

# ===========================================================================
#  Phase 1c — GZ <-> ROS bridge + TF (use the standard bridge.sh)
# ===========================================================================
echo "Starting GZ <-> ROS bridge + TF ..."
GZ_VERSION="${GZ_VERSION}" setsid "${BRIDGE_SCRIPT}" >"${LOG_DIR}/bridge.log" 2>&1 &
PIDS+=("$!")
sleep 3

# Wait for key topics
for _ in $(seq 1 30); do
    if ros2 topic list 2>/dev/null | grep -q "/x500_lidar/scan/points"; then
        echo "Gazebo topics available."
        break
    fi
    sleep 1
done

# ===========================================================================
#  Phase 2 — FAST-LIO chain + PX4 EKF2 fusion
# ===========================================================================
echo ""
echo "Phase 2/3 — FAST-LIO chain + EKF2 fusion"

# Static TFs (no conflict with FAST-LIO's camera_init → body)
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1 body base_link &
PIDS+=("$!")
ros2 run tf2_ros static_transform_publisher 0 0 0.16 0 0 0 1 base_link lidar_link &
PIDS+=("$!")
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1 world camera_init &
PIDS+=("$!")

python3 "${SCRIPT_DIR}/gazebo_imu_bridge.py" &
PIDS+=("$!"); sleep 2

python3 "${SCRIPT_DIR}/add_time_field.py" &
PIDS+=("$!"); sleep 2

# Wait for both sensor streams before starting FAST-LIO: if it comes up
# before the LiDAR/IMU data is stable, it can crash on a regressing
# timestamp ("cannot store a negative time point").
echo "Waiting for LiDAR + IMU streams..."
for _ in $(seq 1 30); do
    if ros2 topic info /x500_lidar/scan/points_timed 2>/dev/null | grep -q "Publisher count: 1" &&
       ros2 topic info /livox/imu 2>/dev/null | grep -q "Publisher count: 1"; then
        echo "Sensor streams ready."
        break
    fi
    sleep 1
done
sleep 2

ros2 run fast_lio fastlio_mapping --ros-args --params-file "${SCRIPT_DIR}/fastlio_gazebo.yaml" \
    >"${LOG_DIR}/fastlio.log" 2>&1 &
PIDS+=("$!"); sleep 5

# FAST-LIO odometry → PX4 EKF2 external vision
python3 "${SCRIPT_DIR}/fastlio_px4_bridge.py" &
PIDS+=("$!"); sleep 2

# Ground-truth trajectory for RViz (truth vs FAST-LIO path comparison)
python3 "${SCRIPT_DIR}/gt_path_node.py" &
PIDS+=("$!"); sleep 1

# NOTE: no cloud_to_world here — super_bridge (in offboard.launch.py) reads the
# PX4 fused vehicle_odometry and transforms the raw lidar cloud into the world
# frame itself (→ /cloud_registered). This mirrors the real-hardware setup
# where there is no Gazebo ground-truth odom.

echo "Waiting for FAST-LIO to initialize (up to 60s)..."
STABLE=0
for _ in $(seq 1 20); do
    if ros2 topic echo /Odometry --once --qos-reliability reliable 2>/dev/null | grep -q "camera_init"; then
        STABLE=$((STABLE + 1))
        echo "FAST-LIO active (${STABLE}/3)"
        [ "${STABLE}" -ge 3 ] && break
    else
        STABLE=0
    fi
    sleep 3
done
if [ "${STABLE}" -lt 3 ]; then
    echo "WARNING: FAST-LIO may not be fully initialized. Proceeding anyway..."
fi
sleep 5

# ===========================================================================
#  Phase 3 — Offboard + Planner + RViz (main's super_bridge architecture)
# ===========================================================================
echo ""
echo "Phase 3/3 — Offboard + Planner + RViz"

# Use main's gazebo.yaml: planner consumes PX4-fused odom via super_bridge
# (/lidar_slam/odom + /cloud_registered) — mirrors real hardware (no Gazebo truth).
if [[ "${NO_RVIZ:-}" == "1" ]]; then
    ros2 launch "${OFFBOARD_LAUNCH}" rviz:=false &
else
    ros2 launch "${OFFBOARD_LAUNCH}" rviz_config:="${SCRIPT_DIR}/x500_fastlio.rviz" &
fi
PIDS+=("$!")

echo ""
echo "======================================================"
echo " All systems running."
echo "  FAST-LIO /Odometry → PX4 EKF2 fusion"
echo "  PX4 fused odom → super_bridge → /lidar_slam/odom → planner"
echo ""
echo " In PX4 xterm: commander takeoff"
echo "                commander mode offboard"
echo " In RViz:      use '2D Goal Pose' to send goals"
echo " Ctrl+C to stop."
echo "======================================================"

wait || true
