#!/usr/bin/env bash
#
# test_fastlio_vs_truth.sh
#
# Compare FAST-LIO odometry against Gazebo ground truth.
# Runs: IMU bridge + FAST-LIO + data recording, then computes ATE/RPE.
#
# Prerequisites (already running):
#   1. PX4 SITL + Gazebo  (./temp/start_sim_gpu.sh)
#   2. uXRCE-DDS agent     (started by start_sim.sh)
#   3. GZ→ROS bridge       (started by start_sim.sh)
#
# Usage:
#   ./temp/test_fastlio_vs_truth.sh              # 60 s default
#   ./temp/test_fastlio_vs_truth.sh 120          # 120 seconds
#

# NOTE: no 'set -e' or 'set -u' — ROS 2 setup.bash often expands unset vars
set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "${SCRIPT_DIR}/.." && pwd)"
DURATION="${1:-60}"

LOG_DIR="/tmp/fastlio_test_$(date +%Y%m%d_%H%M%S)"
mkdir -p "${LOG_DIR}"

echo "=============================================="
echo " FAST-LIO vs Gazebo Truth — Comparison Test"
echo " Duration:    ${DURATION} s"
echo " Log dir:     ${LOG_DIR}"
echo "=============================================="
echo ""

# --- Sanity checks ----------------------------------------------------------
echo "[check] Looking for running processes ..."
if ! pgrep -f "px4" >/dev/null 2>&1; then
    echo "ERROR: PX4 SITL not running. Start it first:"
    echo "  ./temp/start_sim_gpu.sh"
    exit 1
fi
echo "        PX4 SITL — found"

if ! pgrep -f "MicroXRCEAgent" >/dev/null 2>&1; then
    echo "WARNING: MicroXRCEAgent not running. PX4-ROS bridge may be down."
fi

# --- Source workspace -------------------------------------------------------
echo "[check] Sourcing ROS 2 workspace ..."
if ! source "${WORKSPACE}/install/setup.bash" 2>/dev/null; then
    # Try system ROS 2 as fallback
    if [[ -f /opt/ros/humble/setup.bash ]]; then
        source /opt/ros/humble/setup.bash 2>/dev/null || true
        echo "        Using system ROS 2 (/opt/ros/humble)"
    fi
else
    echo "        Workspace sourced"
fi

# Verify critical tools are available
for cmd in python3 ros2; do
    if ! command -v "${cmd}" >/dev/null 2>&1; then
        echo "ERROR: '${cmd}' not found on PATH. Source ROS 2 first."
        exit 1
    fi
done
echo "        python3 + ros2 — OK"

# --- Kill all test nodes on exit --------------------------------------------
PIDS=()
cleanup() {
    echo ""
    echo "Stopping test nodes ..."
    for p in "${PIDS[@]:-}"; do
        kill "$p" 2>/dev/null || true
    done
    sleep 1
    echo "Test nodes stopped."
    echo ""
    echo "=============================================="
    echo " Data saved in: ${LOG_DIR}"
    echo ""
    echo " Next:"
    echo "   python3 temp/compare_odom.py ${LOG_DIR}"
    echo "=============================================="
}
trap cleanup EXIT INT TERM

# ============================================================================
#  1. IMU Bridge  (PX4 SensorCombined → sensor_msgs/Imu on /livox/imu)
# ============================================================================
echo "[1/4] Starting IMU bridge ..."
python3 "${SCRIPT_DIR}/gazebo_imu_bridge.py" &
PIDS+=("$!")
sleep 2
# Check it didn't die immediately
if ! kill -0 "${PIDS[-1]}" 2>/dev/null; then
    echo "ERROR: IMU bridge died immediately. Check:"
    echo "  - Is /fmu/out/sensor_combined being published?"
    echo "  - Is rclpy available?"
    exit 1
fi
echo "      IMU bridge PID=${PIDS[-1]} OK"

# ============================================================================
#  2. PointCloud time-field relay
# ============================================================================
echo "[2/5] Starting pointcloud time-field relay ..."
python3 "${SCRIPT_DIR}/add_time_field.py" &
PIDS+=("$!")
sleep 2
if ! kill -0 "${PIDS[-1]}" 2>/dev/null; then
    echo "ERROR: Time-field relay died."
    exit 1
fi
echo "      Relay PID=${PIDS[-1]} OK"

# ============================================================================
#  3. FAST-LIO
# ============================================================================
echo "[3/5] Starting FAST-LIO (Gazebo config) ..."
ros2 run fast_lio fastlio_mapping --ros-args \
    --params-file "${SCRIPT_DIR}/fastlio_gazebo.yaml" \
    -p use_sim_time:=true \
    > "${LOG_DIR}/fastlio.log" 2>&1 &
PIDS+=("$!")
sleep 4
if ! kill -0 "${PIDS[-1]}" 2>/dev/null; then
    echo "ERROR: FAST-LIO died immediately. Check fastlio.log:"
    tail -20 "${LOG_DIR}/fastlio.log"
    exit 1
fi
echo "      FAST-LIO PID=${PIDS[-1]} OK"

# ============================================================================
#  3. Record ground truth + FAST-LIO odometry
# ============================================================================
echo "[4/5] Recording topics for ${DURATION}s ..."
timeout "${DURATION}" ros2 bag record \
    -o "${LOG_DIR}/test_bag" \
    --max-cache-size 500000000 \
    /odom \
    /Odometry \
    /tf \
    /tf_static \
    > "${LOG_DIR}/bag_record.log" 2>&1 &
BAG_PID="$!"
PIDS+=("${BAG_PID}")

# --- Wait for recording to finish ------------------------------------------
echo ""
echo "=============================================="
echo " Recording — ${DURATION}s. Fly the drone NOW!"
echo ""
echo " In another terminal:"
echo "   ros2 topic pub --once /fmu/in/vehicle_command px4_msgs/msg/VehicleCommand \\"
echo "     '{target_system: 1, target_component: 1, source_system: 1, source_component: 1,"
echo "       from_external: true, timestamp: 0, command: 22, param1: 5.0, param2: 0.0}'"
echo "=============================================="
echo ""

# Countdown
for ((i = DURATION; i > 0; i -= 5)); do
    echo "  ... ${i}s remaining"
    sleep 5
done

wait "${BAG_PID}" 2>/dev/null || true

echo ""
echo "Recording complete."
