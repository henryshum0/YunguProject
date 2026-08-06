#!/usr/bin/env bash
# start_fastlio_checked.sh — Start FAST-LIO chain with data-ready checks.
# Each node waits for its upstream data to actually flow before proceeding.

set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "${SCRIPT_DIR}/.." && pwd)"

# Preserve ROS environment from the calling terminal
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

source /opt/ros/humble/setup.bash 2>/dev/null || true
source "${WORKSPACE}/install/setup.bash" 2>/dev/null || true

PIDS=()
cleanup() {
    for p in "${PIDS[@]:-}"; do kill "$p" 2>/dev/null || true; done
    echo "Stopped."
}
trap cleanup EXIT INT TERM

# Helper: wait until a topic has a publisher (max N s)
wait_publisher() {
    local topic="$1" max="${2:-30}"
    local i
    for i in $(seq 1 "${max}"); do
        local out
        out=$(ros2 topic info "${topic}" 2>&1)
        if echo "${out}" | grep -q "Publisher count: [1-9]"; then
            return 0
        fi
        # Debug: print first check result only
        if [ "${i}" -eq 1 ]; then
            echo "    [debug] 'ros2 topic info ${topic}' output: ${out}" >&2
        fi
        sleep 1
    done
    return 1
}

# Helper: wait until a topic actually has flowing data (max N s)
# QoS: try default first, then best_effort (PX4 topics are best_effort)
wait_data() {
    local topic="$1" max="${2:-30}"
    for _ in $(seq 1 "${max}"); do
        if timeout 3 ros2 topic echo "${topic}" --once >/dev/null 2>&1; then
            return 0
        fi
        if timeout 3 ros2 topic echo "${topic}" --once --qos-reliability best_effort >/dev/null 2>&1; then
            return 0
        fi
        sleep 1
    done
    return 1
}

echo "=== 0. Pre-flight checks ==="

# Sim must be running (raw lidar topic)
if ! wait_publisher "/x500_lidar/scan/points" 30; then
    echo "ERROR: /x500_lidar/scan/points has no publisher. Start simulation first." >&2
    exit 1
fi
echo "  [OK] /x500_lidar/scan/points publishing"

echo "  Waiting for /fmu/out/sensor_combined (PX4 needs time to start publishing, up to 90s)..."
if ! wait_publisher "/fmu/out/sensor_combined" 90; then
    echo "ERROR: /fmu/out/sensor_combined has no publisher after 90s. PX4/uXRCE down?" >&2
    exit 1
fi
echo "  [OK] /fmu/out/sensor_combined publishing"

# Confirm data actually flows
if ! wait_data "/x500_lidar/scan/points" 15; then
    echo "ERROR: /x500_lidar/scan/points has publisher but NO data flowing" >&2
    exit 1
fi
echo "  [OK] /x500_lidar/scan/points data flowing"

if ! wait_data "/fmu/out/sensor_combined" 15; then
    echo "ERROR: /fmu/out/sensor_combined has publisher but NO data flowing" >&2
    exit 1
fi
echo "  [OK] /fmu/out/sensor_combined data flowing"

echo ""
echo "=== 1. IMU bridge ==="
python3 "${SCRIPT_DIR}/gazebo_imu_bridge.py" &
PIDS+=("$!")
if ! wait_data "/livox/imu" 15; then
    echo "ERROR: IMU bridge not producing /livox/imu" >&2
    exit 1
fi
echo "  [OK] /livox/imu data flowing"

echo ""
echo "=== 2. PointCloud relay ==="
python3 "${SCRIPT_DIR}/add_time_field.py" &
PIDS+=("$!")
if ! wait_data "/x500_lidar/scan/points_timed" 15; then
    echo "ERROR: Relay not producing /x500_lidar/scan/points_timed" >&2
    exit 1
fi
echo "  [OK] /x500_lidar/scan/points_timed data flowing"

echo ""
echo "=== 3. FAST-LIO ==="
ros2 run fast_lio fastlio_mapping --ros-args --params-file "${SCRIPT_DIR}/fastlio_gazebo.yaml" &
PIDS+=("$!")

# Wait for odometry to actually publish (FAST-LIO needs ~15-30s to init)
echo "  Waiting for /Odometry (up to 90s)..."
for i in $(seq 1 90); do
    if timeout 3 ros2 topic echo /Odometry --once >/dev/null 2>&1; then
        echo "  [OK] /Odometry publishing after ${i}s"
        echo ""
        echo "=============================================="
        echo " FAST-LIO chain READY."
        echo " /Odometry is publishing in camera_init frame."
        echo " Now run RViz (temp/x500_fastlio.rviz) and fly."
        echo "=============================================="
        # Keep running until Ctrl+C
        wait "${PIDS[2]}" 2>/dev/null || true
        exit 0
    fi
    # Show FAST-LIO status every 15s
    if [ $((i % 15)) -eq 0 ]; then
        echo "  ... still waiting (${i}s). FAST-LIO log tail:"
        tail -2 "${LOG_DIR:-/tmp}/fastlio.log" 2>/dev/null || true
    fi
    sleep 1
done

echo "ERROR: /Odometry never appeared within 90s. FAST-LIO stuck." >&2
exit 1
