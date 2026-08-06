#!/usr/bin/env bash
# Minimal FAST-LIO debug: just IMU bridge + relay + FAST-LIO, no recording.
# Run while start_sim_gpu.sh is running in another terminal.
set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "=== FAST-LIO Debug ==="

# Source ROS
source /opt/ros/humble/setup.bash 2>/dev/null || true
source "${SCRIPT_DIR}/../install/setup.bash" 2>/dev/null || true

PIDS=()
cleanup() { for p in "${PIDS[@]:-}"; do kill "$p" 2>/dev/null || true; done; echo "Stopped."; }
trap cleanup EXIT INT TERM

echo "[1] IMU bridge..."
python3 "${SCRIPT_DIR}/gazebo_imu_bridge.py" --ros-args -p use_sim_time:=true &
PIDS+=("$!"); sleep 2

echo "[2] PointCloud relay..."
python3 "${SCRIPT_DIR}/add_time_field.py" --ros-args -p use_sim_time:=true &
PIDS+=("$!"); sleep 2

echo "[3] FAST-LIO..."
ros2 run fast_lio fastlio_mapping --ros-args \
    --params-file "${SCRIPT_DIR}/fastlio_gazebo.yaml" \
    -p use_sim_time:=true &
PIDS+=("$!"); sleep 5

echo ""
echo "=== Waiting 15s for FAST-LIO to initialize, then checking /Odometry ==="
sleep 15

# Check
echo ""
echo "--- FAST-LIO log tail ---"
tail -10 /tmp/yungu_sim/px4_sitl.log 2>/dev/null || true
echo ""
echo "--- /Odometry check ---"
ros2 topic echo /Odometry --once --timeout 5 2>&1 || echo "(no data after 5s)"
echo ""
echo "--- /Odometry info ---"
ros2 topic info /Odometry 2>&1 || true
