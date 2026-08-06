#!/usr/bin/env bash
#
# start_all_fastlio.sh — GPU sim + FAST-LIO + planner (FAST-LIO odom) + offboard + RViz
#
# Same as start_all.sh but feeds FAST-LIO /Odometry to the planner instead of Gazebo /odom.
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

LOG_DIR="/tmp/yungu_sim"
mkdir -p "${LOG_DIR}"

# --- GPU ------------------------------------------------------
unset LIBGL_ALWAYS_SOFTWARE
unset GZ_SIM_RENDER_ENGINE
unset MESA_D3D12_DEFAULT_ADAPTER_NAME
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA

echo "=== GPU Info ==="
command -v glxinfo &>/dev/null && glxinfo -B | grep -E 'OpenGL vendor|OpenGL renderer|OpenGL version' || true
echo "MESA_D3D12_DEFAULT_ADAPTER_NAME=${MESA_D3D12_DEFAULT_ADAPTER_NAME}"
echo "================="

# --- Sanity checks --------------------------------------------
[[ -d "${PX4_DIR}" ]] || { echo "ERROR: PX4 dir not found" >&2; exit 1; }
[[ -f "${BRIDGE_SCRIPT}" ]] || { echo "ERROR: bridge not found" >&2; exit 1; }
command -v MicroXRCEAgent >/dev/null 2>&1 || { echo "ERROR: MicroXRCEAgent not found" >&2; exit 1; }

if [[ "${NO_XTERM:-}" == "1" ]]; then
    TERMINAL=""
elif command -v xterm >/dev/null 2>&1; then
    TERMINAL="xterm"
else
    TERMINAL=""
fi

# --- Source ROS -----------------------------------------------
if [[ -f /opt/ros/humble/setup.bash ]]; then
    source /opt/ros/humble/setup.bash
fi
if [[ -f "${WORKSPACE}/install/setup.bash" ]]; then
    source "${WORKSPACE}/install/setup.bash"
fi

# --- Cleanup --------------------------------------------------
PIDS=()
cleaned=0
cleanup() {
    [[ "${cleaned}" -eq 1 ]] && return
    cleaned=1
    echo ""; echo "=== Stopping ==="
    for p in "${PIDS[@]:-}"; do kill -- "-${p}" 2>/dev/null || kill "${p}" 2>/dev/null || true; done
    sleep 2
    pkill -9 -x px4 2>/dev/null || true
    pkill -9 -f "gz sim" 2>/dev/null || true
    pkill -9 -x gz-server 2>/dev/null || true
    pkill -9 -x MicroXRCEAgent 2>/dev/null || true
    pkill -9 -f "parameter_bridge" 2>/dev/null || true
    pkill -9 -f "tf_bridge" 2>/dev/null || true
    pkill -9 -f "offboard_node" 2>/dev/null || true
    pkill -9 -f "fsm_node" 2>/dev/null || true
    pkill -9 -f "rviz2" 2>/dev/null || true
    pkill -9 -f "fastlio_mapping" 2>/dev/null || true
    pkill -9 -f "gazebo_imu_bridge" 2>/dev/null || true
    pkill -9 -f "add_time_field" 2>/dev/null || true
    echo "All stopped."
}
trap cleanup EXIT INT TERM

# ==============================================================
#  Phase 1 — PX4 SITL + Gazebo
# ==============================================================
echo ""
echo "Phase 1/3 — PX4 SITL + Gazebo"

echo "Starting PX4 SITL (${PX4_MODEL})..."
if [[ -n "${TERMINAL}" ]]; then
    export PX4_DIR PX4_MODEL LOG_DIR
    setsid xterm -T "PX4 SITL (${PX4_MODEL})" -hold \
        -e bash -c 'cd "$PX4_DIR" && make px4_sitl "$PX4_MODEL" 2>&1 | tee "$LOG_DIR/px4_sitl.log"' &
    PIDS+=("$!")
else
    (cd "${PX4_DIR}" && make px4_sitl "${PX4_MODEL}") >"${LOG_DIR}/px4_sitl.log" 2>&1 &
    PIDS+=("$!")
fi

echo "Waiting for PX4..."
for _ in $(seq 1 120); do
    if grep -qE "Ready for takeoff|INFO *\[commander\]" "${LOG_DIR}/px4_sitl.log" 2>/dev/null; then
        echo "PX4 ready."; break
    fi; sleep 2
done
sleep 2

echo "Starting MicroXRCEAgent..."
setsid MicroXRCEAgent udp4 -p "${XRCE_PORT}" >"${LOG_DIR}/xrce_agent.log" 2>&1 &
PIDS+=("$!"); sleep 2

echo "Starting GZ->ROS bridge (no TF)..."
# Start ros_gz_bridge directly, skip tf_bridge to avoid TF conflict with FAST-LIO
GZ_VERSION="${GZ_VERSION}" ros2 run ros_gz_bridge parameter_bridge --ros-args \
    -p config_file:="${WORKSPACE}/src/utils/gz_bridges/x500_lidar_bridge.yaml" \
    >"${LOG_DIR}/bridge.log" 2>&1 &
PIDS+=("$!"); sleep 3

for _ in $(seq 1 30); do
    if ros2 topic list 2>/dev/null | grep -q "/x500_lidar/scan/points"; then
        echo "Gazebo topics OK."; break
    fi; sleep 1
done

# ==============================================================
#  Phase 2 — FAST-LIO chain
# ==============================================================
echo ""
echo "Phase 2/3 — FAST-LIO chain"

# Use camera_init as the unified world frame (no TF to world needed)
# Static TFs: body = base_link, lidar is 0.16m above
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1 body base_link &
PIDS+=("$!")
ros2 run tf2_ros static_transform_publisher 0 0 0.16 0 0 0 1 base_link lidar_link &
PIDS+=("$!")
# Connect the two TF trees: world (gazebo) <-> camera_init (FAST-LIO)
# Both origins coincide (drone spawns at world origin, yaw aligned)
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1 world camera_init &
PIDS+=("$!")

python3 "${SCRIPT_DIR}/gazebo_imu_bridge.py" &
PIDS+=("$!"); sleep 2

python3 "${SCRIPT_DIR}/add_time_field.py" &
PIDS+=("$!"); sleep 2

ros2 run fast_lio fastlio_mapping --ros-args --params-file "${SCRIPT_DIR}/fastlio_gazebo.yaml" &
PIDS+=("$!"); sleep 5

# Cloud transform using Gazebo TRUTH odom — the world-frame cloud must be
# accurate (not FAST-LIO's noisy pose) so it does not drift/rotate with the drone.
python3 "${SCRIPT_DIR}/cloud_to_world.py" --ros-args \
    -p cloud_topic:=/x500_lidar/scan/points \
    -p odom_topic:=/odom \
    -p out_topic:=/x500_lidar/scan/points_world &
PIDS+=("$!"); sleep 2

echo "Waiting for FAST-LIO to initialize (this takes ~20s)..."
STABLE=0
for _ in $(seq 1 30); do
    # Check if /Odometry has data AND FAST-LIO is producing [mapping] output
    if ros2 topic echo /Odometry --once --qos-reliability reliable 2>/dev/null | grep -q "camera_init"; then
        STABLE=$((STABLE + 1))
        echo "FAST-LIO active (${STABLE}/3 stable checks)"
        if [ "${STABLE}" -ge 3 ]; then
            echo "FAST-LIO initialized and stable."
            break
        fi
    else
        STABLE=0
    fi
    sleep 3
done

if [ "${STABLE}" -lt 3 ]; then
    echo "WARNING: FAST-LIO may not be fully initialized. Proceeding anyway..."
fi

# Extra settling time for the map kdtree to build
echo "Letting FAST-LIO settle for 5s..."
sleep 5

# ==============================================================
#  Phase 3 — Offboard + Planner (with FAST-LIO odom) + RViz
# ==============================================================
echo ""
echo "Phase 3/3 — Offboard + Planner + RViz"

ros2 launch "${OFFBOARD_LAUNCH}" \
    planner_config:=fastlio_live.yaml \
    rviz_config:="${SCRIPT_DIR}/x500_fastlio.rviz" &
PIDS+=("$!")

echo ""
echo "======================================================"
echo " All systems running with FAST-LIO odometry."
echo "  planner odom: /Odometry (camera_init)"
echo ""
echo " In PX4 xterm: commander takeoff"
echo "                commander mode offboard"
echo " In RViz:      use '2D Goal Pose' to send goals"
echo " Ctrl+C to stop."
echo "======================================================"

wait || true
