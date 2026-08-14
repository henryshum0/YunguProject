#!/usr/bin/env bash
#
# start_fastlio.sh — GPU sim + FAST-LIO + PX4-EKF2 fusion + planner + offboard + RViz
#
# One-shot launcher aligned with utils/start_sim.sh (HEADLESS support etc.)
# plus the FAST-LIO chain:
#   - FAST-LIO odometry → PX4 EKF2 (fastlio_px4_bridge → /fmu/in/vehicle_visual_odometry)
#   - PX4 fused odometry → super_bridge → /lidar_slam/odom + /cloud_registered → planner
#   - RViz shows the ikd-Tree *incremental* map (effect_map_en) on top of the
#     accumulated map — old regions fade as the drone moves away.
#
# Usage:
#   ./utils/start_fastlio.sh              # full stack with GUI
#   HEADLESS=1 ./utils/start_fastlio.sh   # no Gazebo GUI
#   NO_RVIZ=1 ./utils/start_fastlio.sh    # skip RViz
#
# Model & lidar topics follow config/simulation.yaml (model / world). The
# FAST-LIO config ships for swan_gamma_v2; for other models, override
# FASTLIO_CONFIG. Other overrides:
#   SIM_MODEL=swan_gamma_v2 SIM_WORLD=yungu ./utils/start_fastlio.sh
#   PX4_MODEL=gz_swan_gamma_v2_yungu ./utils/start_fastlio.sh  # full target
#   LIDAR_TOPIC=/swan_gamma_v2/scan ./utils/start_fastlio.sh
#   FASTLIO_CONFIG=config/fastlio_swan_gamma_effect.yaml ./utils/start_fastlio.sh
#   RVIZ_CONFIG=fastlio_ikdtree.rviz ./utils/start_fastlio.sh
#   BIRDVIEW_CONFIG=birdview.rviz ./utils/start_fastlio.sh  # top-down window config
#
# Two RViz windows (same layout as main's start_sim.sh + offboard.launch.py):
#   - top-down birdview planning window (BIRDVIEW_CONFIG, default birdview.rviz)
#   - 3D window (RVIZ_CONFIG: accumulated map + ikd-Tree incremental map)
#
# Press Ctrl+C to stop everything.
#

set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "${SCRIPT_DIR}/.." && pwd)"
PX4_DIR="${WORKSPACE}/VisionFlow-PX4"
BRIDGE_SCRIPT="${WORKSPACE}/utils/gz_bridges/bridge.sh"
OFFBOARD_LAUNCH="${WORKSPACE}/src/offboard/launch/offboard.launch.py"
SIM_CONFIG="${WORKSPACE}/config/simulation.yaml"
SIM_CONFIG_HELPER="${WORKSPACE}/config/sim_config.py"

# Model / world / lidar topics — default from config/simulation.yaml.
# gz-sim topic convention here: /<model>/scan and /<model>/scan/points.
config_get() {
    python3 "${SIM_CONFIG_HELPER}" --config "${SIM_CONFIG}" get "$1" 2>/dev/null || true
}
SIM_MODEL="${SIM_MODEL:-$(config_get model)}"
SIM_MODEL="${SIM_MODEL:-swan_gamma_v2}"
SIM_WORLD="${SIM_WORLD:-$(config_get world)}"
SIM_WORLD="${SIM_WORLD:-yungu}"
PX4_MODEL="${PX4_MODEL:-gz_${SIM_MODEL}_${SIM_WORLD}}"
LIDAR_TOPIC="${LIDAR_TOPIC:-/${SIM_MODEL}/scan}"
LIDAR_POINTS_TOPIC="${LIDAR_POINTS_TOPIC:-${LIDAR_TOPIC}/points}"
LIDAR_TIMED_TOPIC="${LIDAR_TIMED_TOPIC:-${LIDAR_POINTS_TOPIC}_timed}"
# ikd-Tree incremental-map config (effect_map_en: true → /cloud_effected,
# map_en: true → /Laser_map still published). The yaml hardcodes
# lid_topic: /swan_gamma_v2/scan/points_timed — for another model, override
# FASTLIO_CONFIG with a matching config.
FASTLIO_CONFIG="${FASTLIO_CONFIG:-${WORKSPACE}/config/fastlio_swan_gamma_effect.yaml}"
# Bare file name → resolved via the offboard package share (same as main's
# start_sim.sh + offboard.launch.py flow); needs a colcon build to install.
RVIZ_CONFIG="${RVIZ_CONFIG:-fastlio_ikdtree.rviz}"
BIRDVIEW_CONFIG="${BIRDVIEW_CONFIG:-birdview.rviz}"

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

# Wait for key topics — one blocking "echo --once" instead of a poll loop:
# each `ros2 topic list/info` spawn is a fresh DDS discovery round-trip,
# which is slow on WSL2 where multicast discovery is broken.
if ! timeout 30 ros2 topic echo "${LIDAR_POINTS_TOPIC}" --once --qos-reliability best-effort >/dev/null 2>&1; then
    echo "WARNING: ${LIDAR_POINTS_TOPIC} not seen within 30s — continuing anyway" >&2
fi
echo "Gazebo topics available."

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

# world → camera_init: FAST-LIO's camera_init is the first lidar frame's
# position, i.e. the model spawn pose (PX4_GZ_MODEL_POSE, e.g. swan spawns at
# "0,0,1.15392,0,0,0"). Publishing the real spawn offset keeps FAST-LIO's path
# (camera_init frame) aligned with the Gazebo truth path (/gt_path, world
# frame) in RViz — otherwise the red/blue lines are offset by the spawn xy
# (and z by spawn height + 0.16 m lidar offset).
AIRFRAME_FILE="$(ls "${PX4_DIR}"/ROMFS/px4fmu_common/init.d-posix/airframes/*_gz_${SIM_MODEL} 2>/dev/null | head -1)"
SPAWN_POSE="${PX4_GZ_MODEL_POSE:-}"
if [[ -z "${SPAWN_POSE}" && -n "${AIRFRAME_FILE}" ]]; then
    SPAWN_POSE="$(grep -oE 'PX4_GZ_MODEL_POSE=.*"[-0-9.,]+"' "${AIRFRAME_FILE}" \
        | grep -oE '"[-0-9.,]+"' | tr -d '"' | head -1)"
fi
if [[ -n "${SPAWN_POSE}" ]]; then
    IFS=',' read -r SP_X SP_Y SP_Z _ <<< "${SPAWN_POSE}"
    SP_Z_LIDAR="$(awk "BEGIN{print ${SP_Z}+0.16}")"
    echo "Spawn pose from airframe: x=${SP_X} y=${SP_Y} z=${SP_Z} (world→camera_init z=${SP_Z_LIDAR})"
    ros2 run tf2_ros static_transform_publisher ${SP_X} ${SP_Y} ${SP_Z_LIDAR} 0 0 0 1 world camera_init &
else
    echo "WARNING: no PX4_GZ_MODEL_POSE found — world→camera_init stays identity"
    ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1 world camera_init &
fi
PIDS+=("$!")

# Sensor nodes start in parallel (no serial sleeps); the readiness gate below
# blocks until their data actually flows.
python3 "${SCRIPT_DIR}/fastlio/gazebo_imu_bridge.py" --ros-args -p lidar_topic:="${LIDAR_POINTS_TOPIC}" &
PIDS+=("$!")
python3 "${SCRIPT_DIR}/fastlio/add_time_field.py" --ros-args \
    -p input_topic:="${LIDAR_POINTS_TOPIC}" -p output_topic:="${LIDAR_TIMED_TOPIC}" &
PIDS+=("$!")
# Ground-truth trajectory for RViz (truth vs FAST-LIO path comparison)
python3 "${SCRIPT_DIR}/fastlio/gt_path_node.py" &
PIDS+=("$!")

# Wait for both sensor streams before starting FAST-LIO: if it comes up
# before the LiDAR/IMU data is stable, it can crash on a regressing
# timestamp ("cannot store a negative time point"). Two blocking waits run
# in parallel — "echo --once" exits on the first message, timeout bounds it.
echo "Waiting for LiDAR + IMU streams (${LIDAR_TIMED_TOPIC})..."
timeout 30 ros2 topic echo "${LIDAR_TIMED_TOPIC}" --once --qos-reliability best-effort >/dev/null 2>&1 &
LIDAR_WAIT=$!
timeout 30 ros2 topic echo /livox/imu --once --qos-reliability best-effort >/dev/null 2>&1 &
IMU_WAIT=$!
wait "${LIDAR_WAIT}" || echo "WARNING: ${LIDAR_TIMED_TOPIC} not seen within 30s" >&2
wait "${IMU_WAIT}" || echo "WARNING: /livox/imu not seen within 30s" >&2
echo "Sensor streams ready."

ros2 run fast_lio fastlio_mapping --ros-args --params-file "${FASTLIO_CONFIG}" \
    >"${LOG_DIR}/fastlio.log" 2>&1 &
PIDS+=("$!")

# FAST-LIO odometry → PX4 EKF2 external vision (subscribes best-effort and
# waits for /Odometry — safe to start right away)
python3 "${SCRIPT_DIR}/fastlio/fastlio_px4_bridge.py" &
PIDS+=("$!")

# NOTE: the world-frame cloud comes from super_bridge (in offboard.launch.py):
# it reads the PX4 fused vehicle_odometry and transforms the raw lidar cloud
# into the world frame itself (→ /cloud_registered). This mirrors the
# real-hardware setup where there is no Gazebo ground-truth odom.

echo "Waiting for FAST-LIO to initialize (up to 60s)..."
# FAST-LIO logs "Initialize the map kdtree" once the first scan is accepted —
# grepping the log is cheaper and discovery-free vs. echoing /Odometry.
INIT=0
for _ in $(seq 1 20); do
    if grep -q "Initialize the map kdtree" "${LOG_DIR}/fastlio.log" 2>/dev/null; then
        INIT=1
        echo "FAST-LIO initialized."
        break
    fi
    sleep 3
done
if [ "${INIT}" -ne 1 ]; then
    echo "WARNING: FAST-LIO may not be fully initialized. Proceeding anyway..."
fi

# ===========================================================================
#  Phase 3 — Offboard + Planner + RViz (main's super_bridge architecture)
# ===========================================================================
echo ""
echo "Phase 3/3 — Offboard + Planner + RViz"

# Same offboard.launch.py as main's start_sim.sh flow: planner consumes
# PX4-fused odom via super_bridge (/lidar_slam/odom + /cloud_registered) —
# mirrors real hardware (no Gazebo truth). The model spawns at the world
# origin, so PX4's EKF local frame == world frame and no spawn-pose offset
# is needed. Two RViz windows like main:
#   rviz_config          → top-down birdview planning window (BIRDVIEW_CONFIG)
#   rviz_freelook_config → 3D window (RVIZ_CONFIG: ikd-Tree map view)
if [[ "${NO_RVIZ:-}" == "1" ]]; then
    ros2 launch "${OFFBOARD_LAUNCH}" rviz:=false &
else
    ros2 launch "${OFFBOARD_LAUNCH}" \
        rviz_config:="${BIRDVIEW_CONFIG}" rviz_freelook_config:="${RVIZ_CONFIG}" &
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
