#!/usr/bin/env bash
#
# One-shot launcher for the Yungu simulation stack:
#   1. PX4 SITL + Gazebo (gz-sim)       make px4_sitl gz_x500_lidar_yungu
#   2. MicroXRCEAgent                   uXRCE-DDS agent on UDP 8888
#   3. GZ <-> ROS bridge + TF           utils/gz_bridges/bridge.sh
#
# PX4 SITL runs in its own xterm window (so you can watch its output and
# interact with it). When this script is interrupted (Ctrl+C), that window and
# the whole PX4/Gazebo process tree are terminated as well.
#
# Usage:
#   ./start_sim.sh                                   # defaults from config/simulation.yaml
#   PX4_MODEL=x500_lidar PX4_WORLD=yungu ./start_sim.sh   # override the config
#   PX4_MODEL=swan_gamma_v1 PX4_WORLD=indoor_dining ./start_sim.sh
#   PX4_MODEL=gz_x500_lidar_yungu ./start_sim.sh     # legacy: full make target
#   XRCE_PORT=8888 ./start_sim.sh
#   GZ_VERSION=harmonic ./start_sim.sh
#   HEADLESS=1 ./start_sim.sh                        # run Gazebo without its GUI (server only)
#   ./start_sim.sh --help                            # list available models and maps
#   ./stop_sim.sh                                    # kill any leftover sim processes
#
# Model, map, GZ version and uXRCE port default to config/simulation.yaml
# (env vars override). Model + Map combine into the PX4 make target
# 'gz_<model>_<world>'. Run '--help' for the full list of models and maps.
#
# Press Ctrl+C to stop everything; run ./stop_sim.sh if anything lingers.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "${SCRIPT_DIR}/.." && pwd)"
PX4_DIR="${WORKSPACE}/VisionFlow-PX4"
BRIDGE_SCRIPT="${SCRIPT_DIR}/gz_bridges/bridge.sh"
STOP_SCRIPT="${SCRIPT_DIR}/stop_sim.sh"
SIM_CONFIG="${WORKSPACE}/config/simulation.yaml"
SIM_CONFIG_HELPER="${WORKSPACE}/config/sim_config.py"

[[ -f "${SIM_CONFIG}" ]] || { echo "ERROR: simulation config not found: ${SIM_CONFIG}" >&2; exit 1; }
command -v python3 >/dev/null 2>&1 || { echo "ERROR: python3 not found on PATH" >&2; exit 1; }

# Read the simulation defaults from config/simulation.yaml. Environment
# variables still override the file when set (e.g. PX4_MODEL=... ./start_sim.sh).
config_get() {
  python3 "${SIM_CONFIG_HELPER}" --config "${SIM_CONFIG}" get "$1"
}

# PX4_MODEL = vehicle model, PX4_WORLD = map/world. They combine into the PX4
# make target 'gz_<model>_<world>'. For backwards compatibility PX4_MODEL may
# also be given as a full target (e.g. 'gz_x500_lidar_yungu'), used verbatim.
PX4_MODEL="${PX4_MODEL:-$(config_get model)}"
PX4_WORLD="${PX4_WORLD:-$(config_get world)}"
XRCE_PORT="${XRCE_PORT:-$(config_get xrce_port)}"
GZ_VERSION="${GZ_VERSION:-$(config_get gz_version)}"

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

# ---------------------------------------------------------------------------
#  Model & map selection (PX4_MODEL / PX4_WORLD)
# ---------------------------------------------------------------------------
AIRFRAME_DIR="${PX4_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes"
WORLDS_DIR="${PX4_DIR}/Tools/simulation/gz/worlds"

# Enumerate the valid models (from the gz airframes) and maps (world .sdf files).
valid_models=()
for f in "${AIRFRAME_DIR}"/[0-9]*_gz_*; do
  [[ -f "${f}" ]] || continue
  valid_models+=("${f##*_gz_}")
done
valid_worlds=()
for f in "${WORLDS_DIR}"/*.sdf; do
  [[ -f "${f}" ]] || continue
  valid_worlds+=("$(basename "${f}" .sdf)")
done

print_options() {
  echo "  Available models (PX4_MODEL): ${valid_models[*]:-<none>}"
  echo "  Available maps   (PX4_WORLD): ${valid_worlds[*]:-<none>}"
}

in_list() { # $1 = item, remaining args = candidates
  local item="$1"; shift
  local x
  for x in "$@"; do
    [[ "${x}" == "${item}" ]] && return 0
  done
  return 1
}

usage() {
  cat <<EOF
Usage: $0 [--help]

Model, world, GZ version and uXRCE port default to ${SIM_CONFIG}.
Environment variables override the file:

  PX4_MODEL   Gazebo vehicle model (default: ${PX4_MODEL})
  PX4_WORLD   Gazebo map/world     (default: ${PX4_WORLD})
  XRCE_PORT   uXRCE-DDS agent UDP port (default: ${XRCE_PORT})
  GZ_VERSION  Gazebo distro            (default: ${GZ_VERSION})
  HEADLESS    non-empty to run Gazebo without its GUI
EOF
  print_options
  echo
  echo "If any simulation processes linger after stopping, run: ${STOP_SCRIPT}"
}

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  usage
  exit 0
fi

# Legacy: a PX4_MODEL that already looks like a full target is used as-is.
if [[ "${PX4_MODEL}" == gz_* ]]; then
  PX4_TARGET="${PX4_MODEL}"
else
  in_list "${PX4_MODEL}" "${valid_models[@]}" || {
    echo "ERROR: unknown PX4_MODEL '${PX4_MODEL}'." >&2
    print_options >&2
    exit 1
  }
  in_list "${PX4_WORLD}" "${valid_worlds[@]}" || {
    echo "ERROR: unknown PX4_WORLD '${PX4_WORLD}'." >&2
    print_options >&2
    exit 1
  }
  PX4_TARGET="gz_${PX4_MODEL}_${PX4_WORLD}"
fi

# Resolve the actual Gazebo model/world names (works for both the new
# model/world form and the legacy full-target form) so messages/warnings can
# refer to them accurately. A candidate only counts when the whole remainder
# after 'gz_<model>_' is an exact world name (avoids 'x500' matching 'x500_lidar').
ACTUAL_MODEL=""
ACTUAL_WORLD=""
for m in "${valid_models[@]}"; do
  for w in "${valid_worlds[@]}"; do
    if [[ "${PX4_TARGET}" == "gz_${m}_${w}" ]]; then
      ACTUAL_MODEL="${m}"
      ACTUAL_WORLD="${w}"
      break 2
    fi
  done
  if [[ "${PX4_TARGET}" == "gz_${m}" ]]; then
    ACTUAL_MODEL="${m}"
    break
  fi
done
ACTUAL_MODEL="${ACTUAL_MODEL:-${PX4_MODEL}}"
ACTUAL_WORLD="${ACTUAL_WORLD:-${PX4_WORLD}}"

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
#
# IMPORTANT: this handler must always run to completion. If the user presses
# Ctrl+C a second time while it runs, the terminal delivers SIGINT to the whole
# process group, interrupting e.g. `sleep` below; with `set -e` active that
# would abort the trap and skip the backstop kill, leaving orphaned sim
# processes (the gz server that PX4 detaches into its own session survives the
# group-kill). Disabling errexit at the top makes the trap fail-safe.
cleanup() {
  [[ "${cleaned}" -eq 1 ]] && return
  cleaned=1
  set +e   # never let errexit abort the trap handler
  echo
  echo "Stopping simulation stack ..."

  # 1. Graceful SIGTERM to the process groups we spawned with setsid.
  for p in "${pids[@]:-}"; do
    kill -- "-${p}" 2>/dev/null || kill "${p}" 2>/dev/null
  done

  # 2. Give them a moment to shut down, then SIGKILL anything still alive.
  sleep 2 || true
  for p in "${pids[@]:-}"; do
    kill -9 -- "-${p}" 2>/dev/null || kill -9 "${p}" 2>/dev/null
  done

  # 3. Backstop: remove leftover simulation processes by name (covers children
  #    that PX4 daemonized into their own sessions, e.g. the gz server, which
  #    the group-kill above cannot reach). Also closes the PX4 xterm window.
  if [[ -f "${STOP_SCRIPT}" ]]; then
    "${STOP_SCRIPT}"
  fi

  set -e
  echo "Simulation stack stopped."
}
trap cleanup EXIT INT TERM HUP

# ---------------------------------------------------------------------------
#  1. PX4 SITL + Gazebo
# ---------------------------------------------------------------------------
if [[ -n "${HEADLESS}" ]]; then
  echo "Starting PX4 SITL + Gazebo (target: ${PX4_TARGET}, HEADLESS: Gazebo GUI disabled) ..."
else
  echo "Starting PX4 SITL + Gazebo (target: ${PX4_TARGET}) ..."
fi

# Do not let a successful line from a previous run make this launch look ready.
: >"${LOG_DIR}/px4_sitl.log"

if [[ -n "${TERMINAL}" ]]; then
  export PX4_DIR PX4_TARGET LOG_DIR HEADLESS
  # Use an Xft/fontconfig font. The legacy xterm "fixed" bitmap font is not
  # installed by default in WSLg and makes xterm exit before PX4 is launched.
  setsid xterm -fa Monospace -fs 10 -T "PX4 SITL (${PX4_TARGET})" -hold \
      -e bash -c 'cd "$PX4_DIR" && make px4_sitl "$PX4_TARGET" 2>&1 | tee "$LOG_DIR/px4_sitl.log"' &
  pids+=("$!")
else
  (cd "${PX4_DIR}" && make px4_sitl "${PX4_TARGET}") \
      >"${LOG_DIR}/px4_sitl.log" 2>&1 &
  pids+=("$!")
fi
px4_launcher_pid="$!"

echo "Waiting for PX4 SITL to come up ..."
px4_ready=0
for _ in $(seq 1 90); do
  if grep -qE "Ready for takeoff|INFO *\[commander\]" "${LOG_DIR}/px4_sitl.log" 2>/dev/null; then
    echo "PX4 SITL is up."
    px4_ready=1
    break
  fi
  if ! kill -0 "${px4_launcher_pid}" 2>/dev/null; then
    echo "ERROR: PX4 SITL launcher exited before PX4 became ready." >&2
    tail -n 40 "${LOG_DIR}/px4_sitl.log" >&2 || true
    exit 1
  fi
  sleep 1
done
if [[ "${px4_ready}" -ne 1 ]]; then
  echo "ERROR: PX4 SITL did not become ready within 90 seconds." >&2
  tail -n 40 "${LOG_DIR}/px4_sitl.log" >&2 || true
  exit 1
fi
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
echo "Simulation stack running (model: ${ACTUAL_MODEL}, map: ${ACTUAL_WORLD}, target: ${PX4_TARGET}). Logs in ${LOG_DIR}:"
echo "  - PX4 SITL:    ${LOG_DIR}/px4_sitl.log  (in its own xterm window)"
echo "  - uXRCE agent: ${LOG_DIR}/xrce_agent.log"
echo "  - gz bridge:   ${LOG_DIR}/bridge.log"
echo
echo "Next: ros2 launch offboard offboard.launch.py"

if [[ "${ACTUAL_MODEL}" != "x500_lidar" ]]; then
  echo
  echo "NOTE: the GZ<->ROS bridge topics are configured in ${SIM_CONFIG}"
  echo "(bridge.topics) and may not match model '${ACTUAL_MODEL}'."
  echo "Verify/adjust the bridge topics if needed."
fi
echo "Press Ctrl+C to stop everything (also terminates the PX4 SITL window)."
echo "If any simulation processes linger, run: ${STOP_SCRIPT}"
wait || true
