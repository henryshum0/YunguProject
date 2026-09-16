#!/usr/bin/env bash
#
# Start the whole simulated system in one terminal:
#   1. the simulator        utils/start_sim.sh   (Gazebo, PX4 SITL, uXRCE, bridges)
#   2. the vehicle stack    src/launch/yungu_stack.launch.py
#                           (sensor bridging, navigation, coverage planner, detection)
#   3. optionally the GUI   gui.py               (--gui)
#
# The simulator keeps its own xterm window and its own log files; the vehicle
# stack runs in the foreground here, so its output is what you watch and Ctrl+C
# stops everything.
#
# Usage:
#   ./utils/start_all.sh                  # simulator + vehicle stack
#   ./utils/start_all.sh --gui            # ... and the operator GUI
#   ./utils/start_all.sh --headless       # run Gazebo without its GUI
#   ./utils/start_all.sh detection:=false # any yungu_stack.launch.py argument
#
# Anything of the form name:=value is passed straight to the stack launch; see
# src/launch/yungu_stack.launch.py for the available arguments.
#
# Start the layers separately instead when you want one of them in its own
# terminal (see the workspace README).

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "${SCRIPT_DIR}/.." && pwd)"
SIM_SCRIPT="${SCRIPT_DIR}/start_sim.sh"
STOP_SCRIPT="${SCRIPT_DIR}/stop_sim.sh"
STACK_LAUNCH="${WORKSPACE}/src/launch/yungu_stack.launch.py"
LOG_DIR="/tmp/yungu_sim"
SIM_LOG="${LOG_DIR}/start_all_sim.log"
SIM_READY_TIMEOUT=180

START_GUI=0
LAUNCH_ARGS=()
for argument in "$@"; do
  case "${argument}" in
    --gui)      START_GUI=1 ;;
    --headless) export HEADLESS=1 ;;
    -h|--help)  sed -n '2,30p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *:=*)       LAUNCH_ARGS+=("${argument}") ;;
    *)          echo "ERROR: unknown argument '${argument}'" >&2; exit 1 ;;
  esac
done

for path in "${SIM_SCRIPT}" "${STACK_LAUNCH}"; do
  [[ -f "${path}" ]] || { echo "ERROR: not found: ${path}" >&2; exit 1; }
done
if [[ ! -f "${WORKSPACE}/install/setup.bash" ]]; then
  echo "ERROR: ${WORKSPACE}/install is missing. Run 'colcon build --symlink-install' first." >&2
  exit 1
fi

# ROS is sourced here so every child inherits it. The generated setup scripts
# read variables that may not be set, so nounset is lifted while they run.
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${WORKSPACE}/install/setup.bash"
set -u
export PYTHONPATH="${WORKSPACE}:${PYTHONPATH:-}"

mkdir -p "${LOG_DIR}"
: >"${SIM_LOG}"

sim_pid=""
stack_pid=""
cleaned=0
cleanup() {
  [[ "${cleaned}" -eq 1 ]] && return
  cleaned=1
  echo
  echo "Stopping the vehicle stack and simulator ..."
  # Shut the launch down first, so its nodes stop before the simulator they
  # talk to disappears. It runs in its own process group, so the whole stack can
  # be signalled at once and nothing is orphaned if the launch itself is slow.
  if [[ -n "${stack_pid}" ]] && kill -0 "${stack_pid}" 2>/dev/null; then
    kill -INT -- "-${stack_pid}" 2>/dev/null || kill -INT "${stack_pid}" 2>/dev/null
    for _ in $(seq 1 15); do
      kill -0 "${stack_pid}" 2>/dev/null || break
      sleep 1
    done
    kill -9 -- "-${stack_pid}" 2>/dev/null || kill -9 "${stack_pid}" 2>/dev/null
  fi
  if [[ -n "${sim_pid}" ]]; then
    kill -- "-${sim_pid}" 2>/dev/null || kill "${sim_pid}" 2>/dev/null
    sleep 2
    kill -9 -- "-${sim_pid}" 2>/dev/null || kill -9 "${sim_pid}" 2>/dev/null
  fi
  # Backstop for the processes PX4 detaches into their own sessions.
  [[ -x "${STOP_SCRIPT}" ]] && "${STOP_SCRIPT}" >/dev/null 2>&1
  echo "Stopped."
}
trap cleanup EXIT INT TERM HUP

echo "Starting the simulator (log: ${SIM_LOG}) ..."
setsid "${SIM_SCRIPT}" >"${SIM_LOG}" 2>&1 &
sim_pid=$!

for _ in $(seq 1 "${SIM_READY_TIMEOUT}"); do
  if grep -q "Simulation stack running" "${SIM_LOG}" 2>/dev/null; then
    break
  fi
  if ! kill -0 "${sim_pid}" 2>/dev/null; then
    echo "ERROR: the simulator exited before it was ready. Last output:" >&2
    tail -n 25 "${SIM_LOG}" >&2
    exit 1
  fi
  sleep 1
done
if ! grep -q "Simulation stack running" "${SIM_LOG}" 2>/dev/null; then
  echo "ERROR: the simulator did not become ready within ${SIM_READY_TIMEOUT}s. Last output:" >&2
  tail -n 25 "${SIM_LOG}" >&2
  exit 1
fi
echo "Simulator ready."

if [[ "${START_GUI}" -eq 1 ]]; then
  echo "Starting the operator GUI ..."
  setsid /usr/bin/python3 "${WORKSPACE}/gui.py" >"${LOG_DIR}/gui.log" 2>&1 &
fi

echo
echo "Starting the vehicle stack (sensors, navigation, coverage planner, detection) ..."
echo "Press Ctrl+C to stop everything."
echo
# Backgrounded and waited on rather than run in the foreground: bash defers its
# traps until a foreground command returns, so a plain Ctrl+C would otherwise
# leave the simulator running until the launch happened to exit by itself.
# setsid makes it a process-group leader, which is what lets cleanup reach every
# node it started.
setsid ros2 launch "${STACK_LAUNCH}" ${LAUNCH_ARGS[@]+"${LAUNCH_ARGS[@]}"} &
stack_pid=$!
wait "${stack_pid}"
