#!/usr/bin/env bash
#
# Kill any leftover Yungu simulation processes (PX4 SITL, Gazebo, uXRCE-DDS
# agent, GZ<->ROS bridges, and the PX4 SITL xterm window). Idempotent and safe
# to run at any time.
#
# Usage:
#   ./stop_sim.sh
#
# Use this if start_sim.sh's Ctrl+C cleanup ever leaves processes behind (e.g.
# the gz server that PX4 detaches into its own session, which survives the
# normal process-group kill).

set -u

echo "Stopping any leftover Yungu simulation processes ..."

# PX4 SITL + Gazebo (gz sim is a ruby wrapper; gz-server is the actual server)
pkill -9 -x px4                 2>/dev/null
pkill -9 -f "gz sim"            2>/dev/null
pkill -9 -x gz-server           2>/dev/null

# The PX4 SITL xterm window
pkill -9 -f "PX4 SITL"          2>/dev/null

# uXRCE-DDS agent
pkill -9 -x MicroXRCEAgent      2>/dev/null

# GZ <-> ROS bridges
pkill -9 -f "parameter_bridge"  2>/dev/null
pkill -9 -f "tf_bridge.py"      2>/dev/null

# Report anything that is still alive.
sleep 1
leftover="$(pgrep -af 'px4|gz sim|gz-server|MicroXRCEAgent|parameter_bridge|tf_bridge.py|PX4 SITL' 2>/dev/null | grep -v 'grep' || true)"
if [[ -n "${leftover}" ]]; then
  echo "WARNING: these processes are still running:" >&2
  echo "${leftover}" >&2
  exit 1
fi

echo "All simulation processes stopped."
