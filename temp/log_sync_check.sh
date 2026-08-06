#!/usr/bin/env bash
# log_sync_check.sh — Log FAST-LIO odom + Gazebo truth + planner cmd at the same instant.
# Run this while the drone is flying (or hovering). It appends to a timestamped CSV.
#
# Usage:
#   ./temp/log_sync_check.sh <duration_seconds> [output_file]
#   ./temp/log_sync_check.sh 30 /tmp/coord_sync.csv

set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "${SCRIPT_DIR}/.." && pwd)"

source /opt/ros/humble/setup.bash 2>/dev/null || true
source "${WORKSPACE}/install/setup.bash" 2>/dev/null || true

DUR="${1:-20}"
OUT="${2:-/tmp/coord_sync_$(date +%H%M%S).csv}"

echo "Logging for ${DUR}s → ${OUT}"
echo "t,fastlio_x,fastlio_y,fastlio_z,truth_x,truth_y,truth_z,cmd_x,cmd_y,cmd_z" > "${OUT}"

END=$((SECONDS + DUR))
while [ "${SECONDS}" -lt "${END}" ]; do
    # FAST-LIO odom (camera_init) — -k 1 forces kill after timeout
    FL=$(timeout -k 1 1 ros2 topic echo /Odometry --once 2>/dev/null \
        | awk '/position:/{getline; x=$2; getline; y=$2; getline; z=$2; print x","y","z; exit}')
    # Gazebo truth (world)
    GT=$(timeout -k 1 1 ros2 topic echo /odom --once 2>/dev/null \
        | awk '/position:/{getline; x=$2; getline; y=$2; getline; z=$2; print x","y","z; exit}')
    # Planner command (ENU)
    CMD=$(timeout -k 1 1 ros2 topic echo /planning/pos_cmd --once --qos-reliability best_effort 2>/dev/null \
        | awk '/position:/{getline; x=$2; getline; y=$2; getline; z=$2; print x","y","z; exit}')

    # If no data, use empty markers
    FL="${FL:-NA,NA,NA}"
    GT="${GT:-NA,NA,NA}"
    CMD="${CMD:-NA,NA,NA}"

    echo "$(date +%s.%N),${FL},${GT},${CMD}" >> "${OUT}"
    sleep 0.5
done

echo "Done. Saved to ${OUT}"
echo ""
echo "Sample (first 5 lines):"
head -6 "${OUT}"
