#!/usr/bin/env bash
# Quick check: is the offboard+planner chain healthy?
# Run while start_all.sh is running in another terminal.

echo "=== Key Topics ==="
for t in /goal_pose /planning/pos_cmd /fmu/in/trajectory_setpoint /fmu/in/offboard_control_mode /fmu/out/vehicle_local_position_v1; do
    if ros2 topic info "$t" 2>/dev/null | grep -q "Publisher"; then
        rate=$(ros2 topic hz "$t" --window 3 2>/dev/null | tail -1 | awk '{print $NF}')
        echo "  ✅ $t  (${rate:-?} Hz)"
    else
        echo "  ❌ $t — NO PUBLISHER"
    fi
done

echo ""
echo "=== TrajectorySetpoint last 3 msgs ==="
ros2 topic echo /fmu/in/trajectory_setpoint --once 2>/dev/null | head -12

echo ""
echo "=== Offboard node state (check log) ==="
grep -E "State:|Planner|Goal" /tmp/yungu_sim/px4_sitl.log 2>/dev/null | tail -5 || echo "(no offboard log found — check the terminal running start_all.sh)"
