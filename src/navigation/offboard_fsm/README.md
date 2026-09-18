# EGO/PX4 offboard adapter

`offboard_fsm` is the single-drone boundary between EGO-Planner's ENU
`quadrotor_msgs/msg/PositionCommand` trajectory and PX4's NED offboard
setpoints. It owns takeoff, hover, queued EGO goals, abort/clear, and native
PX4 landing.

If this adapter restarts while PX4 is armed and airborne, it captures the
current pose and recovers directly to `IDLE`; it does not command another
takeoff. Queued goals wait for EGO's `WAIT_TARGET` state and are retransmitted
only until EGO acknowledges planning, preventing a DDS-discovery race from
silently losing the first goal.

It is configured externally by:

- `src/navigation/config/offboard/topics.yaml`
- `src/navigation/config/offboard/offboard_fsm.yaml`

Start PX4/Gazebo, the bridge, and `gz_sensor_interface` first, then run:

```bash
ros2 launch /home/windshape/YunguProject/src/launch/ego_single_drone.launch.py
```

Clients use `/waypoint_buffer`, `/waypoint_buffer/clear`, `/offboard/takeoff`,
and `/offboard/land`; see `docs/ros_interfaces.md` for their contracts. EGO's
goal, state, and position-command topics are internal integration endpoints.
