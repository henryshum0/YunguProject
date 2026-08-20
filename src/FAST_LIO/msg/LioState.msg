# LioState.msg
#
# Published by fastlio_mapping (fast_lio) to report the lidar-inertial
# odometry / mapping state to other nodes (e.g. flight monitor, ground
# station, mission layer).
#
# state   description
# -----   ---------------------------------------------------------
# INIT      Starting up: no sensor data received yet, or the EKF has not
#           finished initialising
# RUNNING   Mapping: EKF initialised and lidar + IMU data are flowing
# ERROR     A sensor stream has been lost for more than 2 s
#
# The convenience bool flags mirror `state` so consumers can switch on the
# enum or on the individual flags.

builtin_interfaces/Time stamp

uint8 state
uint8 STATE_INIT = 0
uint8 STATE_RUNNING = 1
uint8 STATE_ERROR = 2

bool init
bool running
bool error
