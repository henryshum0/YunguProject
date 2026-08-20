# PlannerState.msg
#
# Published by fsm_node (super_planner) to report the high-level planner
# state to other nodes (e.g. flight monitor, ground station, mission layer).
#
# state  description
# -----  ---------------------------------------------------------
# INIT       FSM is initialising (no goal received yet)
# WAIT_GOAL  FSM is idle, waiting for a new goal
# MOVE       A goal is being planned / the planned trajectory is executed
# FAIL       Planning has been failing continuously for more than 2 s
#
# The convenience bool flags below simply mirror `state` so consumers can
# either switch on the enum or on the individual flags.

builtin_interfaces/Time stamp

uint8 state
uint8 STATE_INIT = 0
uint8 STATE_WAIT_GOAL = 1
uint8 STATE_MOVE = 2
uint8 STATE_FAIL = 3

bool init
bool wait_goal
bool move
bool fail
