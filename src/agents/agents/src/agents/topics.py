"""The Gazebo command topics an agent model listens on.

The model file declares these (they are written into it at conversion time) and
the agent node publishes to them, so they are defined once here. A mismatch
between the two would show up only as a robot that silently refuses to move.
"""

from __future__ import annotations


def base_command_topic(model_name: str) -> str:
    """Body-frame velocity command for an agent's base."""
    return f"/model/{model_name}/cmd_vel"


def joint_command_topic(model_name: str, joint_name: str) -> str:
    """Position command for one animated joint."""
    return f"/model/{model_name}/joint/{joint_name}/cmd_pos"


#: Gazebo's own clock topic, which the launch bridges into ROS.
GZ_CLOCK_TOPIC = "/clock"

#: Where that clock is bridged to. Deliberately *not* ``/clock``: publishing the
#: global sim clock would switch every node that asks for ``use_sim_time`` --
#: the whole offboard stack does -- from a frozen clock to a running one, and
#: that is not a change to make as a side effect of fixing the ground agents.
SIM_CLOCK_TOPIC = "/agents/sim_clock"
