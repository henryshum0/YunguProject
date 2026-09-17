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
