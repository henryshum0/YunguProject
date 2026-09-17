"""Scripted ground agents: robots that walk to a goal for the scene to contain.

These are deliberately not controlled robots. A quadruped or humanoid that walks
under physics needs a whole-body locomotion controller, which is a research
project of its own and not what this workspace is about. What it does need is
ground robots that go where they are told and *look* like they walked there, so
the UAV has something to fly over, watch, and plan around.

So an agent is a kinematic body with animated legs: :mod:`agents.navigator`
produces the velocity that takes it to a goal, and :mod:`agents.gait` turns the
distance it covered into joint angles. Legs are driven by distance travelled, not
by a timer, which is what keeps the steps and the motion consistent.
"""

from agents.config import AgentConfigError, AgentSpec, AgentsConfig
from agents.gait import Gait, JointGait, gait_from_mapping
from agents.navigator import AgentLimits, Command, Navigator, Pose2D, Step, wrap_angle
from agents.topics import base_command_topic, joint_command_topic

__all__ = [
    "AgentConfigError",
    "AgentLimits",
    "AgentSpec",
    "AgentsConfig",
    "Command",
    "Gait",
    "JointGait",
    "Navigator",
    "Pose2D",
    "Step",
    "base_command_topic",
    "gait_from_mapping",
    "joint_command_topic",
    "wrap_angle",
]
