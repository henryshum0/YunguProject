# Ground agents

Quadruped and humanoid robots that walk to a goal, so the scene contains moving
ground robots for the UAV to fly over, watch and plan around.

## What these are, and what they are not

They are **kinematic scene actors**, not controlled robots. The base is driven
straight to its goal and the legs are animated to match the distance covered.
There is no locomotion controller, no balance, no foot contact and no state
estimation.

That is a deliberate trade. Physically walking a quadruped or a humanoid needs a
whole-body locomotion controller — a gait generator with inverse kinematics and
impedance control for the quadruped, and a trained RL policy for the humanoid.
Both are projects in their own right, and neither is what this workspace is
about. What it actually needs is ground robots that go where they are told and
*look* like they walked there, which is what this gives you for a few hundred
lines.

So:

* the legs move in step with the body and stop when it stops — the gait is driven
  by **distance travelled**, not by a timer, which is what avoids the ice-skating
  look of an animation running at its own cadence;
* the robot never falls, never tips and never gets wedged on scenery;
* the feet do not push the ground, and the robot walks through obstacles rather
  than around them.

The relationship between motion and joints is the same one the UAV already has:
its rotor joints spin because it is being commanded to fly, not on a clock of
their own.

## Sending an agent somewhere

Goals are ENU `PoseStamped` in the `map` frame — the same frame as coverage
search areas, navigation waypoints and detection targets — so a point read off
the operations map can be sent straight to a robot.

```bash
ros2 topic pub --once /agents/go1/goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: map}, pose: {position: {x: 25.0, y: 8.0}}}'
```

Each agent reports where it believes it is on `/agents/<name>/pose`. Only `x`
and `y` of a goal are used; the agent walks on the ground plane.

An agent turns on the spot until it roughly faces the goal, walks to it, then
stops inside `arrive_radius_m` and holds its standing stance. Sending a new goal
while it walks replaces the old one.

## Configuration

Everything is in [`config/agents.yaml`](config/agents.yaml): which robots exist,
where they start, how fast they walk, and how each joint moves over a gait cycle.
The launch file and the driver node both read it, so adding a robot is a config
change.

A gait is one sinusoid per joint over a cycle completed every `stride_m` metres:

```
angle = bias + amplitude * shape(2*pi * (phase + offset))
```

`bias` is the standing pose, `offset` is what makes it a gait (a trot puts the
diagonal pairs half a cycle apart; a biped walk puts the two legs half a cycle
apart), and `shape` is `sin` for joints that swing both ways or `flex` for knees,
which only bend one way. Joint sign conventions differ between the robots — the
Go1 calf is always negative, the G1 knee always positive — and the config
comments record which is which. A unit test checks that no configured joint is
ever commanded past its hardware limit.

## Models

[`agents/models/`](agents/models) holds `unitree_go1` and `unitree_g1`, generated
from the upstream Unitree descriptions by
[`scripts/convert_unitree_model`](agents/scripts/convert_unitree_model). The
converter keeps the part that is hard to author by hand — the link tree, joint
origins and axes, and the visual meshes — and strips everything that would let
physics take over: Gazebo-classic plugins, the on-board sensors (the Go1
description ships five cameras, which must not start publishing here),
collisions, and gravity. It then adds one `VelocityControl` system for the base
and one `JointPositionController` per animated joint, for exactly the joints the
gait drives.

To regenerate after an upstream change:

```bash
ros2 run agents convert_unitree_model \
  --config src/agents/config/agents.yaml --agent go1 \
  --urdf  <unitree_ros>/robots/go1_description/urdf/go1.urdf \
  --meshes <unitree_ros>/robots/go1_description/meshes \
  --output src/agents/agents/models/unitree_go1
```

Upstream sources: [unitree_ros](https://github.com/unitreerobotics/unitree_ros)
(Go1, and the G1 description used here). Only the meshes the model actually
references are copied.

## Running

`agents.launch.py` spawns every configured agent, bridges the topics its model
listens on, and starts the driver:

```bash
ros2 launch agents agents.launch.py
```

It is included in the full stack, so `./utils/start_all.sh` brings the agents up
with everything else. Leave them out with `agents:=false`.
