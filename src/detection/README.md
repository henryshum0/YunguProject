# Detection

The detection layer answers *what is in the camera frame*, and — one step
further — *where that is on the map*. It is the perception half of the search
skill: coverage planning produces a route, navigation flies it, and detection is
what makes the flight worth doing.

In simulation the detector itself is mocked. That is a deliberate design
decision, not a shortcut, and the reason is in [Why the detector is
mocked](#why-the-detector-is-mocked) below. Everything else in this package —
the camera model, the localizer, the message interface, the skills on top — runs
unchanged with the real detector.

## Layout

| Path | Contents |
|---|---|
| [`config/detection.yaml`](config/detection.yaml) | The shared contract: world frame and ground plane, camera model, topics, localizer and overlay settings. Used by the mock, the localizer, the overlay, and the skills layer. |
| [`config/mock_detector.yaml`](config/mock_detector.yaml) | Simulation-only: detector rate, visibility gates, and the error model. |
| [`config/targets.yaml`](config/targets.yaml) | The simulated ground-truth targets, read by both the Gazebo spawner and the mock detector. |
| [`detection/src/detection/`](detection/src/detection/) | Python package: camera geometry, occlusion, error model, mock detector, localization, overlay drawing, ground-truth matching, and the ROS adapters. |
| [`detection/models/`](detection/models/) | Self-contained Gazebo models for the target classes. |

## Running it

It comes up with the rest of the vehicle stack:

```bash
./utils/start_all.sh                                      # everything
ros2 launch "$PWD/src/launch/yungu_stack.launch.py"       # the vehicle stack
ros2 launch detection detection.launch.py                 # this layer alone
```

That spawns the configured targets into the running Gazebo world and starts three
nodes: the mock detector, the target localizer, and the debug overlay. Useful
arguments:

| Argument | Default | Effect |
|---|---|---|
| `spawn:=false` | `true` | Do not spawn targets (they are already in the world). |
| `mock:=false` | `true` | Do not start the mock detector — use when a real detector publishes. |
| `localizer:=false` | `true` | Do not start the localizer. |
| `overlay:=false` | `true` | Do not start the debug overlay. |
| `world:=<name>` | from `simulation.yaml` | Gazebo world to spawn targets into. |
| `spawn_wait:=<s>` | `60.0` | How long to wait for the Gazebo world before spawning, so this can start alongside the simulator rather than after it. |

Targets can also be managed on their own, which is convenient while moving them
around:

```bash
ros2 run detection spawn_targets \
  --detection-config src/detection/config/detection.yaml \
  --targets-config src/detection/config/targets.yaml \
  --models-dir src/detection/detection/models
```

Add `--remove` to take them out of the world again, or `--dry-run` to print what
would be spawned without contacting Gazebo.

## Interface

These are the topics a real detector has to produce. Nothing downstream knows
whether the mock or a real model is publishing them.

| Topic | Type | Published by | Contents |
|---|---|---|---|
| `/detection/detections` | `vision_msgs/msg/Detection2DArray` | the detector | Image-plane boxes in the camera frame, stamped with the frame's capture time. `class_id` is a VisDrone class name, `id` is unique per detection. |
| `/detection/detections_world` | `vision_msgs/msg/Detection3DArray` | `target_localizer` | The same detections placed on the ground plane in the `map` ENU frame. `id` is carried over so the two can be joined. |
| `/detection/markers` | `visualization_msgs/msg/MarkerArray` | `target_localizer` | RViz markers for the world detections. |
| `/detection/image_overlay` | `sensor_msgs/msg/Image` | `detection_overlay` | Debug view: the front-camera frame with boxes drawn on it. Point the GUI camera panel at this topic to watch the loop work. |
| `/gz/odom_super` | `nav_msgs/msg/Odometry` | `gz_sensor_interface` | Consumed, not produced: the vehicle pose both nodes work from. |

### Replacing the mock with the real detector

A RemDet node (the separate `YunguDetection` repository) has to publish
`Detection2DArray` on `/detection/detections`, with:

| RemDet output | Message field |
|---|---|
| `class_name` (`pedestrian`, `car`, …) | `results[0].hypothesis.class_id` |
| `score` | `results[0].hypothesis.score` |
| `bbox` as `xyxy` | `bbox.center.position` = box centre, `bbox.size_x/size_y` = box size |
| frame capture time | `header.stamp` — the time the image was taken, not the time inference finished |
| tracker or per-frame id | `id`, if a consumer should join frames |

Then launch with `mock:=false` and nothing else changes — the localizer, the
overlay and the skills layer all keep working, because they only ever read
`/detection/detections`. `header.stamp` is the
part worth being careful about: the localizer looks the vehicle pose up by it,
so stamping the publication time instead of the capture time silently
mislocalizes every target by one inference latency of vehicle motion.

## Why the detector is mocked

Everything is validated in simulation before it flies. Navigation and search
survive that honestly. Detection does not, and the two obvious ways around it
both fail:

- **Simulated images into a real detector.** The model is trained on real
  imagery; a Gazebo scene is far too coarse for its output to say anything about
  real accuracy.
- **Real video with simulated flight.** The recorded video has its own camera
  pose and timeline. It cannot be aligned with where the simulated vehicle
  actually is, so every downstream decision rests on a detection that does not
  describe the vehicle's situation.

The mock takes the third road. It renders nothing. It projects the known
ground-truth targets through the camera model using the vehicle's own state at
that instant — so detections are aligned with the flight by construction — and
then applies an error model whose parameters come from measuring the real
detector.

What that buys: the search logic in simulation faces detection behaviour whose
*statistics* match what it will face on the vehicle. What it explicitly does not
buy: any statement about detection accuracy. Accuracy is validated separately,
on real data, and is what calibrates the error model.

### Geometry

Per target, per frame, with no randomness involved — a target that fails any of
these is one the real camera genuinely could not show:

1. The target's eight bounding-box corners go world → body (from the vehicle
   pose) → camera (from the mounting extrinsics).
2. Corners must be in front of the image plane; they are projected with the
   pinhole model and bounded into an image-plane box.
3. Range must be within the camera's far clip. Beyond it the simulated image
   renders nothing, so nothing may be reported there either.
4. Enough of the box must fall inside the image, and the box must be large
   enough to resolve. This is where *flying higher makes targets smaller* enters
   — the box shrinks as `focal_length × size / range`, and detectability follows.
5. The line from camera to target must not cross the world collision mesh.
   Without this check a search could "find" a target through a building wall.

### Error model

Applied to whatever survived the geometry, with one seeded RNG so a run replays
exactly. Each parameter is calibrated from a measurement of the real detector:

| Behaviour | Shape | Calibrated from |
|---|---|---|
| Missed detections | Recall as a logistic in the box's short side | recall versus target size on real Yungu imagery |
| False positives | Poisson-distributed spurious boxes per frame | measured per-frame false-positive rate |
| Localization noise | Gaussian jitter on box centre and size | measured localization variance |
| Confidence | Size-dependent mean plus Gaussian spread, then a score threshold | measured score distribution |
| Class confusion | Per-class probability of reporting a neighbour | measured confusion pairs |
| Latency | Output held back, capture stamp preserved | measured inference latency |

**The numbers currently in `mock_detector.yaml` are provisional placeholders**
with a plausible shape, not measurements. Replacing them with real ones is a
configuration edit; no code changes.

## Reading a result against the truth

What a search reports is what it **estimated**: a detection box back-projected
onto the ground plane and averaged over frames. That is all a real system has,
and the skill layer has no more — nothing in the detection path is allowed to
quietly correct an estimate with knowledge only the simulator holds.

`detection.truth.match_to_truth` exists for reading the result afterwards. It
pairs each estimate with the ground-truth target it most likely refers to
(greedy by distance, one-to-one, within a radius) and reports that target's id,
its true class and the horizontal error. Both
[`run_search_detection_demo.py`](../../run_search_detection_demo.py) and the GUI
use it to print the truth underneath the estimate. An estimate with no target
near it comes back unmatched, which is what a false positive confirmed by
mistake looks like.

Typical errors from a full sweep of the shipped targets: 0.5 to 1.1 m for cars,
which is the box-bottom-edge bias below averaging out over hundreds of
detections from many viewpoints. A target confirmed from only a handful of
distant frames is cruder, two to three metres — which is why `SearchSkill.call`
only confirms a target once the vehicle is within `confirm_within_m` of it and
then observes for `settle_sec` more, so even a "stop on first find" mission
reports a close-range fix (~0.5–1.5 m) instead of the first distant one.

## Coordinates and clocks

Two things are easy to get wrong here, and both are settled in
`config/detection.yaml`.

**Frames.** Targets, detections and waypoints all live in the same ENU frame:
`map`, anchored at the drone launch point (the visualization layer calls the same
frame `world`). Gazebo's own world frame is offset from it by the airframe spawn
pose, `PX4_GZ_MODEL_POSE` — `(0, 0, 1.15392)` for `swan_gamma_v2`. Only the
spawner and the occlusion mesh loader ever apply that offset; everything else
stays in the navigation frame. The Gazebo ground sits at gz `z = -0.50`, which is
`world.ground_z_m = -1.654` in the navigation frame.

**Clocks.** Bridged Gazebo topics carry the simulation clock, which starts at
zero; the offboard stack runs on the wall clock. The detection subsystem is
entirely on the wall clock, because it takes its pose from `/gz/odom_super`,
which `gz_sensor_interface/super_lidar` already restamps with `now()`. The one
exception is the debug overlay, which draws current boxes on the most recent
camera frame rather than on the frame they were computed from — the two are on
different clocks, and at normal rates within a frame of each other anyway.

**Why the overlay is a separate node.** Decoding a 640×480 stream costs a
full-frame copy per published overlay, and ROS deserializes every frame that
arrives whether or not it is drawn. Inside the detector that competed with the
detector's own timing (measured: 42% of a core, and the detector's own loop
starved); split out, the detector holds a steady 10 Hz at ~10%.

## Known limitations

- **The camera the planner assumes is not the camera that exists.** Coverage
  planning models a nadir camera with a 60°×45° field of view; the simulated (and
  real) camera looks forward, pitched 60° down, with a 100° horizontal field of
  view. The real swath is much wider than the planned one, so coverage is
  conservative rather than gappy — but lane spacing is not derived from the
  camera that actually takes the pictures. Left alone deliberately until the
  detection accuracy work settles the usable height envelope.
- **Targets are static.** They are declared with a fixed pose. Moving targets
  would need a pose source instead of a configuration file; the mock's geometry
  would not otherwise change.
- **Targets must be placed out of sight of the launch point.** From the takeoff
  hover the camera already reaches about 35 m ahead and 48 m at the image
  corners, so a target inside that is confirmed before the vehicle has moved and
  the search proves nothing. The shipped targets are all beyond 50 m, and
  `test_shipped_targets_are_not_visible_from_the_takeoff_hover` keeps it that
  way.
- **Height is not observable.** A single image box cannot give a target's height,
  so world detections report a zero z-size.
- **Localization is biased by box geometry.** The bottom edge of a 2D box around
  a long vehicle is its nearest ground edge, not its centre, so world positions
  for cars sit one or two metres short. A real detector has exactly the same
  bias; the target-confirmation radius in the skills layer accommodates it.

## Tests

```bash
colcon test --packages-select detection
```

or directly, without a build:

```bash
PYTHONPATH=src/detection/detection/src:$PYTHONPATH python3 -m pytest src/detection/detection/tests/unit
```
