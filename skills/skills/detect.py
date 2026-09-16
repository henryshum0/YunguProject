"""Detection skill: watch the detector stream and report what was found.

The detector node answers "what is in this frame". This skill is the mission-level
layer on top of it: it is told *what to look for*, filters the stream to those
classes, and turns a scattered sequence of per-frame boxes into a decision —
"there is a car at (11.9, 6.1)".

That last step is the part that matters. A real detector invents boxes, and the
simulated one reproduces that on purpose, so a single frame is never evidence.
A target is confirmed only once several detections of the same class agree on a
world position; spurious boxes land at random places and never accumulate.

The skill subscribes to two topics. ``detections`` is the detector's own output
and is always present. ``detections_world`` carries the ground positions added by
``target_localizer``; when that node is not running, detections are still
reported, with ``world_position`` left empty.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass, field
from math import hypot

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from vision_msgs.msg import Detection2DArray, Detection3DArray

from skills.config import SkillRuntimeConfig
from skills.primitives.base import SkillExecutionError
from skills.skills.base import Skill


#: Classes the detector can report. These are the VisDrone class names RemDet is
#: trained on; the mock detector uses the same names so both are interchangeable.
DETECTOR_CLASSES = (
    "pedestrian", "people", "bicycle", "car", "van", "truck",
    "tricycle", "awning-tricycle", "bus", "motor",
)

#: Convenience names for what an operator actually searches for. "Look for a
#: person" has to cover both VisDrone person classes, and "look for a vehicle"
#: has to cover the classes the detector confuses with one another.
CLASS_GROUPS = {
    "person": ("pedestrian", "people"),
    "vehicle": ("car", "van", "truck", "bus"),
}

#: Defaults for turning per-frame detections into a confirmed target.
DEFAULT_MIN_SCORE = 0.35
DEFAULT_MIN_HITS = 3
DEFAULT_CLUSTER_RADIUS_M = 6.0
DEFAULT_CLUSTER_TIMEOUT_SEC = 8.0

#: How long a detection waits for its world position before being reported without one.
JOIN_GRACE_SEC = 0.25


@dataclass(frozen=True, slots=True)
class Detection:
    """One detection as the skill layer reports it."""

    detection_id: str
    class_id: str
    score: float
    #: ``(center_x, center_y, size_x, size_y)`` in pixels.
    bbox: tuple[float, float, float, float]
    #: Capture time of the frame this detection came from.
    stamp_sec: float
    #: Ground position in the navigation ENU frame, or ``None`` when no localizer
    #: was running.
    world_position: tuple[float, float, float] | None


@dataclass(frozen=True, slots=True)
class ConfirmedTarget:
    """A target that enough consistent detections agree on."""

    class_id: str
    position: tuple[float, float, float]
    hits: int
    best_score: float
    first_seen_sec: float
    last_seen_sec: float


@dataclass(frozen=True, slots=True)
class DetectionResult:
    """Structured result of one detection-skill call."""

    success: bool
    message: str
    detections: tuple[Detection, ...]
    targets: tuple[ConfirmedTarget, ...]
    #: Capture time of the most recent frame seen, or ``None`` if none arrived.
    frame_stamp_sec: float | None
    frames_observed: int

    @property
    def found(self) -> bool:
        """Whether any target was confirmed."""
        return bool(self.targets)


def expand_classes(classes: Sequence[str]) -> tuple[str, ...]:
    """Expand operator-facing class names into detector class names.

    Group names from :data:`CLASS_GROUPS` expand to their members; anything else
    must already be a detector class, so a typo fails here instead of silently
    matching nothing.
    """
    if isinstance(classes, str):
        classes = (classes,)
    if not classes:
        raise ValueError("at least one target class is required")
    expanded: list[str] = []
    for name in classes:
        key = str(name).strip().lower()
        if not key:
            raise ValueError("target class names must not be empty")
        if key in CLASS_GROUPS:
            expanded.extend(CLASS_GROUPS[key])
            continue
        if key not in DETECTOR_CLASSES:
            raise ValueError(
                f"unknown target class '{name}'; use one of "
                f"{', '.join((*CLASS_GROUPS, *DETECTOR_CLASSES))}")
        expanded.append(key)
    return tuple(dict.fromkeys(expanded))


# Identity comparison (eq=False): two clusters with identical field values are
# still different clusters, and membership tests below rely on that.
@dataclass(eq=False)
class _Cluster:
    """Detections of one class that agree on a world position."""

    class_id: str
    position: tuple[float, float, float]
    hits: int
    best_score: float
    first_seen_sec: float
    last_seen_sec: float
    class_counts: dict[str, int] = field(default_factory=dict)

    def as_target(self) -> ConfirmedTarget:
        dominant = max(self.class_counts.items(), key=lambda item: item[1])[0]
        return ConfirmedTarget(
            class_id=dominant,
            position=self.position,
            hits=self.hits,
            best_score=self.best_score,
            first_seen_sec=self.first_seen_sec,
            last_seen_sec=self.last_seen_sec,
        )


class TargetAggregator:
    """Confirm targets from repeated, spatially consistent detections."""

    def __init__(
        self,
        *,
        min_score: float = DEFAULT_MIN_SCORE,
        min_hits: int = DEFAULT_MIN_HITS,
        cluster_radius_m: float = DEFAULT_CLUSTER_RADIUS_M,
        cluster_timeout_sec: float = DEFAULT_CLUSTER_TIMEOUT_SEC,
    ) -> None:
        if min_hits < 1:
            raise ValueError("min_hits must be at least 1")
        if cluster_radius_m <= 0.0:
            raise ValueError("cluster_radius_m must be positive")
        self._min_score = min_score
        self._min_hits = min_hits
        self._radius = cluster_radius_m
        self._timeout = cluster_timeout_sec
        self._clusters: list[_Cluster] = []
        self._confirmed: list[_Cluster] = []

    @property
    def confirmed(self) -> tuple[ConfirmedTarget, ...]:
        """Every target confirmed so far, most hits first."""
        return tuple(cluster.as_target() for cluster in
                     sorted(self._confirmed, key=lambda item: -item.hits))

    def add(self, detection: Detection) -> ConfirmedTarget | None:
        """Fold one detection in; return a target if this detection confirmed it."""
        if detection.world_position is None or detection.score < self._min_score:
            return None
        self._expire(detection.stamp_sec)
        cluster = self._nearest(detection)
        if cluster is None:
            cluster = _Cluster(
                class_id=detection.class_id,
                position=detection.world_position,
                hits=0,
                best_score=0.0,
                first_seen_sec=detection.stamp_sec,
                last_seen_sec=detection.stamp_sec,
            )
            self._clusters.append(cluster)
        cluster.hits += 1
        cluster.best_score = max(cluster.best_score, detection.score)
        cluster.last_seen_sec = detection.stamp_sec
        cluster.class_counts[detection.class_id] = cluster.class_counts.get(detection.class_id, 0) + 1
        # Running mean: every accepted detection pulls the estimate towards itself.
        weight = 1.0 / cluster.hits
        cluster.position = tuple(
            previous + weight * (new - previous)
            for previous, new in zip(cluster.position, detection.world_position)
        )
        if cluster.hits == self._min_hits:
            self._confirmed.append(cluster)
            return cluster.as_target()
        return None

    def _nearest(self, detection: Detection) -> _Cluster | None:
        assert detection.world_position is not None
        x, y, _ = detection.world_position
        best: _Cluster | None = None
        best_distance = self._radius
        for cluster in self._clusters:
            distance = hypot(cluster.position[0] - x, cluster.position[1] - y)
            if distance <= best_distance:
                best, best_distance = cluster, distance
        return best

    def _expire(self, now_sec: float) -> None:
        """Drop unconfirmed clusters that stopped accumulating.

        Without this, unrelated false positives spread over a long flight could
        eventually pile up into a confirmation.
        """
        self._clusters = [
            cluster for cluster in self._clusters
            if cluster in self._confirmed or now_sec - cluster.last_seen_sec <= self._timeout
        ]


class DetectSkill(Skill[Sequence[str], DetectionResult]):
    """Watch the detector stream for the requested target classes."""

    def __init__(self, node: Node, *, config: SkillRuntimeConfig) -> None:
        if config.detection is None:
            raise SkillExecutionError(
                "no detection configuration was loaded; pass detection_config_file to "
                "SkillRuntimeConfig.load before using DetectSkill")
        self._node = node
        self._config = config.detection
        self._classes: tuple[str, ...] = ()
        self._min_score = 0.0
        self._pending: dict[str, Detection] = {}
        self._arrival: list[tuple[float, str]] = []
        self._ready: list[Detection] = []
        self._frames = 0
        self._world_frames = 0
        self._last_stamp: float | None = None
        self._running = False

        reliable = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
        self._detections_sub = node.create_subscription(
            Detection2DArray, self._config.detections_topic, self._on_detections, reliable)
        self._world_sub = node.create_subscription(
            Detection3DArray, self._config.detections_world_topic, self._on_world, reliable)

    @property
    def name(self) -> str:
        return "detect"

    @property
    def detections_topic(self) -> str:
        return self._config.detections_topic

    @property
    def detections_world_topic(self) -> str:
        return self._config.detections_world_topic

    @property
    def classes(self) -> tuple[str, ...]:
        """Detector classes currently being watched for."""
        return self._classes

    def start(self, classes: Sequence[str], *, min_score: float = DEFAULT_MIN_SCORE) -> tuple[str, ...]:
        """Begin collecting detections of ``classes``; returns the expanded class list."""
        self._classes = expand_classes(classes)
        self._min_score = float(min_score)
        self._pending.clear()
        self._arrival.clear()
        self._ready.clear()
        self._frames = 0
        self._world_frames = 0
        self._last_stamp = None
        self._running = True
        return self._classes

    def stop(self) -> None:
        """Stop collecting. Buffered detections stay available for one final drain."""
        self._running = False

    def collect(self, *, join_grace_sec: float = JOIN_GRACE_SEC) -> tuple[Detection, ...]:
        """Drain the detections that have waited long enough for a world position."""
        self._flush(join_grace_sec)
        drained = tuple(self._ready)
        self._ready.clear()
        return drained

    @property
    def frames_observed(self) -> int:
        """Detector frames seen since :meth:`start`."""
        return self._frames

    @property
    def localizer_seen(self) -> bool:
        """Whether any world-frame detection frame arrived since :meth:`start`."""
        return self._world_frames > 0

    def call(
        self,
        request: Sequence[str],
        *,
        timeout_sec: float | None = 5.0,
        min_score: float = DEFAULT_MIN_SCORE,
        min_hits: int = DEFAULT_MIN_HITS,
        cluster_radius_m: float = DEFAULT_CLUSTER_RADIUS_M,
        stop_on_first_target: bool = True,
    ) -> DetectionResult:
        """Watch for ``request`` classes until a target is confirmed or time runs out.

        ``success`` means the detection pipeline ran and was observed, not that
        something was found; use :attr:`DetectionResult.found` for that.
        """
        classes = self.start(request, min_score=min_score)
        aggregator = TargetAggregator(
            min_score=min_score, min_hits=min_hits, cluster_radius_m=cluster_radius_m)
        collected: list[Detection] = []
        deadline = self._now() + float(timeout_sec if timeout_sec is not None else 5.0)
        try:
            while self._now() < deadline:
                rclpy.spin_once(self._node, timeout_sec=0.05)
                for detection in self.collect():
                    collected.append(detection)
                    aggregator.add(detection)
                if stop_on_first_target and aggregator.confirmed:
                    break
        finally:
            self.stop()
        collected.extend(self.collect(join_grace_sec=0.0))
        return self._result(classes, tuple(collected), aggregator.confirmed)

    # -- internals ----------------------------------------------------------
    def _result(
        self,
        classes: Sequence[str],
        detections: tuple[Detection, ...],
        targets: tuple[ConfirmedTarget, ...],
    ) -> DetectionResult:
        if self._frames == 0:
            return DetectionResult(
                success=False,
                message=(f"no detector frames on '{self._config.detections_topic}'; "
                         "is the detector running?"),
                detections=(), targets=(), frame_stamp_sec=None, frames_observed=0)
        message = _describe(detections, targets, classes)
        if detections and not self.localizer_seen:
            message += (f" (no world positions: nothing is publishing "
                        f"'{self._config.detections_world_topic}')")
        return DetectionResult(
            success=True,
            message=message,
            detections=detections,
            targets=targets,
            frame_stamp_sec=self._last_stamp,
            frames_observed=self._frames,
        )

    def _on_detections(self, message: Detection2DArray) -> None:
        if not self._running:
            return
        self._frames += 1
        stamp = _stamp_seconds(message.header.stamp)
        self._last_stamp = stamp
        arrival = self._now()
        for detection in message.detections:
            class_id, score = _best_hypothesis(detection)
            if class_id not in self._classes or score < self._min_score:
                continue
            identifier = detection.id or f"{class_id}@{stamp:.6f}#{len(self._pending)}"
            self._pending[identifier] = Detection(
                detection_id=identifier,
                class_id=class_id,
                score=score,
                bbox=(detection.bbox.center.position.x, detection.bbox.center.position.y,
                      detection.bbox.size_x, detection.bbox.size_y),
                stamp_sec=stamp,
                world_position=None,
            )
            self._arrival.append((arrival, identifier))

    def _on_world(self, message: Detection3DArray) -> None:
        if not self._running:
            return
        self._world_frames += 1
        for detection in message.detections:
            pending = self._pending.get(detection.id)
            if pending is None:
                continue
            centre = detection.bbox.center.position
            self._pending[detection.id] = Detection(
                detection_id=pending.detection_id,
                class_id=pending.class_id,
                score=pending.score,
                bbox=pending.bbox,
                stamp_sec=pending.stamp_sec,
                world_position=(centre.x, centre.y, centre.z),
            )

    def _flush(self, join_grace_sec: float) -> None:
        """Move detections older than the join grace period into the ready queue."""
        horizon = self._now() - max(join_grace_sec, 0.0)
        remaining: list[tuple[float, str]] = []
        for arrival, identifier in self._arrival:
            if arrival > horizon:
                remaining.append((arrival, identifier))
                continue
            detection = self._pending.pop(identifier, None)
            if detection is not None:
                self._ready.append(detection)
        self._arrival = remaining

    def _now(self) -> float:
        return self._node.get_clock().now().nanoseconds * 1e-9


def _best_hypothesis(detection) -> tuple[str, float]:
    if not detection.results:
        return ("", 0.0)
    best = max(detection.results, key=lambda result: result.hypothesis.score)
    return (best.hypothesis.class_id, float(best.hypothesis.score))


def _stamp_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _describe(
    detections: Sequence[Detection],
    targets: Sequence[ConfirmedTarget],
    classes: Sequence[str],
) -> str:
    """Human-readable summary, e.g. 'found 1 car at (11.9, 6.1); 42 detections'."""
    if targets:
        placed = ", ".join(
            f"{target.class_id} at ({target.position[0]:.1f}, {target.position[1]:.1f})"
            for target in targets)
        return f"found {len(targets)} target(s): {placed}; {len(detections)} matching detection(s)"
    if detections:
        counts: dict[str, int] = {}
        for detection in detections:
            counts[detection.class_id] = counts.get(detection.class_id, 0) + 1
        seen = ", ".join(f"{count} {name}" for name, count in sorted(counts.items()))
        return f"no target confirmed; {seen} detection(s) did not accumulate"
    return f"no {'/'.join(classes)} detected"
