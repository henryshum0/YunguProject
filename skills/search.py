"""Search skill: plan a coverage route, fly it, and report what was found.

This is the composite skill of the three. Search owns the mission and orchestrates
the other two: the coverage planner produces a route, ``NavigateSkill`` flies it,
``DetectSkill`` watches the detector stream while it flies, and the result says
whether the thing being looked for was actually found and where.

Two entry points, because two different things are useful:

``plan_and_queue``
    Command level. Plan a route, hand it to navigation, return immediately. This
    is what an operator UI wants behind a "plan and queue" button.
``call``
    Mission level. Everything above plus flying it out with detection running,
    blocking until the route is covered, a target is confirmed, or time runs out,
    and returning a :class:`SearchResult`.

``call`` drives the node's executor while it waits, like the other blocking skill
calls, so the node it is given must not be spun elsewhere at the same time.
"""

from __future__ import annotations

from collections.abc import Callable, Sequence
from dataclasses import dataclass
from math import hypot

from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from skills.base import Skill, SkillExecutionError
from skills.config import SkillRuntimeConfig
from skills.detect import (
    ConfirmedTarget,
    DEFAULT_CLUSTER_RADIUS_M,
    DEFAULT_MIN_HITS,
    DEFAULT_MIN_SCORE,
    DetectSkill,
    TargetAggregator,
)
from skills.navigate import NavigateSkill
from skills.primitives import PlanSearchPrimitive
from skills.primitives.plan_search import SearchArea


#: How long the offboard queue has to report the queued route before the mission
#: gives up waiting for it to start.
ROUTE_START_TIMEOUT_SEC = 15.0

#: Default ceiling on one search mission.
DEFAULT_MISSION_TIMEOUT_SEC = 900.0

#: Termination reasons reported by :class:`SearchResult`.
TERMINATION_REASONS = ("found", "covered", "timeout", "not_started", "aborted")

#: Seconds between progress reports while a mission runs.
PROGRESS_PERIOD_SEC = 0.5

#: A "find" is only trusted once the vehicle is within this far of the target it
#: places, in metres. A detection whose target lands further than this does not
#: count toward confirmation, so the mission does not stop on a distant, few-frame
#: sighting whose back-projected position is unreliable; the coverage sweep flies
#: on and confirms the target once it is close. None disables the gate.
DEFAULT_CONFIRM_WITHIN_M = 20.0

#: After the first confirmation, keep observing this long before stopping, so a
#: few more close-range frames average into the reported position. The vehicle is
#: passing near the target on its sweep at this point, so these are good views.
DEFAULT_SETTLE_SEC = 2.0


@dataclass(frozen=True, slots=True)
class SearchProgress:
    """A snapshot of a mission in flight, for an operator to watch.

    A coverage sweep takes minutes, so a caller that only ever sees the final
    result has nothing to show meanwhile. This is what the mission knows about
    itself at one instant.
    """

    waypoints_total: int
    #: Active plus pending waypoints, or ``None`` before the queue first reports.
    waypoints_remaining: int | None
    targets: tuple[ConfirmedTarget, ...]
    detector_frames: int
    elapsed_sec: float

    @property
    def waypoints_completed(self) -> int:
        if self.waypoints_remaining is None:
            return 0
        return min(max(self.waypoints_total - self.waypoints_remaining, 0), self.waypoints_total)


@dataclass(frozen=True, slots=True)
class SearchResult:
    """Structured outcome of a search mission."""

    success: bool
    message: str
    #: Whether a target of the requested classes was confirmed.
    found: bool
    targets: tuple[ConfirmedTarget, ...]
    #: Waypoints this mission planned, and how many of them were flown.
    waypoints_total: int
    waypoints_completed: int
    #: One of :data:`TERMINATION_REASONS`.
    termination_reason: str
    #: The planned route, so a caller can inspect or draw it.
    path: Path
    detector_frames: int

    @property
    def route_completion(self) -> float:
        """Fraction of the planned route that was flown, in ``[0, 1]``.

        This is route progress, not area coverage: the planner reports the route
        it designed for the requested area, not how much ground was observed.
        """
        if self.waypoints_total <= 0:
            return 0.0
        return min(self.waypoints_completed / self.waypoints_total, 1.0)


class SearchSkill(Skill[SearchArea, SearchResult]):
    """Plan a four-corner coverage area, fly it, and report confirmed targets."""

    def __init__(
        self,
        node: Node,
        *,
        config: SkillRuntimeConfig,
    ) -> None:
        self._node = node
        self._config = config
        self._search = PlanSearchPrimitive(node, config=config)
        self._navigate = NavigateSkill(node, config=config)
        self._detect: DetectSkill | None = None
        self._route = _RouteMonitor(node, config.offboard.queue_status_topic)
        # Only created when a detection config is present: the proximity gate
        # needs to know where the vehicle is. Navigation-only use never builds it.
        self._vehicle = (
            _VehicleMonitor(node, config.detection.vehicle_odom_topic)
            if config.detection is not None else None)

    @property
    def name(self) -> str:
        return "search"

    @property
    def service_name(self) -> str:
        return self._search.service_name

    @property
    def queue_service(self) -> str:
        return self._navigate.queue_service

    def plan_and_queue(
        self,
        request: SearchArea,
        *,
        timeout_sec: float | None = 30.0,
    ) -> Path:
        """Plan a search area and queue its ENU path, without waiting for the flight."""
        path = self._search.call(request, publish_result=True, timeout_sec=timeout_sec)
        self._navigate.call_poses(path.poses, timeout_sec=timeout_sec)
        return path

    def call(
        self,
        request: SearchArea,
        *,
        classes: Sequence[str] = ("vehicle",),
        timeout_sec: float | None = 30.0,
        mission_timeout_sec: float = DEFAULT_MISSION_TIMEOUT_SEC,
        stop_on_first_detection: bool = True,
        clear_existing: bool = True,
        min_score: float = DEFAULT_MIN_SCORE,
        min_hits: int = DEFAULT_MIN_HITS,
        cluster_radius_m: float = DEFAULT_CLUSTER_RADIUS_M,
        confirm_within_m: float | None = DEFAULT_CONFIRM_WITHIN_M,
        settle_sec: float = DEFAULT_SETTLE_SEC,
        stop_requested: Callable[[], bool] | None = None,
        on_progress: Callable[[SearchProgress], None] | None = None,
    ) -> SearchResult:
        """Run a complete search mission and return its structured outcome.

        The vehicle must already be airborne and holding: this queues a route,
        it does not take off. ``timeout_sec`` bounds the individual service calls;
        ``mission_timeout_sec`` bounds the flight itself.

        With ``stop_on_first_detection`` the route is aborted once a target is
        confirmed, which is the behaviour a "go find it" mission wants; set it
        false to always cover the whole area and report everything found.
        ``clear_existing`` empties the waypoint queue first so route progress
        refers to this mission only.

        A confirmation only counts detections placed within ``confirm_within_m``
        of the vehicle, so the mission does not stop on a distant, few-frame
        sighting whose position is unreliable — it keeps sweeping and confirms the
        target once the sweep brings it close, where the fix is trustworthy. After
        that first confirmation it keeps observing for ``settle_sec`` so a few more
        close-range frames average into the reported position. Set
        ``confirm_within_m`` to ``None`` to count detections at any range (the old
        stop-on-first-confirmation behaviour).

        A mission lasts minutes, so it can be watched and called off:
        ``on_progress`` is invoked about twice a second with a
        :class:`SearchProgress` snapshot, and ``stop_requested`` is polled every
        cycle — returning true aborts the route and ends the mission with reason
        ``aborted``.
        """
        detect = self._detector()
        if clear_existing:
            self._navigate.clear(timeout_sec=timeout_sec)

        watched = detect.start(classes, min_score=min_score)
        aggregator = TargetAggregator(
            min_score=min_score, min_hits=min_hits, cluster_radius_m=cluster_radius_m)
        path = self.plan_and_queue(request, timeout_sec=timeout_sec)
        total = len(path.poses)

        reason = "timeout"
        began = self._now()
        deadline = began + float(mission_timeout_sec)
        start_deadline = began + ROUTE_START_TIMEOUT_SEC
        started = False
        last_progress = 0.0
        #: When the first target was confirmed, so the settle window can run.
        confirmed_at: float | None = None
        try:
            while self._now() < deadline:
                rclpy.spin_once(self._node, timeout_sec=0.05)
                for detection in detect.collect():
                    if self._within_confirm_range(detection, confirm_within_m):
                        aggregator.add(detection)
                if on_progress is not None and self._now() - last_progress >= PROGRESS_PERIOD_SEC:
                    last_progress = self._now()
                    on_progress(SearchProgress(
                        waypoints_total=total,
                        waypoints_remaining=self._route.remaining,
                        targets=aggregator.confirmed,
                        detector_frames=detect.frames_observed,
                        elapsed_sec=self._now() - began,
                    ))
                if stop_requested is not None and stop_requested():
                    reason = "aborted"
                    break
                if stop_on_first_detection and aggregator.confirmed:
                    # Confirmed. Keep observing for the settle window before
                    # stopping, so more close-range frames refine the position.
                    if confirmed_at is None:
                        confirmed_at = self._now()
                    if self._now() - confirmed_at >= settle_sec:
                        reason = "found"
                        break
                    continue
                remaining = self._route.remaining
                if remaining is None:
                    continue
                if not started:
                    if remaining > 0:
                        started = True
                    elif self._now() > start_deadline:
                        reason = "not_started"
                        break
                    continue
                if remaining == 0:
                    reason = "covered"
                    break
        finally:
            detect.stop()
        for detection in detect.collect(join_grace_sec=0.0):
            if self._within_confirm_range(detection, confirm_within_m):
                aggregator.add(detection)

        # Read route progress before aborting: clearing empties the queue, and a
        # queue that is empty because the route was cancelled must not be
        # mistaken for a route that was flown to the end.
        remaining = self._route.remaining
        if reason in ("found", "aborted"):
            # Abort the rest of the route: the mission is over, either because
            # its question is answered or because the operator called it off.
            self._navigate.clear(timeout_sec=timeout_sec)

        if reason == "covered":
            completed = total
        elif remaining is None:
            # The offboard queue never reported the route, so none of it was flown.
            completed = 0
        else:
            completed = min(max(total - remaining, 0), total)
        targets = aggregator.confirmed
        return SearchResult(
            success=reason in ("found", "covered"),
            message=_describe(reason, targets, completed, total, watched, detect.frames_observed),
            found=bool(targets),
            targets=targets,
            waypoints_total=total,
            waypoints_completed=completed,
            termination_reason=reason,
            path=path,
            detector_frames=detect.frames_observed,
        )

    def _within_confirm_range(self, detection, confirm_within_m: float | None) -> bool:
        """Whether a detection is close enough to the vehicle to be trusted.

        A detection with no world position cannot be range-checked, so it is
        dropped rather than trusted. With the gate disabled, or before any vehicle
        pose has arrived, every located detection counts.
        """
        if confirm_within_m is None:
            return detection.world_position is not None
        if detection.world_position is None:
            return False
        pose = self._vehicle.latest if self._vehicle is not None else None
        if pose is None:
            return True
        return hypot(detection.world_position[0] - pose[0],
                     detection.world_position[1] - pose[1]) <= confirm_within_m

    def _detector(self) -> DetectSkill:
        if self._detect is None:
            if self._config.detection is None:
                raise SkillExecutionError(
                    "a search mission needs detection; pass detection_config_file to "
                    "SkillRuntimeConfig.load, or use plan_and_queue for the command-level "
                    "plan-and-queue behaviour")
            self._detect = DetectSkill(self._node, config=self._config)
        return self._detect

    def _now(self) -> float:
        return self._node.get_clock().now().nanoseconds * 1e-9


class _VehicleMonitor:
    """Track the latest ENU vehicle position, for the confirmation range gate."""

    def __init__(self, node: Node, topic: str) -> None:
        self._latest: tuple[float, float, float] | None = None
        node.create_subscription(
            Odometry, topic, self._on_odom,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT))

    @property
    def latest(self) -> tuple[float, float, float] | None:
        """Most recent ``(x, y, z)`` position, or ``None`` before the first fix."""
        return self._latest

    def _on_odom(self, message: Odometry) -> None:
        position = message.pose.pose.position
        self._latest = (position.x, position.y, position.z)


class _RouteMonitor:
    """Track how much of the queued route the offboard FSM still has to fly."""

    def __init__(self, node: Node, topic: str) -> None:
        self._remaining: int | None = None
        node.create_subscription(
            Path, topic, self._on_status,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))

    @property
    def remaining(self) -> int | None:
        """Active plus pending waypoints, or ``None`` before the first update."""
        return self._remaining

    def _on_status(self, message: Path) -> None:
        self._remaining = len(message.poses)


def _describe(
    reason: str,
    targets: Sequence[ConfirmedTarget],
    completed: int,
    total: int,
    classes: Sequence[str],
    detector_frames: int,
) -> str:
    progress = f"{completed}/{total} waypoint(s)"
    if reason == "found":
        placed = "; ".join(
            f"{target.class_id} at ({target.position[0]:.1f}, {target.position[1]:.1f}) "
            f"from {target.hits} detection(s)" for target in targets)
        return f"found {len(targets)} target(s) after {progress}: {placed}"
    if reason == "covered":
        if targets:
            placed = "; ".join(
                f"{target.class_id} at ({target.position[0]:.1f}, {target.position[1]:.1f})"
                for target in targets)
            return f"covered {progress} and found {len(targets)} target(s): {placed}"
        looked_for = "/".join(classes)
        if detector_frames == 0:
            return (f"covered {progress} but no detector frames arrived; "
                    f"no {looked_for} could have been found")
        return f"covered {progress}, no {looked_for} found"
    if reason == "aborted":
        found = f" with {len(targets)} target(s) found" if targets else " with nothing found"
        return f"stopped by the operator after {progress}{found}"
    if reason == "not_started":
        return (f"the offboard queue never reported the {total} queued waypoint(s); "
                "is the vehicle airborne and the offboard FSM running?")
    return f"timed out after {progress}" + (
        f" with {len(targets)} target(s) found" if targets else " with nothing found")
