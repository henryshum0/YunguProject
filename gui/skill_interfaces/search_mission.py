"""Search-mission skill tab: plan, fly, and watch detection as one mission.

This is the mission-level counterpart to the Coverage Search tab. It reuses the
four corners entered there, then runs :class:`~skills.SearchMissionSkill` to plan
a coverage route, queue it, and watch the detector while the route is flown.

Concurrency note. The host serializes flight and skill service calls on a single
worker thread, and takeoff/land are ROS services that spin the host node. A search
mission lasts minutes, so running it on that worker would block Land for the whole
flight. Instead the mission runs on its own thread with its own ROS node: the two
nodes spin independently, so **Land stays available while a mission is flying** and
neither call spins the other's node. Progress is marshalled back to the Tk thread
through a queue drained by a periodic poll; the result and any error are picked up
by that same poll when the mission thread finishes, so the worker thread never
touches Tk.
"""

from __future__ import annotations

from queue import Empty, Queue
from threading import Event, Thread
from typing import Any

import tkinter as tk
from tkinter import ttk

import rclpy

from gui.input_parser import parse_mission_timeout, parse_target_classes
from gui.map_view import Viewport
from gui.skill_interfaces.base import Point, SkillInterface
from gui.skill_interfaces.controller import format_progress, format_search_result
from gui.skill_interfaces.coverage_search import CoverageSearchSkillInterface
from skills import SearchMissionSkill, SearchProgress, SkillConfigError


#: How often the Tk thread drains mission progress and checks for completion.
POLL_PERIOD_MS = 200


class SearchMissionSkillInterface(SkillInterface):
    """Run and watch the complete search + navigation + detection mission."""

    tab_title = "Search mission"

    def __init__(self, host: Any) -> None:
        super().__init__(host)
        self.mission_classes = tk.StringVar(value="vehicle")
        self.mission_timeout_sec = tk.StringVar(value="900")
        self.mission_stop_on_first = tk.BooleanVar(value=True)
        self.show_truth_targets = tk.BooleanVar(value=False)
        self.mission_status = tk.StringVar(value="No mission running.")
        # Confirmed targets to draw on the map, as (class_id, x, y).
        self._targets: tuple[tuple[str, float, float], ...] = ()
        self._truth_targets: tuple[tuple[str, float, float], ...] = ()
        self._truth_field: Any = None
        self._truth_source: str | None = None
        # Mission-thread coordination.
        self._stop = Event()
        self._running = False
        self._progress: Queue[SearchProgress] = Queue()
        self._thread: Thread | None = None
        self._result: Any = None
        self._error: Exception | None = None
        self._run_button: ttk.Button | None = None
        self._stop_button: ttk.Button | None = None

    @property
    def map_instruction(self) -> str:
        return "Search mission uses the corners from the Coverage Search tab; no map click needed."

    def build(self, frame: ttk.Frame) -> None:
        frame.columnconfigure(0, weight=1)
        ttk.Label(frame, text=(
            "Runs search + navigation + detection as one mission: it plans a coverage route for "
            "the corners from the Coverage Search tab, queues it, and watches the detector while "
            "the route is flown. The vehicle must already be airborne and holding."),
            wraplength=520).grid(row=0, column=0, columnspan=4, sticky="w")

        parameters = ttk.LabelFrame(frame, text="Mission parameters", padding=6)
        parameters.grid(row=1, column=0, columnspan=4, pady=(10, 0), sticky="ew")
        ttk.Label(parameters, text="Look for").grid(row=0, column=0, padx=(0, 6), pady=3, sticky="w")
        ttk.Combobox(parameters, textvariable=self.mission_classes, width=24,
                     values=("vehicle", "person", "car", "pedestrian", "vehicle, person")).grid(
            row=0, column=1, padx=(0, 16), pady=3, sticky="w")
        ttk.Label(parameters, text="Mission timeout (s)").grid(
            row=0, column=2, padx=(0, 6), pady=3, sticky="w")
        ttk.Entry(parameters, textvariable=self.mission_timeout_sec, width=10).grid(
            row=0, column=3, pady=3, sticky="w")
        ttk.Checkbutton(parameters, text="Stop as soon as a target is confirmed",
                        variable=self.mission_stop_on_first).grid(
            row=1, column=0, columnspan=4, pady=(4, 0), sticky="w")
        ttk.Label(parameters, text=(
            "Group names person and vehicle cover the classes the detector confuses with one "
            "another; a comma-separated list of detector classes also works."),
            wraplength=520).grid(row=2, column=0, columnspan=4, pady=(4, 0), sticky="w")

        controls = ttk.Frame(frame)
        controls.grid(row=2, column=0, columnspan=4, pady=(10, 0), sticky="w")
        # Deliberately not host service buttons: the mission runs on its own
        # thread, so Run/Stop must stay independent of the shared worker.
        self._run_button = ttk.Button(controls, text="Run search mission", command=self._run_mission)
        self._run_button.grid(row=0, column=0, padx=(0, 8))
        self._stop_button = ttk.Button(controls, text="Stop mission", command=self._stop_mission)
        self._stop_button.grid(row=0, column=1, padx=(0, 8))
        self._stop_button.state(["disabled"])
        ttk.Button(controls, text="Clear found targets", command=self._clear_targets).grid(
            row=0, column=2, padx=(0, 8))
        ttk.Checkbutton(controls, text="Show ground truth on map",
                        variable=self.show_truth_targets,
                        command=self._on_show_truth_toggled).grid(row=0, column=3)

        ttk.Label(frame, textvariable=self.mission_status, wraplength=520,
                  foreground="#0d47a1").grid(row=3, column=0, columnspan=4, pady=(10, 0), sticky="w")
        ttk.Label(frame, text=(
            "Land stays available while a mission runs. Confirmed targets are drawn on the "
            "operations map in magenta; the reported position is what the mission estimated by "
            "back-projecting its detections. In simulation the result also lists the true "
            "position and the error, read from the ground-truth YAML."),
            wraplength=520).grid(row=4, column=0, columnspan=4, pady=(8, 0), sticky="w")

    # -- map interaction ----------------------------------------------------
    def on_map_click(self, point: Point) -> None:
        # The mission reuses the Coverage Search corners; a click here does nothing.
        self.host.map_status.set(
            "Search mission uses the Coverage Search corners. Set them on the Coverage Search tab.")

    def map_bounds_points(self) -> tuple[Point, ...]:
        points = [(x, y) for _class, x, y in self._targets]
        if self.show_truth_targets.get():
            points.extend((x, y) for _class, x, y in self._truth_targets)
        return tuple(points)

    def draw_map_overlay(self, canvas: tk.Canvas, viewport: Viewport) -> None:
        if self.show_truth_targets.get():
            for _class, x, y in self._truth_targets:
                cx, cy = viewport.to_canvas((x, y))
                canvas.create_rectangle(cx - 5, cy - 5, cx + 5, cy + 5,
                                        outline="#2e7d32", width=2)
        for class_id, x, y in self._targets:
            cx, cy = viewport.to_canvas((x, y))
            canvas.create_oval(cx - 6, cy - 6, cx + 6, cy + 6, fill="#d500f9", outline="white")
            canvas.create_text(cx + 8, cy - 8, anchor="sw", text=class_id, fill="#aa00c7")

    # -- mission control ----------------------------------------------------
    def _mission_corners(self) -> tuple[Point, ...]:
        """The four corners entered on the Coverage Search tab."""
        for interface in self.host._skill_interfaces:
            if isinstance(interface, CoverageSearchSkillInterface):
                return interface.current_corners()
        return ()

    def _run_mission(self) -> None:
        if self._running:
            return
        try:
            settings = self.host._settings()
            corners = self._mission_corners()
            if not corners:
                raise ValueError(
                    "set four search-area corners on the Coverage Search tab first")
            classes = parse_target_classes(self.mission_classes.get())
            mission_timeout = parse_mission_timeout(self.mission_timeout_sec.get())
        except (ValueError, SkillConfigError) as error:
            self.host._report_error(error)
            return
        if settings.config.detection is None:
            self.host._report_error(ValueError(
                self.host._detection_config_error
                or "no detection configuration loaded; set the detection config YAML"))
            return

        stop_on_first = bool(self.mission_stop_on_first.get())
        self._stop.clear()
        self._result = None
        self._error = None
        self._targets = ()
        while not self._progress.empty():
            self._progress.get_nowait()
        self._running = True
        self._run_button.state(["disabled"])
        self._stop_button.state(["!disabled"])
        self.mission_status.set(
            f"Mission starting: looking for {', '.join(classes)} over the selected area"
            f"{' (stops on the first confirmed target)' if stop_on_first else ' (full sweep)'}.")
        self.host._schedule_map_redraw()

        self._thread = Thread(
            target=self._mission_worker,
            args=(corners, classes, settings, mission_timeout, stop_on_first),
            name="skills-test-gui-mission",
            daemon=True,
        )
        self._thread.start()
        self.host.after(POLL_PERIOD_MS, self._poll)

    def _mission_worker(self, corners, classes, settings, mission_timeout, stop_on_first) -> None:
        """Run the mission on a dedicated ROS node (see the module docstring)."""
        node = rclpy.create_node("skills_test_gui_mission")
        try:
            skill = SearchMissionSkill(node, config=settings.config)
            self._result = skill.call(
                corners,
                classes=classes,
                timeout_sec=settings.timeout_sec,
                mission_timeout_sec=mission_timeout,
                stop_on_first_detection=stop_on_first,
                stop_requested=self._stop.is_set,
                on_progress=self._progress.put,
            )
        except Exception as error:  # noqa: BLE001 - reported to the operator on the Tk thread
            self._error = error
        finally:
            node.destroy_node()

    def _stop_mission(self) -> None:
        if not self._running:
            self.mission_status.set("No mission running.")
            return
        self._stop.set()
        self.mission_status.set("Stop requested; aborting the route...")

    def _clear_targets(self) -> None:
        self._targets = ()
        self.mission_status.set("Found targets cleared from the map.")
        self.host._schedule_map_redraw()

    def _poll(self) -> None:
        if getattr(self.host, "_closed", False):
            return
        progress = None
        while True:
            try:
                progress = self._progress.get_nowait()
            except Empty:
                break
        if progress is not None and self._running:
            self.mission_status.set(format_progress(progress))
            targets = tuple(
                (target.class_id, target.position[0], target.position[1])
                for target in progress.targets)
            if targets != self._targets:
                self._targets = targets
                self.host._schedule_map_redraw()

        if self._thread is not None and not self._thread.is_alive():
            self._finish_mission()
            return
        self.host.after(POLL_PERIOD_MS, self._poll)

    def _finish_mission(self) -> None:
        self._running = False
        self._stop.clear()
        self._thread = None
        self._run_button.state(["!disabled"])
        self._stop_button.state(["disabled"])
        if self._error is not None:
            error, self._error = self._error, None
            self.mission_status.set(f"Mission failed: {error}")
            self.host._report_error(error)
            return
        result = self._result
        if result is None:
            return
        self._targets = tuple(
            (target.class_id, target.position[0], target.position[1]) for target in result.targets)
        self.mission_status.set(result.message)
        self.host._set_result(format_search_result(result, self._truth_matches(result.targets)))
        self.host._schedule_map_redraw()

    # -- ground truth (simulation only) -------------------------------------
    def _on_show_truth_toggled(self) -> None:
        if self.show_truth_targets.get():
            self._load_truth_field()
        self.host._schedule_map_redraw()

    def _truth_matches(self, targets):
        """Pair each estimated target with the simulated ground truth, if available.

        Simulation only, and only for reading the result: the mission itself never
        sees the truth, so what it reports stays the estimate it actually made.
        """
        field = self._load_truth_field()
        if field is None or not targets:
            return None
        try:
            from detection.truth import match_to_truth

            return match_to_truth([target.position for target in targets], field)
        except Exception as error:  # noqa: BLE001 - a convenience, never required
            self.host._report_error(error)
            return None

    def _load_truth_field(self):
        """Load (and cache) the ground-truth targets named in the settings."""
        path = self.host.truth_targets_file.get().strip()
        if not path:
            self._truth_field, self._truth_source, self._truth_targets = None, None, ()
            return None
        if path == self._truth_source:
            return self._truth_field
        try:
            from detection.config import DetectionConfig, load_targets

            detection = DetectionConfig.load(self.host.detection_config_file.get().strip())
            field = load_targets(path, ground_z_m=detection.world.ground_z_m)
        except Exception as error:  # noqa: BLE001 - the truth file is optional
            self._truth_field, self._truth_source, self._truth_targets = None, path, ()
            self.mission_status.set(f"Ground truth unavailable: {error}")
            return None
        self._truth_field = field
        self._truth_source = path
        self._truth_targets = tuple(
            (target.class_id, target.position[0], target.position[1]) for target in field.targets)
        return field
