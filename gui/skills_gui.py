#!/usr/bin/python3
"""Desktop GUI for exercising the workspace navigation and search skills."""

from __future__ import annotations

import argparse
import sys
import tkinter as tk
from concurrent.futures import Future, ThreadPoolExecutor
from pathlib import Path
from queue import Empty, Queue
from threading import Event
from time import monotonic
from tkinter import filedialog, messagebox, ttk

WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
if str(WORKSPACE_ROOT) not in sys.path:
    sys.path.insert(0, str(WORKSPACE_ROOT))

import rclpy
from rclpy.node import Node

from gui.controller import (
    ConnectionSettings,
    SkillController,
    format_path,
    format_progress,
    format_search_result,
)
from gui.camera_view import CameraPreview
from gui.telemetry import OperationsTelemetry, QueueState, VehicleState
from skills import SearchProgress, SkillConfigError, SkillRuntimeConfig
from gui.input_parser import (
    parse_corners,
    parse_frame,
    parse_mission_timeout,
    parse_navigation_goal,
    parse_target_classes,
    parse_timeout,
    parse_waypoints,
)
from gui.map_view import (
    MapLoadError,
    PlannerMap,
    Viewport,
    bounds_for,
    heading_endpoint,
    load_planner_map,
    make_viewport,
    rectangle_from_clicks,
    route_points,
)


class SkillsTestGui(tk.Tk):
    """Tk application with one serialized worker for ROS service actions."""

    def __init__(self, node: Node) -> None:
        super().__init__()
        self.title("Yungu Skills Test GUI")
        # The camera is a native 640 px-wide feed.  Keep enough room for it
        # instead of allowing the skill controls to squeeze the sidebar.
        self.minsize(1880, 760)
        self._node = node
        self._controller = SkillController(node)
        self._worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix="skill-service")
        self._completed_actions: Queue[tuple[Future, object]] = Queue()
        self._service_buttons: list[ttk.Button] = []
        self._closed = False
        self._map_data: PlannerMap | None = None
        self._map_viewport: Viewport | None = None
        self._pending_map_click: tuple[float, float] | None = None
        self._route_preview: tuple[tuple[float, float], ...] = ()
        self._map_redraw_scheduled = False
        self._navigation_goal: tuple[float, float, float, float] | None = None
        self._vehicle_state: VehicleState | None = None
        self._queue_state: QueueState | None = None
        self._operations_telemetry: OperationsTelemetry | None = None
        self._camera_preview: CameraPreview | None = None
        self._camera_photo: tk.PhotoImage | None = None
        self._camera_raw_topic: str | None = None
        self._mission_stop = Event()
        self._mission_running = False
        self._mission_progress: Queue[SearchProgress] = Queue()
        self._mission_targets: tuple[tuple[str, float, float], ...] = ()
        self._truth_targets: tuple[tuple[str, float, float], ...] = ()
        self._truth_field = None
        self._truth_source: str | None = None
        self._detection_config_error: str | None = None
        self._build_variables()
        self._build_layout()
        self._load_map()
        self.protocol("WM_DELETE_WINDOW", self._close)
        self.after(50, self._poll_completed_actions)
        self.after(50, self._poll_camera_preview)
        self.after(100, self._poll_operations_telemetry)
        self.after(200, self._poll_mission_progress)
        self.after_idle(self._start_camera_preview)
        self.after_idle(self._start_operations_telemetry)

    def _build_variables(self) -> None:
        self.navigation_config_dir = tk.StringVar(
            value=str(WORKSPACE_ROOT / "src" / "navigation" / "config" / "offboard"))
        self.planner_config_file = tk.StringVar(
            value=str(WORKSPACE_ROOT / "src" / "search" / "config" / "yungu_planner.json"))
        self.detection_config_file = tk.StringVar(
            value=str(WORKSPACE_ROOT / "src" / "detection" / "config" / "detection.yaml"))
        # Simulation only: lets a finished mission be read against the true target
        # positions. Clear it on a real vehicle, where there is no such file.
        self.truth_targets_file = tk.StringVar(
            value=str(WORKSPACE_ROOT / "src" / "detection" / "config" / "targets.yaml"))
        self.camera_image_topic = tk.StringVar(value="/swan_gamma_v2/front_camera/image")
        self.vehicle_odometry_topic = tk.StringVar(value="/gz/ground_truth/odom")
        self.waypoint_queue_status_topic = tk.StringVar(value="/waypoint_buffer/status")
        self.timeout_sec = tk.StringVar(value="10")
        self.navigate_frame = tk.StringVar(value="ENU")
        self.mission_classes = tk.StringVar(value="vehicle")
        self.mission_timeout_sec = tk.StringVar(value="900")
        self.mission_stop_on_first = tk.BooleanVar(value=True)
        self.mission_status = tk.StringVar(value="No mission running.")
        self.show_truth_targets = tk.BooleanVar(value=False)
        self.corner_values = [(tk.StringVar(), tk.StringVar()) for _ in range(4)]
        self.map_file = tk.StringVar(
            value=str(WORKSPACE_ROOT / "src" / "search" / "config" / "yungu_map.json"))
        self.map_status = tk.StringVar()
        self.map_mode = tk.StringVar(value="Navigate: click selects one ENU goal")
        self.telemetry_status = tk.StringVar(value="Telemetry stopped.")
        self.navigation_goal_x = tk.StringVar()
        self.navigation_goal_y = tk.StringVar()
        self.navigation_goal_z = tk.StringVar(value="5.0")
        self.navigation_goal_heading = tk.StringVar(value="0")
        self.status = tk.StringVar(value="Ready. Source ROS and start the required nodes first.")
        for x_value, y_value in self.corner_values:
            x_value.trace_add("write", self._on_corner_value_changed)
            y_value.trace_add("write", self._on_corner_value_changed)
        for value in (
            self.navigation_goal_x,
            self.navigation_goal_y,
            self.navigation_goal_z,
            self.navigation_goal_heading,
        ):
            value.trace_add("write", self._on_navigation_goal_value_changed)

    def _build_layout(self) -> None:
        outer = ttk.Frame(self, padding=10)
        outer.grid(sticky="nsew")
        self.columnconfigure(0, weight=1)
        self.rowconfigure(0, weight=1)
        outer.columnconfigure(0, weight=1, minsize=1180)
        # 640 px preview + label-frame padding/border.  ``minsize`` is needed
        # because a zero-weight grid column may otherwise be compressed.
        outer.columnconfigure(1, weight=0, minsize=660)
        outer.rowconfigure(3, weight=1)

        settings = ttk.LabelFrame(outer, text="Connection settings", padding=8)
        settings.grid(row=0, column=0, sticky="ew")
        fields = [
            ("Navigation config directory", self.navigation_config_dir),
            ("Planner config JSON", self.planner_config_file),
            ("Detection config YAML", self.detection_config_file),
            ("Ground truth YAML (sim)", self.truth_targets_file),
            ("Camera image topic", self.camera_image_topic),
            ("Vehicle odometry topic", self.vehicle_odometry_topic),
            ("Queue status topic", self.waypoint_queue_status_topic),
            ("Timeout (s)", self.timeout_sec),
        ]
        for index, (label, variable) in enumerate(fields):
            row, column = divmod(index, 2)
            ttk.Label(settings, text=label).grid(row=row, column=column * 2, padx=(0, 6), pady=3, sticky="w")
            ttk.Entry(settings, textvariable=variable, width=38).grid(
                row=row, column=column * 2 + 1, padx=(0, 16), pady=3, sticky="ew")
        settings.columnconfigure(1, weight=1)
        settings.columnconfigure(3, weight=1)

        flight = ttk.LabelFrame(outer, text="Flight control", padding=8)
        flight.grid(row=1, column=0, pady=(8, 0), sticky="ew")
        ttk.Button(flight, text="Take off", command=self._takeoff).grid(row=0, column=0, padx=(0, 8))
        ttk.Button(flight, text="Land", command=self._land).grid(row=0, column=1)
        ttk.Label(flight, text="Both commands require confirmation and publish Bool(data=True).").grid(
            row=0, column=2, padx=16, sticky="w")

        work_area = ttk.PanedWindow(outer, orient=tk.HORIZONTAL)
        work_area.grid(row=2, column=0, pady=(8, 0), sticky="nsew")
        outer.rowconfigure(2, weight=1)
        self._notebook = ttk.Notebook(work_area)
        work_area.add(self._notebook, weight=1)
        self._build_navigate_tab(self._notebook)
        self._build_search_tab(self._notebook)
        self._build_mission_tab(self._notebook)
        self._notebook.bind("<<NotebookTabChanged>>", self._on_skill_tab_changed)
        operations = ttk.LabelFrame(work_area, text="Live operations map", padding=6)
        work_area.add(operations, weight=1)
        self._build_operations_map(operations)

        result = ttk.LabelFrame(outer, text="Status and planner result", padding=8)
        result.grid(row=3, column=0, pady=(8, 0), sticky="nsew")
        result.columnconfigure(0, weight=1)
        result.rowconfigure(1, weight=1)
        ttk.Label(result, textvariable=self.status, foreground="#155724", wraplength=850).grid(
            row=0, column=0, sticky="ew", pady=(0, 6))
        self.output = tk.Text(result, height=12, wrap="none", state="disabled")
        self.output.grid(row=1, column=0, sticky="nsew")
        scrollbar = ttk.Scrollbar(result, command=self.output.yview)
        scrollbar.grid(row=1, column=1, sticky="ns")
        self.output.configure(yscrollcommand=scrollbar.set)

        self._build_camera_pane(outer)

    def _build_navigate_tab(self, notebook: ttk.Notebook) -> None:
        tab = ttk.Frame(notebook, padding=10)
        notebook.add(tab, text="Navigate")
        tab.columnconfigure(0, weight=1)
        tab.rowconfigure(1, weight=1)
        ttk.Label(tab, text="One waypoint per line: x, y, z, heading_deg").grid(
            row=0, column=0, sticky="w")
        self.waypoint_text = tk.Text(tab, height=10, width=80)
        self.waypoint_text.grid(row=1, column=0, pady=6, sticky="nsew")
        self.waypoint_text.insert("1.0", "10, 10, 5, 0\n20, 10, 5, 90")
        controls = ttk.Frame(tab)
        controls.grid(row=2, column=0, sticky="w")
        ttk.Label(controls, text="Input frame").grid(row=0, column=0, padx=(0, 6))
        ttk.Combobox(controls, textvariable=self.navigate_frame, values=("ENU", "NED"),
                     width=8, state="readonly").grid(row=0, column=1, padx=(0, 8))
        self._service_button(controls, "Queue navigation", self._navigate).grid(row=0, column=2, padx=(0, 8))
        self._service_button(controls, "Clear route", self._clear_route).grid(row=0, column=3)

        goal_controls = ttk.LabelFrame(tab, text="Map-selected ENU goal", padding=6)
        goal_controls.grid(row=3, column=0, pady=(10, 0), sticky="ew")
        for index, (label, value) in enumerate((
            ("X", self.navigation_goal_x),
            ("Y", self.navigation_goal_y),
            ("Altitude", self.navigation_goal_z),
            ("Heading°", self.navigation_goal_heading),
        )):
            ttk.Label(goal_controls, text=label).grid(row=0, column=index * 2, padx=(0, 4), pady=3, sticky="w")
            ttk.Entry(goal_controls, textvariable=value, width=9).grid(
                row=0, column=index * 2 + 1, padx=(0, 8), pady=3)
        self._service_button(goal_controls, "Queue selected goal", self._queue_navigation_map_goal).grid(
            row=1, column=0, columnspan=4, pady=(3, 0), sticky="w")
        ttk.Button(goal_controls, text="Clear selection", command=self._clear_navigation_map_selection).grid(
            row=1, column=4, columnspan=4, padx=(8, 0), pady=(3, 0), sticky="w")
        ttk.Label(tab, text=("Select the Navigate tab, then click the persistent operations map. "
                             "The map is display-only and does not validate flight paths."),
                  wraplength=500).grid(row=4, column=0, pady=(8, 0), sticky="w")

    def _build_search_tab(self, notebook: ttk.Notebook) -> None:
        tab = ttk.Frame(notebook, padding=10)
        notebook.add(tab, text="Coverage search")
        ttk.Label(tab, text="Four search-area corners in ENU metres (x, y), or choose two map points.").grid(
            row=0, column=0, columnspan=3, sticky="w")
        for index, (x_value, y_value) in enumerate(self.corner_values, start=1):
            ttk.Label(tab, text=f"Corner {index}").grid(row=index, column=0, padx=(0, 8), pady=3, sticky="w")
            ttk.Entry(tab, textvariable=x_value, width=16).grid(row=index, column=1, padx=(0, 6), pady=3)
            ttk.Entry(tab, textvariable=y_value, width=16).grid(row=index, column=2, pady=3)
        controls = ttk.Frame(tab)
        controls.grid(row=5, column=0, columnspan=3, pady=(8, 0), sticky="w")
        self._service_button(controls, "Plan only", self._plan_search).grid(row=0, column=0, padx=(0, 8))
        self._service_button(controls, "Plan and queue", self._search_and_queue).grid(row=0, column=1, padx=(0, 8))
        ttk.Button(controls, text="Reset selection", command=self._reset_map_selection).grid(row=0, column=2)

        ttk.Label(tab, text=("Select the Coverage Search tab, then use two clicks on the persistent "
                             "operations map to fill SW, SE, NE, NW."), wraplength=500).grid(
            row=6, column=0, columnspan=3, pady=(10, 0), sticky="w")

    def _build_mission_tab(self, notebook: ttk.Notebook) -> None:
        """The complete skill: plan a route, fly it, and watch detection while it flies."""
        tab = ttk.Frame(notebook, padding=10)
        notebook.add(tab, text="Search mission")
        tab.columnconfigure(0, weight=1)
        ttk.Label(tab, text=(
            "Runs search + navigation + detection as one mission: it plans a coverage route for "
            "the corners from the Coverage Search tab, queues it, and watches the detector while "
            "the route is flown. The vehicle must already be airborne and holding."),
            wraplength=520).grid(row=0, column=0, columnspan=4, sticky="w")

        parameters = ttk.LabelFrame(tab, text="Mission parameters", padding=6)
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

        controls = ttk.Frame(tab)
        controls.grid(row=2, column=0, columnspan=4, pady=(10, 0), sticky="w")
        self._mission_run_button = self._service_button(
            controls, "Run search mission", self._run_search_mission)
        self._mission_run_button.grid(row=0, column=0, padx=(0, 8))
        # Deliberately not a service button: it must stay usable while the
        # mission occupies the worker, which is the whole point of a stop.
        self._mission_stop_button = ttk.Button(controls, text="Stop mission",
                                               command=self._stop_search_mission)
        self._mission_stop_button.grid(row=0, column=1, padx=(0, 8))
        self._mission_stop_button.state(["disabled"])
        ttk.Button(controls, text="Clear found targets",
                   command=self._clear_mission_targets).grid(row=0, column=2, padx=(0, 8))
        ttk.Checkbutton(controls, text="Show ground truth on map",
                        variable=self.show_truth_targets,
                        command=self._on_show_truth_toggled).grid(row=0, column=3)

        ttk.Label(tab, textvariable=self.mission_status, wraplength=520,
                  foreground="#0d47a1").grid(row=3, column=0, columnspan=4, pady=(10, 0), sticky="w")
        ttk.Label(tab, text=(
            "Land stays available while a mission runs. Confirmed targets are drawn on the "
            "operations map in magenta; the reported position is what the mission estimated by "
            "back-projecting its detections. In simulation the result also lists the true "
            "position and the error, read from the ground-truth YAML."),
            wraplength=520).grid(row=4, column=0, columnspan=4, pady=(8, 0), sticky="w")

    def _build_operations_map(self, panel: ttk.LabelFrame) -> None:
        panel.columnconfigure(0, weight=1)
        panel.rowconfigure(3, weight=1)
        ttk.Label(panel, text="Map JSON").grid(row=0, column=0, sticky="w")
        ttk.Entry(panel, textvariable=self.map_file, width=52).grid(
            row=1, column=0, sticky="ew", padx=(0, 6))
        controls = ttk.Frame(panel)
        controls.grid(row=1, column=1, sticky="e")
        ttk.Button(controls, text="Browse…", command=self._browse_map).grid(row=0, column=0, padx=(0, 5))
        ttk.Button(controls, text="Reload", command=self._load_map).grid(row=0, column=1)
        ttk.Button(controls, text="Reconnect telemetry", command=self._start_operations_telemetry).grid(
            row=0, column=2, padx=(8, 0))
        ttk.Label(panel, textvariable=self.map_mode, wraplength=620).grid(
            row=2, column=0, columnspan=2, pady=(6, 2), sticky="w")
        self.map_canvas = tk.Canvas(panel, width=620, height=410, background="#f8f9fa",
                                    highlightthickness=1, highlightbackground="#a0a0a0",
                                    cursor="crosshair")
        self.map_canvas.grid(row=3, column=0, columnspan=2, sticky="nsew")
        self.map_canvas.bind("<Button-1>", self._on_operations_map_click)
        self.map_canvas.bind("<Configure>", self._on_map_resize)
        ttk.Label(panel, textvariable=self.telemetry_status, wraplength=620).grid(
            row=4, column=0, columnspan=2, pady=(4, 0), sticky="w")
        ttk.Label(panel, textvariable=self.map_status, wraplength=620).grid(
            row=5, column=0, columnspan=2, pady=(2, 0), sticky="w")

    def _build_camera_pane(self, outer: ttk.Frame) -> None:
        pane = ttk.LabelFrame(outer, text="Front camera", padding=8, width=660)
        pane.grid(row=0, column=1, rowspan=4, padx=(10, 0), sticky="nsew")
        pane.columnconfigure(0, weight=1)
        pane.rowconfigure(2, weight=1)
        ttk.Label(
            pane,
            text=("Live preview remains visible while changing skill tabs. Set the topic in Connection "
                  "settings, then reconnect if it changes."),
            wraplength=390,
        ).grid(row=0, column=0, sticky="w")
        controls = ttk.Frame(pane)
        controls.grid(row=1, column=0, pady=(8, 6), sticky="w")
        ttk.Button(controls, text="Start / reconnect preview", command=self._start_camera_preview).grid(
            row=0, column=0, padx=(0, 8))
        ttk.Button(controls, text="Stop preview", command=self._stop_camera_preview).grid(
            row=0, column=1, padx=(0, 8))
        self._overlay_button = ttk.Button(
            controls, text="Show detection boxes", command=self._toggle_detection_overlay)
        self._overlay_button.grid(row=0, column=2)
        self.camera_status = tk.StringVar(value="Preview stopped.")
        ttk.Label(controls, textvariable=self.camera_status, wraplength=220).grid(
            row=1, column=0, columnspan=2, pady=(5, 0), sticky="w")
        preview_frame = tk.Frame(pane, width=640, height=480, background="#202020")
        preview_frame.grid(row=2, column=0, sticky="nsew")
        preview_frame.grid_propagate(False)
        self.camera_label = tk.Label(
            preview_frame, text="Starting camera preview...", background="#202020",
            foreground="#f0f0f0", anchor="center")
        self.camera_label.pack(fill="both", expand=True)

    def _service_button(self, parent: tk.Misc, text: str, command) -> ttk.Button:
        button = ttk.Button(parent, text=text, command=command)
        self._service_buttons.append(button)
        return button

    def _settings(self) -> ConnectionSettings:
        timeout = parse_timeout(self.timeout_sec.get())
        navigation_config_dir = self.navigation_config_dir.get().strip()
        planner_config_file = self.planner_config_file.get().strip()
        detection_config_file = self.detection_config_file.get().strip() or None
        if not navigation_config_dir:
            raise ValueError("navigation config directory must not be empty")
        if not planner_config_file:
            raise ValueError("planner config JSON must not be empty")
        try:
            config = SkillRuntimeConfig.load(
                navigation_config_dir, planner_config_file, detection_config_file)
        except SkillConfigError as error:
            if detection_config_file is None:
                raise
            # Detection is only needed by the mission. Navigation and coverage
            # planning must keep working without it, so fall back and let the
            # mission report the exact reason it cannot run.
            config = SkillRuntimeConfig.load(navigation_config_dir, planner_config_file)
            self._detection_config_error = str(error)
        else:
            self._detection_config_error = None
        return ConnectionSettings(config=config, timeout_sec=timeout)

    def _start_camera_preview(self) -> None:
        topic = self.camera_image_topic.get().strip()
        try:
            self._stop_camera_preview(update_status=False)
            self._camera_preview = CameraPreview(topic)
        except Exception as error:
            self.camera_status.set(f"Preview error: {error}")
            self._report_error(error)
            return
        self.camera_status.set(f"Waiting for images on {topic}...")
        self.camera_label.configure(image="", text="Waiting for camera frames...")

    def _toggle_detection_overlay(self) -> None:
        """Switch the preview between the raw camera and the detector's overlay.

        The overlay is published by the detection layer, so its topic comes from
        the detection configuration rather than being spelled out here.
        """
        if self._camera_raw_topic is not None:
            topic, self._camera_raw_topic = self._camera_raw_topic, None
            self.camera_image_topic.set(topic)
            self._overlay_button.configure(text="Show detection boxes")
            self._start_camera_preview()
            return
        try:
            settings = self._settings()
        except (ValueError, SkillConfigError) as error:
            self._report_error(error)
            return
        if settings.config.detection is None:
            self._report_error(ValueError(
                self._detection_config_error
                or "no detection configuration loaded; set the detection config YAML"))
            return
        self._camera_raw_topic = self.camera_image_topic.get()
        self.camera_image_topic.set(settings.config.detection.image_overlay_topic)
        self._overlay_button.configure(text="Show raw camera")
        self._start_camera_preview()

    def _stop_camera_preview(self, *, update_status: bool = True) -> None:
        preview, self._camera_preview = self._camera_preview, None
        if preview is not None:
            preview.close()
        self._camera_photo = None
        self.camera_label.configure(image="", text="Camera preview stopped.")
        if update_status:
            self.camera_status.set("Preview stopped.")

    def _poll_camera_preview(self) -> None:
        if self._closed:
            return
        preview = self._camera_preview
        if preview is not None:
            error = preview.latest_error()
            if error is not None:
                self.camera_status.set(f"Preview error: {error}")
            frame = preview.latest_frame()
            if frame is not None:
                try:
                    photo = tk.PhotoImage(data=frame.ppm_bytes(), format="PPM")
                except tk.TclError as image_error:
                    self.camera_status.set(f"Preview display error: {image_error}")
                    self.after(50, self._poll_camera_preview)
                    return
                self._camera_photo = photo
                self.camera_label.configure(image=self._camera_photo, text="")
                self.camera_status.set(
                    f"Receiving {frame.width}x{frame.height} RGB frames on {preview.topic}.")
        self.after(50, self._poll_camera_preview)

    def _takeoff(self) -> None:
        self._confirm_and_publish("Take off", "Publish a takeoff command to the offboard FSM?", "takeoff")

    def _land(self) -> None:
        self._confirm_and_publish("Land", "Publish a landing command to the offboard FSM?", "land")

    def _confirm_and_publish(self, title: str, prompt: str, operation: str) -> None:
        if not messagebox.askyesno(title, prompt, parent=self):
            self.status.set(f"{title} cancelled.")
            return
        try:
            settings = self._settings()
            getattr(self._controller, operation)(settings)
        except Exception as error:
            self._report_error(error)
            return
        self.status.set(f"{title} command published.")

    def _navigate(self) -> None:
        try:
            settings = self._settings()
            waypoints = parse_waypoints(self.waypoint_text.get("1.0", "end"))
            frame = parse_frame(self.navigate_frame.get())
        except ValueError as error:
            self._report_error(error)
            return
        self._run_service_action(
            "Queueing navigation route...",
            lambda: self._controller.navigate(waypoints, frame=frame, settings=settings),
            lambda count: self._set_result(f"Queued {count} navigation waypoint(s)."),
        )

    def _queue_navigation_map_goal(self) -> None:
        try:
            settings = self._settings()
            goal = parse_navigation_goal(
                self.navigation_goal_x.get(),
                self.navigation_goal_y.get(),
                self.navigation_goal_z.get(),
                self.navigation_goal_heading.get(),
            )
        except ValueError as error:
            self._report_error(error)
            return
        self._run_service_action(
            "Queueing selected map goal...",
            lambda: self._controller.navigate((goal,), frame="enu", settings=settings),
            lambda count: self._show_queued_navigation_goal(goal, count),
        )

    def _show_queued_navigation_goal(self, goal: tuple[float, float, float, float], count: int) -> None:
        self._set_result(
            f"Queued {count} selected map goal(s): x={goal[0]:.2f}, y={goal[1]:.2f}, "
            f"z={goal[2]:.2f}, yaw={goal[3]:.1f} deg.")

    def _clear_route(self) -> None:
        try:
            settings = self._settings()
        except ValueError as error:
            self._report_error(error)
            return
        self._run_service_action(
            "Clearing active and queued route...",
            lambda: self._controller.clear(settings),
            lambda count: self._set_result(f"Cleared {count} active/queued waypoint(s)."),
        )

    def _plan_search(self) -> None:
        self._run_search_action(queue=False)

    def _search_and_queue(self) -> None:
        self._run_search_action(queue=True)

    def _run_search_action(self, *, queue: bool) -> None:
        try:
            settings = self._settings()
            corners = parse_corners(tuple((x.get(), y.get()) for x, y in self.corner_values))
        except ValueError as error:
            self._report_error(error)
            return
        label = "Planning and queueing coverage route..." if queue else "Planning coverage route..."
        action = self._controller.search_and_queue if queue else self._controller.plan_search
        prefix = "Planned and queued" if queue else "Planned"
        self._run_service_action(
            label,
            lambda: action(corners, settings=settings),
            lambda path: self._show_search_result(prefix, path),
        )

    def _run_search_mission(self) -> None:
        try:
            settings = self._settings()
            corners = parse_corners(tuple((x.get(), y.get()) for x, y in self.corner_values))
            classes = parse_target_classes(self.mission_classes.get())
            mission_timeout = parse_mission_timeout(self.mission_timeout_sec.get())
        except (ValueError, SkillConfigError) as error:
            self._report_error(error)
            return
        if settings.config.detection is None:
            self._report_error(ValueError(
                self._detection_config_error
                or "no detection configuration loaded; set the detection config YAML"))
            return

        stop_on_first = bool(self.mission_stop_on_first.get())
        # Land must stay usable for the whole flight, and the worker thread is
        # about to start spinning this node, so create its publisher now.
        self._controller.prepare_flight_commands(settings)
        self._mission_stop.clear()
        self._mission_running = True
        self._mission_targets = ()
        while not self._mission_progress.empty():
            self._mission_progress.get_nowait()
        self._mission_stop_button.state(["!disabled"])
        self.mission_status.set(
            f"Mission starting: looking for {', '.join(classes)} over the selected area"
            f"{' (stops on the first confirmed target)' if stop_on_first else ' (full sweep)'}.")
        self._schedule_map_redraw()
        self._run_service_action(
            "Running search mission...",
            lambda: self._controller.run_search_mission(
                corners,
                classes=classes,
                settings=settings,
                mission_timeout_sec=mission_timeout,
                stop_on_first_detection=stop_on_first,
                stop_requested=self._mission_stop.is_set,
                on_progress=self._mission_progress.put,
            ),
            self._show_mission_result,
        )

    def _stop_search_mission(self) -> None:
        if not self._mission_running:
            self.mission_status.set("No mission running.")
            return
        self._mission_stop.set()
        self.mission_status.set("Stop requested; aborting the route...")

    def _clear_mission_targets(self) -> None:
        self._mission_targets = ()
        self.mission_status.set("Found targets cleared from the map.")
        self._schedule_map_redraw()

    def _show_mission_result(self, result: object) -> None:
        self._mission_finished()
        self._mission_targets = tuple(
            (target.class_id, target.position[0], target.position[1]) for target in result.targets)
        self.mission_status.set(result.message)
        self._set_result(format_search_result(result, self._truth_matches(result.targets)))
        self._schedule_map_redraw()

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
            self._report_error(error)
            return None

    def _load_truth_field(self):
        """Load (and cache) the ground-truth targets named in the settings."""
        path = self.truth_targets_file.get().strip()
        if not path:
            self._truth_field, self._truth_source, self._truth_targets = None, None, ()
            return None
        if path == self._truth_source:
            return self._truth_field
        try:
            from detection.config import DetectionConfig, load_targets

            detection = DetectionConfig.load(self.detection_config_file.get().strip())
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

    def _on_show_truth_toggled(self) -> None:
        if self.show_truth_targets.get():
            self._load_truth_field()
        self._schedule_map_redraw()

    def _mission_finished(self) -> None:
        self._mission_running = False
        self._mission_stop.clear()
        self._mission_stop_button.state(["disabled"])

    def _poll_mission_progress(self) -> None:
        if self._closed:
            return
        progress = None
        while True:
            try:
                progress = self._mission_progress.get_nowait()
            except Empty:
                break
        if progress is not None and self._mission_running:
            self.mission_status.set(format_progress(progress))
            targets = tuple(
                (target.class_id, target.position[0], target.position[1])
                for target in progress.targets)
            if targets != self._mission_targets:
                self._mission_targets = targets
                self._schedule_map_redraw()
        self.after(200, self._poll_mission_progress)

    def _on_skill_tab_changed(self, _event: tk.Event) -> None:
        index = self._notebook.index("current")
        if index == 0:
            self.map_mode.set("Navigate mode: one click selects an ENU goal.")
        elif index == 1:
            self.map_mode.set("Coverage Search mode: two clicks select an ENU rectangle.")
        else:
            self.map_mode.set(
                "Search mission mode: two clicks select the ENU rectangle the mission searches.")
        self._schedule_map_redraw()

    def _on_operations_map_click(self, event: tk.Event) -> None:
        if self._map_viewport is None:
            return
        if self._notebook.index("current") == 0:
            self._on_navigation_map_click(event)
        else:
            self._on_map_click(event)

    def _on_navigation_map_click(self, event: tk.Event) -> None:
        if self._map_viewport is None:
            return
        x, y = self._map_viewport.to_enu((float(event.x), float(event.y)))
        self.navigation_goal_x.set(f"{x:.3f}")
        self.navigation_goal_y.set(f"{y:.3f}")
        self._navigation_goal = self._current_navigation_goal()
        self.map_status.set(
            f"Selected ENU goal ({x:.2f}, {y:.2f}). Review altitude/heading, then queue it.")
        self._schedule_map_redraw()

    def _on_navigation_goal_value_changed(self, *_args: str) -> None:
        self._navigation_goal = self._current_navigation_goal()
        self._schedule_map_redraw()

    def _current_navigation_goal(self) -> tuple[float, float, float, float] | None:
        try:
            return parse_navigation_goal(
                self.navigation_goal_x.get(),
                self.navigation_goal_y.get(),
                self.navigation_goal_z.get(),
                self.navigation_goal_heading.get(),
            )
        except ValueError:
            return None

    def _clear_navigation_map_selection(self) -> None:
        self.navigation_goal_x.set("")
        self.navigation_goal_y.set("")
        self._navigation_goal = None
        self.map_status.set("Navigation goal selection cleared.")
        self._schedule_map_redraw()

    def _draw_navigation_goal(self, goal: tuple[float, float, float, float], color: str, label: str) -> None:
        assert self._map_viewport is not None
        x, y, _altitude, heading = goal
        span = max(
            self._map_viewport.bounds.max_x - self._map_viewport.bounds.min_x,
            self._map_viewport.bounds.max_y - self._map_viewport.bounds.min_y,
            1.0,
        )
        endpoint = heading_endpoint((x, y), heading, max(1.0, span * 0.05))
        start_x, start_y = self._map_viewport.to_canvas((x, y))
        end_x, end_y = self._map_viewport.to_canvas(endpoint)
        self.map_canvas.create_line(start_x, start_y, end_x, end_y, fill=color, width=2.5, arrow=tk.LAST)
        self.map_canvas.create_oval(start_x - 5, start_y - 5, start_x + 5, start_y + 5,
                                    fill=color, outline="white")
        self.map_canvas.create_text(
            start_x + 8, start_y - 8, anchor="sw", fill=color,
            text=f"{label}: z={goal[2]:.1f}, yaw={heading:.0f}°")

    def _browse_map(self) -> None:
        initial = Path(self.map_file.get()).expanduser()
        selected = filedialog.askopenfilename(
            parent=self,
            title="Select a coverage planner map JSON",
            initialdir=str(initial.parent if initial.parent.is_dir() else WORKSPACE_ROOT),
            initialfile=initial.name,
            filetypes=(("JSON files", "*.json"), ("All files", "*")),
        )
        if selected:
            self._load_map(selected)

    def _load_map(self, path: str | None = None) -> None:
        candidate = path or self.map_file.get().strip()
        try:
            map_data = load_planner_map(candidate)
        except (MapLoadError, ValueError) as error:
            if self._map_data is not None:
                self.map_file.set(str(self._map_data.source))
            self.map_status.set(f"Map load failed: {error}")
            self._report_error(error)
            return
        self._map_data = map_data
        self.map_file.set(str(map_data.source))
        self._pending_map_click = None
        self._route_preview = ()
        self.map_status.set(
            f"Loaded {map_data.source.name}: {len(map_data.occupied_areas)} occupied area(s).")
        self._schedule_map_redraw()

    def _on_map_click(self, event: tk.Event) -> None:
        if self._map_viewport is None:
            return
        point = self._map_viewport.to_enu((float(event.x), float(event.y)))
        if self._pending_map_click is None:
            self._pending_map_click = point
            self.map_status.set(
                f"First corner: ({point[0]:.2f}, {point[1]:.2f}). Click the opposite corner.")
            self._schedule_map_redraw()
            return
        try:
            corners = rectangle_from_clicks(self._pending_map_click, point)
        except ValueError as error:
            self._pending_map_click = None
            self.map_status.set(str(error))
            self._schedule_map_redraw()
            return
        self._pending_map_click = None
        self._set_corners(corners)
        self.map_status.set("Search rectangle selected: SW, SE, NE, NW populated in the corner fields.")

    def _on_map_resize(self, _event: tk.Event) -> None:
        self._schedule_map_redraw()

    def _on_corner_value_changed(self, *_args: str) -> None:
        self._schedule_map_redraw()

    def _schedule_map_redraw(self) -> None:
        if self._map_redraw_scheduled or not hasattr(self, "map_canvas"):
            return
        self._map_redraw_scheduled = True
        self.after_idle(self._draw_map)

    def _draw_map(self) -> None:
        self._map_redraw_scheduled = False
        canvas = self.map_canvas
        canvas.delete("all")
        if self._map_data is None:
            canvas.create_text(12, 12, anchor="nw", text="Load a planner map JSON to visualize it.")
            return
        corners = self._current_corners()
        overlays = [self._route_preview]
        if corners:
            overlays.append(corners)
        if self._pending_map_click is not None:
            overlays.append((self._pending_map_click,))
        selected_goal = self._current_navigation_goal()
        if selected_goal is not None:
            overlays.append(((selected_goal[0], selected_goal[1]),))
        if self._vehicle_state is not None:
            overlays.append(((self._vehicle_state.x, self._vehicle_state.y),))
        if self._queue_state is not None:
            overlays.append(self._queue_state.points)
        if self._mission_targets:
            overlays.append(tuple((x, y) for _class_id, x, y in self._mission_targets))
        if self.show_truth_targets.get() and self._truth_targets:
            overlays.append(tuple((x, y) for _class_id, x, y in self._truth_targets))
        width = max(float(canvas.winfo_width()), 100.0)
        height = max(float(canvas.winfo_height()), 100.0)
        self._map_viewport = make_viewport(bounds_for(self._map_data, *overlays), width, height)

        for area in self._map_data.occupied_areas:
            canvas.create_polygon(
                self._canvas_coordinates(area.points), fill="#e57373", outline="#9f2b2b", width=1.5)
        origin_x, origin_y = self._map_viewport.to_canvas(self._map_data.origin)
        canvas.create_oval(origin_x - 5, origin_y - 5, origin_x + 5, origin_y + 5,
                           fill="#202020", outline="white", width=1)
        canvas.create_text(origin_x + 8, origin_y - 8, anchor="sw", text="origin", fill="#202020")
        if corners:
            canvas.create_line(
                self._canvas_coordinates((*corners, corners[0])), fill="#009688", width=2.5)
            for index, point in enumerate(corners, start=1):
                x, y = self._map_viewport.to_canvas(point)
                canvas.create_oval(x - 4, y - 4, x + 4, y + 4, fill="#009688", outline="white")
                canvas.create_text(x + 6, y - 6, anchor="sw", text=str(index), fill="#00695c")
        if self._pending_map_click is not None:
            x, y = self._map_viewport.to_canvas(self._pending_map_click)
            canvas.create_oval(x - 5, y - 5, x + 5, y + 5, fill="#ff9800", outline="white")
        if selected_goal is not None:
            self._draw_navigation_goal(selected_goal, "#00838f", "selected")
        if self._route_preview:
            if len(self._route_preview) > 1:
                canvas.create_line(self._canvas_coordinates(self._route_preview), fill="#1565c0", width=2.5)
            for point in self._route_preview:
                x, y = self._map_viewport.to_canvas(point)
                canvas.create_oval(x - 3, y - 3, x + 3, y + 3, fill="#1565c0", outline="white")
        if self._queue_state is not None and self._queue_state.points:
            queue_points = self._queue_state.points
            if len(queue_points) > 1:
                canvas.create_line(self._canvas_coordinates(queue_points), fill="#7e57c2", width=2.5)
            active_x, active_y = self._map_viewport.to_canvas(queue_points[0])
            canvas.create_oval(active_x - 6, active_y - 6, active_x + 6, active_y + 6,
                               fill="#ff7043", outline="white", width=1.5)
            canvas.create_text(active_x + 8, active_y - 8, anchor="sw", text="active", fill="#d84315")
            for point in queue_points[1:]:
                x, y = self._map_viewport.to_canvas(point)
                canvas.create_oval(x - 4, y - 4, x + 4, y + 4, fill="#7e57c2", outline="white")
        if self.show_truth_targets.get():
            for class_id, x, y in self._truth_targets:
                self._draw_truth_target(class_id, x, y)
        for class_id, x, y in self._mission_targets:
            self._draw_found_target(class_id, x, y)
        if self._vehicle_state is not None:
            self._draw_vehicle(self._vehicle_state)
        canvas.create_text(
            8, 8, anchor="nw", fill="#303030",
            text=(f"{self._map_data.source.name} | red: occupied | green: search area | "
                  "blue: planned route | orange: active queue | purple: pending queue | "
                  "magenta: found target | grey: ground truth | black: vehicle"),
        )

    def _draw_truth_target(self, class_id: str, x: float, y: float) -> None:
        """Mark where a target really is, to read the estimate against."""
        assert self._map_viewport is not None
        canvas_x, canvas_y = self._map_viewport.to_canvas((x, y))
        self.map_canvas.create_polygon(
            canvas_x, canvas_y - 9, canvas_x + 9, canvas_y, canvas_x, canvas_y + 9,
            canvas_x - 9, canvas_y, fill="", outline="#546e7a", width=2)
        self.map_canvas.create_text(
            canvas_x + 11, canvas_y + 11, anchor="nw", fill="#546e7a",
            text=f"truth {class_id}")

    def _draw_found_target(self, class_id: str, x: float, y: float) -> None:
        """Mark a target the mission confirmed, in the frame the map already uses."""
        assert self._map_viewport is not None
        canvas_x, canvas_y = self._map_viewport.to_canvas((x, y))
        self.map_canvas.create_polygon(
            canvas_x, canvas_y - 8, canvas_x + 8, canvas_y, canvas_x, canvas_y + 8,
            canvas_x - 8, canvas_y, fill="#d500f9", outline="white", width=1.5)
        self.map_canvas.create_text(
            canvas_x + 10, canvas_y - 10, anchor="sw", fill="#aa00c7",
            text=f"{class_id} ({x:.1f}, {y:.1f})")

    def _draw_vehicle(self, vehicle: VehicleState) -> None:
        assert self._map_viewport is not None
        span = max(
            self._map_viewport.bounds.max_x - self._map_viewport.bounds.min_x,
            self._map_viewport.bounds.max_y - self._map_viewport.bounds.min_y,
            1.0,
        )
        endpoint = heading_endpoint((vehicle.x, vehicle.y), vehicle.heading_deg, max(1.0, span * 0.04))
        x, y = self._map_viewport.to_canvas((vehicle.x, vehicle.y))
        end_x, end_y = self._map_viewport.to_canvas(endpoint)
        self.map_canvas.create_line(x, y, end_x, end_y, fill="#202020", width=3, arrow=tk.LAST)
        self.map_canvas.create_oval(x - 6, y - 6, x + 6, y + 6, fill="#202020", outline="white", width=1.5)
        self.map_canvas.create_text(x + 8, y + 8, anchor="nw", fill="#202020",
                                    text=f"vehicle z={vehicle.z:.1f}, yaw={vehicle.heading_deg:.0f}°")

    def _canvas_coordinates(self, points: tuple[tuple[float, float], ...]) -> tuple[float, ...]:
        assert self._map_viewport is not None
        return tuple(value for point in points for value in self._map_viewport.to_canvas(point))

    def _current_corners(self) -> tuple[tuple[float, float], ...]:
        try:
            return parse_corners(tuple((x.get(), y.get()) for x, y in self.corner_values))
        except ValueError:
            return ()

    def _set_corners(self, corners: tuple[tuple[float, float], ...]) -> None:
        for (x_value, y_value), (x, y) in zip(self.corner_values, corners):
            x_value.set(f"{x:.3f}")
            y_value.set(f"{y:.3f}")
        self._schedule_map_redraw()

    def _reset_map_selection(self) -> None:
        self._pending_map_click = None
        for x_value, y_value in self.corner_values:
            x_value.set("")
            y_value.set("")
        self.map_status.set("Search rectangle cleared. Click two map points or enter four corners.")
        self._schedule_map_redraw()

    def _show_search_result(self, prefix: str, path: object) -> None:
        self._route_preview = route_points(path)
        self._schedule_map_redraw()
        self._set_result(f"{prefix} coverage route.\n{format_path(path)}")

    def _start_operations_telemetry(self) -> None:
        try:
            self._stop_operations_telemetry(update_status=False)
            self._operations_telemetry = OperationsTelemetry(
                self.vehicle_odometry_topic.get(), self.waypoint_queue_status_topic.get())
        except Exception as error:
            self.telemetry_status.set(f"Telemetry error: {error}")
            self._report_error(error)
            return
        self._vehicle_state = None
        self._queue_state = None
        self.telemetry_status.set(
            f"Waiting for vehicle pose on {self.vehicle_odometry_topic.get().strip()} and queue state on "
            f"{self.waypoint_queue_status_topic.get().strip()}...")
        self._schedule_map_redraw()

    def _stop_operations_telemetry(self, *, update_status: bool = True) -> None:
        telemetry, self._operations_telemetry = self._operations_telemetry, None
        if telemetry is not None:
            telemetry.close()
        if update_status:
            self.telemetry_status.set("Telemetry stopped.")

    def _poll_operations_telemetry(self) -> None:
        if self._closed:
            return
        telemetry = self._operations_telemetry
        if telemetry is not None:
            error = telemetry.latest_error()
            if error is not None:
                self.telemetry_status.set(f"Telemetry error: {error}")
            vehicle = telemetry.latest_vehicle()
            queue_state = telemetry.latest_queue()
            changed = False
            if vehicle is not None:
                self._vehicle_state = vehicle
                changed = True
            if queue_state is not None:
                self._queue_state = queue_state
                changed = True
            if changed:
                self._schedule_map_redraw()
            self._update_telemetry_status(telemetry)
        self.after(100, self._poll_operations_telemetry)

    def _update_telemetry_status(self, telemetry: OperationsTelemetry) -> None:
        now = monotonic()
        vehicle = self._vehicle_state
        queue_state = self._queue_state
        vehicle_text = "vehicle: waiting"
        if vehicle is not None:
            age = now - vehicle.received_at
            prefix = "vehicle: stale" if age > 2.0 else "vehicle"
            vehicle_text = (f"{prefix} x={vehicle.x:.2f}, y={vehicle.y:.2f}, z={vehicle.z:.2f}, "
                            f"yaw={vehicle.heading_deg:.1f}° ({age:.1f}s ago)")
        queue_text = "queue: waiting"
        if queue_state is not None:
            age = now - queue_state.received_at
            prefix = "queue: stale" if age > 2.0 else "queue"
            queue_text = f"{prefix} {len(queue_state.points)} waypoint(s) ({age:.1f}s ago)"
        self.telemetry_status.set(
            f"{vehicle_text}; {queue_text}. Topics: {telemetry.odometry_topic}, {telemetry.queue_status_topic}")

    def _run_service_action(self, message: str, action, on_success) -> None:
        self.status.set(message)
        for button in self._service_buttons:
            button.state(["disabled"])
        future = self._worker.submit(action)
        future.add_done_callback(
            lambda completed: self._completed_actions.put((completed, on_success)))

    def _poll_completed_actions(self) -> None:
        if self._closed:
            return
        while True:
            try:
                future, on_success = self._completed_actions.get_nowait()
            except Empty:
                break
            self._complete_service_action(future, on_success)
        self.after(50, self._poll_completed_actions)

    def _complete_service_action(self, future: Future, on_success: object) -> None:
        if self._closed:
            return
        for button in self._service_buttons:
            button.state(["!disabled"])
        try:
            on_success(future.result())  # type: ignore[operator]
        except Exception as error:
            if self._mission_running:
                self._mission_finished()
                self.mission_status.set(f"Mission failed: {error}")
            self._report_error(error)

    def _set_result(self, result: str) -> None:
        self.status.set(result.splitlines()[0])
        self.output.configure(state="normal")
        self.output.delete("1.0", "end")
        self.output.insert("1.0", result)
        self.output.configure(state="disabled")

    def _report_error(self, error: Exception) -> None:
        self.status.set(f"Error: {error}")
        self.output.configure(state="normal")
        self.output.insert("end", f"\nError: {error}\n")
        self.output.see("end")
        self.output.configure(state="disabled")

    def _close(self) -> None:
        self._closed = True
        self._worker.shutdown(wait=False, cancel_futures=True)
        self._stop_camera_preview(update_status=False)
        self._stop_operations_telemetry(update_status=False)
        self._node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        self.destroy()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="verify imports without opening a window")
    args = parser.parse_args()
    if args.check:
        print("Skills test GUI imports are available.")
        return
    rclpy.init()
    app = SkillsTestGui(rclpy.create_node("skills_test_gui"))
    app.mainloop()


if __name__ == "__main__":
    main()
