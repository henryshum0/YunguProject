#!/usr/bin/python3
"""Desktop GUI for exercising the workspace navigation and search skills."""

from __future__ import annotations

import argparse
import sys
from collections.abc import Sequence
import tkinter as tk
from concurrent.futures import Future, ThreadPoolExecutor
from pathlib import Path
from queue import Empty, Queue
from time import monotonic
from tkinter import filedialog, messagebox, ttk

WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
if str(WORKSPACE_ROOT) not in sys.path:
    sys.path.insert(0, str(WORKSPACE_ROOT))

# The sidebar now carries a feed per robot — the UAV's two plus one for each
# ground agent — so the previews are laid out two to a row at half width rather
# than stacked full width, which keeps four of them on screen at once.
CAMERA_PREVIEW_WIDTH = 320
CAMERA_PREVIEW_HEIGHT = 240
CAMERA_FEED_COLUMNS = 2

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from gui.skill_interfaces.controller import ConnectionSettings, SkillController
from gui.camera_view import CameraPreview
from gui.telemetry import OperationsTelemetry, QueueState, VehicleState
from skills import SkillConfigError, SkillRuntimeConfig
from gui.input_parser import parse_timeout
from gui.map_view import (
    MapLoadError,
    PlannerMap,
    Viewport,
    bounds_for,
    heading_endpoint,
    load_planner_map,
    make_viewport,
)
from gui.skill_interfaces import DEFAULT_SKILL_INTERFACES, SkillInterface


class SkillsTestGui(tk.Tk):
    """Tk application with one serialized worker for ROS service actions."""

    def __init__(
        self,
        node: Node,
        *,
        skill_interfaces: Sequence[type[SkillInterface]] = DEFAULT_SKILL_INTERFACES,
    ) -> None:
        super().__init__()
        self.title("Yungu Skills Test GUI")
        # Stacked 4:3 feeds keep both camera views wide and readable.
        self.minsize(1920, 1080)
        self._node = node
        self._controller = SkillController(node)
        self._worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix="skill-service")
        self._completed_actions: Queue[tuple[Future, object]] = Queue()
        self._service_buttons: list[ttk.Button] = []
        self._closed = False
        self._map_data: PlannerMap | None = None
        self._map_viewport: Viewport | None = None
        self._route_preview: tuple[tuple[float, float], ...] = ()
        self._map_redraw_scheduled = False
        self._vehicle_state: VehicleState | None = None
        self._queue_state: QueueState | None = None
        self._operations_telemetry: OperationsTelemetry | None = None
        self._camera_previews: dict[str, CameraPreview] = {}
        self._camera_photos: dict[str, tk.PhotoImage] = {}
        self._camera_labels: dict[str, tk.Label] = {}
        self._camera_statuses: dict[str, tk.StringVar] = {}
        # Overlay toggle swaps the front feed to the detector's overlay topic and
        # remembers the raw topic so it can switch back.
        self._front_camera_raw_topic: str | None = None
        # Set when a detection config was named but could not be loaded, so the
        # search-mission tab can report the exact reason it cannot start.
        self._detection_config_error: str | None = None
        # Ground agents are optional: the GUI is still a complete UAV client
        # without them, so a missing or unbuilt agents package degrades to "no
        # agents" rather than failing to start.
        self._agent_specs: tuple = ()
        self._agents_error: str | None = None
        self._agent_goal_publishers: dict[str, object] = {}
        self._build_variables()
        self._load_agents()
        if not skill_interfaces:
            raise ValueError("at least one skill interface is required")
        self._skill_interfaces = tuple(interface(self) for interface in skill_interfaces)
        self._build_layout()
        self._load_map()
        self.protocol("WM_DELETE_WINDOW", self._close)
        self.after(50, self._poll_completed_actions)
        self.after(50, self._poll_camera_previews)
        self.after(100, self._poll_operations_telemetry)
        self.after_idle(self._start_camera_previews)
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
        self.agents_config_file = tk.StringVar(
            value=str(WORKSPACE_ROOT / "src" / "agents" / "config" / "agents.yaml"))
        self.front_camera_image_topic = tk.StringVar(value="/swan_gamma_v2/front_camera/image")
        self.follow_camera_image_topic = tk.StringVar(value="/swan_gamma_v2/follow_camera/image")
        self.vehicle_odometry_topic = tk.StringVar(value="/gz/ground_truth/odom")
        self.waypoint_queue_status_topic = tk.StringVar(value="/waypoint_buffer/status")
        self.timeout_sec = tk.StringVar(value="10")
        self.map_file = tk.StringVar(
            value=str(WORKSPACE_ROOT / "src" / "search" / "config" / "yungu_map.json"))
        self.map_status = tk.StringVar()
        self.map_mode = tk.StringVar(value="Navigate: click selects one ENU goal")
        self.telemetry_status = tk.StringVar(value="Telemetry stopped.")
        self.status = tk.StringVar(value="Ready. Source ROS and start the required nodes first.")

    def _build_layout(self) -> None:
        outer = ttk.Frame(self, padding=10)
        outer.grid(sticky="nsew")
        self.columnconfigure(0, weight=1)
        self.rowconfigure(0, weight=1)
        outer.columnconfigure(0, weight=1, minsize=1200)
        # A native-width preview plus frame padding/borders. ``minsize`` prevents
        # the zero-weight sidebar from being compressed by the main controls.
        outer.columnconfigure(1, weight=0, minsize=680)
        outer.rowconfigure(3, weight=1)

        settings = ttk.LabelFrame(outer, text="Connection settings", padding=4)
        settings.grid(row=0, column=0, sticky="ew")
        fields = [
            ("Navigation config directory", self.navigation_config_dir),
            ("Planner config JSON", self.planner_config_file),
            ("Detection config YAML", self.detection_config_file),
            ("Ground truth YAML (sim)", self.truth_targets_file),
            ("Agents config YAML", self.agents_config_file),
            ("Front camera image topic", self.front_camera_image_topic),
            ("Follow camera image topic", self.follow_camera_image_topic),
            ("Vehicle odometry topic", self.vehicle_odometry_topic),
            ("Queue status topic", self.waypoint_queue_status_topic),
            ("Timeout (s)", self.timeout_sec),
        ]
        for index, (label, variable) in enumerate(fields):
            row, column = divmod(index, 4)
            field = ttk.Frame(settings)
            field.grid(row=row, column=column, padx=(0, 8), pady=1, sticky="ew")
            ttk.Label(field, text=label).grid(row=0, column=0, sticky="w")
            ttk.Entry(field, textvariable=variable, width=31).grid(row=1, column=0, sticky="ew")
            field.columnconfigure(0, weight=1)
            settings.columnconfigure(column, weight=1)

        flight = ttk.LabelFrame(outer, text="Flight control", padding=4)
        flight.grid(row=1, column=0, pady=(8, 0), sticky="ew")
        self._service_button(flight, "Take off", self._takeoff).grid(row=0, column=0, padx=(0, 8))
        self._service_button(flight, "Land", self._land).grid(row=0, column=1)
        ttk.Label(flight, text="Both commands require confirmation and send a service request.").grid(
            row=0, column=2, padx=16, sticky="w")

        work_area = ttk.PanedWindow(outer, orient=tk.HORIZONTAL)
        work_area.grid(row=2, column=0, pady=(8, 0), sticky="nsew")
        outer.rowconfigure(2, weight=1)
        self._notebook = ttk.Notebook(work_area)
        work_area.add(self._notebook, weight=1)
        for interface in self._skill_interfaces:
            interface.install(self._notebook)
        self._notebook.bind("<<NotebookTabChanged>>", self._on_skill_tab_changed)
        operations = ttk.LabelFrame(work_area, text="Live operations map", padding=4)
        work_area.add(operations, weight=2)
        self._build_operations_map(operations)

        result = ttk.LabelFrame(outer, text="Status and planner result", padding=4)
        result.grid(row=3, column=0, pady=(8, 0), sticky="nsew")
        result.columnconfigure(0, weight=1)
        result.rowconfigure(1, weight=1)
        ttk.Label(result, textvariable=self.status, foreground="#155724", wraplength=850).grid(
            row=0, column=0, sticky="ew", pady=(0, 6))
        self.output = tk.Text(result, height=6, wrap="none", state="disabled")
        self.output.grid(row=1, column=0, sticky="nsew")
        scrollbar = ttk.Scrollbar(result, command=self.output.yview)
        scrollbar.grid(row=1, column=1, sticky="ns")
        self.output.configure(yscrollcommand=scrollbar.set)

        self._build_camera_pane(outer)

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
        ttk.Label(panel, textvariable=self.map_mode, wraplength=720).grid(
            row=2, column=0, columnspan=2, pady=(6, 2), sticky="w")
        self.map_canvas = tk.Canvas(panel, width=740, height=540, background="#f8f9fa",
                                    highlightthickness=1, highlightbackground="#a0a0a0",
                                    cursor="crosshair")
        self.map_canvas.grid(row=3, column=0, columnspan=2, sticky="nsew")
        self.map_canvas.bind("<Button-1>", self._on_operations_map_click)
        self.map_canvas.bind("<Configure>", self._on_map_resize)
        ttk.Label(panel, textvariable=self.telemetry_status, wraplength=720).grid(
            row=4, column=0, columnspan=2, pady=(4, 0), sticky="w")
        ttk.Label(panel, textvariable=self.map_status, wraplength=720).grid(
            row=5, column=0, columnspan=2, pady=(2, 0), sticky="w")

    def _build_camera_pane(self, outer: ttk.Frame) -> None:
        pane = ttk.LabelFrame(outer, text="Camera feeds", padding=4, width=680)
        pane.grid(row=0, column=1, rowspan=4, padx=(10, 0), sticky="nsew")
        pane.columnconfigure(0, weight=1)
        pane.rowconfigure(2, weight=1)
        ttk.Label(
            pane,
            text=("One feed per robot: the UAV's front and chase cameras, plus the front camera "
                  "each ground agent carries. They stay visible while changing skill tabs. Set "
                  "the UAV topics in Connection settings, then reconnect."),
            wraplength=650,
        ).grid(row=0, column=0, sticky="w")
        controls = ttk.Frame(pane)
        controls.grid(row=1, column=0, pady=(8, 6), sticky="w")
        ttk.Button(controls, text="Start / reconnect feeds", command=self._start_camera_previews).grid(
            row=0, column=0, padx=(0, 8))
        ttk.Button(controls, text="Stop feeds", command=self._stop_camera_previews).grid(
            row=0, column=1, padx=(0, 8))
        self._overlay_button = ttk.Button(
            controls, text="Show detection boxes", command=self._toggle_detection_overlay)
        self._overlay_button.grid(row=0, column=2)

        feeds = ttk.Frame(pane)
        feeds.grid(row=2, column=0, sticky="nsew")
        for column in range(CAMERA_FEED_COLUMNS):
            feeds.columnconfigure(column, weight=1)
        for index, (key, title, _topic) in enumerate(self._camera_feeds()):
            row, column = divmod(index, CAMERA_FEED_COLUMNS)
            feeds.rowconfigure(row, weight=1)
            feed = ttk.LabelFrame(feeds, text=title, padding=4)
            feed.grid(row=row, column=column, padx=(0, 6), pady=(0, 6), sticky="nsew")
            status = tk.StringVar(value="Preview stopped.")
            ttk.Label(feed, textvariable=status, wraplength=CAMERA_PREVIEW_WIDTH).grid(
                row=0, column=0, pady=(0, 2), sticky="w")
            preview_frame = tk.Frame(
                feed,
                width=CAMERA_PREVIEW_WIDTH,
                height=CAMERA_PREVIEW_HEIGHT,
                background="#202020",
            )
            preview_frame.grid(row=1, column=0, sticky="nsew")
            preview_frame.grid_propagate(False)
            label = tk.Label(
                preview_frame, text="Starting camera preview...", background="#202020",
                foreground="#f0f0f0", anchor="center")
            label.pack(fill="both", expand=True)
            self._camera_labels[key] = label
            self._camera_statuses[key] = status

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

    def _load_agents(self) -> None:
        """Load the ground-agent roster, or record why there is none.

        Agents are optional. The GUI is a complete UAV client without them, so a
        missing file or an unbuilt agents package leaves the roster empty and the
        reason on hand for the Navigate tab to show, rather than refusing to start.
        """
        path = self.agents_config_file.get().strip()
        if not path:
            self._agent_specs, self._agents_error = (), None
            return
        try:
            from agents.config import AgentsConfig

            config = AgentsConfig.load(path)
        except Exception as error:  # noqa: BLE001 - any failure means "no agents"
            self._agent_specs = ()
            self._agents_error = str(error)
            return
        self._agent_specs = config.agents
        self._agents_error = None
        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._agent_goal_publishers = {
            spec.name: self._node.create_publisher(PoseStamped, spec.goal_topic, reliable)
            for spec in self._agent_specs
        }

    def send_agent_goal(self, name: str, x: float, y: float) -> None:
        """Send one ground agent to an ENU position on the map plane.

        Published straight from the Tk thread rather than through the service
        worker: it is one non-blocking publish, and routing it through the worker
        would make sending a goal wait behind whatever the UAV is doing.
        """
        publisher = self._agent_goal_publishers.get(name)
        if publisher is None:
            raise ValueError(f"no ground agent named '{name}'")
        message = PoseStamped()
        message.header.stamp = self._node.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.pose.position.x = float(x)
        message.pose.position.y = float(y)
        message.pose.orientation.w = 1.0
        publisher.publish(message)

    def _camera_feeds(self) -> tuple[tuple[str, str, str], ...]:
        """Every camera the sidebar shows, as ``(key, title, topic)``.

        The UAV topics stay editable in the connection settings; an agent's comes
        from the agents config, because it is fixed by the model that carries it.
        """
        feeds = [
            ("front", "UAV front camera", self.front_camera_image_topic.get().strip()),
            ("follow", "UAV follow camera", self.follow_camera_image_topic.get().strip()),
        ]
        for spec in self._agent_specs:
            if spec.camera is not None:
                feeds.append((f"agent_{spec.name}", f"{spec.name} front camera", spec.camera.topic))
        return tuple(feeds)

    def _start_camera_previews(self) -> None:
        """Start an independent ROS image subscriber for each configured feed."""
        self._stop_camera_previews(update_status=False)
        for key, _title, topic in self._camera_feeds():
            status = self._camera_statuses.get(key)
            label = self._camera_labels.get(key)
            if status is None or label is None:
                # The pane was built before this feed existed; it appears after
                # the next restart rather than being wired up half-built.
                continue
            try:
                self._camera_previews[key] = CameraPreview(
                    topic, node_name=f"skills_test_gui_{key}_camera")
            except Exception as error:
                status.set(f"Preview error: {error}")
                label.configure(image="", text="Camera preview unavailable.")
                self._report_error(error)
                continue
            status.set(f"Waiting for images on {topic}...")
            label.configure(image="", text="Waiting for camera frames...")

    def _stop_camera_previews(self, *, update_status: bool = True) -> None:
        previews, self._camera_previews = self._camera_previews, {}
        for preview in previews.values():
            preview.close()
        self._camera_photos.clear()
        for key, label in self._camera_labels.items():
            label.configure(image="", text="Camera preview stopped.")
            if update_status:
                self._camera_statuses[key].set("Preview stopped.")

    def _toggle_detection_overlay(self) -> None:
        """Switch the front feed between the raw camera and the detector's overlay.

        The overlay is published by the detection layer, so its topic comes from
        the detection configuration rather than being spelled out here. Only the
        front feed is switched; the follow camera keeps showing the raw stream.
        """
        if self._front_camera_raw_topic is not None:
            topic, self._front_camera_raw_topic = self._front_camera_raw_topic, None
            self.front_camera_image_topic.set(topic)
            self._overlay_button.configure(text="Show detection boxes")
            self._start_camera_previews()
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
        self._front_camera_raw_topic = self.front_camera_image_topic.get()
        self.front_camera_image_topic.set(settings.config.detection.image_overlay_topic)
        self._overlay_button.configure(text="Show raw camera")
        self._start_camera_previews()

    def _poll_camera_previews(self) -> None:
        if self._closed:
            return
        for key, preview in tuple(self._camera_previews.items()):
            status = self._camera_statuses[key]
            label = self._camera_labels[key]
            error = preview.latest_error()
            if error is not None:
                status.set(f"Preview error: {error}")
            frame = preview.latest_frame()
            if frame is not None:
                try:
                    display_frame = frame.resized_to_fit(
                        CAMERA_PREVIEW_WIDTH, CAMERA_PREVIEW_HEIGHT)
                    photo = tk.PhotoImage(data=display_frame.ppm_bytes(), format="PPM")
                except tk.TclError as image_error:
                    status.set(f"Preview display error: {image_error}")
                    continue
                self._camera_photos[key] = photo
                label.configure(image=photo, text="")
                status.set(
                    f"Receiving {frame.width}x{frame.height} RGB frames on {preview.topic}.")
        self.after(100, self._poll_camera_previews)

    def _takeoff(self) -> None:
        self._confirm_and_request("Take off", "Request takeoff from the offboard FSM?", "takeoff")

    def _land(self) -> None:
        self._confirm_and_request("Land", "Request native PX4 landing from the offboard FSM?", "land")

    def _confirm_and_request(self, title: str, prompt: str, operation: str) -> None:
        if not messagebox.askyesno(title, prompt, parent=self):
            self.status.set(f"{title} cancelled.")
            return
        try:
            settings = self._settings()
        except Exception as error:
            self._report_error(error)
            return
        self._run_service_action(
            f"Requesting {title.lower()}...",
            lambda: getattr(self._controller, operation)(settings),
            lambda message: self.status.set(f"{title} accepted: {message}"),
        )

    def _on_skill_tab_changed(self, _event: tk.Event) -> None:
        self._active_skill_interface().on_selected()
        self._schedule_map_redraw()

    def _on_operations_map_click(self, event: tk.Event) -> None:
        if self._map_viewport is None:
            return
        point = self._map_viewport.to_enu((float(event.x), float(event.y)))
        self._active_skill_interface().on_map_click(point)

    def _active_skill_interface(self) -> SkillInterface:
        return self._skill_interfaces[self._notebook.index("current")]

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
        for interface in self._skill_interfaces:
            interface.on_map_reloaded()
        self._route_preview = ()
        self.map_status.set(
            f"Loaded {map_data.source.name}: {len(map_data.occupied_areas)} occupied area(s).")
        self._schedule_map_redraw()

    def _on_map_resize(self, _event: tk.Event) -> None:
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
        overlays = [self._route_preview]
        overlays.extend(interface.map_bounds_points() for interface in self._skill_interfaces)
        if self._vehicle_state is not None:
            overlays.append(((self._vehicle_state.x, self._vehicle_state.y),))
        if self._queue_state is not None:
            overlays.append(self._queue_state.points)
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
        for interface in self._skill_interfaces:
            interface.draw_map_overlay(canvas, self._map_viewport)
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
        if self._vehicle_state is not None:
            self._draw_vehicle(self._vehicle_state)
        canvas.create_text(
            8, 8, anchor="nw", fill="#303030",
            text=(f"{self._map_data.source.name} | red: occupied | green: search area | "
                  "blue: planned route | orange: active queue | purple: pending queue | black: vehicle"),
        )

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
        self._stop_camera_previews(update_status=False)
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
