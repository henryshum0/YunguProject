"""Pluggable Tk views for the ROS-backed skills exposed by the GUI."""

from __future__ import annotations

from abc import ABC, abstractmethod
from collections.abc import Sequence
from typing import Any

import tkinter as tk
from tkinter import ttk

from gui.controller import format_path
from gui.input_parser import (
    parse_corners,
    parse_frame,
    parse_navigation_goal,
    parse_waypoints,
)
from gui.map_view import Viewport, rectangle_from_clicks, route_points


Point = tuple[float, float]
NavigationGoal = tuple[float, float, float, float]


class SkillInterface(ABC):
    """A self-contained skill tab and its persistent-map interaction contract.

    Implementations own their Tk variables, controls, validation, and map
    interaction.  The host supplies shared ROS configuration, the serialized
    action worker, status/result output, and the persistent map canvas.
    """

    tab_title: str

    def __init__(self, host: Any) -> None:
        self.host = host
        self.frame: ttk.Frame | None = None

    def install(self, notebook: ttk.Notebook) -> None:
        """Create and register this interface's tab exactly once."""
        if self.frame is not None:
            raise RuntimeError(f"{self.tab_title} interface is already installed")
        self.frame = ttk.Frame(notebook, padding=10)
        notebook.add(self.frame, text=self.tab_title)
        self.build(self.frame)

    @property
    @abstractmethod
    def map_instruction(self) -> str:
        """Explain the active interface's map-click behavior."""

    def on_selected(self) -> None:
        self.host.map_mode.set(self.map_instruction)

    @abstractmethod
    def build(self, frame: ttk.Frame) -> None:
        """Populate this interface's notebook tab."""

    @abstractmethod
    def on_map_click(self, point: Point) -> None:
        """Handle an ENU point selected on the shared operations map."""

    def on_map_reloaded(self) -> None:
        """Reset only transient map interaction state after a map replacement."""

    def map_bounds_points(self) -> tuple[Point, ...]:
        """Return interface-owned points that must remain visible on the map."""
        return ()

    def draw_map_overlay(self, canvas: tk.Canvas, viewport: Viewport) -> None:
        """Draw persistent interface state after the common map geometry."""


class NavigateSkillInterface(SkillInterface):
    """Controls and map interaction for :class:`skills.NavigateSkill`."""

    tab_title = "Navigate"

    def __init__(self, host: Any) -> None:
        super().__init__(host)
        self.input_frame = tk.StringVar(value="ENU")
        self.goal_x = tk.StringVar()
        self.goal_y = tk.StringVar()
        self.goal_z = tk.StringVar(value="5.0")
        self.goal_heading = tk.StringVar(value="0")
        for value in (self.goal_x, self.goal_y, self.goal_z, self.goal_heading):
            value.trace_add("write", self._on_goal_changed)
        self.waypoint_text: tk.Text | None = None

    @property
    def map_instruction(self) -> str:
        return "Navigate mode: one click selects an ENU goal."

    def build(self, frame: ttk.Frame) -> None:
        frame.columnconfigure(0, weight=1)
        frame.rowconfigure(1, weight=1)
        ttk.Label(frame, text="One waypoint per line: x, y, z, heading_deg").grid(
            row=0, column=0, sticky="w")
        self.waypoint_text = tk.Text(frame, height=6, width=60)
        self.waypoint_text.grid(row=1, column=0, pady=6, sticky="nsew")
        self.waypoint_text.insert("1.0", "10, 10, 5, 0\n20, 10, 5, 90")
        controls = ttk.Frame(frame)
        controls.grid(row=2, column=0, sticky="w")
        ttk.Label(controls, text="Input frame").grid(row=0, column=0, padx=(0, 6))
        ttk.Combobox(controls, textvariable=self.input_frame, values=("ENU", "NED"),
                     width=8, state="readonly").grid(row=0, column=1, padx=(0, 8))
        self.host._service_button(controls, "Queue navigation", self._navigate).grid(
            row=0, column=2, padx=(0, 8))
        self.host._service_button(controls, "Clear route", self._clear_route).grid(row=0, column=3)

        goal_controls = ttk.LabelFrame(frame, text="Map-selected ENU goal", padding=6)
        goal_controls.grid(row=3, column=0, pady=(10, 0), sticky="ew")
        for index, (label, value) in enumerate((
            ("X", self.goal_x),
            ("Y", self.goal_y),
            ("Altitude", self.goal_z),
            ("Heading°", self.goal_heading),
        )):
            ttk.Label(goal_controls, text=label).grid(
                row=0, column=index * 2, padx=(0, 4), pady=3, sticky="w")
            ttk.Entry(goal_controls, textvariable=value, width=9).grid(
                row=0, column=index * 2 + 1, padx=(0, 8), pady=3)
        self.host._service_button(goal_controls, "Queue selected goal", self._queue_selected_goal).grid(
            row=1, column=0, columnspan=4, pady=(3, 0), sticky="w")
        ttk.Button(goal_controls, text="Clear selection", command=self._clear_selection).grid(
            row=1, column=4, columnspan=4, padx=(8, 0), pady=(3, 0), sticky="w")
        ttk.Label(frame, text=("Select the Navigate tab, then click the persistent operations map. "
                               "The map is display-only and does not validate flight paths."),
                  wraplength=500).grid(row=4, column=0, pady=(8, 0), sticky="w")

    def on_map_click(self, point: Point) -> None:
        self.goal_x.set(f"{point[0]:.3f}")
        self.goal_y.set(f"{point[1]:.3f}")
        self.host.map_status.set(
            f"Selected ENU goal ({point[0]:.2f}, {point[1]:.2f}). Review altitude/heading, then queue it.")
        self.host._schedule_map_redraw()

    def map_bounds_points(self) -> tuple[Point, ...]:
        goal = self.current_goal()
        return ((goal[0], goal[1]),) if goal is not None else ()

    def draw_map_overlay(self, _canvas: tk.Canvas, _viewport: Viewport) -> None:
        goal = self.current_goal()
        if goal is not None:
            self.host._draw_navigation_goal(goal, "#00838f", "selected")

    def current_goal(self) -> NavigationGoal | None:
        try:
            return parse_navigation_goal(
                self.goal_x.get(), self.goal_y.get(), self.goal_z.get(), self.goal_heading.get())
        except ValueError:
            return None

    def _on_goal_changed(self, *_args: str) -> None:
        self.host._schedule_map_redraw()

    def _clear_selection(self) -> None:
        self.goal_x.set("")
        self.goal_y.set("")
        self.host.map_status.set("Navigation goal selection cleared.")
        self.host._schedule_map_redraw()

    def _navigate(self) -> None:
        assert self.waypoint_text is not None
        try:
            settings = self.host._settings()
            waypoints = parse_waypoints(self.waypoint_text.get("1.0", "end"))
            frame = parse_frame(self.input_frame.get())
        except ValueError as error:
            self.host._report_error(error)
            return
        self.host._run_service_action(
            "Queueing navigation route...",
            lambda: self.host._controller.navigate(waypoints, frame=frame, settings=settings),
            lambda count: self.host._set_result(f"Queued {count} navigation waypoint(s)."),
        )

    def _queue_selected_goal(self) -> None:
        try:
            settings = self.host._settings()
            goal = parse_navigation_goal(
                self.goal_x.get(), self.goal_y.get(), self.goal_z.get(), self.goal_heading.get())
        except ValueError as error:
            self.host._report_error(error)
            return
        self.host._run_service_action(
            "Queueing selected map goal...",
            lambda: self.host._controller.navigate((goal,), frame="enu", settings=settings),
            lambda count: self.host._set_result(
                f"Queued {count} selected map goal(s): x={goal[0]:.2f}, y={goal[1]:.2f}, "
                f"z={goal[2]:.2f}, yaw={goal[3]:.1f} deg."),
        )

    def _clear_route(self) -> None:
        try:
            settings = self.host._settings()
        except ValueError as error:
            self.host._report_error(error)
            return
        self.host._run_service_action(
            "Clearing active and queued route...",
            lambda: self.host._controller.clear(settings),
            lambda count: self.host._set_result(f"Cleared {count} active/queued waypoint(s)."),
        )


class CoverageSearchSkillInterface(SkillInterface):
    """Controls and map interaction for coverage planning and search queueing."""

    tab_title = "Coverage search"

    def __init__(self, host: Any) -> None:
        super().__init__(host)
        self.corner_values = [(tk.StringVar(), tk.StringVar()) for _ in range(4)]
        self.pending_click: Point | None = None
        for x_value, y_value in self.corner_values:
            x_value.trace_add("write", self._on_corner_changed)
            y_value.trace_add("write", self._on_corner_changed)

    @property
    def map_instruction(self) -> str:
        return "Coverage Search mode: two clicks select an ENU rectangle."

    def build(self, frame: ttk.Frame) -> None:
        ttk.Label(frame, text="Four search-area corners in ENU metres (x, y), or choose two map points.").grid(
            row=0, column=0, columnspan=3, sticky="w")
        for index, (x_value, y_value) in enumerate(self.corner_values, start=1):
            ttk.Label(frame, text=f"Corner {index}").grid(
                row=index, column=0, padx=(0, 8), pady=3, sticky="w")
            ttk.Entry(frame, textvariable=x_value, width=16).grid(
                row=index, column=1, padx=(0, 6), pady=3)
            ttk.Entry(frame, textvariable=y_value, width=16).grid(row=index, column=2, pady=3)
        controls = ttk.Frame(frame)
        controls.grid(row=5, column=0, columnspan=3, pady=(8, 0), sticky="w")
        self.host._service_button(controls, "Plan only", self._plan_only).grid(
            row=0, column=0, padx=(0, 8))
        self.host._service_button(controls, "Plan and queue", self._plan_and_queue).grid(
            row=0, column=1, padx=(0, 8))
        ttk.Button(controls, text="Reset selection", command=self._reset_selection).grid(row=0, column=2)
        ttk.Label(frame, text=("Select the Coverage Search tab, then use two clicks on the persistent "
                               "operations map to fill SW, SE, NE, NW."), wraplength=500).grid(
            row=6, column=0, columnspan=3, pady=(10, 0), sticky="w")

    def on_map_click(self, point: Point) -> None:
        if self.pending_click is None:
            self.pending_click = point
            self.host.map_status.set(
                f"First corner: ({point[0]:.2f}, {point[1]:.2f}). Click the opposite corner.")
            self.host._schedule_map_redraw()
            return
        try:
            corners = rectangle_from_clicks(self.pending_click, point)
        except ValueError as error:
            self.pending_click = None
            self.host.map_status.set(str(error))
            self.host._schedule_map_redraw()
            return
        self.pending_click = None
        self._set_corners(corners)
        self.host.map_status.set("Search rectangle selected: SW, SE, NE, NW populated in the corner fields.")

    def on_map_reloaded(self) -> None:
        self.pending_click = None

    def map_bounds_points(self) -> tuple[Point, ...]:
        points = list(self.current_corners())
        if self.pending_click is not None:
            points.append(self.pending_click)
        return tuple(points)

    def draw_map_overlay(self, canvas: tk.Canvas, viewport: Viewport) -> None:
        corners = self.current_corners()
        if corners:
            coordinates = tuple(value for point in (*corners, corners[0])
                                for value in viewport.to_canvas(point))
            canvas.create_line(coordinates, fill="#009688", width=2.5)
            for index, point in enumerate(corners, start=1):
                x, y = viewport.to_canvas(point)
                canvas.create_oval(x - 4, y - 4, x + 4, y + 4, fill="#009688", outline="white")
                canvas.create_text(x + 6, y - 6, anchor="sw", text=str(index), fill="#00695c")
        if self.pending_click is not None:
            x, y = viewport.to_canvas(self.pending_click)
            canvas.create_oval(x - 5, y - 5, x + 5, y + 5, fill="#ff9800", outline="white")

    def current_corners(self) -> tuple[Point, ...]:
        try:
            return parse_corners(tuple((x.get(), y.get()) for x, y in self.corner_values))
        except ValueError:
            return ()

    def _on_corner_changed(self, *_args: str) -> None:
        self.host._schedule_map_redraw()

    def _set_corners(self, corners: Sequence[Point]) -> None:
        for (x_value, y_value), (x, y) in zip(self.corner_values, corners):
            x_value.set(f"{x:.3f}")
            y_value.set(f"{y:.3f}")
        self.host._schedule_map_redraw()

    def _reset_selection(self) -> None:
        self.pending_click = None
        for x_value, y_value in self.corner_values:
            x_value.set("")
            y_value.set("")
        self.host.map_status.set("Search rectangle cleared. Click two map points or enter four corners.")
        self.host._schedule_map_redraw()

    def _plan_only(self) -> None:
        self._run_plan(queue=False)

    def _plan_and_queue(self) -> None:
        self._run_plan(queue=True)

    def _run_plan(self, *, queue: bool) -> None:
        try:
            settings = self.host._settings()
            corners = parse_corners(tuple((x.get(), y.get()) for x, y in self.corner_values))
        except ValueError as error:
            self.host._report_error(error)
            return
        label = "Planning and queueing coverage route..." if queue else "Planning coverage route..."
        action = self.host._controller.search_and_queue if queue else self.host._controller.plan_search
        prefix = "Planned and queued" if queue else "Planned"
        self.host._run_service_action(
            label,
            lambda: action(corners, settings=settings),
            lambda path: self._show_result(prefix, path),
        )

    def _show_result(self, prefix: str, path: object) -> None:
        self.host._route_preview = route_points(path)
        self.host._schedule_map_redraw()
        self.host._set_result(f"{prefix} coverage route.\n{format_path(path)}")


DEFAULT_SKILL_INTERFACES: tuple[type[SkillInterface], ...] = (
    NavigateSkillInterface,
    CoverageSearchSkillInterface,
)

