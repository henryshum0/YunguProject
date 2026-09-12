"""Navigate skill tab for the skills test GUI."""

from __future__ import annotations

from typing import Any

import tkinter as tk
from tkinter import ttk

from gui.input_parser import parse_frame, parse_navigation_goal, parse_waypoints
from gui.map_view import Viewport
from gui.skill_interfaces.base import NavigationGoal, Point, SkillInterface


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
