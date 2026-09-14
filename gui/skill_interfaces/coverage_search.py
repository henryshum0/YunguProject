"""Coverage Search skill tab for the skills test GUI."""

from __future__ import annotations

from collections.abc import Sequence
from typing import Any

import tkinter as tk
from tkinter import ttk

from gui.skill_interfaces.controller import format_path
from gui.input_parser import parse_corners
from gui.map_view import Viewport, rectangle_from_clicks, route_points
from gui.skill_interfaces.base import Point, SkillInterface


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
