"""Base contract for pluggable GUI skill interfaces."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any

import tkinter as tk
from tkinter import ttk

from gui.map_view import Viewport


Point = tuple[float, float]
NavigationGoal = tuple[float, float, float, float]


class SkillInterface(ABC):
    """A self-contained skill tab and its persistent-map interaction contract.

    Implementations own their Tk variables, controls, validation, and map
    interaction. The host supplies shared ROS configuration, the serialized
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
