#!/usr/bin/python3
"""Desktop GUI for exercising the workspace navigation and search skills."""

from __future__ import annotations

import argparse
import sys
import tkinter as tk
from concurrent.futures import Future, ThreadPoolExecutor
from pathlib import Path
from queue import Empty, Queue
from tkinter import messagebox, ttk

WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
if str(WORKSPACE_ROOT) not in sys.path:
    sys.path.insert(0, str(WORKSPACE_ROOT))

import rclpy
from rclpy.node import Node

from gui.controller import ConnectionSettings, SkillController, format_path
from gui.input_parser import parse_corners, parse_frame, parse_timeout, parse_waypoints


class SkillsTestGui(tk.Tk):
    """Tk application with one serialized worker for ROS service actions."""

    def __init__(self, node: Node) -> None:
        super().__init__()
        self.title("Yungu Skills Test GUI")
        self.minsize(920, 700)
        self._node = node
        self._controller = SkillController(node)
        self._worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix="skill-service")
        self._completed_actions: Queue[tuple[Future, object]] = Queue()
        self._service_buttons: list[ttk.Button] = []
        self._closed = False
        self._build_variables()
        self._build_layout()
        self.protocol("WM_DELETE_WINDOW", self._close)
        self.after(50, self._poll_completed_actions)

    def _build_variables(self) -> None:
        self.frame_id = tk.StringVar(value="map")
        self.planner_service = tk.StringVar(value="/coverage_planner/plan_coverage")
        self.queue_service = tk.StringVar(value="/waypoint_buffer")
        self.clear_service = tk.StringVar(value="/waypoint_buffer/clear")
        self.takeoff_topic = tk.StringVar(value="/takeoff_cmd")
        self.land_topic = tk.StringVar(value="/land_cmd")
        self.timeout_sec = tk.StringVar(value="10")
        self.navigate_frame = tk.StringVar(value="ENU")
        self.corner_values = [(tk.StringVar(), tk.StringVar()) for _ in range(4)]
        self.status = tk.StringVar(value="Ready. Source ROS and start the required nodes first.")

    def _build_layout(self) -> None:
        outer = ttk.Frame(self, padding=10)
        outer.grid(sticky="nsew")
        self.columnconfigure(0, weight=1)
        self.rowconfigure(0, weight=1)
        outer.columnconfigure(0, weight=1)
        outer.rowconfigure(3, weight=1)

        settings = ttk.LabelFrame(outer, text="Connection settings", padding=8)
        settings.grid(row=0, column=0, sticky="ew")
        fields = [
            ("Frame ID", self.frame_id),
            ("Planner service", self.planner_service),
            ("Queue service", self.queue_service),
            ("Clear service", self.clear_service),
            ("Takeoff topic", self.takeoff_topic),
            ("Land topic", self.land_topic),
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

        notebook = ttk.Notebook(outer)
        notebook.grid(row=2, column=0, pady=(8, 0), sticky="nsew")
        outer.rowconfigure(2, weight=1)
        self._build_navigate_tab(notebook)
        self._build_search_tab(notebook)

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

    def _build_navigate_tab(self, notebook: ttk.Notebook) -> None:
        tab = ttk.Frame(notebook, padding=10)
        notebook.add(tab, text="Navigate")
        tab.columnconfigure(0, weight=1)
        ttk.Label(tab, text="One waypoint per line: x, y, z, heading_deg").grid(row=0, column=0, sticky="w")
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

    def _build_search_tab(self, notebook: ttk.Notebook) -> None:
        tab = ttk.Frame(notebook, padding=10)
        notebook.add(tab, text="Coverage search")
        ttk.Label(tab, text="Four search-area corners in ENU metres (x, y)").grid(
            row=0, column=0, columnspan=3, sticky="w")
        for index, (x_value, y_value) in enumerate(self.corner_values, start=1):
            ttk.Label(tab, text=f"Corner {index}").grid(row=index, column=0, padx=(0, 8), pady=3, sticky="w")
            ttk.Entry(tab, textvariable=x_value, width=16).grid(row=index, column=1, padx=(0, 6), pady=3)
            ttk.Entry(tab, textvariable=y_value, width=16).grid(row=index, column=2, pady=3)
        controls = ttk.Frame(tab)
        controls.grid(row=5, column=0, columnspan=3, pady=(8, 0), sticky="w")
        self._service_button(controls, "Plan only", self._plan_search).grid(row=0, column=0, padx=(0, 8))
        self._service_button(controls, "Plan and queue", self._search_and_queue).grid(row=0, column=1)

    def _service_button(self, parent: tk.Misc, text: str, command) -> ttk.Button:
        button = ttk.Button(parent, text=text, command=command)
        self._service_buttons.append(button)
        return button

    def _settings(self) -> ConnectionSettings:
        timeout = parse_timeout(self.timeout_sec.get())
        fields = {
            "frame ID": self.frame_id.get(),
            "planner service": self.planner_service.get(),
            "queue service": self.queue_service.get(),
            "clear service": self.clear_service.get(),
            "takeoff topic": self.takeoff_topic.get(),
            "land topic": self.land_topic.get(),
        }
        for label, value in fields.items():
            if not value.strip():
                raise ValueError(f"{label} must not be empty")
        return ConnectionSettings(
            frame_id=fields["frame ID"].strip(),
            planner_service=fields["planner service"].strip(),
            queue_service=fields["queue service"].strip(),
            clear_service=fields["clear service"].strip(),
            takeoff_topic=fields["takeoff topic"].strip(),
            land_topic=fields["land topic"].strip(),
            timeout_sec=timeout,
        )

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
            lambda path: self._set_result(f"{prefix} coverage route.\n{format_path(path)}"),
        )

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
