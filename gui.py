#!/usr/bin/env python3
"""Start the skills GUI with ROS Humble and the workspace overlay loaded."""

from __future__ import annotations

import os
from pathlib import Path
import subprocess
import sys


WORKSPACE_ROOT = Path(__file__).resolve().parent
ROS_SETUP = Path("/opt/ros/humble/setup.bash")
WORKSPACE_SETUP = WORKSPACE_ROOT / "install" / "setup.bash"
GUI_ENTRYPOINT = WORKSPACE_ROOT / "gui" / "skills_gui.py"


def _sourced_environment() -> dict[str, str]:
    """Return an environment produced by sourcing the ROS and workspace setups."""
    for path, description in (
        (ROS_SETUP, "ROS 2 Humble setup file"),
        (WORKSPACE_SETUP, "workspace overlay"),
        (GUI_ENTRYPOINT, "GUI entrypoint"),
    ):
        if not path.is_file():
            raise FileNotFoundError(f"{description} not found: {path}")

    command = "source \"$1\" && source \"$2\" && env -0"
    result = subprocess.run(
        ["bash", "-c", command, "gui.py", str(ROS_SETUP), str(WORKSPACE_SETUP)],
        check=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    environment = {
        entry.split(b"=", 1)[0].decode(): entry.split(b"=", 1)[1].decode()
        for entry in result.stdout.split(b"\0")
        if entry
    }
    environment["PYTHONPATH"] = str(WORKSPACE_ROOT) + (
        ":" + environment["PYTHONPATH"] if environment.get("PYTHONPATH") else "")
    return environment


def main() -> None:
    try:
        environment = _sourced_environment()
    except (FileNotFoundError, subprocess.CalledProcessError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        if not WORKSPACE_SETUP.is_file():
            print("Run 'colcon build --symlink-install' from the workspace root first.", file=sys.stderr)
        raise SystemExit(1) from error
    os.execvpe("/usr/bin/python3", ["/usr/bin/python3", str(GUI_ENTRYPOINT), *sys.argv[1:]], environment)


if __name__ == "__main__":
    main()
