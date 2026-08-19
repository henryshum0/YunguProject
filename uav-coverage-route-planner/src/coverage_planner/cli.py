"""Headless command-line interface."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import yaml
from pydantic import ValidationError

from coverage_planner import __version__
from coverage_planner.io import load_polygonal_geojson, load_semantic_map
from coverage_planner.models import CameraConfig
from coverage_planner.planner import CoveragePlanner
from coverage_planner.reporting import export_plan


def main() -> None:
    parser = argparse.ArgumentParser(prog="coverage-planner")
    parser.add_argument("--version", action="version", version=__version__)
    subparsers = parser.add_subparsers(dest="command")
    plan_parser = subparsers.add_parser("plan")
    plan_parser.add_argument("--semantic-map", required=True)
    plan_parser.add_argument("--search-area", required=True)
    plan_parser.add_argument("--config", required=True)
    plan_parser.add_argument("--output", required=True)
    args = parser.parse_args()
    if args.command is None:
        print(f"coverage-search-planner {__version__}")
        return
    try:
        config = yaml.safe_load(Path(args.config).read_text(encoding="utf-8"))
        camera = CameraConfig.model_validate(config.pop("camera"))
        result = CoveragePlanner().plan(
            semantic_map=load_semantic_map(args.semantic_map),
            search_geometry=load_polygonal_geojson(args.search_area), camera=camera, **config)
        output = export_plan(result, args.output)
        print(f"Wrote coverage plan to {output}")
        if result.warnings:
            print("Warnings: " + "; ".join(result.warnings), file=sys.stderr)
    except (OSError, ValueError, TypeError, ValidationError, yaml.YAMLError) as exc:
        parser.exit(2, f"coverage-planner: error: {exc}\n")


if __name__ == "__main__":
    main()
