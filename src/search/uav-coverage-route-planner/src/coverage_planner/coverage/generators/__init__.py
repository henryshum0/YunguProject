"""Interchangeable geometric coverage-structure generators."""

from coverage_planner.coverage.generators.base import CoverageStructureGenerator
from coverage_planner.coverage.generators.bcd import (
    BCDGenerator,
    build_boustrophedon_planning_cells,
    decompose_boustrophedon_cells,
    merge_small_boustrophedon_cells,
)
from coverage_planner.coverage.generators.global_scanline import GlobalScanlineGenerator

__all__ = [
    "BCDGenerator",
    "CoverageStructureGenerator",
    "GlobalScanlineGenerator",
    "build_boustrophedon_planning_cells",
    "decompose_boustrophedon_cells",
    "merge_small_boustrophedon_cells",
]
