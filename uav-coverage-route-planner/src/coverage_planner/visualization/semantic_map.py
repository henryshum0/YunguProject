"""Headless overview rendering for a validated semantic map."""

from __future__ import annotations

from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
from matplotlib import pyplot as plt
from matplotlib.patches import Polygon as PolygonPatch
from matplotlib.transforms import Affine2D

from coverage_planner.geometry.calibration import MapCalibration
from coverage_planner.models.semantic_map import SemanticMap


def render_semantic_map(
    semantic_map: SemanticMap,
    output_path: str | Path,
    *,
    background_image_path: str | Path | None = None,
    calibration: MapCalibration | None = None,
) -> Path:
    if (background_image_path is None) != (calibration is None):
        raise ValueError("background_image_path and calibration must be provided together")
    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    figure, axes = plt.subplots(figsize=(12, 8))
    display_points = list(semantic_map.search_area.coords)
    if background_image_path is not None and calibration is not None:
        image = plt.imread(background_image_path)
        height, width = np.asarray(image).shape[:2]
        calibration.validate_image_size(width, height)
        corners = [
            calibration.pixel_to_enu((0.0, 0.0)),
            calibration.pixel_to_enu((float(width - 1), 0.0)),
            calibration.pixel_to_enu((0.0, float(height - 1))),
            calibration.pixel_to_enu((float(width - 1), float(height - 1))),
        ]
        matrix = calibration.pixel_to_enu_matrix
        image_transform = Affine2D.from_values(
            matrix[0, 0],
            matrix[1, 0],
            matrix[0, 1],
            matrix[1, 1],
            matrix[0, 2],
            matrix[1, 2],
        )
        axes.imshow(
            image,
            extent=(0.0, float(width - 1), float(height - 1), 0.0),
            origin="upper",
            transform=image_transform + axes.transData,
        )
        display_points.extend(corners)
    colors = {"building": "#555b66", "area": "#7dbb72", "trans_facility": "#d5aa62"}
    for node in semantic_map.nodes:
        low = node.shape.min_corner
        high = node.shape.max_corner
        corners = [(low[0], low[1]), (high[0], low[1]), (high[0], high[1]), (low[0], high[1])]
        axes.add_patch(
            PolygonPatch(
                corners,
                facecolor=colors.get(node.properties.category, "#999999"),
                alpha=0.25 if background_image_path is not None else 0.7,
                edgecolor="#ff3030" if node.properties.category == "building" else "white",
                linewidth=0.8,
            )
        )
    for region in semantic_map.excluded_search_regions:
        low = region.shape.min_corner
        high = region.shape.max_corner
        corners = [(low[0], low[1]), (high[0], low[1]),
                   (high[0], high[1]), (low[0], high[1])]
        axes.add_patch(PolygonPatch(
            corners, facecolor="#343a40", alpha=0.55, edgecolor="#111111",
            linewidth=1.0, hatch="////"))
    boundary = semantic_map.search_area.coords + [semantic_map.search_area.coords[0]]
    axes.plot([point[0] for point in boundary], [point[1] for point in boundary], color="#c53131", linewidth=1.5)
    axes.set_title(f"{semantic_map.world_name} semantic map")
    axes.set_xlabel("East x (m)")
    axes.set_ylabel("North y (m)")
    axes.set_aspect("equal", adjustable="box")
    x_values = [point[0] for point in display_points]
    y_values = [point[1] for point in display_points]
    axes.set_xlim(min(x_values), max(x_values))
    axes.set_ylim(min(y_values), max(y_values))
    axes.grid(alpha=0.2)
    figure.tight_layout()
    figure.savefig(output, dpi=160)
    plt.close(figure)
    return output


def semantic_map_display_bounds(
    semantic_map: SemanticMap,
    *,
    image_size_px: tuple[int, int] | None = None,
    calibration: MapCalibration | None = None,
) -> tuple[float, float, float, float]:
    """Return ENU display bounds for semantic geometry and an optional image."""
    if (image_size_px is None) != (calibration is None):
        raise ValueError("image_size_px and calibration must be provided together")
    points = list(semantic_map.search_area.coords)
    if image_size_px is not None and calibration is not None:
        width, height = image_size_px
        calibration.validate_image_size(width, height)
        points.extend(
            calibration.pixel_to_enu(pixel)
            for pixel in (
                (0.0, 0.0),
                (float(width - 1), 0.0),
                (0.0, float(height - 1)),
                (float(width - 1), float(height - 1)),
            )
        )
    return (
        min(point[0] for point in points),
        min(point[1] for point in points),
        max(point[0] for point in points),
        max(point[1] for point in points),
    )
