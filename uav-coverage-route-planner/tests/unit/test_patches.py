import pytest
from pydantic import ValidationError
from shapely.geometry import MultiPolygon, Polygon, box

from coverage_planner.camera import GroundFootprintDimensions
from coverage_planner.coverage import CoverageEvaluationError, evaluate_patch_coverage
from coverage_planner.models.patch import PatchGridConfig
from coverage_planner.partition import PatchGenerationError, generate_patches


def test_generates_clipped_patches_with_stable_ids_and_conserved_area() -> None:
    effective = Polygon(
        [(0, 0), (9, 0), (9, 7), (0, 7)],
        holes=[[(3, 2), (6, 2), (6, 5), (3, 5)]],
    )
    patches = generate_patches(
        effective,
        config=PatchGridConfig(width_m=4, height_m=3, origin_enu_m=(0, 0)),
    )
    assert [patch.id for patch in patches] == [
        "patch_r0_c0", "patch_r0_c1", "patch_r0_c2",
        "patch_r1_c0", "patch_r1_c1", "patch_r1_c2",
        "patch_r2_c0", "patch_r2_c1", "patch_r2_c2",
    ]
    assert sum(patch.area_m2 for patch in patches) == pytest.approx(effective.area)
    assert patches[-1].area_m2 == 1
    assert all(patch.geometry.intersection(box(3, 2, 6, 5)).area == 0 for patch in patches)


def test_supports_disconnected_geometry_and_negative_grid_indices() -> None:
    geometry = MultiPolygon([box(-3, -3, -1, -1), box(1, 1, 3, 3)])
    patches = generate_patches(
        geometry,
        config=PatchGridConfig(width_m=2, height_m=2),
    )
    assert [patch.id for patch in patches] == [
        "patch_r-2_c-2", "patch_r-2_c-1", "patch_r-1_c-2", "patch_r-1_c-1",
        "patch_r0_c0", "patch_r0_c1", "patch_r1_c0", "patch_r1_c1",
    ]
    assert sum(patch.area_m2 for patch in patches) == pytest.approx(geometry.area)


def test_derives_patch_size_from_camera_spacing() -> None:
    dimensions = GroundFootprintDimensions(
        width_m=20,
        length_m=10,
        capture_spacing_m=7,
        scan_line_spacing_m=12,
        height_above_ground_m=30,
    )
    patches = generate_patches(box(0, 0, 24, 14), camera_dimensions=dimensions)
    assert len(patches) == 4
    assert {patch.area_m2 for patch in patches} == {84}


def test_requires_explicit_or_camera_derived_patch_dimensions() -> None:
    with pytest.raises(PatchGenerationError, match="required unless derived"):
        generate_patches(box(0, 0, 1, 1))
    with pytest.raises(ValidationError):
        PatchGridConfig(width_m=0, height_m=1)


def test_area_ratio_uses_union_without_double_counting() -> None:
    patch = generate_patches(
        box(0, 0, 10, 10), config=PatchGridConfig(width_m=10, height_m=10)
    )[0]
    evaluated = evaluate_patch_coverage(
        [patch],
        {"wp_1": box(0, 0, 6, 10), "wp_2": box(4, 0, 8, 10)},
        minimum_coverage_ratio=0.75,
    )[0]
    assert evaluated.coverage_ratio == pytest.approx(0.8)
    assert evaluated.covered
    assert evaluated.covered_by_waypoint_ids == ("wp_1", "wp_2")


def test_center_mode_does_not_claim_full_area_coverage() -> None:
    patch = generate_patches(
        box(0, 0, 10, 10), config=PatchGridConfig(width_m=10, height_m=10)
    )[0]
    footprint = {"wp_center": box(4, 4, 6, 6)}
    center_result = evaluate_patch_coverage([patch], footprint, mode="center")[0]
    area_result = evaluate_patch_coverage([patch], footprint, mode="area_ratio")[0]
    assert center_result.covered
    assert center_result.coverage_ratio == pytest.approx(0.04)
    assert not area_result.covered


def test_clipped_patch_uses_searchable_area_as_ratio_denominator() -> None:
    patch = generate_patches(
        box(0, 0, 2, 10), config=PatchGridConfig(width_m=10, height_m=10)
    )[0]
    result = evaluate_patch_coverage(
        [patch], {"wp": box(0, 0, 2, 9)}, minimum_coverage_ratio=0.9
    )[0]
    assert patch.area_m2 == 20
    assert result.coverage_ratio == pytest.approx(0.9)
    assert result.covered


def test_rejects_invalid_coverage_settings() -> None:
    with pytest.raises(CoverageEvaluationError, match="between 0 and 1"):
        evaluate_patch_coverage([], {}, minimum_coverage_ratio=1.1)
    with pytest.raises(CoverageEvaluationError, match="unsupported"):
        evaluate_patch_coverage([], {}, mode="unknown")  # type: ignore[arg-type]
