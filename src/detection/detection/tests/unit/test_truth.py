"""Pairing estimated positions with the simulated ground truth."""

from __future__ import annotations

from pathlib import Path

import pytest

from detection.config import DetectionConfig, load_targets
from detection.targets import GroundTruthTarget, TargetField
from detection.truth import match_to_truth


def field(*entries) -> TargetField:
    return TargetField(
        targets=tuple(
            GroundTruthTarget(target_id=name, class_id=class_id, position=(x, y, -1.65),
                              yaw_deg=0.0, size=(4.4, 1.8, 1.5), model="m")
            for name, class_id, x, y in entries),
        world_origin_in_gz=(0.0, 0.0, 1.15392))


def test_an_estimate_is_paired_with_the_target_it_refers_to() -> None:
    truth = field(("car_03", "car", 52.0, -2.0), ("car_01", "car", 58.0, -14.0))
    (match,) = match_to_truth([(49.62, -2.47, -1.65)], truth)
    assert match is not None
    assert match.target_id == "car_03"
    assert match.position == (52.0, -2.0, -1.65)
    assert match.error_m == pytest.approx(2.43, abs=0.01)


def test_the_truth_class_is_reported_so_a_class_mistake_shows() -> None:
    truth = field(("car_03", "car", 52.0, -2.0))
    (match,) = match_to_truth([(52.5, -2.0, -1.65)], truth)
    assert match is not None and match.class_id == "car"   # even if reported as 'van'


def test_an_estimate_with_nothing_near_it_has_no_match() -> None:
    truth = field(("car_03", "car", 52.0, -2.0))
    assert match_to_truth([(0.0, 0.0, -1.65)], truth) == (None,)
    # A false positive confirmed by mistake looks exactly like this.
    assert match_to_truth([(52.0, -2.0, -1.65)], truth, max_distance_m=0.1)[0] is not None


def test_two_estimates_cannot_claim_the_same_target() -> None:
    truth = field(("car_03", "car", 52.0, -2.0), ("car_01", "car", 58.0, -14.0))
    first, second = match_to_truth([(51.0, -2.0, -1.65), (52.5, -1.5, -1.65)], truth)
    assert {first.target_id, second.target_id} == {"car_03", "car_01"}
    # The closer estimate wins the nearer target.
    assert second.target_id == "car_03"


def test_results_keep_the_order_of_their_inputs() -> None:
    truth = field(("a", "car", 0.0, 0.0), ("b", "car", 40.0, 0.0))
    matches = match_to_truth([(39.0, 0.0, 0.0), (500.0, 0.0, 0.0), (1.0, 0.0, 0.0)], truth)
    assert [match.target_id if match else None for match in matches] == ["b", None, "a"]


def test_matching_works_against_the_shipped_targets() -> None:
    config_dir = Path(__file__).resolve().parents[3] / "config"
    detection = DetectionConfig.load(config_dir / "detection.yaml")
    shipped = load_targets(config_dir / "targets.yaml", ground_z_m=detection.world.ground_z_m)
    first = shipped.targets[0]
    (match,) = match_to_truth([(first.position[0] + 1.5, first.position[1], 0.0)], shipped)
    assert match is not None and match.target_id == first.target_id
    assert match.error_m == pytest.approx(1.5, abs=1e-6)
