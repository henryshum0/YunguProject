"""The gait turns distance into joint angles; these pin the properties that matter."""

from __future__ import annotations

import pytest

from agents.gait import Gait, JointGait, gait_from_mapping


def _gait(**joints: JointGait) -> Gait:
    return Gait(stride_m=0.4, joints=joints or {"j": JointGait(amplitude=1.0)})


def test_phase_advances_with_distance_not_time() -> None:
    gait = _gait()
    assert gait.advance(0.0, 0.2) == pytest.approx(0.5)
    # A full stride is exactly one cycle, and wraps rather than growing.
    assert gait.advance(0.0, 0.4) == pytest.approx(0.0)
    assert gait.advance(0.25, 0.4) == pytest.approx(0.25)


def test_standing_still_never_advances_the_phase() -> None:
    """A stopped agent must not keep cycling its legs."""
    gait = _gait()
    assert gait.advance(0.3, 0.0) == pytest.approx(0.3)


def test_walking_backwards_still_cycles_the_legs() -> None:
    gait = _gait()
    assert gait.advance(0.0, -0.2) == pytest.approx(0.5)


def test_opposite_offsets_put_legs_in_antiphase() -> None:
    """The whole point of `offset`: a trot's diagonal pairs move oppositely."""
    gait = _gait(
        near=JointGait(bias=0.8, amplitude=0.35, offset=0.0),
        far=JointGait(bias=0.8, amplitude=0.35, offset=0.5),
    )
    angles = gait.positions(0.25)
    assert angles["near"] == pytest.approx(0.8 + 0.35)
    assert angles["far"] == pytest.approx(0.8 - 0.35)


def test_flex_shape_bends_one_way_only() -> None:
    """Knees only fold one way; a plain sine would hyperextend them."""
    joint = JointGait(bias=-1.5, amplitude=-0.5, shape="flex")
    samples = [joint.angle(phase / 100.0) for phase in range(100)]
    assert min(samples) == pytest.approx(-2.0)
    # Never straightens past its neutral angle.
    assert max(samples) == pytest.approx(-1.5)


def test_stance_is_every_joint_at_its_bias() -> None:
    gait = _gait(
        a=JointGait(bias=0.8, amplitude=0.35),
        b=JointGait(bias=-1.5, amplitude=-0.5, shape="flex"),
    )
    assert gait.stance() == {"a": pytest.approx(0.8), "b": pytest.approx(-1.5)}


def test_zero_amplitude_joint_holds_its_bias() -> None:
    gait = _gait(hip=JointGait(bias=0.1, amplitude=0.0))
    assert gait.positions(0.37)["hip"] == pytest.approx(0.1)


@pytest.mark.parametrize("stride", [0.0, -1.0])
def test_stride_must_be_positive(stride: float) -> None:
    with pytest.raises(ValueError, match="stride_m"):
        Gait(stride_m=stride, joints={"j": JointGait()})


def test_gait_needs_at_least_one_joint() -> None:
    with pytest.raises(ValueError, match="at least one joint"):
        Gait(stride_m=0.4, joints={})


def test_unknown_shape_is_rejected() -> None:
    with pytest.raises(ValueError, match="unknown gait shape"):
        JointGait(shape="bounce")


class TestFromMapping:
    def test_builds_a_gait(self) -> None:
        gait = gait_from_mapping({
            "stride_m": 0.5,
            "joints": {"knee": {"bias": 0.1, "amplitude": 0.6, "offset": 0.25, "shape": "flex"}},
        })
        assert gait.stride_m == pytest.approx(0.5)
        assert gait.joint_names == ("knee",)
        assert gait.joints["knee"].shape == "flex"

    def test_rejects_a_missing_stride(self) -> None:
        with pytest.raises(ValueError, match="stride_m"):
            gait_from_mapping({"joints": {"knee": {}}})

    def test_rejects_empty_joints(self) -> None:
        with pytest.raises(ValueError, match="joints"):
            gait_from_mapping({"stride_m": 0.5, "joints": {}})

    def test_rejects_an_unknown_key(self) -> None:
        """A typo in a joint spec must fail loudly, not animate nothing."""
        with pytest.raises(ValueError, match="unknown key"):
            gait_from_mapping({"stride_m": 0.5, "joints": {"knee": {"amplitud": 0.6}}})
