"""The shipped model.sdf files, checked against the config they are built for.

These models are generated from upstream URDFs by ``model_convert``, but they are
committed artifacts: nothing rebuilds them, so nothing else notices when one of
them stops matching ``agents.yaml`` or loses a property the converter is supposed
to give it. Every check here stands for a fault that was visible in the simulator
and invisible everywhere else -- a base that slowly rolled, limbs that waved
about, a humanoid standing with its arms folded across its chest.
"""

from __future__ import annotations

from pathlib import Path
from xml.etree import ElementTree

import pytest

from agents.config import AgentsConfig
from agents.model_convert import BASE_INERTIA
from agents.topics import base_command_topic, joint_command_topic

_AGENTS = Path(__file__).resolve().parents[3]
SHIPPED_CONFIG = _AGENTS / "config" / "agents.yaml"
MODELS_DIR = _AGENTS / "agents" / "models"

CONFIG = AgentsConfig.load(SHIPPED_CONFIG)


def _model(spec) -> ElementTree.Element:
    model_file = MODELS_DIR / spec.model / "model.sdf"
    assert model_file.is_file(), f"{spec.name}: no model at {model_file}"
    model = ElementTree.parse(model_file).getroot().find("model")
    assert model is not None, f"{model_file} has no <model>"
    return model


def _controllers(model: ElementTree.Element) -> dict[str, ElementTree.Element]:
    found: dict[str, ElementTree.Element] = {}
    for plugin in model.findall("plugin"):
        if "JointPositionController" not in (plugin.get("name") or ""):
            continue
        name = plugin.findtext("joint_name", "").strip()
        assert name not in found, f"two controllers fight over '{name}'"
        found[name] = plugin
    return found


def _driven_joints(model: ElementTree.Element) -> list[str]:
    return [joint.get("name") for joint in model.iter("joint")
            if joint.get("type") in ("revolute", "prismatic")]


ALL_SPECS = [pytest.param(spec, id=spec.name) for spec in CONFIG.agents]


@pytest.mark.parametrize("spec", ALL_SPECS)
def test_every_link_has_exactly_one_gravity_tag_and_it_is_off(spec) -> None:
    """A second tag silently wins, and one link falling is worse than none.

    The Go1's trunk carried both a ``false`` and a ``true``; sdformat honours the
    last, so the trunk fell while its twelve gravity-free leg links did not, and
    the joints turned that into a slow roll.
    """
    for link in _model(spec).iter("link"):
        tags = link.findall("gravity")
        name = link.get("name")
        assert len(tags) == 1, f"{spec.name}/{name}: {len(tags)} <gravity> tags, want 1"
        assert tags[0].text.strip() == "false", f"{spec.name}/{name}: gravity is on"


@pytest.mark.parametrize("spec", ALL_SPECS)
def test_the_base_is_too_heavy_for_its_limbs_to_turn(spec) -> None:
    """Velocity control alone does not hold a light base still; see _anchor_base."""
    model = _model(spec)
    children = {joint.findtext("child", "").strip() for joint in model.iter("joint")}
    roots = [link for link in model.iter("link") if link.get("name") not in children]
    assert len(roots) == 1, f"{spec.name}: expected one root link, found {len(roots)}"

    inertia = roots[0].find("inertial/inertia")
    assert inertia is not None, f"{spec.name}: root link has no <inertia>"
    for axis in ("ixx", "iyy", "izz"):
        value = float(inertia.findtext(axis))
        assert value >= BASE_INERTIA, (
            f"{spec.name}: root {axis} is {value}, below {BASE_INERTIA} -- the "
            "limbs will be able to roll the base")
    for product in ("ixy", "ixz", "iyz"):
        assert float(inertia.findtext(product)) == 0.0, (
            f"{spec.name}: root {product} is non-zero, so the axes stay coupled")


@pytest.mark.parametrize("spec", ALL_SPECS)
def test_every_joint_is_held_by_a_controller(spec) -> None:
    """An uncontrolled joint is flung about: nothing damps it without contacts."""
    model = _model(spec)
    controlled = set(_controllers(model))
    free = [name for name in _driven_joints(model) if name not in controlled]
    assert not free, f"{spec.name}: {len(free)} joint(s) with no controller: {free}"


@pytest.mark.parametrize("spec", ALL_SPECS)
def test_each_joint_rests_where_the_config_says(spec) -> None:
    """``initial_position`` is the resting pose, and the config owns it.

    The G1's elbow is why this exists: at zero its forearm is horizontal and
    pointing forwards, so it stood with both arms folded up across its belly.
    The config holds it at 1.5; if that stops reaching the model, the pose
    silently reverts and only a look at the simulator would show it.
    """
    for joint, plugin in _controllers(_model(spec)).items():
        commanded = float(plugin.findtext("initial_position", "0"))
        assert commanded == pytest.approx(spec.rest_angle(joint), abs=1e-6), (
            f"{spec.name}/{joint}: model rests at {commanded}, "
            f"config says {spec.rest_angle(joint)}")


@pytest.mark.parametrize("spec", ALL_SPECS)
def test_a_resting_angle_is_never_outside_the_joint_limits(spec) -> None:
    """Commanding past a limit is what makes a limb visibly snap or invert."""
    model = _model(spec)
    controllers = _controllers(model)
    for joint in model.iter("joint"):
        limit = joint.find("axis/limit")
        plugin = controllers.get(joint.get("name"))
        if limit is None or plugin is None:
            continue
        low = float(limit.findtext("lower", "-inf"))
        high = float(limit.findtext("upper", "inf"))
        angle = float(plugin.findtext("initial_position", "0"))
        assert low <= angle <= high, (
            f"{spec.name}/{joint.get('name')}: rests at {angle}, "
            f"outside its limit [{low}, {high}]")


@pytest.mark.parametrize("spec", ALL_SPECS)
def test_the_gait_joints_are_the_ones_that_can_be_commanded(spec) -> None:
    """Exactly the animated joints take a topic; holding the rest costs nothing."""
    controllers = _controllers(_model(spec))
    animated = set(spec.gait.joint_names)
    for joint, plugin in controllers.items():
        topic = plugin.findtext("topic")
        if joint in animated:
            assert topic == joint_command_topic(spec.model, joint), (
                f"{spec.name}/{joint} is animated but listens on {topic!r}")
        else:
            assert topic is None, f"{spec.name}/{joint} is not animated but has a topic"
    assert animated <= set(controllers), (
        f"{spec.name}: gait drives joints the model does not have: "
        f"{sorted(animated - set(controllers))}")


@pytest.mark.parametrize("spec", ALL_SPECS)
def test_the_base_listens_on_the_topic_the_node_publishes(spec) -> None:
    plugins = [p for p in _model(spec).findall("plugin")
               if "VelocityControl" in (p.get("name") or "")]
    assert len(plugins) == 1, f"{spec.name}: {len(plugins)} VelocityControl plugins"
    assert plugins[0].findtext("topic") == base_command_topic(spec.model)


@pytest.mark.parametrize("spec", ALL_SPECS)
def test_the_camera_matches_the_config(spec) -> None:
    sensors = [s for s in _model(spec).iter("sensor") if s.get("type") == "camera"]
    assert len(sensors) == 1, f"{spec.name}: {len(sensors)} camera sensors"
    assert sensors[0].findtext("topic") == spec.camera.topic


@pytest.mark.parametrize("spec", ALL_SPECS)
def test_every_mesh_the_model_references_is_shipped_with_it(spec) -> None:
    """A missing mesh renders as nothing at all, with no error anywhere."""
    model_dir = MODELS_DIR / spec.model
    for uri in (element.text.strip() for element in _model(spec).iter("uri")):
        prefix = f"model://{spec.model}/"
        assert uri.startswith(prefix), f"{spec.name}: unresolvable mesh uri {uri!r}"
        mesh = model_dir / uri[len(prefix):]
        assert mesh.is_file(), f"{spec.name}: {uri} is not shipped ({mesh} missing)"
