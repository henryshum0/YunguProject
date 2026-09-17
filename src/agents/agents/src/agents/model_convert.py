"""Turn an upstream robot URDF into a kinematic Gazebo agent model.

The Unitree descriptions are ROS 1 / Gazebo-classic packages: they carry
``libgazebo_ros_*`` plugins, on-board sensors, inertial and collision geometry
for a robot that is meant to be *controlled*. None of that is wanted here. These
agents are scripted scene actors: something drives the base along a path and
animates the legs, and physics is deliberately not in the loop — see
``agents/README.md`` for why that is the right trade for this workspace.

So the conversion keeps the part that is genuinely hard to author by hand — the
link tree, joint origins and axes, and the visual meshes — and drops everything
that would let physics take over:

``<plugin>``
    Gazebo-classic controllers and sensor plugins. They do not exist in
    Gazebo Harmonic and would fail to load.
``<sensor>``
    The Go1 description ships five cameras. An agent walking past the UAV must
    not start publishing its own image topics.
``<collision>``
    Removed so contacts can never push the agent off its commanded path, and so
    it can never get wedged against world geometry. This is what makes the
    kinematic drive predictable.
``gravity``
    Forced off on every link: with no locomotion controller holding it up, a
    physical robot would simply fall over.

What is added is the minimum needed to command it: one ``VelocityControl``
system for the base and one ``JointPositionController`` per animated joint.
"""

from __future__ import annotations

import re
import shutil
import subprocess
import xml.etree.ElementTree as ElementTree
from collections.abc import Sequence
from pathlib import Path

from agents.topics import base_command_topic, joint_command_topic


class ModelConvertError(RuntimeError):
    """Raised when an upstream description cannot be converted."""


#: Element tags removed outright; see the module docstring for the reasoning.
_STRIPPED_TAGS = ("plugin", "sensor", "collision")


def convert(
    urdf_file: Path,
    *,
    model_name: str,
    mesh_source_dir: Path,
    output_dir: Path,
    animated_joints: Sequence[str],
    description: str = "",
    camera=None,
    rest_angle=None,
) -> Path:
    """Write ``output_dir`` as a self-contained kinematic agent model.

    ``animated_joints`` are the joints the gait animator drives; each gets its
    own position-controller topic. ``camera``, when given, mounts one
    forward-facing camera on the root link. Returns the written ``model.sdf``.
    """
    sdf_text = _run_gz_sdf(urdf_file)
    root = _parse(sdf_text, urdf_file)
    model = root.find("model")
    if model is None:
        raise ModelConvertError(f"converted SDF from '{urdf_file}' has no <model>")

    model.set("name", model_name)
    _strip_unwanted(model)
    _fix_classic_materials(model)
    _rewrite_mesh_uris(model, model_name)
    _disable_gravity(model)
    _check_joints(model, animated_joints, urdf_file)
    if camera is not None:
        _add_camera(model, camera, model_name)
    _add_control_plugins(model, model_name, animated_joints,
                         rest_angle or (lambda _joint: 0.0))

    meshes_out = output_dir / "meshes"
    if meshes_out.exists():
        shutil.rmtree(meshes_out)
    meshes_out.mkdir(parents=True, exist_ok=True)
    _copy_referenced_meshes(model, mesh_source_dir, meshes_out)

    model_file = output_dir / "model.sdf"
    ElementTree.indent(root, space="  ")
    model_file.write_text(
        '<?xml version="1.0" ?>\n' + ElementTree.tostring(root, encoding="unicode") + "\n",
        encoding="utf-8",
    )
    (output_dir / "model.config").write_text(
        _model_config(model_name, description), encoding="utf-8")
    return model_file


def _run_gz_sdf(urdf_file: Path) -> str:
    """Convert URDF to SDF with sdformat, which owns the kinematics conversion."""
    if not urdf_file.is_file():
        raise ModelConvertError(f"URDF does not exist: '{urdf_file}'")
    try:
        # Run from the URDF's directory: some descriptions use mesh paths
        # relative to it, and sdformat resolves them against the process cwd.
        completed = subprocess.run(
            ["gz", "sdf", "-p", urdf_file.name],
            cwd=urdf_file.parent,
            capture_output=True,
            text=True,
            check=False,
        )
    except FileNotFoundError as exc:
        raise ModelConvertError(
            "the 'gz' command is unavailable; source the Gazebo environment") from exc
    if completed.returncode != 0 or not completed.stdout.strip():
        raise ModelConvertError(
            f"'gz sdf -p {urdf_file.name}' failed: {completed.stderr.strip() or 'no output'}")
    return completed.stdout


def _parse(sdf_text: str, urdf_file: Path) -> ElementTree.Element:
    text = _sanitise_vendor_tags(sdf_text)
    try:
        return ElementTree.fromstring(text)
    except ElementTree.ParseError as exc:
        raise ModelConvertError(f"converted SDF from '{urdf_file}' is not valid XML: {exc}") from exc


def _sanitise_vendor_tags(sdf_text: str) -> str:
    """Rename ``<gz::x>`` vendor tags so the document is parseable XML.

    sdformat emits these inside the Gazebo-classic plugins it carries across.
    A double colon cannot be made legal by declaring a namespace — the part after
    the prefix would start with ':' — so the tag itself has to be renamed. Every
    such tag lives inside a ``<plugin>``, which is stripped straight afterwards,
    so the renamed form never reaches the written model.
    """
    return re.sub(r"(</?)gz::", r"\1gz__", sdf_text)


def _strip_unwanted(model: ElementTree.Element) -> None:
    """Remove every element the agent must not carry (see module docstring)."""
    for parent in model.iter():
        for child in list(parent):
            if child.tag in _STRIPPED_TAGS:
                parent.remove(child)


def _rewrite_mesh_uris(model: ElementTree.Element, model_name: str) -> None:
    """Point every mesh at this model's own ``meshes/`` directory.

    Upstream URIs are either ``model://<upstream_pkg>/meshes/x`` or a bare
    relative ``meshes/x``; both become ``model://<model_name>/meshes/x`` so the
    model resolves through the normal Gazebo resource path with no dependency on
    the upstream package being installed.
    """
    for uri in model.iter("uri"):
        text = (uri.text or "").strip()
        if not text or "meshes/" not in text:
            continue
        uri.text = f"model://{model_name}/meshes/{text.rsplit('meshes/', 1)[1]}"


#: Gazebo-classic script material names, as approximate RGB. Only used for
#: primitive geometry; meshes carry their own materials and are left alone.
_CLASSIC_COLOURS = {
    "Gazebo/Grey": "0.7 0.7 0.7 1",
    "Gazebo/DarkGrey": "0.3 0.3 0.3 1",
    "Gazebo/Black": "0.1 0.1 0.1 1",
    "Gazebo/White": "1 1 1 1",
    "Gazebo/Red": "1 0 0 1",
    "Gazebo/Green": "0 1 0 1",
    "Gazebo/Blue": "0 0 1 1",
    "Gazebo/Yellow": "1 1 0 1",
    "Gazebo/Orange": "1 0.5 0 1",
}


def _fix_classic_materials(model: ElementTree.Element) -> None:
    """Replace Gazebo-classic script materials, which Harmonic cannot load.

    A ``<material><script>`` names a material from ``gazebo.material``, a file
    that only exists in Gazebo-classic. Harmonic's ogre2 cannot resolve it, and
    the script still overrides whatever the geometry would otherwise use — so a
    mesh that ships perfectly good materials of its own ends up rendering wrong.
    The Go1 description does this for all twelve leg visuals, which is why the
    robot appeared incomplete.

    Meshes get the material dropped entirely, so their embedded material applies
    again. Primitives have nothing to fall back on, so the script name is
    translated to an explicit colour and they keep the shade they were given.
    """
    for visual in model.iter("visual"):
        material = visual.find("material")
        if material is None:
            continue
        script = material.find("script")
        if script is None:
            continue
        material.remove(script)
        geometry = visual.find("geometry")
        is_mesh = geometry is not None and geometry.find("mesh") is not None
        name_element = script.find("name")
        colour = _CLASSIC_COLOURS.get((name_element.text or "").strip() if name_element is not None
                                      else "")
        if is_mesh or colour is None:
            # Nothing useful to say about the colour: let the mesh's own
            # material show, rather than flattening it to a guess.
            if not list(material):
                visual.remove(material)
            continue
        ElementTree.SubElement(material, "ambient").text = colour
        ElementTree.SubElement(material, "diffuse").text = colour


def _copy_referenced_meshes(
    model: ElementTree.Element,
    source_dir: Path,
    output_dir: Path,
) -> None:
    """Copy only the meshes the stripped model still points at.

    Upstream description packages ship meshes for every variant of the robot —
    the G1 directory carries 167 files for a model that uses 29 — and stripping
    sensors drops more. Copying the whole directory would commit a large pile of
    binaries that nothing loads.
    """
    if not source_dir.is_dir():
        raise ModelConvertError(f"meshes directory does not exist: '{source_dir}'")
    wanted = {
        (uri.text or "").rsplit("/", 1)[-1]
        for uri in model.iter("uri")
        if "meshes/" in (uri.text or "")
    }
    missing: list[str] = []
    for name in sorted(wanted):
        source = source_dir / name
        if not source.is_file():
            missing.append(name)
            continue
        shutil.copy2(source, output_dir / name)
    if missing:
        raise ModelConvertError(
            f"meshes referenced by the model are not in '{source_dir}': {', '.join(missing)}")


def _disable_gravity(model: ElementTree.Element) -> None:
    for link in model.iter("link"):
        existing = link.find("gravity")
        if existing is None:
            existing = ElementTree.SubElement(link, "gravity")
        existing.text = "false"


def _check_joints(
    model: ElementTree.Element,
    animated_joints: Sequence[str],
    urdf_file: Path,
) -> None:
    """Fail loudly when a configured joint is not in the converted model.

    A typo here would otherwise show up only as a leg that never moves.
    """
    present = {joint.get("name") for joint in model.iter("joint")}
    missing = [name for name in animated_joints if name not in present]
    if missing:
        raise ModelConvertError(
            f"'{urdf_file.name}' has no joint(s) {', '.join(missing)}; "
            f"available: {', '.join(sorted(name for name in present if name))}")


def _add_camera(model: ElementTree.Element, camera, model_name: str) -> None:
    """Give the agent one forward-facing camera on its root link.

    The upstream sensors were stripped wholesale, so this adds back exactly one
    deliberate sensor rather than inheriting whatever the description shipped.
    It is mounted on the root link — the trunk of the quadruped, the pelvis of
    the humanoid — so the view is carried by the body and is not swung around by
    the animated legs.
    """
    root = model.find("link")
    if root is None or not root.get("name"):
        raise ModelConvertError(f"model '{model_name}' has no link to mount a camera on")
    root_name = root.get("name")
    link_name = "front_camera_link"

    link = ElementTree.SubElement(model, "link", {"name": link_name})
    x, y, z = camera.pose
    pose = ElementTree.SubElement(link, "pose", {"relative_to": root_name})
    pose.text = f"{x} {y} {z} 0 {camera.pitch_rad} 0"
    ElementTree.SubElement(link, "gravity").text = "false"

    inertial = ElementTree.SubElement(link, "inertial")
    ElementTree.SubElement(inertial, "mass").text = "0.05"
    inertia = ElementTree.SubElement(inertial, "inertia")
    for axis, value in (("ixx", "1e-5"), ("iyy", "1e-5"), ("izz", "1e-5"),
                        ("ixy", "0"), ("ixz", "0"), ("iyz", "0")):
        ElementTree.SubElement(inertia, axis).text = value

    visual = ElementTree.SubElement(link, "visual", {"name": "housing"})
    geometry = ElementTree.SubElement(visual, "geometry")
    ElementTree.SubElement(ElementTree.SubElement(geometry, "box"), "size").text = "0.05 0.03 0.03"
    material = ElementTree.SubElement(visual, "material")
    ElementTree.SubElement(material, "diffuse").text = "0.1 0.1 0.1 1"

    sensor = ElementTree.SubElement(link, "sensor", {"name": "front_camera", "type": "camera"})
    ElementTree.SubElement(sensor, "always_on").text = "1"
    ElementTree.SubElement(sensor, "update_rate").text = str(camera.update_rate_hz)
    ElementTree.SubElement(sensor, "topic").text = camera.topic
    ElementTree.SubElement(sensor, "gz_frame_id").text = link_name
    camera_element = ElementTree.SubElement(sensor, "camera", {"name": "front_camera"})
    ElementTree.SubElement(camera_element, "horizontal_fov").text = str(camera.hfov_rad)
    image = ElementTree.SubElement(camera_element, "image")
    ElementTree.SubElement(image, "width").text = str(camera.width)
    ElementTree.SubElement(image, "height").text = str(camera.height)
    ElementTree.SubElement(image, "format").text = "R8G8B8"
    clip = ElementTree.SubElement(camera_element, "clip")
    ElementTree.SubElement(clip, "near").text = "0.05"
    ElementTree.SubElement(clip, "far").text = "100.0"

    joint = ElementTree.SubElement(
        model, "joint", {"name": f"{root_name}_front_camera_joint", "type": "fixed"})
    ElementTree.SubElement(joint, "parent").text = root_name
    ElementTree.SubElement(joint, "child").text = link_name


def _add_control_plugins(
    model: ElementTree.Element,
    model_name: str,
    animated_joints: Sequence[str],
    rest_angle,
) -> None:
    """Add the base velocity control and a position controller for every joint.

    *Every* joint, not just the animated ones. With gravity and collisions gone,
    a joint nobody holds is free: the reaction torques of the driven joints fling
    it about and nothing ever damps it out, which is what had the humanoid waving
    its arms and the quadruped splaying its legs. Joints the gait does not drive
    are pinned at their rest angle through ``initial_position`` and given no
    command topic, so holding them costs no messages at all.
    """
    velocity = ElementTree.SubElement(model, "plugin", {
        "filename": "gz-sim-velocity-control-system",
        "name": "gz::sim::systems::VelocityControl",
    })
    ElementTree.SubElement(velocity, "topic").text = base_command_topic(model_name)

    animated = set(animated_joints)
    for joint in model.iter("joint"):
        name = joint.get("name")
        if not name or joint.get("type") not in ("revolute", "prismatic"):
            continue
        plugin = ElementTree.SubElement(model, "plugin", {
            "filename": "gz-sim-joint-position-controller-system",
            "name": "gz::sim::systems::JointPositionController",
        })
        ElementTree.SubElement(plugin, "joint_name").text = name
        if name in animated:
            ElementTree.SubElement(plugin, "topic").text = joint_command_topic(model_name, name)
        # Start where the agent stands, so it does not snap out of the pose it
        # was spawned in on the first simulation step.
        ElementTree.SubElement(plugin, "initial_position").text = f"{rest_angle(name):.6f}"
        # The joints carry only their own inertia here, so a stiff proportional
        # term tracks the gait closely and the derivative term settles it.
        ElementTree.SubElement(plugin, "p_gain").text = "200.0"
        ElementTree.SubElement(plugin, "d_gain").text = "10.0"


def main(argv: Sequence[str] | None = None) -> int:
    """CLI: regenerate one agent model from its upstream description.

    The animated joint list is read from the agents config rather than passed in,
    so the model is always built with controllers for exactly the joints the gait
    drives, and a joint renamed upstream fails here instead of at runtime.
    """
    import argparse

    from agents.config import AgentsConfig

    parser = argparse.ArgumentParser(description="Convert a robot URDF into an agent model.")
    parser.add_argument("--config", required=True, help="agents YAML, for the animated joints")
    parser.add_argument("--agent", required=True, help="agent name in that config")
    parser.add_argument("--urdf", required=True, type=Path, help="upstream URDF file")
    parser.add_argument("--meshes", required=True, type=Path, help="upstream meshes directory")
    parser.add_argument("--output", required=True, type=Path, help="model directory to write")
    parser.add_argument("--description", default="", help="model.config description text")
    arguments = parser.parse_args(argv)

    config = AgentsConfig.load(arguments.config)
    spec = config.agent(arguments.agent)
    written = convert(
        arguments.urdf,
        model_name=spec.model,
        mesh_source_dir=arguments.meshes,
        output_dir=arguments.output,
        animated_joints=spec.gait.joint_names,
        description=arguments.description,
        camera=spec.camera,
        rest_angle=spec.rest_angle,
    )
    camera_note = f", camera on {spec.camera.topic}" if spec.camera else ", no camera"
    print(f"wrote {written} ({len(spec.gait.joint_names)} animated joint(s){camera_note})")
    return 0


def _model_config(model_name: str, description: str) -> str:
    return (
        '<?xml version="1.0"?>\n'
        "<model>\n"
        f"  <name>{model_name}</name>\n"
        "  <version>1.0</version>\n"
        "  <sdf version='1.9'>model.sdf</sdf>\n"
        "  <description>\n"
        f"    {description or model_name}\n"
        "  </description>\n"
        "</model>\n"
    )
