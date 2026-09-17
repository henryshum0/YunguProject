"""Conversion details that decide whether a robot renders correctly."""

from __future__ import annotations

import xml.etree.ElementTree as ElementTree

import pytest

from agents.model_convert import (
    BASE_INERTIA,
    ModelConvertError,
    _anchor_base,
    _disable_gravity,
    _fix_classic_materials,
    _sanitise_vendor_tags,
)
from pathlib import Path

URDF = Path("/tmp/robot.urdf")


def _model(*visuals: str) -> ElementTree.Element:
    return ElementTree.fromstring(f"<model name='r'><link name='l'>{''.join(visuals)}</link></model>")


def _visual(geometry: str, material: str = "") -> str:
    return f"<visual name='v'><geometry>{geometry}</geometry>{material}</visual>"


MESH = "<mesh><uri>model://r/meshes/hip.dae</uri></mesh>"
BOX = "<box><size>1 1 1</size></box>"


def _script(name: str) -> str:
    return (f"<material><script><name>{name}</name>"
            "<uri>file://media/materials/scripts/gazebo.material</uri></script></material>")


class TestClassicMaterials:
    """Gazebo-classic script materials cannot be resolved by Harmonic's ogre2."""

    def test_a_mesh_loses_the_material_so_its_own_one_applies(self) -> None:
        """The Go1 legs ship materials inside the DAE; the script hid them."""
        model = _model(_visual(MESH, _script("Gazebo/DarkGrey")))
        _fix_classic_materials(model)
        visual = model.find(".//visual")
        assert visual.find("material") is None
        assert visual.find("geometry/mesh") is not None

    def test_a_primitive_keeps_its_colour_as_an_explicit_value(self) -> None:
        """A box has nothing to fall back on, so the shade must be preserved."""
        model = _model(_visual(BOX, _script("Gazebo/Red")))
        _fix_classic_materials(model)
        material = model.find(".//visual/material")
        assert material.find("diffuse").text == "1 0 0 1"
        assert material.find("ambient").text == "1 0 0 1"
        assert material.find("script") is None

    def test_an_unknown_script_name_is_dropped_rather_than_guessed(self) -> None:
        model = _model(_visual(BOX, _script("Gazebo/SomethingCustom")))
        _fix_classic_materials(model)
        assert model.find(".//visual/material") is None

    def test_a_material_without_a_script_is_left_alone(self) -> None:
        model = _model(_visual(BOX, "<material><diffuse>0 1 0 1</diffuse></material>"))
        _fix_classic_materials(model)
        assert model.find(".//visual/material/diffuse").text == "0 1 0 1"

    def test_other_material_settings_survive_the_script_removal(self) -> None:
        model = _model(_visual(
            BOX, "<material><script><name>Gazebo/Red</name></script>"
                 "<specular>0 0 1 1</specular></material>"))
        _fix_classic_materials(model)
        material = model.find(".//visual/material")
        assert material.find("specular").text == "0 0 1 1"
        assert material.find("diffuse").text == "1 0 0 1"

    def test_no_material_is_untouched(self) -> None:
        model = _model(_visual(MESH))
        _fix_classic_materials(model)
        assert model.find(".//visual/material") is None


class TestVendorTags:
    def test_double_colon_tags_are_renamed_so_the_document_parses(self) -> None:
        """sdformat emits <gz::x>, which is not well-formed XML at all."""
        text = "<sdf><plugin><gz::corrected_offsets>1</gz::corrected_offsets></plugin></sdf>"
        sanitised = _sanitise_vendor_tags(text)
        assert "gz::" not in sanitised
        ElementTree.fromstring(sanitised)  # must not raise

    def test_documents_without_vendor_tags_are_unchanged(self) -> None:
        text = "<sdf><model name='r'/></sdf>"
        assert _sanitise_vendor_tags(text) == text


@pytest.mark.parametrize("name", ["Gazebo/Grey", "Gazebo/DarkGrey", "Gazebo/Black", "Gazebo/White"])
def test_common_classic_greys_are_all_translated(name: str) -> None:
    model = _model(_visual(BOX, _script(name)))
    _fix_classic_materials(model)
    assert model.find(".//visual/material/diffuse") is not None


def _linked(*links: str) -> ElementTree.Element:
    return ElementTree.fromstring(f"<model name='r'>{''.join(links)}</model>")


class TestGravity:
    """Every link must end up with one gravity tag, and it must be off."""

    def test_a_link_without_a_tag_gets_one(self) -> None:
        model = _linked("<link name='a'/>")
        _disable_gravity(model)
        assert [tag.text for tag in model.find("link").findall("gravity")] == ["false"]

    def test_a_second_tag_is_removed_rather_than_left_behind(self) -> None:
        """sdformat honours the last tag, so a stray 'true' after ours wins.

        This is what left the Go1's trunk falling while its legs did not.
        """
        model = _linked("<link name='a'><gravity>false</gravity><gravity>true</gravity></link>")
        _disable_gravity(model)
        assert [tag.text for tag in model.find("link").findall("gravity")] == ["false"]


class TestBaseAnchor:
    """The base is driven, not simulated; its limbs must not be able to turn it."""

    LEGGED = ("<link name='base'><inertial><mass>5</mass>"
              "<inertia><ixx>0.018</ixx><ixy>0.1</ixy><ixz>0.1</ixz>"
              "<iyy>0.068</iyy><iyz>0.1</iyz><izz>0.077</izz></inertia></inertial></link>"
              "<link name='thigh'><inertial><inertia><ixx>0.001</ixx></inertia></inertial></link>"
              "<joint name='hip' type='revolute'><parent>base</parent><child>thigh</child></joint>")

    def test_the_root_link_is_given_a_large_isotropic_inertia(self) -> None:
        model = _linked(self.LEGGED)
        _anchor_base(model, URDF)
        inertia = model.find("link[@name='base']/inertial/inertia")
        for axis in ("ixx", "iyy", "izz"):
            assert float(inertia.findtext(axis)) == pytest.approx(BASE_INERTIA)
        for product in ("ixy", "ixz", "iyz"):
            assert float(inertia.findtext(product)) == 0.0

    def test_the_limbs_are_left_alone(self) -> None:
        """Only the driven base is fictional; the legs keep their real inertia."""
        model = _linked(self.LEGGED)
        _anchor_base(model, URDF)
        thigh = model.find("link[@name='thigh']/inertial/inertia")
        assert float(thigh.findtext("ixx")) == pytest.approx(0.001)

    def test_the_root_is_the_link_no_joint_is_a_child_of(self) -> None:
        """Not simply the first link: SDF does not promise that order."""
        model = _linked(
            "<link name='thigh'/><link name='base'/>"
            "<joint name='hip' type='revolute'><parent>base</parent><child>thigh</child></joint>")
        _anchor_base(model, URDF)
        assert model.find("link[@name='base']/inertial") is not None
        assert model.find("link[@name='thigh']/inertial") is None

    def test_a_model_with_two_roots_is_rejected(self) -> None:
        """Two roots means there is no single body to drive; say so loudly."""
        with pytest.raises(ModelConvertError, match="2 root links"):
            _anchor_base(_linked("<link name='a'/><link name='b'/>"), URDF)
