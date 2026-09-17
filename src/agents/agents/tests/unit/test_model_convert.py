"""Conversion details that decide whether a robot renders correctly."""

from __future__ import annotations

import xml.etree.ElementTree as ElementTree

import pytest

from agents.model_convert import _fix_classic_materials, _sanitise_vendor_tags


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
