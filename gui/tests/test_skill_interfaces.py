from __future__ import annotations

import pytest

from gui.skill_interfaces import (
    DEFAULT_SKILL_INTERFACES,
    CoverageSearchSkillInterface,
    NavigateSkillInterface,
    SkillInterface,
)


class _CompleteInterface(SkillInterface):
    tab_title = "Test"

    @property
    def map_instruction(self) -> str:
        return "Test map mode"

    def build(self, frame) -> None:
        self.built_frame = frame

    def on_map_click(self, point) -> None:
        self.point = point


def test_skill_interface_requires_tab_and_map_contracts() -> None:
    with pytest.raises(TypeError):
        SkillInterface(object())

    interface = _CompleteInterface(object())
    assert interface.tab_title == "Test"
    assert interface.map_bounds_points() == ()


def test_default_interfaces_are_pluggable_skill_interface_classes() -> None:
    assert DEFAULT_SKILL_INTERFACES == (
        NavigateSkillInterface,
        CoverageSearchSkillInterface,
    )
    assert all(issubclass(interface, SkillInterface) for interface in DEFAULT_SKILL_INTERFACES)
