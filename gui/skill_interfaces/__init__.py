"""Built-in pluggable skill interfaces for the skills test GUI."""

from gui.skill_interfaces.base import NavigationGoal, Point, SkillInterface
from gui.skill_interfaces.coverage_search import CoverageSearchSkillInterface
from gui.skill_interfaces.navigate import NavigateSkillInterface


DEFAULT_SKILL_INTERFACES: tuple[type[SkillInterface], ...] = (
    NavigateSkillInterface,
    CoverageSearchSkillInterface,
)


__all__ = [
    "CoverageSearchSkillInterface",
    "DEFAULT_SKILL_INTERFACES",
    "NavigateSkillInterface",
    "NavigationGoal",
    "Point",
    "SkillInterface",
]
