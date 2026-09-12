"""Regression checks for the role-oriented skills package layout."""

from skills import NavigateSkill, Primitive, SearchSkill, Skill
from skills.helper.frames import to_enu_waypoints
from skills.primitives.base import Primitive as PrimitiveBase
from skills.skills.base import Skill as SkillBase
from skills.skills.navigate import NavigateSkill as NavigateSkillImpl
from skills.skills.search import SearchSkill as SearchSkillImpl


def test_public_exports_match_the_role_oriented_subpackages() -> None:
    assert Primitive is PrimitiveBase
    assert Skill is SkillBase
    assert NavigateSkill is NavigateSkillImpl
    assert SearchSkill is SearchSkillImpl
    assert to_enu_waypoints(((1.0, 2.0, 3.0, 0.0),), frame="enu") == (
        (1.0, 2.0, 3.0, 0.0),)
