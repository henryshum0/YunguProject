"""Base class for composed high-level skills."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Generic

from skills.primitives.base import RequestT, ResultT


class Skill(ABC, Generic[RequestT, ResultT]):
    """A composition of one or more ROS-backed primitives."""

    @property
    @abstractmethod
    def name(self) -> str:
        """Stable, human-readable skill name."""

    @abstractmethod
    def call(self, request: RequestT, *, timeout_sec: float | None = None) -> ResultT:
        """Execute the skill or raise a skill exception."""
