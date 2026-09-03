"""Base interfaces shared by skills and ROS-backed primitives."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Generic, TypeVar


RequestT = TypeVar("RequestT")
ResultT = TypeVar("ResultT")


class SkillExecutionError(RuntimeError):
    """A background ROS node rejected or could not execute a request."""


class SkillTimeoutError(TimeoutError):
    """A background ROS node did not become ready or respond in time."""


class Primitive(ABC, Generic[RequestT, ResultT]):
    """A single synchronous operation exposed by a background ROS node."""

    @property
    @abstractmethod
    def name(self) -> str:
        """Stable, human-readable primitive name."""

    @abstractmethod
    def call(self, request: RequestT, *, timeout_sec: float | None = None) -> ResultT:
        """Execute the operation or raise a skill exception."""


class Skill(ABC, Generic[RequestT, ResultT]):
    """Base class for future higher-level compositions of primitives."""

    @property
    @abstractmethod
    def name(self) -> str:
        """Stable, human-readable skill name."""

    @abstractmethod
    def call(self, request: RequestT, *, timeout_sec: float | None = None) -> ResultT:
        """Execute the skill or raise a skill exception."""
