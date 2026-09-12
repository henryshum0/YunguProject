"""Trigger-service primitives for the offboard FSM flight controls."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from skills.primitives.base import Primitive, SkillExecutionError, SkillTimeoutError
from skills.config import SkillRuntimeConfig


class _FlightCommandPrimitive(Primitive[None, str]):
    """Common synchronous client behavior for an offboard Trigger service."""

    _operation = "flight command"

    def __init__(self, node: Node, *, service_name: str) -> None:
        self._node = node
        self._service_name = service_name
        self._client = node.create_client(Trigger, service_name)

    @property
    def service_name(self) -> str:
        return self._service_name

    def call(self, request: None = None, *, timeout_sec: float | None = 10.0) -> str:
        if request is not None:
            raise ValueError(f"{self.name} does not accept a request payload")
        if not self._client.wait_for_service(timeout_sec=timeout_sec):
            raise SkillTimeoutError(
                f"{self._operation} service '{self._service_name}' is unavailable")

        future = self._client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=timeout_sec)
        if not future.done():
            future.cancel()
            raise SkillTimeoutError(
                f"{self._operation} service '{self._service_name}' did not respond")
        try:
            response = future.result()
        except Exception as exc:
            raise SkillExecutionError(
                f"{self._operation} service '{self._service_name}' failed: {exc}") from exc
        if response is None:
            raise SkillExecutionError(f"{self._operation} service returned no response")
        if not response.success:
            raise SkillExecutionError(response.message or f"{self._operation} request was rejected")
        return response.message or f"{self._operation} accepted"


class TakeoffPrimitive(_FlightCommandPrimitive):
    """Request the normal offboard FSM arming and takeoff sequence."""

    _operation = "takeoff"

    def __init__(self, node: Node, *, config: SkillRuntimeConfig) -> None:
        super().__init__(node, service_name=config.offboard.takeoff_service)

    @property
    def name(self) -> str:
        return "takeoff"


class LandPrimitive(_FlightCommandPrimitive):
    """Request native PX4 landing through the offboard FSM."""

    _operation = "land"

    def __init__(self, node: Node, *, config: SkillRuntimeConfig) -> None:
        super().__init__(node, service_name=config.offboard.land_service)

    @property
    def name(self) -> str:
        return "land"
