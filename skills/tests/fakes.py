"""Fake ROS node and clock used by the skills unit tests."""

from __future__ import annotations

from types import SimpleNamespace


class FakeClock:
    """A clock the tests advance by hand."""

    def __init__(self, seconds: float = 1000.0) -> None:
        self.seconds = seconds

    def now(self) -> SimpleNamespace:
        return SimpleNamespace(nanoseconds=int(self.seconds * 1e9))


class FakeNode:
    """Records subscriptions so a test can deliver messages to them directly."""

    def __init__(self, seconds: float = 1000.0) -> None:
        self.clock = FakeClock(seconds)
        self.subscriptions: dict[str, object] = {}

    def create_subscription(self, message_type, topic, callback, qos):
        self.subscriptions[topic] = callback
        return SimpleNamespace(topic=topic, message_type=message_type, qos=qos)

    def get_clock(self) -> FakeClock:
        return self.clock

    def deliver(self, topic: str, message) -> None:
        """Invoke the subscription callback registered for ``topic``."""
        self.subscriptions[topic](message)

    def advance(self, seconds: float) -> None:
        self.clock.seconds += seconds
