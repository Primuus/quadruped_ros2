from __future__ import annotations

from typing import Protocol, runtime_checkable

from ..types import ImuState, JointCommand, JointState


class BackendError(RuntimeError):
    """Base exception for real deployment input and output backends."""


class BackendUnavailableError(BackendError):
    """Raised when a required SDK, device, or operating-system resource is missing."""


class BackendCommunicationError(BackendError):
    """Raised when an opened backend cannot exchange a complete state."""


@runtime_checkable
class MotorBackend(Protocol):
    """Hardware-independent interface for one complete motor I/O cycle."""

    @property
    def num_joints(self) -> int:
        ...

    def open(self) -> None:
        ...

    def cycle(self, command: JointCommand) -> JointState:
        ...

    def safe_stop(self) -> None:
        ...

    def close(self) -> None:
        ...


@runtime_checkable
class ImuBackend(Protocol):
    """Hardware-independent interface for body-frame IMU snapshots."""

    def open(self) -> None:
        ...

    def read(self) -> ImuState:
        ...

    def close(self) -> None:
        ...
