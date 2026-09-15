from __future__ import annotations

from collections.abc import Callable
import math
import threading
import time
from typing import Any

import numpy as np

from ..config import RealImuConfig
from ..types import ImuState
from .base import BackendCommunicationError, BackendError


class MockImuBackend:
    """Configurable body-frame IMU source for offline deployment tests."""

    def __init__(
        self,
        config: RealImuConfig,
        *,
        quaternion_wxyz: Any = (1.0, 0.0, 0.0, 0.0),
        angular_velocity: Any = (0.0, 0.0, 0.0),
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        if config.backend != 'mock':
            raise ValueError('MockImuBackend requires imu.backend=mock')
        self.config = config
        self._clock = clock
        self._quaternion = self._validated_quaternion(quaternion_wxyz)
        self._angular_velocity = self._validated_vector(
            angular_velocity,
            3,
            'angular_velocity',
        )

        self._lock = threading.Lock()
        self._is_open = False
        self._stale_age_s = 0.0
        self._tilt_fault: np.ndarray | None = None
        self._nan_fields: set[tuple[str, int]] = set()
        self._invalid_state = False

    @property
    def is_open(self) -> bool:
        with self._lock:
            return self._is_open

    def open(self) -> None:
        self._read_clock()
        with self._lock:
            self._is_open = True

    def read(self) -> ImuState:
        now = self._read_clock()
        with self._lock:
            if not self._is_open:
                raise BackendError('Mock IMU backend is not open')

            quaternion = (
                self._quaternion.copy()
                if self._tilt_fault is None
                else self._tilt_fault.copy()
            )
            angular_velocity = self._angular_velocity.copy()
            for field, index in self._nan_fields:
                if field == 'quaternion_wxyz':
                    quaternion[index] = np.nan
                else:
                    angular_velocity[index] = np.nan

            return ImuState(
                quaternion_wxyz=quaternion,
                angular_velocity=angular_velocity,
                valid=not self._invalid_state,
                timestamp=max(0.0, now - self._stale_age_s),
            )

    def close(self) -> None:
        with self._lock:
            self._is_open = False

    def set_state(
        self,
        *,
        quaternion_wxyz: Any,
        angular_velocity: Any,
    ) -> None:
        quaternion = self._validated_quaternion(quaternion_wxyz)
        velocity = self._validated_vector(
            angular_velocity,
            3,
            'angular_velocity',
        )
        with self._lock:
            self._quaternion = quaternion
            self._angular_velocity = velocity

    def inject_stale_state(self, age_s: float) -> None:
        age = float(age_s)
        if not np.isfinite(age) or age < 0.0:
            raise ValueError('age_s must be finite and non-negative')
        with self._lock:
            self._stale_age_s = age

    def inject_nan(
        self,
        field: str = 'quaternion_wxyz',
        index: int = 0,
    ) -> None:
        sizes = {'quaternion_wxyz': 4, 'angular_velocity': 3}
        if field not in sizes:
            raise ValueError(
                'field must be quaternion_wxyz or angular_velocity'
            )
        if isinstance(index, bool) or not isinstance(index, int):
            raise ValueError('index must be an integer')
        if not 0 <= index < sizes[field]:
            raise ValueError(f'index must be in [0, {sizes[field] - 1}]')
        with self._lock:
            self._nan_fields.add((field, index))

    def inject_fall(
        self,
        *,
        roll_rad: float = math.pi / 2.0,
        pitch_rad: float = 0.0,
        yaw_rad: float = 0.0,
    ) -> None:
        angles = np.asarray([roll_rad, pitch_rad, yaw_rad], dtype=np.float64)
        if not np.all(np.isfinite(angles)):
            raise ValueError('fall angles must be finite')
        quaternion = self._rpy_to_quaternion(*angles)
        with self._lock:
            self._tilt_fault = quaternion

    def inject_invalid_state(self) -> None:
        with self._lock:
            self._invalid_state = True

    def clear_faults(self) -> None:
        with self._lock:
            self._stale_age_s = 0.0
            self._tilt_fault = None
            self._nan_fields.clear()
            self._invalid_state = False

    @staticmethod
    def _validated_vector(value: Any, size: int, field_name: str) -> np.ndarray:
        vector = np.asarray(value, dtype=np.float32)
        if vector.shape != (size,):
            raise ValueError(
                f'{field_name} must have shape ({size},), got {vector.shape}'
            )
        if not np.all(np.isfinite(vector)):
            raise ValueError(f'{field_name} must contain only finite values')
        return np.array(vector, dtype=np.float32, copy=True)

    @classmethod
    def _validated_quaternion(cls, value: Any) -> np.ndarray:
        quaternion = cls._validated_vector(value, 4, 'quaternion_wxyz')
        norm = float(np.linalg.norm(quaternion))
        if not np.isclose(norm, 1.0, rtol=0.0, atol=1e-3):
            raise ValueError('quaternion_wxyz must be normalized')
        return (quaternion / norm).astype(np.float32)

    @staticmethod
    def _rpy_to_quaternion(
        roll_rad: float,
        pitch_rad: float,
        yaw_rad: float,
    ) -> np.ndarray:
        cr = math.cos(roll_rad * 0.5)
        sr = math.sin(roll_rad * 0.5)
        cp = math.cos(pitch_rad * 0.5)
        sp = math.sin(pitch_rad * 0.5)
        cy = math.cos(yaw_rad * 0.5)
        sy = math.sin(yaw_rad * 0.5)
        return np.asarray(
            [
                cr * cp * cy + sr * sp * sy,
                sr * cp * cy - cr * sp * sy,
                cr * sp * cy + sr * cp * sy,
                cr * cp * sy - sr * sp * cy,
            ],
            dtype=np.float32,
        )

    def _read_clock(self) -> float:
        timestamp = float(self._clock())
        if not np.isfinite(timestamp) or timestamp < 0.0:
            raise BackendCommunicationError(
                'Mock IMU clock must return a finite, non-negative value'
            )
        return timestamp
