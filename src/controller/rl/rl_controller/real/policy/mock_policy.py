from __future__ import annotations

import threading
from typing import Any

import numpy as np

from ..config import RealPolicyConfig


class MockPolicy:
    """Deterministic policy implementation that requires no model runtime."""

    backend_name = 'mock'

    def __init__(
        self,
        config: RealPolicyConfig,
        *,
        action: Any | None = None,
    ) -> None:
        if config.backend != 'mock':
            raise ValueError('MockPolicy requires policy.backend=mock')
        self.config = config
        self.num_observations = config.num_observations
        self.num_actions = config.num_actions
        self._lock = threading.Lock()
        self._action = np.zeros(self.num_actions, dtype=np.float32)
        self._nan_indices: set[int] = set()
        if action is not None:
            self.set_action(action)

    def infer(self, observation: np.ndarray) -> np.ndarray:
        obs = np.asarray(observation, dtype=np.float32)
        if obs.shape != (self.num_observations,):
            raise ValueError(
                'Mock policy observation must have shape '
                f'({self.num_observations},), got {obs.shape}'
            )
        if not np.all(np.isfinite(obs)):
            raise ValueError('Mock policy observation must contain only finite values')

        with self._lock:
            action = self._action.copy()
            for index in self._nan_indices:
                action[index] = np.nan
            return action

    def set_action(self, action: Any) -> None:
        value = np.asarray(action, dtype=np.float32)
        if value.shape != (self.num_actions,):
            raise ValueError(
                f'action must have shape ({self.num_actions},), got {value.shape}'
            )
        if not np.all(np.isfinite(value)):
            raise ValueError('action must contain only finite values')
        with self._lock:
            self._action = np.array(value, dtype=np.float32, copy=True)

    def inject_nan(self, action_index: int = 0) -> None:
        if isinstance(action_index, bool) or not isinstance(action_index, int):
            raise ValueError('action_index must be an integer')
        if not 0 <= action_index < self.num_actions:
            raise ValueError(
                f'action_index must be in [0, {self.num_actions - 1}]'
            )
        with self._lock:
            self._nan_indices.add(action_index)

    def clear_faults(self) -> None:
        with self._lock:
            self._nan_indices.clear()
