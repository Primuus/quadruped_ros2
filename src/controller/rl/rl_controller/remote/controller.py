from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .input import RemoteSnapshot


@dataclass(frozen=True, slots=True)
class RemoteKeyEffect:
    reset_sim: bool = False
    zero_last_action: bool = False
    log_message: str | None = None


class RemoteCommandController:
    def __init__(
        self,
        *,
        command_limits: np.ndarray,
        forward_preset: np.ndarray,
        initial_command: np.ndarray | None = None,
        deadzone: float = 0.12,
        speed_trim_scale: float = 0.5,
    ) -> None:
        self.command_limits = np.asarray(command_limits, dtype=np.float32).reshape(3)
        self.forward_preset = self._clip_command(forward_preset)
        self.initial_command = self._clip_command(
            np.zeros(3, dtype=np.float32) if initial_command is None else initial_command
        )
        self.deadzone = float(deadzone)
        self.speed_trim_scale = float(speed_trim_scale)
        self.enabled = True
        self.base_command = self.initial_command.copy()
        self.current_command = np.zeros(3, dtype=np.float32)
        self.last_snapshot: RemoteSnapshot | None = None

    def reset(self) -> None:
        self.enabled = True
        self.base_command = self.initial_command.copy()
        self.current_command.fill(0.0)
        self.last_snapshot = None

    def apply_key(self, key: str) -> RemoteKeyEffect:
        if not key:
            return RemoteKeyEffect()

        raw_key = key[0]
        if raw_key == 'F':
            self.enabled = True
            self.base_command = self.initial_command.copy()
            self.current_command.fill(0.0)
            return RemoteKeyEffect(reset_sim=True, zero_last_action=True, log_message='remote key F: reset simulation')

        key_lower = raw_key.lower()
        if key_lower == 'f':
            self.enabled = True
            return RemoteKeyEffect(zero_last_action=True, log_message='remote key f: arm control')
        if key_lower == 'i':
            self.enabled = False
            self.base_command.fill(0.0)
            self.current_command.fill(0.0)
            return RemoteKeyEffect(zero_last_action=True, log_message='remote key i: disarm control')
        if key_lower == 'h':
            self.enabled = True
            self._set_base_command(np.zeros(3, dtype=np.float32))
            return RemoteKeyEffect(zero_last_action=True, log_message='remote key h: stand preset')
        if key_lower == 'b':
            self.enabled = True
            self._set_base_command(self.forward_preset)
            return RemoteKeyEffect(zero_last_action=True, log_message='remote key b: forward preset')
        if key_lower == 'g':
            self.enabled = True
            self._set_base_command(self._preset_turn_left())
            return RemoteKeyEffect(zero_last_action=True, log_message='remote key g: turn left preset')
        if key_lower == 'c':
            self.enabled = True
            self._set_base_command(self._preset_turn_right())
            return RemoteKeyEffect(zero_last_action=True, log_message='remote key c: turn right preset')
        if key_lower == 'a':
            self.enabled = True
            self._set_base_command(self._preset_strafe_left())
            return RemoteKeyEffect(zero_last_action=True, log_message='remote key a: strafe left preset')
        if key_lower == 'u':
            self.enabled = True
            self._set_base_command(self._preset_strafe_right())
            return RemoteKeyEffect(zero_last_action=True, log_message='remote key u: strafe right preset')
        if key_lower == 'd':
            return RemoteKeyEffect(log_message=self.describe())
        return RemoteKeyEffect(log_message=f'ignored remote key {raw_key!r}')

    def update_snapshot(self, snapshot: RemoteSnapshot | None) -> np.ndarray:
        if snapshot is None:
            self.current_command.fill(0.0)
            return self.current_command.copy()

        self.last_snapshot = snapshot
        if not self.enabled or not snapshot.mode_active:
            self.current_command.fill(0.0)
            return self.current_command.copy()

        # Align the stick axes with the traditional PD convention:
        # left stick Y -> forward/backward, left stick X -> lateral, right stick X -> yaw.
        stick = np.array([-snapshot.joy_ly, snapshot.joy_lx, snapshot.joy_rx], dtype=np.float32)
        stick = self._apply_deadzone(stick)
        speed_trim = float(np.clip(1.0 + snapshot.joy_ry * self.speed_trim_scale, 0.2, 1.8))
        command = self.base_command + stick * self.command_limits * speed_trim
        self.current_command = self._clip_command(command)
        return self.current_command.copy()

    def describe(self) -> str:
        base = ', '.join(f'{value:.3f}' for value in self.base_command)
        current = ', '.join(f'{value:.3f}' for value in self.current_command)
        mode = 'enabled' if self.enabled else 'disabled'
        if self.last_snapshot is None:
            return f'remote {mode}: base=[{base}] current=[{current}]'
        return (
            f'remote {mode}: mode={1.0 if self.last_snapshot.mode_active else 0.0}, '
            f'key={self.last_snapshot.key!r}, base=[{base}], current=[{current}]'
        )

    def _set_base_command(self, command: np.ndarray) -> None:
        self.base_command = self._clip_command(command)

    def _preset_turn_left(self) -> np.ndarray:
        return np.array([0.0, 0.0, 0.5 * float(self.command_limits[2])], dtype=np.float32)

    def _preset_turn_right(self) -> np.ndarray:
        return np.array([0.0, 0.0, -0.5 * float(self.command_limits[2])], dtype=np.float32)

    def _preset_strafe_left(self) -> np.ndarray:
        return np.array([0.0, 0.5 * float(self.command_limits[1]), 0.0], dtype=np.float32)

    def _preset_strafe_right(self) -> np.ndarray:
        return np.array([0.0, -0.5 * float(self.command_limits[1]), 0.0], dtype=np.float32)

    def _apply_deadzone(self, values: np.ndarray) -> np.ndarray:
        filtered = np.zeros_like(values, dtype=np.float32)
        for index, value in enumerate(np.asarray(values, dtype=np.float32)):
            magnitude = float(abs(value))
            if magnitude <= self.deadzone:
                filtered[index] = 0.0
                continue
            filtered[index] = float(np.sign(value) * (magnitude - self.deadzone) / (1.0 - self.deadzone))
        return filtered

    def _clip_command(self, command: np.ndarray) -> np.ndarray:
        command = np.asarray(command, dtype=np.float32).reshape(3)
        return np.clip(command, -self.command_limits, self.command_limits).astype(np.float32)
