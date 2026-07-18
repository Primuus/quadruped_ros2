from __future__ import annotations

from collections.abc import Iterable
from dataclasses import dataclass
from pathlib import Path
import queue
import select
import threading
import time
from typing import Any


@dataclass(frozen=True, slots=True)
class KeyboardSnapshot:
    timestamp: float
    pressed_keys: frozenset[str]


class KeyboardInput:
    """Read local Linux EV_KEY events without depending on a terminal or viewer."""

    def __init__(
        self,
        device_path: str | Path | None = None,
        *,
        tracked_keys: Iterable[str],
        read_timeout: float = 0.1,
    ) -> None:
        try:
            import evdev
        except ImportError as exc:  # pragma: no cover - depends on runtime environment.
            raise RuntimeError(
                'Linux keyboard input requires evdev. Install the deployment package or run: '
                'pip install evdev'
            ) from exc

        self._evdev = evdev
        self._read_timeout = max(0.01, float(read_timeout))
        self._tracked_keys = frozenset(str(key) for key in tracked_keys)
        if not self._tracked_keys:
            raise ValueError('tracked_keys must not be empty')

        self._code_to_key = self._resolve_key_codes(self._tracked_keys)
        selected_paths = self._select_device_paths(device_path)
        devices: list[Any] = []
        try:
            for selected_path in selected_paths:
                device = evdev.InputDevice(selected_path)
                try:
                    self._validate_device_capabilities(device)
                except Exception:
                    device.close()
                    raise
                devices.append(device)
        except Exception:
            for opened_device in devices:
                opened_device.close()
            raise
        self._devices = devices
        self._devices_lock = threading.Lock()
        self._exclusive = False

        self.device_paths = tuple(str(device.path) for device in self._devices)
        self.device_names = tuple(
            str(device.name or 'unknown keyboard')
            for device in self._devices
        )
        self._pressed_keys_by_fd: dict[int, set[str]] = {
            int(device.fd): set()
            for device in self._devices
        }
        self._pressed_keys: set[str] = set()
        self._state_lock = threading.Lock()
        self._key_events: queue.Queue[str] = queue.Queue()
        self._last_event_time = time.monotonic()
        self._last_error: Exception | None = None
        self._running = threading.Event()
        self._running.set()

        self._thread = threading.Thread(
            target=self._read_loop,
            name='linux-keyboard-input',
            daemon=True,
        )
        self._thread.start()

    def _resolve_key_codes(self, tracked_keys: frozenset[str]) -> dict[int, str]:
        code_to_key: dict[int, str] = {}
        unknown_keys: list[str] = []
        for key_name in sorted(tracked_keys):
            key_code = getattr(self._evdev.ecodes, key_name, None)
            if not isinstance(key_code, int):
                unknown_keys.append(key_name)
                continue
            code_to_key[key_code] = key_name
        if unknown_keys:
            raise ValueError(f'Unknown Linux key names: {", ".join(unknown_keys)}')
        if len(code_to_key) != len(tracked_keys):
            raise ValueError('Configured keyboard bindings resolve to duplicate Linux key codes')
        return code_to_key

    def _select_device_paths(self, requested_path: str | Path | None) -> list[str]:
        if requested_path is not None:
            path = str(Path(requested_path).expanduser())
            device = self._evdev.InputDevice(path)
            try:
                self._validate_device_capabilities(device)
            finally:
                device.close()
            return [path]

        candidates: list[str] = []
        by_path = Path('/dev/input/by-path')
        if by_path.exists():
            keyboard_paths = sorted(
                by_path.glob('*-event-kbd'),
                key=lambda path: (not path.name.startswith('platform-'), path.name),
            )
            candidates.extend(str(path) for path in keyboard_paths)
        candidates.extend(str(path) for path in self._evdev.list_devices())

        seen_devices: set[str] = set()
        permission_errors: list[str] = []
        selected_paths: list[str] = []
        for path in candidates:
            real_path = str(Path(path).resolve())
            if real_path in seen_devices:
                continue
            seen_devices.add(real_path)
            try:
                device = self._evdev.InputDevice(path)
                try:
                    self._validate_device_capabilities(device)
                finally:
                    device.close()
            except PermissionError:
                permission_errors.append(path)
                continue
            except (FileNotFoundError, OSError, RuntimeError):
                continue
            selected_paths.append(path)

        if selected_paths:
            return selected_paths

        if permission_errors:
            raise PermissionError(
                'Keyboard devices were found but are not readable. Add the user to the input group '
                'or install a udev rule. Do not run the whole RL controller as root. '
                f'Devices: {", ".join(permission_errors)}'
            )
        raise RuntimeError(
            'No readable Linux keyboard device with the configured control keys was found. '
            'Connect a keyboard or pass --keyboard-device /dev/input/by-path/...-event-kbd.'
        )

    def _validate_device_capabilities(self, device: Any) -> None:
        capabilities = device.capabilities().get(self._evdev.ecodes.EV_KEY, [])
        supported_codes = {int(code) for code in capabilities}
        missing_keys = [
            key_name
            for key_code, key_name in self._code_to_key.items()
            if key_code not in supported_codes
        ]
        if missing_keys:
            raise RuntimeError(
                f'Input device {device.path} does not provide required keys: '
                f'{", ".join(sorted(missing_keys))}'
            )

    def _read_loop(self) -> None:
        try:
            while self._running.is_set():
                with self._devices_lock:
                    devices_by_fd = {
                        int(device.fd): device
                        for device in self._devices
                    }
                if not devices_by_fd:
                    raise RuntimeError('All Linux keyboard devices have disconnected')
                ready, _, _ = select.select(
                    list(devices_by_fd),
                    [],
                    [],
                    self._read_timeout,
                )
                if not ready:
                    continue
                for device_fd in ready:
                    device = devices_by_fd[int(device_fd)]
                    try:
                        events = device.read()
                    except BlockingIOError:
                        continue
                    except OSError as exc:
                        if not self._drop_device(int(device_fd)):
                            raise RuntimeError(
                                f'All Linux keyboard devices have disconnected: {exc}'
                            ) from exc
                        continue
                    for event in events:
                        if event.type != self._evdev.ecodes.EV_KEY:
                            continue
                        key_name = self._code_to_key.get(int(event.code))
                        if key_name is None:
                            continue
                        self._handle_key_event(
                            int(device_fd),
                            key_name,
                            int(event.value),
                        )
        except Exception as exc:  # pragma: no cover - requires a device disconnect.
            if self._running.is_set():
                with self._state_lock:
                    self._last_error = exc
        finally:
            with self._state_lock:
                for pressed_keys in self._pressed_keys_by_fd.values():
                    pressed_keys.clear()
                self._pressed_keys.clear()
                self._last_event_time = time.monotonic()
            self._running.clear()

    def _drop_device(self, device_fd: int) -> bool:
        with self._devices_lock:
            remaining_devices: list[Any] = []
            for device in self._devices:
                if int(device.fd) == device_fd:
                    device.close()
                else:
                    remaining_devices.append(device)
            self._devices = remaining_devices
            if not self._devices:
                self._exclusive = False

        with self._state_lock:
            self._pressed_keys_by_fd.pop(device_fd, None)
            self._pressed_keys = set().union(*self._pressed_keys_by_fd.values())
            self._last_event_time = time.monotonic()
        return bool(remaining_devices)

    def _handle_key_event(self, device_fd: int, key_name: str, value: int) -> None:
        with self._state_lock:
            device_pressed_keys = self._pressed_keys_by_fd.setdefault(device_fd, set())
            if value in (1, 2):
                device_pressed_keys.add(key_name)
            elif value == 0:
                device_pressed_keys.discard(key_name)
            else:
                return
            self._pressed_keys = set().union(*self._pressed_keys_by_fd.values())
            self._last_event_time = time.monotonic()

        if value == 1:
            self._key_events.put(key_name)

    def latest_snapshot(self) -> KeyboardSnapshot:
        with self._state_lock:
            return KeyboardSnapshot(
                timestamp=self._last_event_time,
                pressed_keys=frozenset(self._pressed_keys),
            )

    def drain_key_events(self) -> list[str]:
        events: list[str] = []
        while True:
            try:
                events.append(self._key_events.get_nowait())
            except queue.Empty:
                return events

    @property
    def last_error(self) -> Exception | None:
        with self._state_lock:
            return self._last_error

    @property
    def exclusive(self) -> bool:
        with self._devices_lock:
            return self._exclusive

    def set_exclusive(self, enabled: bool) -> None:
        """Prevent other applications from receiving keyboard events while enabled."""
        enabled = bool(enabled)
        with self._devices_lock:
            if self._exclusive == enabled:
                return
            if not enabled:
                for device in self._devices:
                    try:
                        device.ungrab()
                    except OSError:
                        pass
                self._exclusive = False
                return

            if not self._devices:
                raise RuntimeError('No connected keyboard device is available')
            grabbed_devices: list[Any] = []
            try:
                for device in self._devices:
                    device.grab()
                    grabbed_devices.append(device)
            except OSError as exc:
                failed_device_path = str(device.path)
                for grabbed_device in reversed(grabbed_devices):
                    try:
                        grabbed_device.ungrab()
                    except OSError:
                        pass
                raise RuntimeError(
                    f'Could not acquire exclusive access to {failed_device_path}: {exc}'
                ) from exc
            self._exclusive = True

    def close(self) -> None:
        if not hasattr(self, '_running'):
            return
        self._running.clear()
        if hasattr(self, '_thread') and self._thread.is_alive():
            self._thread.join(timeout=self._read_timeout + 0.5)
        with self._devices_lock:
            if self._exclusive:
                for device in self._devices:
                    try:
                        device.ungrab()
                    except OSError:
                        pass
                self._exclusive = False
            for device in self._devices:
                device.close()
            self._devices = []
        if hasattr(self, '_thread') and self._thread.is_alive():
            self._thread.join(timeout=0.5)
