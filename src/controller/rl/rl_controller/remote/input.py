from __future__ import annotations

from dataclasses import dataclass
import queue
import threading
import time
from typing import Optional

from .qlp_protocol import FC_JOYSTICK_REPORT, QlpFrame, QlpStreamParser, unpack_joystick_report


@dataclass(frozen=True, slots=True)
class RemoteSnapshot:
    timestamp: float
    mode: int
    mode_active: bool
    joy_lx: float
    joy_ly: float
    joy_rx: float
    joy_ry: float
    key: str


class QlpRemoteInput:
    def __init__(self, serial_port: str, baud_rate: int = 115200, read_timeout: float = 0.05) -> None:
        try:
            from serial import Serial, SerialException
        except ImportError as exc:  # pragma: no cover - depends on runtime env.
            raise RuntimeError('pyserial is required when --remote-port is used') from exc

        self.serial_port = str(serial_port)
        self.baud_rate = int(baud_rate)
        self._SerialException = SerialException
        self._serial = Serial(self.serial_port, self.baud_rate, timeout=float(read_timeout))
        self._parser = QlpStreamParser()
        self._latest_snapshot: RemoteSnapshot | None = None
        self._latest_lock = threading.Lock()
        self._key_events: queue.Queue[str] = queue.Queue()
        self._last_key = ''
        self._last_error: Exception | None = None
        self._running = True

        self._thread = threading.Thread(target=self._read_loop, name='qlp-remote-input', daemon=True)
        self._thread.start()

    def _read_loop(self) -> None:
        while self._running:
            try:
                pending = max(1, int(self._serial.in_waiting))
                data = self._serial.read(pending)
                if not data:
                    continue
                for frame in self._parser.parse_bytes(data):
                    self._handle_frame(frame)
            except self._SerialException as exc:
                self._last_error = exc
                self._running = False
                break
            except Exception as exc:  # pragma: no cover - defensive guard.
                self._last_error = exc
                self._running = False
                break

    def _handle_frame(self, frame: QlpFrame) -> None:
        if frame.fc != FC_JOYSTICK_REPORT:
            return

        report = unpack_joystick_report(frame)
        snapshot = RemoteSnapshot(
            timestamp=time.monotonic(),
            mode=report.mode,
            mode_active=self._is_mode_active(report.mode),
            joy_lx=report.joy_lx,
            joy_ly=report.joy_ly,
            joy_rx=report.joy_rx,
            joy_ry=report.joy_ry,
            key=report.key_char,
        )

        with self._latest_lock:
            self._latest_snapshot = snapshot

        if snapshot.key:
            if snapshot.key != self._last_key:
                self._key_events.put(snapshot.key)
            self._last_key = snapshot.key
        else:
            self._last_key = ''

    @staticmethod
    def _is_mode_active(mode_value: int) -> bool:
        if mode_value == 0:
            return False
        if mode_value == ord('0'):
            return False
        return True

    def latest_snapshot(self) -> RemoteSnapshot | None:
        with self._latest_lock:
            return self._latest_snapshot

    def drain_key_events(self) -> list[str]:
        events: list[str] = []
        while True:
            try:
                events.append(self._key_events.get_nowait())
            except queue.Empty:
                break
        return events

    @property
    def last_error(self) -> Exception | None:
        return self._last_error

    def close(self) -> None:
        self._running = False
        try:
            if getattr(self, '_serial', None) is not None and self._serial.is_open:
                self._serial.close()
        finally:
            if hasattr(self, '_thread') and self._thread.is_alive():
                self._thread.join(timeout=1.0)
