"""Python codec for the QLP serial protocol."""

from __future__ import annotations

from dataclasses import dataclass
import struct
from typing import Iterable, Optional, Union


QLP_SOF = 0x23
QLP_HEADER_SIZE = 6
QLP_CRC_SIZE = 2
QLP_OVERHEAD_SIZE = QLP_HEADER_SIZE + QLP_CRC_SIZE
QLP_MAX_DATA_LEN = 256
QLP_MAX_FRAME_SIZE = QLP_OVERHEAD_SIZE + QLP_MAX_DATA_LEN

FC_JOYSTICK_REPORT = 0x90
JOYSTICK_REPORT_LEN = 18


class QlpProtocolError(ValueError):
    """Base exception for QLP protocol failures."""


class QlpCrcError(QlpProtocolError):
    """Raised when a frame CRC check fails."""


@dataclass(frozen=True)
class QlpFrame:
    """Decoded QLP frame."""

    addr: int
    src: int
    fc: int
    data: bytes = b""

    def __post_init__(self) -> None:
        object.__setattr__(self, "addr", _validate_u8(self.addr, "addr"))
        object.__setattr__(self, "src", _validate_u8(self.src, "src"))
        object.__setattr__(self, "fc", _validate_u8(self.fc, "fc"))

        payload = bytes(self.data)
        if len(payload) > QLP_MAX_DATA_LEN:
            raise QlpProtocolError(
                f"data length {len(payload)} exceeds max {QLP_MAX_DATA_LEN}"
            )
        object.__setattr__(self, "data", payload)

    @property
    def length(self) -> int:
        return len(self.data)


@dataclass(frozen=True)
class JoystickReport:
    """Decoded payload for FC=0x90."""

    mode: int
    joy_lx: float
    joy_ly: float
    joy_rx: float
    joy_ry: float
    key: int

    @property
    def mode_char(self) -> str:
        return _decode_alpha_value(self.mode)

    @property
    def key_char(self) -> str:
        return _decode_key_value(self.key)


def encode_address(device_type: int, instance_id: int) -> int:
    """Pack TYPE and ID into one address byte."""
    if not 0 <= device_type <= 0x0F:
        raise QlpProtocolError("device_type must be in [0, 15]")
    if not 0 <= instance_id <= 0x0F:
        raise QlpProtocolError("instance_id must be in [0, 15]")
    return (device_type << 4) | instance_id


def decode_address(address: int) -> tuple[int, int]:
    """Split an address byte into TYPE and ID."""
    address = _validate_u8(address, "address")
    return (address >> 4) & 0x0F, address & 0x0F


def crc16(data: Union[bytes, bytearray, memoryview]) -> int:
    """CRC-16/MODBUS."""
    crc = 0xFFFF
    for byte in bytes(data):
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def pack_frame(frame: QlpFrame) -> bytes:
    """Encode one QLP frame."""
    header = struct.pack(
        "<BBBBH", QLP_SOF, frame.addr, frame.src, frame.fc, frame.length
    )
    crc = crc16(header[1:] + frame.data)
    return header + frame.data + struct.pack("<H", crc)


def unpack_frame(raw: Union[bytes, bytearray, memoryview]) -> QlpFrame:
    """Decode and validate one complete QLP frame."""
    raw_bytes = bytes(raw)
    if len(raw_bytes) < QLP_OVERHEAD_SIZE:
        raise QlpProtocolError("frame is shorter than minimum size")
    if raw_bytes[0] != QLP_SOF:
        raise QlpProtocolError(f"invalid SOF 0x{raw_bytes[0]:02X}")

    data_len = struct.unpack_from("<H", raw_bytes, 4)[0]
    if data_len > QLP_MAX_DATA_LEN:
        raise QlpProtocolError(f"invalid data length {data_len}")

    expected_size = QLP_OVERHEAD_SIZE + data_len
    if len(raw_bytes) != expected_size:
        raise QlpProtocolError(
            f"frame size {len(raw_bytes)} does not match LEN {data_len}"
        )

    received_crc = struct.unpack_from("<H", raw_bytes, expected_size - QLP_CRC_SIZE)[0]
    calculated_crc = crc16(raw_bytes[1 : expected_size - QLP_CRC_SIZE])
    if received_crc != calculated_crc:
        raise QlpCrcError(
            f"CRC mismatch: received 0x{received_crc:04X}, expected 0x{calculated_crc:04X}"
        )
    # print(raw_bytes[QLP_HEADER_SIZE : expected_size - QLP_CRC_SIZE])
    return QlpFrame(
        addr=raw_bytes[1],
        src=raw_bytes[2],
        fc=raw_bytes[3],
        data=raw_bytes[QLP_HEADER_SIZE : expected_size - QLP_CRC_SIZE],
    )


def pack_joystick_report(
    *,
    addr: int = 0x11,
    src: int = 0x31,
    mode: Union[int, str] = 0,
    joy_lx: float = 0.0,
    joy_ly: float = 0.0,
    joy_rx: float = 0.0,
    joy_ry: float = 0.0,
    key: Union[int, str] = 0,
) -> bytes:
    """Encode a joystick report frame (FC=0x90)."""
    payload = struct.pack(
        "<BffffB",
        _coerce_u8(mode, "mode"),
        float(joy_lx),
        float(joy_ly),
        float(joy_rx),
        float(joy_ry),
        _coerce_u8(key, "key"),
    )
    return pack_frame(QlpFrame(addr=addr, src=src, fc=FC_JOYSTICK_REPORT, data=payload))


def unpack_joystick_report(
    frame_or_raw: Union[QlpFrame, bytes, bytearray, memoryview]
):
    """Decode the payload of a joystick report frame."""
    frame = frame_or_raw if isinstance(frame_or_raw, QlpFrame) else unpack_frame(frame_or_raw)
    if frame.fc != FC_JOYSTICK_REPORT:
        raise QlpProtocolError(f"unexpected FC 0x{frame.fc:02X}, expected 0x90")
    if frame.length != JOYSTICK_REPORT_LEN:
        raise QlpProtocolError(
            f"unexpected joystick payload length {frame.length}, expected {JOYSTICK_REPORT_LEN}"
        )

    mode, joy_lx, joy_ly, joy_rx, joy_ry, key = struct.unpack("<BffffB", frame.data)
    return JoystickReport(
        mode=mode,
        joy_lx=(joy_lx-20)*1/20,
        joy_ly=(joy_ly-20)*1/20,
        joy_rx=(joy_rx-20)*1/20,
        joy_ry=(joy_ry-20)*1/20,
        # joy_lx=joy_lx,
        # joy_ly=joy_ly,
        # joy_rx=joy_rx,
        # joy_ry=joy_ry,
        key=key,
    )


class QlpStreamParser:
    """Streaming parser equivalent to the C state machine."""

    STATE_IDLE = 0
    STATE_HEADER = 1
    STATE_DATA = 2
    STATE_CRC = 3

    def __init__(self) -> None:
        self._state = self.STATE_IDLE
        self._data_len = 0
        self._buffer = bytearray()

    def reset(self) -> None:
        self._state = self.STATE_IDLE
        self._data_len = 0
        self._buffer.clear()

    def parse_byte(self, byte: int) -> Optional[QlpFrame]:
        byte = _validate_u8(byte, "byte")

        if byte == QLP_SOF:
            self._buffer.clear()
            self._buffer.append(byte)
            self._state = self.STATE_HEADER
            self._data_len = 0
            return None

        if self._state == self.STATE_IDLE:
            return None

        if len(self._buffer) >= QLP_MAX_FRAME_SIZE:
            self.reset()
            return None

        self._buffer.append(byte)

        if self._state == self.STATE_HEADER and len(self._buffer) >= QLP_HEADER_SIZE:
            self._data_len = struct.unpack_from("<H", self._buffer, 4)[0]
            if self._data_len > QLP_MAX_DATA_LEN:
                self.reset()
                return None
            self._state = self.STATE_CRC if self._data_len == 0 else self.STATE_DATA

        if (
            self._state == self.STATE_DATA
            and len(self._buffer) >= QLP_HEADER_SIZE + self._data_len
        ):
            self._state = self.STATE_CRC

        frame_len = QLP_OVERHEAD_SIZE + self._data_len
        if self._state == self.STATE_CRC and len(self._buffer) >= frame_len:
            raw = bytes(self._buffer[:frame_len])
            try:
                frame = unpack_frame(raw)
            except QlpProtocolError:
                self.reset()
                return None
            self.reset()
            return frame

        return None

    def parse_bytes(
        self, data: Union[bytes, bytearray, memoryview, Iterable[int]]
    ) -> list[QlpFrame]:
        frames = []
        for byte in data:
            frame = self.parse_byte(byte)
            if frame is not None:
                frames.append(frame)
        return frames


def _validate_u8(value: int, field_name: str) -> int:
    if not 0 <= int(value) <= 0xFF:
        raise QlpProtocolError(f"{field_name} must be in [0, 255]")
    return int(value)


def _coerce_u8(value: Union[int, str], field_name: str) -> int:
    if isinstance(value, str):
        if len(value) != 1:
            raise QlpProtocolError(f"{field_name} string must be a single character")
        return ord(value)
    return _validate_u8(value, field_name)


def _printable_char(value: int) -> str:
    if 32 <= value <= 126:
        return chr(value)
    return ""


def _decode_alpha_value(value: int) -> str:
    if 1 <= value <= 26:
        return chr(ord("a") + value - 1)
    return _printable_char(value)


def _decode_key_value(value: int) -> str:
    if 0x11 <= value <= 0x1F:
        return chr(ord("A") + value - 0x11)
    if 0x01 <= value <= 0x0F:
        return chr(ord("a") + value - 0x01)
    return _printable_char(value)


__all__ = [
    "FC_JOYSTICK_REPORT",
    "JOYSTICK_REPORT_LEN",
    "JoystickReport",
    "QLP_CRC_SIZE",
    "QLP_HEADER_SIZE",
    "QLP_MAX_DATA_LEN",
    "QLP_MAX_FRAME_SIZE",
    "QLP_OVERHEAD_SIZE",
    "QLP_SOF",
    "QlpCrcError",
    "QlpFrame",
    "QlpProtocolError",
    "QlpStreamParser",
    "crc16",
    "decode_address",
    "encode_address",
    "pack_frame",
    "pack_joystick_report",
    "unpack_frame",
    "unpack_joystick_report",
]
