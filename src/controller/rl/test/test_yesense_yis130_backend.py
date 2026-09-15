from __future__ import annotations

from dataclasses import replace
import math
import struct
import sys
import unittest
from pathlib import Path
from typing import Any

import numpy as np


RL_PACKAGE_ROOT = Path(__file__).resolve().parents[1]
MOCK_CONFIG_PATH = RL_PACKAGE_ROOT / 'config' / 'real' / 'mock.yaml'
sys.path.insert(0, str(RL_PACKAGE_ROOT))

from rl_controller.real import RealDeploymentConfig
from rl_controller.real.hardware import (
    BackendCommunicationError,
    BackendError,
    ImuBackend,
    YesenseFrameParser,
    YesenseProtocolError,
    YesenseYis130Backend,
    decode_yesense_imu_frame,
)


DOCUMENTED_SAMPLE_FRAME = bytes.fromhex(
    '59 53 34 12 20 '
    '20 0c 80 4a 5d 05 c0 5a 51 fd 00 95 ba 0a '
    '41 10 23 ca 0a 00 00 00 00 00 00 00 00 00 23 ca 0a 00 '
    'be 77'
)


class FakeClock:
    def __init__(self, initial_time: float = 100.0) -> None:
        self.current_time = float(initial_time)

    def __call__(self) -> float:
        return self.current_time

    def advance(self, duration_s: float) -> None:
        self.current_time += float(duration_s)


class FakeSerial:
    def __init__(
        self,
        *,
        clock: FakeClock,
        chunks: list[bytes] | None = None,
        read_advance_s: float = 0.001,
        empty_advance_s: float = 0.005,
    ) -> None:
        self.clock = clock
        self.chunks = list(chunks or [])
        self.read_advance_s = float(read_advance_s)
        self.empty_advance_s = float(empty_advance_s)
        self.closed = False
        self.raise_on_read = False
        self.reset_called = False

    @property
    def in_waiting(self) -> int:
        return len(self.chunks[0]) if self.chunks else 0

    def reset_input_buffer(self) -> None:
        self.reset_called = True

    def read(self, size: int) -> bytes:
        if self.raise_on_read:
            raise OSError('injected read failure')
        if not self.chunks:
            self.clock.advance(self.empty_advance_s)
            return b''
        chunk = self.chunks.pop(0)
        result = chunk[:size]
        remainder = chunk[size:]
        if remainder:
            self.chunks.insert(0, remainder)
        self.clock.advance(self.read_advance_s)
        return result

    def close(self) -> None:
        self.closed = True


def make_frame(tid: int, packets: list[tuple[int, bytes]]) -> bytes:
    message = b''.join(
        bytes((data_id, len(payload))) + payload
        for data_id, payload in packets
    )
    checksum_data = struct.pack('<H', tid) + bytes((len(message),)) + message
    ck1 = 0
    ck2 = 0
    for value in checksum_data:
        ck1 = (ck1 + value) & 0xFF
        ck2 = (ck2 + ck1) & 0xFF
    return b'\x59\x53' + checksum_data + bytes((ck1, ck2))


def make_imu_frame(
    *,
    tid: int = 1,
    angular_values: tuple[int, int, int] = (0, 0, 0),
    quaternion_values: tuple[int, int, int, int] = (1_000_000, 0, 0, 0),
) -> bytes:
    return make_frame(
        tid,
        [
            (0x20, struct.pack('<3i', *angular_values)),
            (0x41, struct.pack('<4i', *quaternion_values)),
        ],
    )


class YesenseFrameParserTest(unittest.TestCase):
    def test_parses_documented_layout_incrementally(self) -> None:
        parser = YesenseFrameParser()

        self.assertEqual(parser.feed(b'noise' + DOCUMENTED_SAMPLE_FRAME[:9]), [])
        frames = parser.feed(DOCUMENTED_SAMPLE_FRAME[9:])

        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].tid, 0x1234)
        quaternion, angular_velocity = decode_yesense_imu_frame(
            frames[0],
            quaternion_order='wxyz',
            angular_velocity_unit='deg/s',
        )
        np.testing.assert_allclose(
            quaternion,
            [math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5)],
            atol=2e-6,
        )
        np.testing.assert_allclose(
            angular_velocity,
            [math.pi / 2.0, -math.pi / 4.0, math.pi],
            atol=1e-6,
        )
        self.assertEqual(parser.discarded_byte_count, 5)

    def test_recovers_after_bad_checksum(self) -> None:
        parser = YesenseFrameParser()
        corrupted = bytearray(DOCUMENTED_SAMPLE_FRAME)
        corrupted[-1] ^= 0xFF

        frames = parser.feed(corrupted + DOCUMENTED_SAMPLE_FRAME)

        self.assertEqual([frame.tid for frame in frames], [0x1234])
        self.assertEqual(parser.checksum_error_count, 1)

    def test_reorders_xyzw_and_rejects_invalid_payload(self) -> None:
        frame_bytes = make_imu_frame(
            quaternion_values=(0, 707_107, 0, 707_107),
        )
        frame = YesenseFrameParser().feed(frame_bytes)[0]

        quaternion, _ = decode_yesense_imu_frame(
            frame,
            quaternion_order='xyzw',
            angular_velocity_unit='rad/s',
        )
        np.testing.assert_allclose(
            quaternion,
            [math.sqrt(0.5), 0.0, math.sqrt(0.5), 0.0],
            atol=2e-6,
        )

        quaternion_only = YesenseFrameParser().feed(
            make_frame(2, [(0x41, struct.pack('<4i', 1_000_000, 0, 0, 0))])
        )[0]
        with self.assertRaisesRegex(YesenseProtocolError, 'missing angular'):
            decode_yesense_imu_frame(
                quaternion_only,
                quaternion_order='wxyz',
                angular_velocity_unit='deg/s',
            )


class YesenseYis130BackendTest(unittest.TestCase):
    def setUp(self) -> None:
        base = RealDeploymentConfig.from_yaml(MOCK_CONFIG_PATH)
        self.clock = FakeClock()
        self.config = replace(
            base.imu,
            backend='yesense_yis130',
            device='/dev/fake-yis130',
            baud_rate=460_800,
            quaternion_order='wxyz',
            mounting_quaternion_wxyz=np.array(
                [1.0, 0.0, 0.0, 0.0],
                dtype=np.float32,
            ),
            angular_velocity_unit='deg/s',
            state_timeout_s=0.03,
        )

    def make_backend(
        self,
        fake_serial: FakeSerial,
        config: Any | None = None,
    ) -> tuple[YesenseYis130Backend, dict[str, Any]]:
        factory_arguments: dict[str, Any] = {}

        def factory(**kwargs: Any) -> FakeSerial:
            factory_arguments.update(kwargs)
            return fake_serial

        backend = YesenseYis130Backend(
            config or self.config,
            serial_factory=factory,
            clock=self.clock,
        )
        return backend, factory_arguments

    def test_reads_body_frame_state_and_reports_health(self) -> None:
        fake_serial = FakeSerial(
            clock=self.clock,
            chunks=[DOCUMENTED_SAMPLE_FRAME],
        )
        backend, factory_arguments = self.make_backend(fake_serial)
        self.assertIsInstance(backend, ImuBackend)
        with self.assertRaisesRegex(BackendError, 'not open'):
            backend.read()

        backend.open()
        state = backend.read()

        self.assertEqual(factory_arguments['port'], '/dev/fake-yis130')
        self.assertEqual(factory_arguments['baudrate'], 460_800)
        self.assertEqual(factory_arguments['timeout'], 0.005)
        self.assertTrue(fake_serial.reset_called)
        self.assertEqual(backend.last_frame_tid, 0x1234)
        self.assertIsNone(backend.last_error)
        self.assertEqual(state.timestamp, 100.001)
        self.assertAlmostEqual(backend.last_state_age_s, 0.0)
        self.assertTrue(state.valid)
        np.testing.assert_allclose(
            state.angular_velocity,
            [math.pi / 2.0, -math.pi / 4.0, math.pi],
            atol=1e-6,
        )

        backend.close()
        self.assertTrue(fake_serial.closed)
        self.assertFalse(backend.is_open)

    def test_applies_sensor_to_body_mounting_rotation(self) -> None:
        half_sqrt = math.sqrt(0.5)
        mounting = np.array(
            [half_sqrt, 0.0, 0.0, half_sqrt],
            dtype=np.float32,
        )
        config = replace(
            self.config,
            mounting_quaternion_wxyz=mounting,
        )
        fake_serial = FakeSerial(
            clock=self.clock,
            chunks=[make_imu_frame(angular_values=(90_000_000, 0, 0))],
        )
        backend, _ = self.make_backend(fake_serial, config)
        backend.open()

        state = backend.read()

        np.testing.assert_allclose(
            state.quaternion_wxyz,
            [half_sqrt, 0.0, 0.0, -half_sqrt],
            atol=1e-6,
        )
        np.testing.assert_allclose(
            state.angular_velocity,
            [0.0, math.pi / 2.0, 0.0],
            atol=1e-6,
        )

    def test_skips_bad_checksum_then_uses_next_frame(self) -> None:
        corrupted = bytearray(DOCUMENTED_SAMPLE_FRAME)
        corrupted[-2] ^= 0x01
        fake_serial = FakeSerial(
            clock=self.clock,
            chunks=[bytes(corrupted) + DOCUMENTED_SAMPLE_FRAME],
        )
        backend, _ = self.make_backend(fake_serial)
        backend.open()

        state = backend.read()

        self.assertTrue(state.valid)
        self.assertEqual(backend.checksum_error_count, 1)
        self.assertGreater(backend.discarded_byte_count, 0)

    def test_times_out_after_incomplete_protocol_frames(self) -> None:
        quaternion_only = make_frame(
            7,
            [(0x41, struct.pack('<4i', 1_000_000, 0, 0, 0))],
        )
        fake_serial = FakeSerial(
            clock=self.clock,
            chunks=[quaternion_only],
        )
        backend, _ = self.make_backend(fake_serial)
        backend.open()

        with self.assertRaisesRegex(
            BackendCommunicationError,
            'missing angular velocity',
        ):
            backend.read()

        self.assertIn('Timed out', backend.last_error or '')
        self.assertIsNone(backend.last_state_age_s)

    def test_reports_timeout_and_serial_errors(self) -> None:
        timeout_serial = FakeSerial(clock=self.clock)
        timeout_backend, _ = self.make_backend(timeout_serial)
        timeout_backend.open()
        with self.assertRaisesRegex(BackendCommunicationError, 'Timed out'):
            timeout_backend.read()

        failing_serial = FakeSerial(clock=self.clock)
        failing_serial.raise_on_read = True
        failing_backend, _ = self.make_backend(failing_serial)
        failing_backend.open()
        with self.assertRaisesRegex(BackendCommunicationError, 'serial read failed'):
            failing_backend.read()


if __name__ == '__main__':
    unittest.main()
