from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np


RL_PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RL_PACKAGE_ROOT))

from rl_controller.real import ImuState, JointCommand, JointState, RealRobotState
from rl_controller.real.hardware import ImuBackend, MotorBackend


def make_joint_state(num_joints: int = 12, timestamp: float = 1.0) -> JointState:
    return JointState(
        position=np.zeros(num_joints),
        velocity=np.zeros(num_joints),
        torque=np.zeros(num_joints),
        temperature=np.full(num_joints, 25.0),
        error=np.zeros(num_joints, dtype=np.int64),
        valid=True,
        timestamp=timestamp,
    )


def make_imu_state(timestamp: float = 1.0) -> ImuState:
    return ImuState(
        quaternion_wxyz=np.array([1.0, 0.0, 0.0, 0.0]),
        angular_velocity=np.zeros(3),
        valid=True,
        timestamp=timestamp,
    )


class RealStateTypeTest(unittest.TestCase):
    def test_joint_state_copies_and_freezes_input(self) -> None:
        source = np.arange(12, dtype=np.float64)
        state = JointState(
            position=source,
            velocity=np.zeros(12),
            torque=np.zeros(12),
            temperature=np.full(12, 25.0),
            error=np.zeros(12),
            valid=True,
            timestamp=1.5,
        )

        source[0] = 100.0
        self.assertEqual(state.position.dtype, np.float32)
        self.assertEqual(state.error.dtype, np.int64)
        self.assertEqual(float(state.position[0]), 0.0)
        self.assertFalse(state.position.flags.writeable)
        self.assertEqual(state.num_joints, 12)

    def test_joint_state_rejects_mismatched_shapes(self) -> None:
        with self.assertRaisesRegex(ValueError, 'velocity must have shape'):
            JointState(
                position=np.zeros(12),
                velocity=np.zeros(11),
                torque=np.zeros(12),
                temperature=np.zeros(12),
                error=np.zeros(12),
                valid=False,
                timestamp=0.0,
            )

    def test_joint_command_validates_every_vector(self) -> None:
        command = JointCommand(
            position=np.zeros(12),
            velocity=np.zeros(12),
            kp=np.full(12, 20.0),
            kd=np.full(12, 0.5),
            torque=np.zeros(12),
            timestamp=2.0,
        )
        self.assertEqual(command.num_joints, 12)
        self.assertFalse(command.kp.flags.writeable)

        with self.assertRaisesRegex(ValueError, 'torque must have shape'):
            JointCommand(
                position=np.zeros(12),
                velocity=np.zeros(12),
                kp=np.zeros(12),
                kd=np.zeros(12),
                torque=np.zeros(10),
                timestamp=2.0,
            )

    def test_imu_and_robot_state_validate_shapes_and_types(self) -> None:
        joints = make_joint_state(timestamp=3.0)
        imu = make_imu_state(timestamp=3.0)
        state = RealRobotState(joints=joints, imu=imu, valid=True, timestamp=3.0)
        self.assertEqual(state.num_joints, 12)
        self.assertFalse(state.imu.quaternion_wxyz.flags.writeable)

        with self.assertRaisesRegex(ValueError, 'quaternion_wxyz must have shape'):
            ImuState(
                quaternion_wxyz=np.zeros(3),
                angular_velocity=np.zeros(3),
                valid=False,
                timestamp=0.0,
            )
        with self.assertRaisesRegex(TypeError, 'joints must be a JointState'):
            RealRobotState(joints=None, imu=imu, valid=False, timestamp=0.0)  # type: ignore[arg-type]

    def test_timestamps_must_be_monotonic_time_values(self) -> None:
        with self.assertRaisesRegex(ValueError, 'finite, non-negative'):
            make_joint_state(timestamp=-1.0)
        with self.assertRaisesRegex(ValueError, 'finite, non-negative'):
            make_imu_state(timestamp=float('nan'))


class BackendProtocolTest(unittest.TestCase):
    def test_structural_backends_match_protocols(self) -> None:
        class FakeMotorBackend:
            num_joints = 12

            def open(self) -> None:
                pass

            def cycle(self, command: JointCommand) -> JointState:
                return make_joint_state(timestamp=command.timestamp)

            def safe_stop(self) -> None:
                pass

            def close(self) -> None:
                pass

        class FakeImuBackend:
            def open(self) -> None:
                pass

            def read(self) -> ImuState:
                return make_imu_state()

            def close(self) -> None:
                pass

        self.assertIsInstance(FakeMotorBackend(), MotorBackend)
        self.assertIsInstance(FakeImuBackend(), ImuBackend)


if __name__ == '__main__':
    unittest.main()
