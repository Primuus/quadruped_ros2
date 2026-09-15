from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

import numpy as np


RL_PACKAGE_ROOT = Path(__file__).resolve().parents[1]
MOCK_CONFIG_PATH = RL_PACKAGE_ROOT / 'config' / 'real' / 'mock.yaml'
sys.path.insert(0, str(RL_PACKAGE_ROOT))

from rl_controller.real import (
    ImuState,
    JointCommand,
    JointState,
    MockImuBackend,
    MockMotorBackend,
    MockPolicy,
    ObservationContinuityError,
    RealDeploymentConfig,
    RealObservationHistory,
    RealRobotState,
    RealStateAggregator,
    StateUnavailableError,
    build_real_one_step_observation,
)


class FakeClock:
    def __init__(self, initial_time: float = 100.0) -> None:
        self.current_time = float(initial_time)

    def __call__(self) -> float:
        return self.current_time

    def advance(self, duration_s: float) -> None:
        self.current_time += float(duration_s)

    def set(self, timestamp: float) -> None:
        self.current_time = float(timestamp)


def make_joint_state(
    *,
    timestamp: float = 100.0,
    num_joints: int = 12,
    valid: bool = True,
    position: np.ndarray | None = None,
    velocity: np.ndarray | None = None,
    error: np.ndarray | None = None,
) -> JointState:
    if position is None:
        position = np.linspace(-0.3, 0.3, num_joints, dtype=np.float32)
    if velocity is None:
        velocity = np.linspace(-1.0, 1.0, num_joints, dtype=np.float32)
    if error is None:
        error = np.zeros(num_joints, dtype=np.int64)
    return JointState(
        position=position,
        velocity=velocity,
        torque=np.zeros(num_joints, dtype=np.float32),
        temperature=np.full(num_joints, 30.0, dtype=np.float32),
        error=error,
        valid=valid,
        timestamp=timestamp,
    )


def make_imu_state(
    *,
    timestamp: float = 100.0,
    valid: bool = True,
    quaternion: np.ndarray | None = None,
    angular_velocity: np.ndarray | None = None,
) -> ImuState:
    if quaternion is None:
        quaternion = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
    if angular_velocity is None:
        angular_velocity = np.array([1.0, 2.0, 3.0], dtype=np.float32)
    return ImuState(
        quaternion_wxyz=quaternion,
        angular_velocity=angular_velocity,
        valid=valid,
        timestamp=timestamp,
    )


def make_robot_state(
    *,
    timestamp: float = 100.0,
    quaternion: np.ndarray | None = None,
) -> RealRobotState:
    return RealRobotState(
        joints=make_joint_state(timestamp=timestamp),
        imu=make_imu_state(timestamp=timestamp, quaternion=quaternion),
        valid=True,
        timestamp=timestamp,
    )


class RealStateAggregatorTest(unittest.TestCase):
    def setUp(self) -> None:
        self.config = RealDeploymentConfig.from_yaml(MOCK_CONFIG_PATH)
        self.clock = FakeClock(100.02)
        self.aggregator = RealStateAggregator.from_config(
            self.config,
            clock=self.clock,
        )

    def test_requires_complete_joint_and_imu_feedback(self) -> None:
        self.assertIsNone(self.aggregator.try_snapshot())
        self.assertIn('motor', self.aggregator.last_rejection or '')

        with self.assertRaisesRegex(ValueError, '11 joints, expected 12'):
            self.aggregator.update_joints(make_joint_state(num_joints=11))

        self.aggregator.update_joints(make_joint_state())
        with self.assertRaisesRegex(StateUnavailableError, 'IMU'):
            self.aggregator.snapshot()

    def test_publishes_oldest_timestamp_after_source_synchronization(self) -> None:
        joints = make_joint_state(timestamp=100.0)
        imu = make_imu_state(timestamp=100.015)
        self.aggregator.update_joints(
            joints,
            source_timestamps={
                'left_bus': 100.0,
                'right_bus': 100.01,
            },
        )
        self.aggregator.update_imu(imu)

        snapshot = self.aggregator.snapshot()

        self.assertTrue(snapshot.valid)
        self.assertIs(snapshot.joints, joints)
        self.assertIs(snapshot.imu, imu)
        self.assertEqual(snapshot.timestamp, 100.0)
        self.assertIsNone(self.aggregator.last_rejection)
        self.assertEqual(
            dict(self.aggregator.motor_source_timestamps),
            {'left_bus': 100.0, 'right_bus': 100.01},
        )

    def test_rejects_mismatched_motor_cycle_timestamp(self) -> None:
        with self.assertRaisesRegex(ValueError, 'oldest motor source'):
            self.aggregator.update_joints(
                make_joint_state(timestamp=100.01),
                source_timestamps={
                    'left_bus': 100.0,
                    'right_bus': 100.01,
                },
            )

    def test_rejects_invalid_nonfinite_stale_and_skewed_states(self) -> None:
        cases = []
        cases.append(
            (
                make_joint_state(valid=False),
                make_imu_state(),
                {'motor': 100.0},
                100.02,
                'motor feedback is marked invalid',
            )
        )
        nan_position = np.zeros(12, dtype=np.float32)
        nan_position[3] = np.nan
        cases.append(
            (
                make_joint_state(position=nan_position),
                make_imu_state(),
                {'motor': 100.0},
                100.02,
                'NaN or Inf',
            )
        )
        cases.append(
            (
                make_joint_state(),
                make_imu_state(valid=False),
                {'motor': 100.0},
                100.02,
                'IMU feedback is marked invalid',
            )
        )
        cases.append(
            (
                make_joint_state(),
                make_imu_state(),
                {'motor': 100.0},
                100.10,
                'old',
            )
        )
        cases.append(
            (
                make_joint_state(),
                make_imu_state(timestamp=100.02),
                {'left_bus': 100.0, 'right_bus': 100.04},
                100.04,
                'source skew',
            )
        )

        for joints, imu, sources, now, message in cases:
            with self.subTest(message=message):
                clock = FakeClock(now)
                aggregator = RealStateAggregator.from_config(
                    self.config,
                    clock=clock,
                )
                aggregator.update_joints(joints, source_timestamps=sources)
                aggregator.update_imu(imu)
                with self.assertRaisesRegex(StateUnavailableError, message):
                    aggregator.snapshot()
                self.assertIsNone(aggregator.try_snapshot())
                self.assertIn(message, aggregator.last_rejection or '')

    def test_leaves_motor_error_codes_for_the_safety_stage(self) -> None:
        error = np.zeros(12, dtype=np.int64)
        error[4] = 17
        self.aggregator.update_joints(make_joint_state(error=error))
        self.aggregator.update_imu(make_imu_state())

        snapshot = self.aggregator.snapshot()

        self.assertEqual(int(snapshot.joints.error[4]), 17)


class RealObservationTest(unittest.TestCase):
    def setUp(self) -> None:
        self.config = RealDeploymentConfig.from_yaml(MOCK_CONFIG_PATH)
        self.clock = FakeClock()

    def test_builds_exact_training_side_45_dimension_order(self) -> None:
        state = make_robot_state()
        command = np.array([0.5, -0.25, 1.0], dtype=np.float32)
        last_action = np.linspace(-0.5, 0.5, 12, dtype=np.float32)

        observation = build_real_one_step_observation(
            state,
            command,
            last_action,
            self.config,
        )

        self.assertEqual(observation.shape, (45,))
        self.assertEqual(observation.dtype, np.float32)
        np.testing.assert_allclose(observation[0:3], [1.0, -0.5, 0.25])
        np.testing.assert_allclose(observation[3:6], [0.25, 0.5, 0.75])
        np.testing.assert_allclose(observation[6:9], [0.0, 0.0, -1.0])
        np.testing.assert_allclose(observation[9:21], state.joints.position)
        np.testing.assert_allclose(
            observation[21:33],
            state.joints.velocity * 0.05,
        )
        np.testing.assert_allclose(observation[33:45], last_action)

    def test_projects_world_gravity_into_the_body_frame(self) -> None:
        half_sqrt = math.sqrt(0.5)
        state = make_robot_state(
            quaternion=np.array(
                [half_sqrt, half_sqrt, 0.0, 0.0],
                dtype=np.float32,
            )
        )

        observation = build_real_one_step_observation(
            state,
            np.zeros(3),
            np.zeros(12),
            self.config,
        )

        np.testing.assert_allclose(observation[6:9], [0.0, -1.0, 0.0], atol=1e-6)

    def test_waits_for_six_continuous_newest_first_frames(self) -> None:
        history = RealObservationHistory(self.config, clock=self.clock)
        result = None
        for frame_index in range(1, 7):
            timestamp = 100.0 + frame_index * 0.02
            self.clock.set(timestamp)
            result = history.update(
                make_robot_state(timestamp=timestamp),
                np.array([float(frame_index), 0.0, 0.0]),
                np.full(12, frame_index / 10.0, dtype=np.float32),
            )
            if frame_index < 6:
                self.assertIsNone(result)

        assert result is not None
        self.assertTrue(history.ready)
        self.assertEqual(history.frames_seen, 6)
        self.assertEqual(result.shape, (270,))
        self.assertEqual(result.dtype, np.float32)
        command_x = result[0::45][:6]
        np.testing.assert_allclose(command_x, [12.0, 10.0, 8.0, 6.0, 4.0, 2.0])
        previous_actions = np.asarray(
            [result[index * 45 + 33] for index in range(6)]
        )
        np.testing.assert_allclose(previous_actions, [0.6, 0.5, 0.4, 0.3, 0.2, 0.1])

    def test_clips_ready_history_like_training_environment(self) -> None:
        history = RealObservationHistory(self.config, clock=self.clock)
        result = None
        for frame_index in range(6):
            timestamp = 100.0 + frame_index * 0.02
            self.clock.set(timestamp)
            state = RealRobotState(
                joints=make_joint_state(timestamp=timestamp),
                imu=make_imu_state(
                    timestamp=timestamp,
                    angular_velocity=np.full(3, 1000.0, dtype=np.float32),
                ),
                valid=True,
                timestamp=timestamp,
            )
            result = history.update(state, np.zeros(3), np.zeros(12))

        assert result is not None
        self.assertLessEqual(
            float(np.max(np.abs(result))),
            self.config.policy.clip_observations,
        )
        np.testing.assert_array_equal(result[3:6], np.full(3, 100.0))

    def test_resets_on_duplicate_or_gapped_state_timestamps(self) -> None:
        history = RealObservationHistory(self.config, clock=self.clock)
        self.clock.set(100.0)
        state = make_robot_state(timestamp=100.0)
        history.update(state, np.zeros(3), np.zeros(12))

        with self.assertRaisesRegex(
            ObservationContinuityError,
            'must increase',
        ):
            history.update(state, np.zeros(3), np.zeros(12))
        self.assertEqual(history.frames_seen, 0)

        self.clock.set(100.1)
        result = history.update(
            make_robot_state(timestamp=100.1),
            np.zeros(3),
            np.zeros(12),
        )
        self.assertIsNone(result)
        self.assertEqual(history.frames_seen, 1)

        self.clock.set(100.2)
        result = history.update(
            make_robot_state(timestamp=100.2),
            np.zeros(3),
            np.zeros(12),
        )
        self.assertIsNone(result)
        self.assertEqual(history.frames_seen, 1)
        self.assertIn('frame gap', history.last_reset_reason or '')


class RealOfflineObservationChainTest(unittest.TestCase):
    def test_mock_hardware_produces_stable_actor_input(self) -> None:
        config = RealDeploymentConfig.from_yaml(MOCK_CONFIG_PATH)
        clock = FakeClock()
        motor = MockMotorBackend(
            config.motors,
            initial_position=config.robot.default_angles,
            clock=clock,
        )
        imu = MockImuBackend(config.imu, clock=clock)
        aggregator = RealStateAggregator.from_config(config, clock=clock)
        history = RealObservationHistory(config, clock=clock)
        policy = MockPolicy(config.policy)
        motor.open()
        imu.open()
        observation = None

        for _ in range(config.runtime.history_warmup_frames):
            clock.advance(1.0 / config.policy.frequency_hz)
            command = JointCommand(
                position=config.robot.default_angles,
                velocity=np.zeros(12),
                kp=np.asarray([joint.kp for joint in config.motors.joints]),
                kd=np.asarray([joint.kd for joint in config.motors.joints]),
                torque=np.zeros(12),
                timestamp=clock(),
            )
            aggregator.update_joints(motor.cycle(command))
            aggregator.update_imu(imu.read())
            observation = history.update(
                aggregator.snapshot(),
                np.zeros(3),
                np.zeros(12),
            )

        assert observation is not None
        self.assertEqual(observation.shape, (270,))
        self.assertTrue(np.all(np.isfinite(observation)))
        action = policy.infer(observation)
        self.assertEqual(action.shape, (12,))
        self.assertTrue(np.all(np.isfinite(action)))


if __name__ == '__main__':
    unittest.main()
