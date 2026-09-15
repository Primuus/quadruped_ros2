from __future__ import annotations

import copy
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import yaml


RL_PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REAL_CONFIG_ROOT = RL_PACKAGE_ROOT / 'config' / 'real'
MOCK_CONFIG_PATH = REAL_CONFIG_ROOT / 'mock.yaml'
REAL_TEMPLATE_PATH = REAL_CONFIG_ROOT / 'custom_quadruped.example.yaml'
sys.path.insert(0, str(RL_PACKAGE_ROOT))

from rl_controller.real import RealConfigError, RealDeploymentConfig


class RealDeploymentConfigTest(unittest.TestCase):
    def setUp(self) -> None:
        with MOCK_CONFIG_PATH.open('r', encoding='utf-8') as handle:
            self.mock_raw = yaml.safe_load(handle)
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary_directory.cleanup)
        self._config_index = 0

    def write_config(self, raw: dict) -> Path:
        self._config_index += 1
        path = (
            Path(self.temporary_directory.name)
            / f'config_{self._config_index}.yaml'
        )
        with path.open('w', encoding='utf-8') as handle:
            yaml.safe_dump(raw, handle, sort_keys=False)
        return path

    def mutated_config(self) -> dict:
        return copy.deepcopy(self.mock_raw)

    def test_mock_config_parses_without_hardware(self) -> None:
        config = RealDeploymentConfig.from_yaml(MOCK_CONFIG_PATH)

        self.assertEqual(config.schema_version, 1)
        self.assertEqual(config.package_root, RL_PACKAGE_ROOT.resolve())
        self.assertEqual(config.robot.num_actions, 12)
        self.assertEqual(config.robot.num_one_step_obs, 45)
        self.assertEqual(config.robot.history_length, 6)
        self.assertEqual(config.policy.num_observations, 270)
        self.assertEqual(config.motors.backend, 'mock')
        self.assertEqual(config.imu.backend, 'mock')
        self.assertFalse(config.robot.default_angles.flags.writeable)
        with self.assertRaises(TypeError):
            config.keyboard.bindings['forward'] = 'KEY_UP'  # type: ignore[index]

    def test_unsupported_schema_version_is_rejected(self) -> None:
        raw = self.mutated_config()
        raw['schema_version'] = 2

        with self.assertRaisesRegex(
            RealConfigError,
            'Unsupported schema_version=2',
        ):
            RealDeploymentConfig.from_yaml(self.write_config(raw))

    def test_observation_dimension_relation_is_enforced(self) -> None:
        raw = self.mutated_config()
        raw['robot']['num_obs'] = 269

        with self.assertRaisesRegex(
            RealConfigError,
            r'robot\.num_obs must equal',
        ):
            RealDeploymentConfig.from_yaml(self.write_config(raw))

    def test_onnx_path_is_resolved_relative_to_config_root(self) -> None:
        raw = self.mutated_config()
        raw['policy']['backend'] = 'onnx'
        raw['policy']['model_path'] = 'policy.onnx'
        model_path = Path(self.temporary_directory.name) / 'policy.onnx'
        model_path.touch()

        config = RealDeploymentConfig.from_yaml(self.write_config(raw))
        self.assertEqual(config.policy.model_path, model_path.resolve())

    def test_missing_onnx_model_is_rejected(self) -> None:
        raw = self.mutated_config()
        raw['policy']['backend'] = 'onnx'
        raw['policy']['model_path'] = 'missing.onnx'

        with self.assertRaisesRegex(
            RealConfigError,
            r'policy\.model_path does not exist',
        ):
            RealDeploymentConfig.from_yaml(self.write_config(raw))

    def test_real_template_rejects_unfilled_values(self) -> None:
        with self.assertRaisesRegex(
            RealConfigError,
            r'robot\.default_angles\[0\] is required',
        ):
            RealDeploymentConfig.from_yaml(REAL_TEMPLATE_PATH)

    def test_duplicate_motor_id_on_same_bus_is_rejected(self) -> None:
        raw = self.mutated_config()
        raw['motors']['joints'][1]['motor_id'] = 0

        with self.assertRaisesRegex(RealConfigError, 'duplicates motor ID 0'):
            RealDeploymentConfig.from_yaml(self.write_config(raw))

    def test_wrong_robot_array_length_is_rejected(self) -> None:
        raw = self.mutated_config()
        raw['robot']['default_angles'].pop()

        with self.assertRaisesRegex(
            RealConfigError,
            r'robot\.default_angles must contain 12 values',
        ):
            RealDeploymentConfig.from_yaml(self.write_config(raw))

    def test_invalid_motor_direction_is_rejected(self) -> None:
        raw = self.mutated_config()
        raw['motors']['joints'][3]['direction'] = 0

        with self.assertRaisesRegex(
            RealConfigError,
            r'motors\.joints\[3\]\.direction must be -1 or 1',
        ):
            RealDeploymentConfig.from_yaml(self.write_config(raw))

    def test_unknown_motor_bus_is_rejected(self) -> None:
        raw = self.mutated_config()
        raw['motors']['joints'][5]['bus'] = 'missing_bus'

        with self.assertRaisesRegex(RealConfigError, 'references unknown bus'):
            RealDeploymentConfig.from_yaml(self.write_config(raw))

    def test_default_angle_outside_joint_limit_is_rejected(self) -> None:
        raw = self.mutated_config()
        raw['robot']['default_angles'][2] = -4.0

        with self.assertRaisesRegex(
            RealConfigError,
            r'robot\.default_angles\[2\].*is outside',
        ):
            RealDeploymentConfig.from_yaml(self.write_config(raw))

    def test_real_motor_backend_cannot_use_mock_policy(self) -> None:
        raw = self.mutated_config()
        raw['motors']['backend'] = 'unitree_m8010'

        with self.assertRaisesRegex(
            RealConfigError,
            'unitree_m8010 cannot be used with policy.backend=mock',
        ):
            RealDeploymentConfig.from_yaml(self.write_config(raw))

    def test_real_motor_backend_requires_bus_device(self) -> None:
        raw = self.mutated_config()
        raw['policy']['backend'] = 'onnx'
        raw['policy']['model_path'] = 'policy.onnx'
        Path(self.temporary_directory.name, 'policy.onnx').touch()
        raw['motors']['backend'] = 'unitree_m8010'
        raw['motors']['buses'][0]['baud_rate'] = 4_800_000
        raw['imu']['backend'] = 'yesense_yis130'
        raw['imu']['device'] = '/dev/yesense_imu'
        raw['imu']['baud_rate'] = 460_800

        with self.assertRaisesRegex(
            RealConfigError,
            r'motors\.buses\[0\]\.device is required',
        ):
            RealDeploymentConfig.from_yaml(self.write_config(raw))

    def test_unknown_fields_are_rejected(self) -> None:
        raw = self.mutated_config()
        raw['safety']['max_torque'] = 10.0

        with self.assertRaisesRegex(
            RealConfigError,
            'safety contains unknown fields: max_torque',
        ):
            RealDeploymentConfig.from_yaml(self.write_config(raw))

    def test_config_import_does_not_load_optional_runtime_modules(self) -> None:
        environment = os.environ.copy()
        environment['PYTHONPATH'] = str(RL_PACKAGE_ROOT)
        script = (
            'import sys\n'
            'from rl_controller.real.config import RealDeploymentConfig\n'
            "assert 'mujoco' not in sys.modules\n"
            "assert 'serial' not in sys.modules\n"
            "assert 'unitree_actuator_sdk' not in sys.modules\n"
            'assert RealDeploymentConfig is not None\n'
        )

        result = subprocess.run(
            [sys.executable, '-c', script],
            check=False,
            capture_output=True,
            text=True,
            env=environment,
        )
        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == '__main__':
    unittest.main()
