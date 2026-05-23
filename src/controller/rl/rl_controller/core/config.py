from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import yaml

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:  # pragma: no cover - fallback for source-only usage.
    get_package_share_directory = None


DEFAULT_JOINT_NAMES = (
    'FL_hip_joint',
    'FL_thigh_joint',
    'FL_calf_joint',
    'FR_hip_joint',
    'FR_thigh_joint',
    'FR_calf_joint',
    'RL_hip_joint',
    'RL_thigh_joint',
    'RL_calf_joint',
    'RR_hip_joint',
    'RR_thigh_joint',
    'RR_calf_joint',
)
DEFAULT_ACTUATOR_NAMES = tuple(name.replace('_joint', '') for name in DEFAULT_JOINT_NAMES)
DEFAULT_DEFAULT_ANGLES = np.array(
    [
        0.1,
        0.8,
        -1.5,
        -0.1,
        0.8,
        -1.5,
        0.1,
        1.0,
        -1.5,
        -0.1,
        1.0,
        -1.5,
    ],
    dtype=np.float32,
)
DEFAULT_KPS = np.full(12, 20.0, dtype=np.float32)
DEFAULT_KDS = np.full(12, 0.5, dtype=np.float32)
DEFAULT_COMMAND_INIT = np.array([0.5, 0.0, 0.0], dtype=np.float32)
DEFAULT_COMMAND_LIMITS = np.array([1.0, 1.0, 1.0], dtype=np.float32)
DEFAULT_COMMAND_SCALE = np.array([2.0, 2.0, 0.25], dtype=np.float32)
DEFAULT_INITIAL_BASE_POS = np.array([0.0, 0.0, 0.42], dtype=np.float32)
DEFAULT_INITIAL_BASE_QUAT = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)


def _as_float_array(value: Any, expected_size: int, default: np.ndarray) -> np.ndarray:
    array = np.asarray(default if value is None else value, dtype=np.float32)
    if array.shape != (expected_size,):
        raise ValueError(f'Expected shape ({expected_size},), got {array.shape}')
    return array


def _resolve_path(base_dir: Path, value: str | Path) -> Path:
    path = Path(value).expanduser()
    if path.is_absolute():
        return path.resolve()
    return (base_dir / path).resolve()


def discover_package_root() -> Path:
    if get_package_share_directory is not None:
        try:
            return Path(get_package_share_directory('rl_controller'))
        except Exception:
            pass
    return Path(__file__).resolve().parents[2]


def default_config_path() -> Path:
    return discover_package_root() / 'config' / 'go2.yaml'


@dataclass(slots=True)
class Go2Config:
    package_root: Path
    policy_path: Path
    xml_path: Path
    simulation_duration: float
    simulation_dt: float
    control_decimation: int
    num_actions: int
    num_obs: int
    joint_names: tuple[str, ...]
    actuator_names: tuple[str, ...]
    kps: np.ndarray
    kds: np.ndarray
    default_angles: np.ndarray
    command_init: np.ndarray
    command_limits: np.ndarray
    command_scale: np.ndarray
    obs_scale_lin_vel: float
    obs_scale_ang_vel: float
    obs_scale_dof_pos: float
    obs_scale_dof_vel: float
    initial_base_pos: np.ndarray
    initial_base_quat: np.ndarray

    @classmethod
    def from_yaml(cls, yaml_path: str | Path) -> 'Go2Config':
        config_path = Path(yaml_path).expanduser().resolve()
        if not config_path.exists():
            raise FileNotFoundError(f'Config file not found: {config_path}')

        with config_path.open('r', encoding='utf-8') as handle:
            raw = yaml.safe_load(handle) or {}

        if not isinstance(raw, dict):
            raise ValueError('Go2 config must be a mapping.')

        package_root = config_path.parent.parent

        policy_path = _resolve_path(package_root, raw.get('policy_path', 'models/go2_policy.pt'))
        xml_path = _resolve_path(package_root, raw.get('xml_path', 'resources/robots/go2/scene.xml'))

        num_actions = int(raw.get('num_actions', 12))
        num_obs = int(raw.get('num_obs', 48))

        cfg = cls(
            package_root=package_root,
            policy_path=policy_path,
            xml_path=xml_path,
            simulation_duration=float(raw.get('simulation_duration', 60.0)),
            simulation_dt=float(raw.get('simulation_dt', 0.002)),
            control_decimation=int(raw.get('control_decimation', 10)),
            num_actions=num_actions,
            num_obs=num_obs,
            joint_names=tuple(str(item) for item in (raw.get('joint_names') or DEFAULT_JOINT_NAMES)),
            actuator_names=tuple(str(item) for item in (raw.get('actuator_names') or DEFAULT_ACTUATOR_NAMES)),
            kps=_as_float_array(raw.get('kps'), num_actions, DEFAULT_KPS),
            kds=_as_float_array(raw.get('kds'), num_actions, DEFAULT_KDS),
            default_angles=_as_float_array(raw.get('default_angles'), num_actions, DEFAULT_DEFAULT_ANGLES),
            command_init=_as_float_array(raw.get('command_init'), 3, DEFAULT_COMMAND_INIT),
            command_limits=_as_float_array(raw.get('command_limits'), 3, DEFAULT_COMMAND_LIMITS),
            command_scale=_as_float_array(raw.get('command_scale'), 3, DEFAULT_COMMAND_SCALE),
            obs_scale_lin_vel=float(raw.get('obs_scale_lin_vel', 2.0)),
            obs_scale_ang_vel=float(raw.get('obs_scale_ang_vel', 0.25)),
            obs_scale_dof_pos=float(raw.get('obs_scale_dof_pos', 1.0)),
            obs_scale_dof_vel=float(raw.get('obs_scale_dof_vel', 0.05)),
            initial_base_pos=_as_float_array(raw.get('initial_base_pos'), 3, DEFAULT_INITIAL_BASE_POS),
            initial_base_quat=_as_float_array(raw.get('initial_base_quat'), 4, DEFAULT_INITIAL_BASE_QUAT),
        )
        cfg.validate()
        return cfg

    def validate(self) -> None:
        if self.num_actions != 12:
            raise ValueError(f'Go2 expects 12 actions, got {self.num_actions}')
        if self.num_obs != 48:
            raise ValueError(f'Go2 expects 48 observations, got {self.num_obs}')
        if len(self.joint_names) != self.num_actions:
            raise ValueError('joint_names length must match num_actions')
        if len(self.actuator_names) != self.num_actions:
            raise ValueError('actuator_names length must match num_actions')
