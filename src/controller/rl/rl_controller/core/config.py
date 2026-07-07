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


# 新增配置字段时请同步补充中文说明，确保训练/部署配置一致可读。

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
)  # Go2 关节默认顺序
DEFAULT_ACTUATOR_NAMES = tuple(name.replace('_joint', '') for name in DEFAULT_JOINT_NAMES)  # 执行器默认顺序
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
)  # action=0 时的默认关节角
DEFAULT_POLICY_PATH = 'models/policy_1.pt'  # 默认策略文件名
DEFAULT_KPS = np.full(12, 40.0, dtype=np.float32)  # 默认比例增益，和训练侧 Go2 配置一致
DEFAULT_KDS = np.full(12, 1.0, dtype=np.float32)  # 默认微分增益，和训练侧 Go2 配置一致
DEFAULT_COMMAND_INIT = np.array([0.5, 0.0, 0.0], dtype=np.float32)  # 默认初始速度命令 [vx, vy, yaw_rate]
DEFAULT_COMMAND_LIMITS = np.array([1.0, 1.0, 1.0], dtype=np.float32)  # 命令上限 [vx, vy, yaw_rate]
DEFAULT_COMMAND_SCALE = np.array([2.0, 2.0, 0.25], dtype=np.float32)  # 命令观测缩放
DEFAULT_INITIAL_BASE_POS = np.array([0.0, 0.0, 0.42], dtype=np.float32)  # 初始机身位置 [m]
DEFAULT_INITIAL_BASE_QUAT = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)  # 初始机身姿态四元数
DEFAULT_FALL_HEIGHT_THRESHOLD = 0.18  # 跌倒判定高度阈值
DEFAULT_FALL_GRAVITY_Z_THRESHOLD = -0.2  # 跌倒判定时的重力方向阈值
DEFAULT_OBS_MODE = 'go2_45'  # 默认使用 HIMLoco 风格 45 维观测


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
    package_root: Path  # 包根目录
    policy_path: Path  # 策略权重文件路径
    xml_path: Path  # MuJoCo 场景文件路径
    simulation_duration: float  # 仿真总时长 [s]
    simulation_dt: float  # 仿真步长 [s]
    control_decimation: int  # 一个动作持续的仿真步数
    num_actions: int  # 动作维度
    num_obs: int  # 观测维度
    obs_mode: str  # 观测模式
    joint_names: tuple[str, ...]  # 关节顺序
    actuator_names: tuple[str, ...]  # 执行器顺序
    kps: np.ndarray  # 每个关节的比例增益
    kds: np.ndarray  # 每个关节的微分增益
    default_angles: np.ndarray  # 默认关节角
    action_scale: float  # 动作缩放系数
    command_init: np.ndarray  # 初始速度命令
    command_limits: np.ndarray  # 速度命令上限
    command_scale: np.ndarray  # 命令观测缩放
    fall_height_threshold: float  # 跌倒高度阈值
    fall_gravity_z_threshold: float  # 跌倒重力方向阈值
    obs_scale_lin_vel: float  # 线速度观测缩放
    obs_scale_ang_vel: float  # 角速度观测缩放
    obs_scale_dof_pos: float  # 关节位置观测缩放
    obs_scale_dof_vel: float  # 关节速度观测缩放
    initial_base_pos: np.ndarray  # 初始机身位置
    initial_base_quat: np.ndarray  # 初始机身姿态

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

        # 默认策略与场景文件，和 go2.yaml 中的注释保持一致。
        policy_path = _resolve_path(package_root, raw.get('policy_path', DEFAULT_POLICY_PATH))
        xml_path = _resolve_path(package_root, raw.get('xml_path', 'resources/robots/go2/scene.xml'))

        # Go2 固定为 12 维动作，当前策略默认使用 45 维观测。
        num_actions = int(raw.get('num_actions', 12))
        num_obs = int(raw.get('num_obs', 45))

        cfg = cls(
            package_root=package_root,
            policy_path=policy_path,
            xml_path=xml_path,
            simulation_duration=float(raw.get('simulation_duration', 60.0)),
            simulation_dt=float(raw.get('simulation_dt', 0.002)),
            control_decimation=int(raw.get('control_decimation', 10)),
            num_actions=num_actions,
            num_obs=num_obs,
            obs_mode=str(raw.get('obs_mode', DEFAULT_OBS_MODE)),
            joint_names=tuple(str(item) for item in (raw.get('joint_names') or DEFAULT_JOINT_NAMES)),
            actuator_names=tuple(str(item) for item in (raw.get('actuator_names') or DEFAULT_ACTUATOR_NAMES)),
            kps=_as_float_array(raw.get('kps'), num_actions, DEFAULT_KPS),
            kds=_as_float_array(raw.get('kds'), num_actions, DEFAULT_KDS),
            default_angles=_as_float_array(raw.get('default_angles'), num_actions, DEFAULT_DEFAULT_ANGLES),
            action_scale=float(raw.get('action_scale', 0.25)),
            command_init=_as_float_array(raw.get('command_init'), 3, DEFAULT_COMMAND_INIT),
            command_limits=_as_float_array(raw.get('command_limits'), 3, DEFAULT_COMMAND_LIMITS),
            command_scale=_as_float_array(raw.get('command_scale'), 3, DEFAULT_COMMAND_SCALE),
            fall_height_threshold=float(raw.get('fall_height_threshold', DEFAULT_FALL_HEIGHT_THRESHOLD)),
            fall_gravity_z_threshold=float(
                raw.get('fall_gravity_z_threshold', DEFAULT_FALL_GRAVITY_Z_THRESHOLD)
            ),
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
        if self.obs_mode != 'go2_45':
            raise ValueError(f'Go2 currently expects obs_mode=go2_45, got {self.obs_mode}')
        if self.num_obs != 45:
            raise ValueError(f'Go2 45-dim policy expects 45 observations, got {self.num_obs}')
        if len(self.joint_names) != self.num_actions:
            raise ValueError('joint_names length must match num_actions')
        if len(self.actuator_names) != self.num_actions:
            raise ValueError('actuator_names length must match num_actions')
