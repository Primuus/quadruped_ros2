from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .config import Go2Config

GRAVITY_VECTOR = np.array([0.0, 0.0, -1.0], dtype=np.float32)


@dataclass(slots=True)
class Go2State:
    base_pos: np.ndarray
    base_quat: np.ndarray
    base_lin_vel: np.ndarray
    base_ang_vel: np.ndarray
    joint_pos: np.ndarray
    joint_vel: np.ndarray


def quat_rotate(quat: np.ndarray, vec: np.ndarray) -> np.ndarray:
    quat = np.asarray(quat, dtype=np.float32)
    vec = np.asarray(vec, dtype=np.float32)
    q_xyz = quat[1:]
    t = 2.0 * np.cross(q_xyz, vec)
    return vec + quat[0] * t + np.cross(q_xyz, t)


def quat_rotate_inverse(quat: np.ndarray, vec: np.ndarray) -> np.ndarray:
    conj = np.asarray(quat, dtype=np.float32).copy()
    conj[1:] *= -1.0
    return quat_rotate(conj, vec)


def build_one_step_observation(state: Go2State, command: np.ndarray, last_action: np.ndarray, cfg: Go2Config) -> np.ndarray:
    obs = np.zeros(cfg.num_one_step_obs, dtype=np.float32)

    # 45 维观测顺序必须和 quadruped_train 中的 compute_observations 保持一致。
    base_ang_vel = np.asarray(state.base_ang_vel, dtype=np.float32) * cfg.obs_scale_ang_vel
    projected_gravity = quat_rotate_inverse(state.base_quat, GRAVITY_VECTOR)
    command_obs = np.asarray(command, dtype=np.float32).reshape(3) * cfg.command_scale
    joint_pos = (np.asarray(state.joint_pos, dtype=np.float32) - cfg.default_angles) * cfg.obs_scale_dof_pos
    joint_vel = np.asarray(state.joint_vel, dtype=np.float32) * cfg.obs_scale_dof_vel
    last_action = np.asarray(last_action, dtype=np.float32).reshape(cfg.num_actions)

    obs[0:3] = command_obs
    obs[3:6] = base_ang_vel
    obs[6:9] = projected_gravity
    obs[9:21] = joint_pos
    obs[21:33] = joint_vel
    obs[33:45] = last_action
    return obs


class Go2ObservationHistory:
    """维护 Go2 的 45*6 历史观测缓存。"""

    def __init__(self, cfg: Go2Config) -> None:
        self.cfg = cfg
        self.buffer = np.zeros(cfg.num_obs, dtype=np.float32)

    def reset(self) -> None:
        self.buffer.fill(0.0)

    def update(self, state: Go2State, command: np.ndarray, last_action: np.ndarray) -> np.ndarray:
        one_step_obs = build_one_step_observation(state, command, last_action, self.cfg)
        if self.cfg.history_length == 1:
            return one_step_obs

        step = self.cfg.num_one_step_obs
        self.buffer[step:] = self.buffer[:-step].copy()
        self.buffer[:step] = one_step_obs
        return self.buffer.copy()


def build_observation(state: Go2State, command: np.ndarray, last_action: np.ndarray, cfg: Go2Config) -> np.ndarray:
    """单次构造 policy 观测；部署主循环应优先使用 Go2ObservationHistory。"""
    history = Go2ObservationHistory(cfg)
    return history.update(state, command, last_action)
