from __future__ import annotations

import numpy as np

from .config import Go2Config


def compute_torques(
    action: np.ndarray,
    joint_pos: np.ndarray,
    joint_vel: np.ndarray,
    cfg: Go2Config,
    torque_limits: np.ndarray | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    action = np.asarray(action, dtype=np.float32).reshape(cfg.num_actions)
    joint_pos = np.asarray(joint_pos, dtype=np.float32).reshape(cfg.num_actions)
    joint_vel = np.asarray(joint_vel, dtype=np.float32).reshape(cfg.num_actions)

    target_pos = cfg.default_angles + action * cfg.action_scale
    torques = (target_pos - joint_pos) * cfg.kps - joint_vel * cfg.kds

    if torque_limits is not None:
        torque_limits = np.asarray(torque_limits, dtype=np.float32).reshape(cfg.num_actions)
        torques = np.clip(torques, -torque_limits, torque_limits)

    return torques.astype(np.float32), target_pos.astype(np.float32)

