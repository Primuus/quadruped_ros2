from __future__ import annotations

import argparse
import time
from dataclasses import replace
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

from .core.config import Go2Config, default_config_path
from .core.observer import Go2State, build_observation, quat_rotate_inverse
from .core.pd import compute_torques
from .core.policy import TorchScriptPolicy


class Go2MujocoRunner:
    def __init__(self, cfg: Go2Config, device: str = 'cpu') -> None:
        self.cfg = cfg
        self.model = mujoco.MjModel.from_xml_path(str(cfg.xml_path))
        self.model.opt.timestep = cfg.simulation_dt
        self.data = mujoco.MjData(self.model)
        self.policy = TorchScriptPolicy(cfg.policy_path, device=device)

        self.joint_qpos_adr = self._resolve_joint_qpos_adr()
        self.joint_qvel_adr = self._resolve_joint_qvel_adr()
        self.actuator_ids = self._resolve_actuator_ids()
        self.actuator_torque_limits = self._resolve_actuator_torque_limits()

        self.command = np.clip(cfg.command_init.copy(), -cfg.command_limits, cfg.command_limits)
        self.last_action = np.zeros(cfg.num_actions, dtype=np.float32)
        self.current_torque = np.zeros(cfg.num_actions, dtype=np.float32)
        self.target_joint_pos = cfg.default_angles.copy()

        self._reset_state()

    def _resolve_joint_qpos_adr(self) -> np.ndarray:
        indices = np.zeros(self.cfg.num_actions, dtype=np.int32)
        for i, joint_name in enumerate(self.cfg.joint_names):
            joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            if joint_id < 0:
                raise ValueError(f'Joint not found in model: {joint_name}')
            indices[i] = self.model.jnt_qposadr[joint_id]
        return indices

    def _resolve_joint_qvel_adr(self) -> np.ndarray:
        indices = np.zeros(self.cfg.num_actions, dtype=np.int32)
        for i, joint_name in enumerate(self.cfg.joint_names):
            joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            if joint_id < 0:
                raise ValueError(f'Joint not found in model: {joint_name}')
            indices[i] = self.model.jnt_dofadr[joint_id]
        return indices

    def _resolve_actuator_ids(self) -> np.ndarray:
        indices = np.zeros(self.cfg.num_actions, dtype=np.int32)
        for i, actuator_name in enumerate(self.cfg.actuator_names):
            actuator_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_name)
            if actuator_id < 0:
                raise ValueError(f'Actuator not found in model: {actuator_name}')
            indices[i] = actuator_id
        return indices

    def _resolve_actuator_torque_limits(self) -> np.ndarray:
        limits = np.full(self.cfg.num_actions, np.inf, dtype=np.float32)
        for i, actuator_id in enumerate(self.actuator_ids):
            if bool(self.model.actuator_ctrllimited[actuator_id]):
                low, high = self.model.actuator_ctrlrange[actuator_id]
                limits[i] = float(max(abs(low), abs(high)))
        return limits

    def _reset_state(self) -> None:
        self.data.qpos[:] = 0.0
        self.data.qvel[:] = 0.0
        self.data.qpos[0:3] = self.cfg.initial_base_pos
        self.data.qpos[3:7] = self.cfg.initial_base_quat
        for index, qpos_addr in enumerate(self.joint_qpos_adr):
            self.data.qpos[qpos_addr] = self.cfg.default_angles[index]
        mujoco.mj_forward(self.model, self.data)

    def _read_state(self) -> Go2State:
        base_pos = self.data.qpos[0:3].copy()
        base_quat = self.data.qpos[3:7].copy()
        base_lin_vel_world = self.data.qvel[0:3].copy()
        base_ang_vel_world = self.data.qvel[3:6].copy()
        base_lin_vel = quat_rotate_inverse(base_quat, base_lin_vel_world)
        base_ang_vel = quat_rotate_inverse(base_quat, base_ang_vel_world)
        joint_pos = np.array([self.data.qpos[idx] for idx in self.joint_qpos_adr], dtype=np.float32)
        joint_vel = np.array([self.data.qvel[idx] for idx in self.joint_qvel_adr], dtype=np.float32)
        return Go2State(
            base_pos=base_pos,
            base_quat=base_quat,
            base_lin_vel=base_lin_vel,
            base_ang_vel=base_ang_vel,
            joint_pos=joint_pos,
            joint_vel=joint_vel,
        )

    def _compute_policy_output(self) -> None:
        state = self._read_state()
        observation = build_observation(state, self.command, self.last_action, self.cfg)
        action = self.policy.infer(observation)
        action = np.clip(action, -1.0, 1.0)
        torques, target_joint_pos = compute_torques(
            action,
            state.joint_pos,
            state.joint_vel,
            self.cfg,
            self.actuator_torque_limits,
        )
        self.last_action = action.astype(np.float32, copy=False)
        self.current_torque = torques
        self.target_joint_pos = target_joint_pos

    def _write_control(self) -> None:
        self.data.ctrl[:] = 0.0
        for torque, actuator_id in zip(self.current_torque, self.actuator_ids):
            self.data.ctrl[actuator_id] = float(torque)

    def run(self, headless: bool = False) -> None:
        total_steps = max(1, int(np.ceil(self.cfg.simulation_duration / self.cfg.simulation_dt)))
        decimation = max(1, int(self.cfg.control_decimation))

        if headless:
            self._run_loop(total_steps, None, decimation)
            return

        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            self._run_loop(total_steps, viewer, decimation)

    def _run_loop(self, total_steps: int, viewer: mujoco.viewer.Handle | None, decimation: int) -> None:
        for step in range(total_steps):
            step_start = time.time()

            if step % decimation == 0:
                self._compute_policy_output()

            self._write_control()
            mujoco.mj_step(self.model, self.data)

            if viewer is not None:
                viewer.sync()
                if not viewer.is_running():
                    break

            elapsed = time.time() - step_start
            sleep_time = self.model.opt.timestep - elapsed
            if sleep_time > 0.0:
                time.sleep(sleep_time)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description='Run the Go2 RL policy in MuJoCo.')
    parser.add_argument('--config', type=Path, default=None, help='Path to the Go2 YAML config.')
    parser.add_argument('--policy-path', type=Path, default=None, help='Override policy path.')
    parser.add_argument('--xml-path', type=Path, default=None, help='Override MuJoCo XML path.')
    parser.add_argument('--duration', type=float, default=None, help='Override simulation duration in seconds.')
    parser.add_argument(
        '--command',
        type=float,
        nargs=3,
        metavar=('VX', 'VY', 'WZ'),
        default=None,
        help='Override command vector [m/s, m/s, rad/s].',
    )
    parser.add_argument('--device', type=str, default='cpu', help='Torch device used for policy inference.')
    parser.add_argument('--headless', action='store_true', help='Run without the MuJoCo viewer.')
    return parser


def main(argv: list[str] | None = None) -> None:
    parser = build_arg_parser()
    args = parser.parse_args(argv)

    config_path = args.config or default_config_path()
    cfg = Go2Config.from_yaml(config_path)

    if args.policy_path is not None:
        cfg = replace(cfg, policy_path=Path(args.policy_path).expanduser().resolve())
    if args.xml_path is not None:
        cfg = replace(cfg, xml_path=Path(args.xml_path).expanduser().resolve())
    if args.duration is not None:
        cfg = replace(cfg, simulation_duration=float(args.duration))
    if args.command is not None:
        command = np.asarray(args.command, dtype=np.float32)
        cfg = replace(cfg, command_init=np.clip(command, -cfg.command_limits, cfg.command_limits))

    runner = Go2MujocoRunner(cfg, device=args.device)
    runner.run(headless=args.headless)


if __name__ == '__main__':
    main()
