from __future__ import annotations

import argparse
import sys
import time
from dataclasses import replace
from pathlib import Path

if __package__ in (None, ''):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import mujoco
import mujoco.viewer
import numpy as np

from rl_controller.core.config import RobotRLConfig, default_config_path
from rl_controller.core.observer import ObservationHistory, RobotState, quat_rotate_inverse
from rl_controller.core.pd import compute_torques
from rl_controller.core.policy import TorchScriptPolicy
from rl_controller.remote import QlpRemoteInput, RemoteCommandController

GRAVITY_VECTOR = np.array([0.0, 0.0, -1.0], dtype=np.float32)


class MujocoRLRunner:
    def __init__(
        self,
        cfg: RobotRLConfig,
        device: str = 'cpu',
        remote_input: QlpRemoteInput | None = None,
        remote_controller: RemoteCommandController | None = None,
        debug_steps: int = 0,
    ) -> None:
        self.cfg = cfg
        self.model = mujoco.MjModel.from_xml_path(str(cfg.xml_path))
        self.model.opt.timestep = cfg.simulation_dt
        self.data = mujoco.MjData(self.model)
        self.policy = TorchScriptPolicy(cfg.policy_path, device=device)
        self.remote_input = remote_input
        self.remote_controller = remote_controller

        self.joint_qpos_adr = self._resolve_joint_qpos_adr()
        self.joint_qvel_adr = self._resolve_joint_qvel_adr()
        self.actuator_ids = self._resolve_actuator_ids()
        self.actuator_torque_limits = self._resolve_actuator_torque_limits()

        if self.remote_controller is not None:
            self.command = self.remote_controller.current_command.copy()
        else:
            self.command = np.clip(cfg.command_init.copy(), -cfg.command_limits, cfg.command_limits)
        self.last_action = np.zeros(cfg.num_actions, dtype=np.float32)
        self.observation_history = ObservationHistory(cfg)
        self.current_torque = np.zeros(cfg.num_actions, dtype=np.float32)
        self.target_joint_pos = cfg.default_angles.copy()
        self.is_fallen = False
        self._fall_reason: str | None = None
        self._last_remote_mode_active: bool | None = None
        self.debug_steps = max(0, int(debug_steps))
        self._debug_policy_steps = 0

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
        self.last_action.fill(0.0)
        self.observation_history.reset()
        self.current_torque.fill(0.0)
        self.target_joint_pos = self.cfg.default_angles.copy()
        self.is_fallen = False
        self._fall_reason = None
        self._debug_policy_steps = 0
        mujoco.mj_forward(self.model, self.data)

    def _read_state(self) -> RobotState:
        base_pos = self.data.qpos[0:3].copy()
        base_quat = self.data.qpos[3:7].copy()
        base_lin_vel_world = self.data.qvel[0:3].copy()
        base_ang_vel_world = self.data.qvel[3:6].copy()
        base_lin_vel = quat_rotate_inverse(base_quat, base_lin_vel_world)
        base_ang_vel = quat_rotate_inverse(base_quat, base_ang_vel_world)
        joint_pos = np.array([self.data.qpos[idx] for idx in self.joint_qpos_adr], dtype=np.float32)
        joint_vel = np.array([self.data.qvel[idx] for idx in self.joint_qvel_adr], dtype=np.float32)
        return RobotState(
            base_pos=base_pos,
            base_quat=base_quat,
            base_lin_vel=base_lin_vel,
            base_ang_vel=base_ang_vel,
            joint_pos=joint_pos,
            joint_vel=joint_vel,
        )

    def _compute_policy_output(self) -> None:
        if self.is_fallen:
            self.last_action.fill(0.0)
            self.current_torque.fill(0.0)
            return

        state = self._read_state()
        fallen, reason = self._detect_fall(state)
        if fallen:
            self._mark_fallen(reason)
            return

        observation = self.observation_history.update(state, self.command, self.last_action)
        raw_action = self.policy.infer(observation)
        clipped_action = np.clip(raw_action, -1.0, 1.0)
        torques, target_joint_pos = compute_torques(
            clipped_action,
            state.joint_pos,
            state.joint_vel,
            self.cfg,
            self.actuator_torque_limits,
        )
        self._print_debug_snapshot(state, raw_action, clipped_action, torques, target_joint_pos)
        self.last_action = clipped_action.astype(np.float32, copy=False)
        self.current_torque = torques
        self.target_joint_pos = target_joint_pos

    def _print_debug_snapshot(
        self,
        state: RobotState,
        raw_action: np.ndarray,
        clipped_action: np.ndarray,
        torques: np.ndarray,
        target_joint_pos: np.ndarray,
    ) -> None:
        if self._debug_policy_steps >= self.debug_steps:
            return

        projected_gravity = quat_rotate_inverse(state.base_quat, GRAVITY_VECTOR)
        print(f'[mujoco-rl-debug] policy_step={self._debug_policy_steps}')
        print(f'  base_z: {float(state.base_pos[2]):.6f}')
        print(f'  projected_gravity: {np.array2string(projected_gravity, precision=5, suppress_small=True)}')
        print(f'  command: {np.array2string(self.command, precision=5, suppress_small=True)}')
        print(f'  joint_pos: {np.array2string(state.joint_pos, precision=5, suppress_small=True)}')
        print(f'  raw_action: {np.array2string(raw_action, precision=5, suppress_small=True)}')
        print(f'  clipped_action: {np.array2string(clipped_action, precision=5, suppress_small=True)}')
        print(f'  torques: {np.array2string(torques, precision=5, suppress_small=True)}')
        print(f'  target_joint_pos: {np.array2string(target_joint_pos, precision=5, suppress_small=True)}')
        print('-' * 80)
        self._debug_policy_steps += 1

    def _detect_fall(self, state: RobotState) -> tuple[bool, str]:
        projected_gravity = quat_rotate_inverse(state.base_quat, GRAVITY_VECTOR)
        reasons: list[str] = []

        if float(state.base_pos[2]) < self.cfg.fall_height_threshold:
            reasons.append(
                f'base_z={float(state.base_pos[2]):.3f} < {self.cfg.fall_height_threshold:.3f}'
            )
        if float(projected_gravity[2]) > self.cfg.fall_gravity_z_threshold:
            reasons.append(
                f'gravity_z={float(projected_gravity[2]):.3f} > {self.cfg.fall_gravity_z_threshold:.3f}'
            )

        return (len(reasons) > 0, '; '.join(reasons))

    def _mark_fallen(self, reason: str) -> None:
        if not self.is_fallen:
            self.is_fallen = True
            self._fall_reason = reason
            print(f'Fallen detected, torque output disabled: {reason}')
        self.last_action.fill(0.0)
        self.observation_history.reset()
        self.current_torque.fill(0.0)

    def _process_remote_input(self) -> None:
        if self.remote_input is None or self.remote_controller is None:
            return

        if self.remote_input.last_error is not None:
            print(f'remote input stopped, disable remote control: {self.remote_input.last_error}')
            self.remote_input = None
            self.remote_controller.enabled = False
            self.remote_controller.current_command.fill(0.0)
            self.command.fill(0.0)
            return

        for key in self.remote_input.drain_key_events():
            effect = self.remote_controller.apply_key(key)
            if effect.log_message:
                print(effect.log_message)
            if effect.zero_last_action:
                self.last_action.fill(0.0)
                self.observation_history.reset()
            if effect.reset_sim:
                self._reset_state()

        snapshot = self.remote_input.latest_snapshot()
        if snapshot is not None and snapshot.mode_active != self._last_remote_mode_active:
            print(f'Remote mode: {1.0 if snapshot.mode_active else 0.0}')
            self._last_remote_mode_active = snapshot.mode_active

        self.command = self.remote_controller.update_snapshot(snapshot)

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

            self._process_remote_input()
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
    parser = argparse.ArgumentParser(description='Run a quadruped RL policy in MuJoCo.')
    parser.add_argument('--config', type=Path, default=None, help='Path to the robot YAML config.')
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
    parser.add_argument(
        '--debug-steps',
        type=int,
        default=0,
        help='Print state/action/torque details for the first N policy steps. Default: 0.',
    )
    parser.add_argument('--remote-port', type=str, default=None, help='QLP remote serial device.')
    parser.add_argument('--remote-baud-rate', type=int, default=115200, help='QLP remote serial baud rate.')
    parser.add_argument('--remote-deadzone', type=float, default=0.12, help='Deadzone applied to remote sticks.')
    parser.add_argument(
        '--remote-speed-trim-scale',
        type=float,
        default=0.5,
        help='Scale factor for the right stick Y speed trim.',
    )
    return parser


def main(argv: list[str] | None = None) -> None:
    parser = build_arg_parser()
    args = parser.parse_args(argv)

    config_path = args.config or default_config_path()
    cfg = RobotRLConfig.from_yaml(config_path)
    forward_preset = cfg.command_init.copy()

    if args.policy_path is not None:
        cfg = replace(cfg, policy_path=Path(args.policy_path).expanduser().resolve())
    if args.xml_path is not None:
        cfg = replace(cfg, xml_path=Path(args.xml_path).expanduser().resolve())
    if args.duration is not None:
        cfg = replace(cfg, simulation_duration=float(args.duration))
    if args.command is not None:
        command = np.asarray(args.command, dtype=np.float32)
        cfg = replace(cfg, command_init=np.clip(command, -cfg.command_limits, cfg.command_limits))

    remote_input = None
    remote_controller = None
    if args.remote_port is not None:
        initial_command = np.zeros(3, dtype=np.float32)
        if args.command is not None:
            initial_command = np.clip(np.asarray(args.command, dtype=np.float32), -cfg.command_limits, cfg.command_limits)

        remote_input = QlpRemoteInput(args.remote_port, args.remote_baud_rate)
        remote_controller = RemoteCommandController(
            command_limits=cfg.command_limits,
            forward_preset=forward_preset,
            initial_command=initial_command,
            deadzone=args.remote_deadzone,
            speed_trim_scale=args.remote_speed_trim_scale,
        )
        print(f'Remote input enabled: {args.remote_port} @ {args.remote_baud_rate}')
        print(f'Remote initial command: {remote_controller.base_command}')

    runner = MujocoRLRunner(
        cfg,
        device=args.device,
        remote_input=remote_input,
        remote_controller=remote_controller,
        debug_steps=args.debug_steps,
    )
    try:
        runner.run(headless=args.headless)
    finally:
        if remote_input is not None:
            remote_input.close()


Go2MujocoRunner = MujocoRLRunner


if __name__ == '__main__':
    main()
