#!/usr/bin/env python3

import argparse
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys


def find_setup_file(explicit_setup):
    if explicit_setup:
        setup_path = Path(explicit_setup).expanduser()
        if setup_path.is_file():
            return setup_path
        raise FileNotFoundError(f"setup file not found: {setup_path}")

    candidates = []
    for prefix in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep):
        if not prefix:
            continue
        prefix_path = Path(prefix)
        candidates.append(prefix_path.parent / "setup.bash")
        candidates.append(prefix_path / "setup.bash")

    cwd = Path.cwd()
    candidates.append(cwd / "install" / "setup.bash")
    candidates.extend(parent / "install" / "setup.bash" for parent in cwd.parents)

    for candidate in candidates:
        if candidate.is_file():
            return candidate

    raise FileNotFoundError(
        "could not find install/setup.bash; source the workspace first or pass --setup"
    )


def build_terminal_command(executable, title, command):
    if executable == "gnome-terminal":
        return [executable, "--title", title, "--", "bash", "-lc", command]
    if executable == "xfce4-terminal":
        return [executable, "--title", title, "--command", f"bash -lc {shlex.quote(command)}"]
    if executable == "konsole":
        return [executable, "-p", f"tabtitle={title}", "-e", "bash", "-lc", command]
    if executable == "xterm":
        return [executable, "-T", title, "-e", "bash", "-lc", command]
    if executable == "x-terminal-emulator":
        return [executable, "-T", title, "-e", "bash", "-lc", command]
    return [executable, "-e", "bash", "-lc", command]


def choose_terminal(requested_terminal):
    if requested_terminal and requested_terminal != "auto":
        if shutil.which(requested_terminal):
            return requested_terminal
        raise FileNotFoundError(f"terminal executable not found: {requested_terminal}")

    for candidate in (
        "gnome-terminal",
        "xfce4-terminal",
        "konsole",
        "xterm",
        "x-terminal-emulator",
    ):
        if shutil.which(candidate):
            return candidate

    raise FileNotFoundError(
        "no supported terminal emulator found; install xterm or pass --terminal"
    )


def shell_command(setup_file, ros_command, title):
    return (
        f"source {shlex.quote(str(setup_file))} && "
        f"{ros_command}; "
        f"echo; echo '[{title}] process exited.'; exec bash"
    )


def parse_args():
    parser = argparse.ArgumentParser(
        description="Open separate real-robot terminals for quadruped control and micro-ROS."
    )
    parser.add_argument("--serial-port", default="/dev/ttyUSB0")
    parser.add_argument("--baud-rate", default="115200")
    parser.add_argument("--mcu-port", default="/dev/ttyUSB1")
    parser.add_argument("--mcu-baud-rate", default="115200")
    parser.add_argument("--micro-ros-verbosity", default="6")
    parser.add_argument("--terminal", default="auto")
    parser.add_argument("--setup", default="")
    return parser.parse_args()


def main():
    args = parse_args()

    try:
        setup_file = find_setup_file(args.setup)
        terminal = choose_terminal(args.terminal)
    except FileNotFoundError as error:
        print(f"real_robot_terminals: {error}", file=sys.stderr)
        return 1

    robot_command = " ".join(
        [
            "ros2 launch quadruped traditional_pd_control.launch.py",
            "use_sim_time:=false",
            "output_mode:=1.0",
            "lock_output_mode:=true",
            "require_joint_state:=false",
            "start_serial:=true",
            "start_kbd:=false",
            f"serial_port:={shlex.quote(args.serial_port)}",
            f"baud_rate:={shlex.quote(args.baud_rate)}",
            "start_micro_ros_agent:=false",
        ]
    )
    micro_ros_command = " ".join(
        [
            "ros2 run micro_ros_agent micro_ros_agent serial",
            "--dev",
            shlex.quote(args.mcu_port),
            "-b",
            shlex.quote(args.mcu_baud_rate),
            f"-v{shlex.quote(args.micro_ros_verbosity)}",
        ]
    )

    processes = [
        (
            "quadruped-control",
            shell_command(setup_file, robot_command, "quadruped-control"),
        ),
        (
            "micro-ros-agent",
            shell_command(setup_file, micro_ros_command, "micro-ros-agent"),
        ),
    ]

    for title, command in processes:
        subprocess.Popen(build_terminal_command(terminal, title, command))

    print("Opened real-robot terminals:")
    print(f"  quadruped-control: remote={args.serial_port}, output_mode=real")
    print(f"  micro-ros-agent: mcu={args.mcu_port}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
