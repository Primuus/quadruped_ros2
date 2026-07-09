from __future__ import annotations

import sys
from pathlib import Path


RL_PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RL_PACKAGE_ROOT))

from rl_controller.deploy_mujoco_rl import main


if __name__ == '__main__':
    default_args = [
        '--command',
        '0.0',
        '0.0',
        '0.0',
        '--duration',
        '30',
        '--debug-steps',
        '200',
        '--remote-port',
        '/dev/ttyUSB0',
        '--remote-baud-rate',
        '115200',
    ]
    main(default_args + sys.argv[1:])
