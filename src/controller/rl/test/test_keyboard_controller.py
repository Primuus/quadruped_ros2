from __future__ import annotations

import queue
import sys
import threading
import time
import unittest
from pathlib import Path

import numpy as np


RL_PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RL_PACKAGE_ROOT))

from rl_controller.core.config import DEFAULT_KEYBOARD_BINDINGS
from rl_controller.input import KeyboardCommandController, KeyboardInput


class KeyboardCommandControllerTest(unittest.TestCase):
    def setUp(self) -> None:
        self.controller = KeyboardCommandController(
            command_limits=np.ones(3, dtype=np.float32),
            bindings=DEFAULT_KEYBOARD_BINDINGS,
            speed_levels=np.array([0.25, 0.5, 1.0], dtype=np.float32),
            default_speed_level=2,
        )

    def assert_command(self, expected: list[float], pressed_keys: set[str]) -> None:
        np.testing.assert_allclose(
            self.controller.update_pressed_keys(pressed_keys),
            np.asarray(expected, dtype=np.float32),
        )

    def test_control_starts_disabled_and_enable_resets_policy_state(self) -> None:
        self.assert_command([0.0, 0.0, 0.0], {'KEY_W'})
        direction_effect = self.controller.apply_key('KEY_W')
        self.assertFalse(direction_effect.reset_policy_state)
        effect = self.controller.apply_key('KEY_F')
        self.assertTrue(effect.reset_policy_state)
        self.assertTrue(effect.exclusive_input)
        repeated_enable_effect = self.controller.apply_key('KEY_F')
        self.assertFalse(repeated_enable_effect.reset_policy_state)
        self.assertTrue(repeated_enable_effect.exclusive_input)
        self.assert_command([0.5, 0.0, 0.0], {'KEY_W'})

    def test_combined_and_opposite_directions(self) -> None:
        self.controller.apply_key('KEY_F')
        self.assert_command([0.5, 0.5, 0.5], {'KEY_W', 'KEY_A', 'KEY_Q'})
        self.assert_command(
            [0.0, 0.0, 0.0],
            {'KEY_W', 'KEY_S', 'KEY_A', 'KEY_D', 'KEY_Q', 'KEY_E'},
        )

    def test_speed_levels_scale_all_axes(self) -> None:
        self.controller.apply_key('KEY_F')
        self.controller.apply_key('KEY_1')
        self.assert_command([0.25, 0.25, 0.25], {'KEY_W', 'KEY_A', 'KEY_Q'})
        self.controller.apply_key('KEY_3')
        self.assert_command([1.0, 1.0, 1.0], {'KEY_W', 'KEY_A', 'KEY_Q'})

    def test_stop_requires_direction_release_before_motion_resumes(self) -> None:
        self.controller.apply_key('KEY_F')
        self.assert_command([0.5, 0.0, 0.0], {'KEY_W'})
        self.controller.apply_key('KEY_SPACE')
        self.assert_command([0.0, 0.0, 0.0], {'KEY_W', 'KEY_SPACE'})
        self.assert_command([0.0, 0.0, 0.0], {'KEY_W'})
        self.assert_command([0.0, 0.0, 0.0], set())
        self.assert_command([0.5, 0.0, 0.0], {'KEY_W'})

    def test_disable_clears_command(self) -> None:
        self.controller.apply_key('KEY_F')
        self.assert_command([0.5, 0.0, 0.0], {'KEY_W'})
        effect = self.controller.apply_key('KEY_I')
        self.assertFalse(self.controller.enabled)
        self.assertFalse(effect.exclusive_input)
        self.assert_command([0.0, 0.0, 0.0], {'KEY_W'})

    def test_reset_is_a_simulation_event(self) -> None:
        effect = self.controller.apply_key('KEY_R')
        self.assertTrue(effect.reset_sim)
        self.assertFalse(effect.reset_policy_state)


class KeyboardInputStateTest(unittest.TestCase):
    def test_pressed_keys_are_merged_across_devices(self) -> None:
        keyboard = KeyboardInput.__new__(KeyboardInput)
        keyboard._pressed_keys_by_fd = {10: set(), 11: set()}
        keyboard._pressed_keys = set()
        keyboard._state_lock = threading.Lock()
        keyboard._key_events = queue.Queue()
        keyboard._last_event_time = time.monotonic()

        keyboard._handle_key_event(10, 'KEY_W', 1)
        keyboard._handle_key_event(11, 'KEY_W', 1)
        keyboard._handle_key_event(10, 'KEY_W', 0)
        self.assertEqual(keyboard.latest_snapshot().pressed_keys, frozenset({'KEY_W'}))

        keyboard._handle_key_event(11, 'KEY_W', 0)
        self.assertEqual(keyboard.latest_snapshot().pressed_keys, frozenset())

    def test_exclusive_mode_grabs_and_releases_every_device(self) -> None:
        class FakeDevice:
            def __init__(self, path: str, fail_grab: bool = False) -> None:
                self.path = path
                self.fail_grab = fail_grab
                self.grabbed = False

            def grab(self) -> None:
                if self.fail_grab:
                    raise OSError('already grabbed')
                self.grabbed = True

            def ungrab(self) -> None:
                self.grabbed = False

        keyboard = KeyboardInput.__new__(KeyboardInput)
        keyboard._devices = [FakeDevice('/dev/input/event1'), FakeDevice('/dev/input/event2')]
        keyboard._devices_lock = threading.Lock()
        keyboard._exclusive = False

        keyboard.set_exclusive(True)
        self.assertTrue(keyboard.exclusive)
        self.assertTrue(all(device.grabbed for device in keyboard._devices))

        keyboard.set_exclusive(False)
        self.assertFalse(keyboard.exclusive)
        self.assertFalse(any(device.grabbed for device in keyboard._devices))

    def test_failed_exclusive_mode_releases_previously_grabbed_devices(self) -> None:
        class FakeDevice:
            def __init__(self, path: str, fail_grab: bool = False) -> None:
                self.path = path
                self.fail_grab = fail_grab
                self.grabbed = False

            def grab(self) -> None:
                if self.fail_grab:
                    raise OSError('already grabbed')
                self.grabbed = True

            def ungrab(self) -> None:
                self.grabbed = False

        keyboard = KeyboardInput.__new__(KeyboardInput)
        keyboard._devices = [
            FakeDevice('/dev/input/event1'),
            FakeDevice('/dev/input/event2', fail_grab=True),
        ]
        keyboard._devices_lock = threading.Lock()
        keyboard._exclusive = False

        with self.assertRaisesRegex(RuntimeError, '/dev/input/event2'):
            keyboard.set_exclusive(True)
        self.assertFalse(keyboard.exclusive)
        self.assertFalse(any(device.grabbed for device in keyboard._devices))


if __name__ == '__main__':
    unittest.main()
