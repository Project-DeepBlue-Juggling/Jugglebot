"""The hand ball-detect GPIO pin must be flashed as an input on the drive.

Pattern sibling of ``test_leg_torque_ff.py::
test_yaml_kt_odrive_config_matches_the_flashed_odrive_json``: the repo's copy of a
drive setting is a MIRROR of what is flashed, and nothing in the system ever reads
the setting back off the drive.

``jugglebot_ball_detect.gpio_pin`` selects which bit of the ODrive's
``get_gpio_states`` word carries the switch. If that pin is not flashed as
``DIGITAL_PULL_UP`` (mode 1), the bit is whatever the pin's other function drives —
a live-looking, never-changing sensor, with no timeout to diagnose it. A re-dump of
the hand config from a drive that lost the setting, or a pin move in the YAML,
breaks here instead of on the robot.
"""

from __future__ import annotations

import json
import os

import yaml

import jugglebot.hardware_config as hw

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))

#: ODrive ``GpioMode.DIGITAL_PULL_UP`` — the switch shorts the pin to GND.
_DIGITAL_PULL_UP = 1


def test_ball_detect_pin_is_flashed_as_a_pulled_up_input():
    json_path = os.path.join(_REPO_ROOT, 'config', 'ODrive config Files',
                             'odrive_pro_hand_config.json')
    with open(json_path) as f:
        cfg = json.load(f)

    pin = hw.JB_BD_GPIO_PIN
    key = f'gpio{pin}_mode'
    flashed = cfg['config'][key]

    assert flashed == _DIGITAL_PULL_UP, (
        f'jugglebot_ball_detect.gpio_pin={pin} but odrive_pro_hand_config.json '
        f'flashes {key}={flashed}, not {_DIGITAL_PULL_UP} (DIGITAL_PULL_UP)')


def test_generated_pin_matches_the_yaml():
    """generate_config.py was actually re-run after the YAML was edited — without
    this leg, a pin move in the YAML alone leaves the drift test reading the old
    generated pin and passing against the wrong gpioN_mode key."""
    yaml_path = os.path.join(_REPO_ROOT, 'config', 'hardware_config.yaml')
    with open(yaml_path) as f:
        yml = yaml.safe_load(f)

    assert yml['jugglebot_ball_detect']['gpio_pin'] == hw.JB_BD_GPIO_PIN, (
        'hardware_config.yaml jugglebot_ball_detect.gpio_pin != generated '
        'JB_BD_GPIO_PIN — run: python config/generate_config.py')
