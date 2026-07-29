"""The (board, fw)-qualified endpoint-id table must round-trip through codegen.

ODrive endpoint ids are firmware-build-specific, and a wrong-but-existing id
ANSWERS well-formed TxSdo replies (700 on the hand Pro is
``encoder_estimator1.status`` — a live-looking sensor that never changes), so
the qualified table and its generated constants must never drift apart.

Deliberately NOT gated behind ``pytest.importorskip("can")`` — this is a config
test with no CAN content, and it must keep running on boxes without python-can
(the Win10 sim clone).
"""

from __future__ import annotations

import os

import yaml

import jugglebot.protocol_config as proto

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))


def test_endpoint_yaml_roundtrips_to_generated_constants():
    """generate_config.py was actually re-run after the endpoints: block was
    edited, and the (group, name) -> ENDPOINT_{GROUP}_{NAME} name-mangling
    contract that consumers quote verbatim holds for every entry."""
    yaml_path = os.path.join(_REPO_ROOT, 'config', 'protocol_config.yaml')
    with open(yaml_path) as f:
        cfg = yaml.safe_load(f)

    seen = 0
    for group, entries in cfg['endpoints'].items():
        for name, val in entries.items():
            pyname = f'ENDPOINT_{group.upper()}_{name.upper()}'
            assert getattr(proto, pyname) == val, (
                f'{pyname} != endpoints.{group}.{name} — '
                'run: python config/generate_config.py')
            seen += 1
    assert seen >= 3   # pro get_gpio_states + s1 get_gpio_states + s1 commutation


def test_ros_delivered_copy_matches_generated():
    """The ROS-delivered protocol_config.py (what the tests above resolve
    through, and what the live nodes import) is byte-identical to the
    canonical generated artifact."""
    gen = os.path.join(_REPO_ROOT, 'config', 'generated', 'protocol_config.py')
    ros = os.path.join(_REPO_ROOT, 'ros_ws', 'src', 'jugglebot', 'jugglebot',
                       'protocol_config.py')
    with open(gen) as f_gen, open(ros) as f_ros:
        assert f_gen.read() == f_ros.read(), (
            'ros_ws delivered protocol_config.py != config/generated copy — '
            'run: python config/generate_config.py')
