# -*- coding: utf-8 -*-
"""Behaviour of ``ros_ws/gui/js/clock.js`` (replay design § 2, § 7 timer rule).

The shipped ``clock.js`` is copied verbatim next to ``tests/ros/js/clock_harness.js``
and a ``{"type": "module"}`` package.json (same sandbox pattern as
``test_gui_fk_golden.py``); the harness prints observations as JSON and every
assertion is made here so failures read as pytest diffs.
"""
from __future__ import annotations

import json
import os
import shutil
import subprocess

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
_CLOCK = os.path.join(_HERE, '..', '..', 'ros_ws', 'gui', 'js', 'clock.js')
_HARNESS = os.path.join(_HERE, 'js', 'clock_harness.js')
_NODE = shutil.which('node') or shutil.which('nodejs')

pytestmark = pytest.mark.skipif(_NODE is None, reason='node unavailable')


@pytest.fixture(scope='module')
def obs(tmp_path_factory):
    sb = str(tmp_path_factory.mktemp('gui_clock'))
    shutil.copy(_CLOCK, os.path.join(sb, 'clock.js'))
    shutil.copy(_HARNESS, os.path.join(sb, 'clock_harness.js'))
    with open(os.path.join(sb, 'package.json'), 'w') as f:
        f.write('{"type": "module"}\n')
    proc = subprocess.run([_NODE, os.path.join(sb, 'clock_harness.js')],
                          stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=30)
    assert proc.returncode == 0, proc.stderr.decode('utf-8', 'replace')
    return json.loads(proc.stdout.decode('utf-8'))


def test_wall_mode_delegates_to_date_now(obs):
    assert obs['wall_now_in_range'] is True
    assert obs['wall_isReplay'] is False
    assert obs['wall_settimeout_is_real'] is True
    assert obs['wall_pending'] == 0


def test_replay_now_follows_set_now(obs):
    assert obs['replay_isReplay'] is True
    assert obs['replay_now_a'] == 123456
    assert obs['replay_now_b'] == 999000


def test_timer_at_speed_1_fires_at_tau_1000_not_before(obs):
    assert obs['s1_before'] == []
    assert obs['s1_at'] == ['a']


def test_timer_at_speed_4_fires_at_tau_4000(obs):
    assert obs['s4_before'] == []
    assert obs['s4_at'] == ['b']


def test_speed_below_1_does_not_stretch_timers(obs):
    assert obs['slow_at'] == ['c']


def test_reverse_travel_still_ages_timers(obs):
    assert obs['reverse_at'] == ['r']


def test_timers_frozen_without_travel(obs):
    assert obs['frozen_fired'] == []
    assert obs['frozen_pending'] == 1
    assert obs['zero_travel_fired'] == []


def test_clear_timers_drops_pending(obs):
    assert obs['cleared_fired'] == []
    assert obs['cleared_pending'] == 0


def test_clear_timeout_cancels_one(obs):
    assert obs['clear_one'] == ['y']


def test_deadline_order_and_once(obs):
    assert obs['order'] == ['t100', 't200', 't300']


def test_timer_deadline_relative_to_arming(obs):
    assert obs['rel_before'] == []
    assert obs['rel_at'] == ['late']


def test_speed_change_mid_wait_uses_wall_equivalent_time(obs):
    assert obs['mix_before'] == []
    assert obs['mix_at'] == ['mix']


def test_exit_replay_cancels_and_restores_wall(obs):
    assert obs['exit_pending'] == 0
    assert obs['exit_isReplay'] is False
    assert obs['exit_fired'] == []
    assert obs['exit_now_wall'] is True


def test_virtual_ids_are_negative_and_never_reach_real_clear_timeout(obs):
    assert obs['virtual_id_negative'] is True
    assert obs['stale_virtual_forwarded'] is False


def test_real_timer_cleared_during_replay_is_cancelled(obs):
    assert obs['real_id_forwarded'] is True
    assert obs['real_timer_fired'] is False


def test_throwing_timer_callback_is_logged_not_propagated(obs):
    t = obs['throw_timer']
    assert t['propagated'] is False and t['logged'] is True
    assert t['later_fired'] == ['after']


def test_zero_delay_rearming_timer_hits_the_fuse(obs):
    f = obs['fuse']
    assert f['count'] == 10000 and f['pending'] == 0 and f['logged'] is True
