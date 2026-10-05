"""``tools/probes/feed_lateral_miss.py`` — ``parse_feeds``'s pattern-NAME
match (R5 sitting 6, 2026-10-05).

Regression for the bug the sitting-6 unit fixed: ``oneball`` used to test
``'columns_1ball' in head``, a SUBSTRING check, so a ``columns_1ball_fed``
attempt (the fed pattern this probe exists to measure) was wrongly treated
as a ``columns_1ball`` one (parked-cup control, excluded by design) and its
feed silently dropped. The fix matches the pattern NAME
(``head.split(' ')[0]``) instead.

Offline and read-only: builds a synthetic launch-log text file under
``tmp_path`` in the exact format ``_RE_LINE``/``_RE_AIM`` parse, no bag, no
ROS2, no network.
"""

from __future__ import annotations

import importlib.util
import os

import pytest

_PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_SCRIPT_PATH = os.path.join(_PROJECT_ROOT, 'tools', 'probes', 'feed_lateral_miss.py')


def _load_script():
    spec = importlib.util.spec_from_file_location('feed_lateral_miss', _SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope='module')
def tool():
    return _load_script()


_LOG = (
    '1000.100 [skill_node-1] [INFO] [123.456] [skill_node]: '
    'columns_1ball_fed (reload) started: ball A at P2, feed at P1\n'
    '1000.200 [skill_node-1] [INFO] [123.556] [skill_node]: '
    'CATCH-AIM skill 1: source=schedule landing=(-52.5, 0.0, 830.0) mm '
    't_land=1000.403\n'
    '2000.100 [skill_node-1] [INFO] [223.456] [skill_node]: '
    'columns_1ball (reload) started: ball A at P1 parked\n'
    '2000.200 [skill_node-1] [INFO] [223.556] [skill_node]: '
    'CATCH-AIM skill 1: source=schedule landing=(10.0, 0.0, 830.0) mm '
    't_land=2000.123\n'
)


def test_parse_feeds_finds_the_fed_pattern_and_excludes_the_parked_one(
        tool, tmp_path):
    log_path = tmp_path / 'launch.log'
    log_path.write_text(_LOG)

    feeds = tool.parse_feeds(str(log_path))

    assert len(feeds) == 1, (
        'expected exactly the columns_1ball_fed feed; the columns_1ball '
        '(parked-cup) goal must be excluded, not matched as a substring')
    assert feeds[0]['t_land'] == pytest.approx(1000.403)
    assert feeds[0]['req_x'] == pytest.approx(-52.5)
    assert feeds[0]['req_y'] == pytest.approx(0.0)


def test_parse_feeds_excludes_a_bare_columns_1ball_goal_entirely(tool, tmp_path):
    """A log with ONLY the parked-cup pattern yields zero feeds -- the whole
    point of the pattern-NAME fix, isolated from the fed case above."""
    log_path = tmp_path / 'launch.log'
    log_path.write_text(
        '2000.100 [skill_node-1] [INFO] [223.456] [skill_node]: '
        'columns_1ball (reload) started: ball A at P1 parked\n'
        '2000.200 [skill_node-1] [INFO] [223.556] [skill_node]: '
        'CATCH-AIM skill 1: source=schedule landing=(10.0, 0.0, 830.0) mm '
        't_land=2000.123\n')

    assert tool.parse_feeds(str(log_path)) == []
