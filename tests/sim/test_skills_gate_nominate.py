"""``sim/skills_gate.py --nominate`` from the command line (P3b I3): the phase
gate numbers pinned by ``tests/motion/test_skills_nominate.py``, run through
``main()`` against the real planner (no simulation)."""
from __future__ import annotations

import pytest

from sim import skills_gate as sg

GATE = ['350', '5000', '200000', '4500']
HIGHLOW = ['--nominate', '--pattern', 'columns_highlow', '--apex-m', '1.0',
           '--apex-ball2', '0.9', '--dwell-s', '0.27', '--limits'] + GATE


def _run(capsys, argv):
    rc = sg.main(argv)
    return rc, capsys.readouterr().out


def test_highlow_separation_125_refused_names_100(capsys):
    rc, out = _run(capsys, HIGHLOW[:9] + ['--separation-mm', '125']
                   + HIGHLOW[9:])
    assert rc == 1
    assert out.startswith('nomination REFUSED: columns_highlow')
    assert 'nearest viable: separation 100 mm' in out
    assert 'LIMIT_VEL leg 2 at t=0.188 s' in out


def test_highlow_separation_100_viable(capsys):
    rc, out = _run(capsys, HIGHLOW + ['--separation-mm', '100'])
    assert rc == 0
    assert out.startswith('nomination OK: columns_highlow')


def test_oval_dwell_030_at_working_point_names_035(capsys):
    rc, out = _run(capsys, ['--nominate', '--pattern', 'oval_1ball',
                            '--dwell-s', '0.30', '--limits', '600', '8000',
                            '200000', '4500'])
    assert rc == 1
    assert 'dwell 0.35 s, longer' in out.splitlines()[0]


def test_oval_no_search_omits_nearest(capsys):
    # the module LRU may hold the searched report from the test above
    from jugglebot.motion.skills import nominate as nom
    nom.clear_cache()
    rc, out = _run(capsys, ['--nominate', '--pattern', 'oval_1ball',
                            '--dwell-s', '0.30', '--no-search', '--limits',
                            '600', '8000', '200000', '4500'])
    assert rc == 1
    assert 'nearest viable' not in out


def test_columns_default_working_point_viable(capsys):
    rc, out = _run(capsys, ['--nominate', '--pattern', 'columns'])
    assert rc == 0
    assert 'separation 125 mm, dwell 0.27 s' in out


@pytest.mark.parametrize('extra', [['--learn'], ['--reload'],
                                   ['--seeds', '1'], ['--viewer']])
def test_conflicting_flags_error(extra, capsys):
    with pytest.raises(SystemExit) as e:
        sg.main(['--nominate'] + extra)
    assert e.value.code == 2
    assert 'cannot combine' in capsys.readouterr().err


@pytest.mark.parametrize('argv', [['--nominate', '--pattern', 'columns',
                                   '--apex-ball2', '0.9'],
                                  ['--nominate', '--pattern', 'hop',
                                   '--angle-deg', '45'],
                                  ['--limits', '1', '2', '3', '4'],
                                  ['--no-search']])
def test_nonsense_flags_error(argv):
    with pytest.raises(SystemExit) as e:
        sg.main(argv)
    assert e.value.code == 2
