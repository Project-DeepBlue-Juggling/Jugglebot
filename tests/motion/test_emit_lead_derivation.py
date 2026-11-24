"""I-WIRE-7 — the planner's wire horizon derives from the emitter's lead.

``schedule.WIRE_READ_KNOTS = WIRE_READ_BASE_KNOTS (3) + EMIT_LEAD_KNOTS`` and
every splice lead, freeze and cliff fence derives from it; two numbers must
NOT follow the naive rule (``FRESH_REST_REBASE_S`` derives from the Hand's
resume budget ``REST_RETURN_ONSET_S``, ``FRESH_REST_STALL_FENCE_S`` stays at
``0.5*3*dt``).  The constants are fixed at import, so each lead is checked in
its own subprocess that sets the generated config constant BEFORE importing
``schedule`` / ``executor`` / ``bump`` (parallel-safe: no shared state, no
``importlib.reload``).  The table is the one in
``logbook/2026-11-25-emit-lead-planner-derivation.md``; the configured lead is
still 0.
"""
from __future__ import annotations

import json
import os
import subprocess
import sys

import pytest

_REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
_PKG = os.path.join(_REPO, 'ros_ws', 'src', 'jugglebot')

_SCRIPT = r'''
import json, sys
import jugglebot.hardware_config as hw
hw.JB_TRAJ_EMIT_LEAD_KNOTS = int(sys.argv[1])
from jugglebot.motion.skills import schedule as sc, executor as ex, bump as bp
dt = float(hw.JB_TRAJ_KNOT_DT_S)
print(json.dumps({
    'EMIT_LEAD_KNOTS': sc.EMIT_LEAD_KNOTS,
    'WIRE_READ_KNOTS': sc.WIRE_READ_KNOTS,
    'ex_WIRE_READ_KNOTS': ex.WIRE_READ_KNOTS,
    'ex_EMIT_LEAD_KNOTS': ex.EMIT_LEAD_KNOTS,
    'LEAD_KNOTS': sc.LEAD_KNOTS, 'LEAD_S': sc.LEAD_S,
    'CATCH_FREEZE_S': ex.CATCH_FREEZE_S,
    'HANDOFF_LEAD_KNOTS': sc.HANDOFF_LEAD_KNOTS, 'HANDOFF_LEAD_S': sc.HANDOFF_LEAD_S,
    'FRESH_THROW_LEAD_KNOTS': sc.FRESH_THROW_LEAD_KNOTS,
    'FRESH_THROW_LEAD_S': sc.FRESH_THROW_LEAD_S,
    'BUMP_LEAD_KNOTS': bp.BUMP_LEAD_KNOTS,
    'BUMP_FREEZE_LEAD_S': bp.BUMP_FREEZE_LEAD_S,
    'RESEND_PINNED_BUDGET_FLOOR_S': ex.RESEND_PINNED_BUDGET_FLOOR_S,
    'rest_solve_budget_s_base': ex.rest_solve_budget_s(0.1),
    'FRESH_REST_REBASE_S': sc.FRESH_REST_REBASE_S,
    'ex_FRESH_REST_REBASE_S': ex.FRESH_REST_REBASE_S,
    'FRESH_REST_STALL_FENCE_S': sc.FRESH_REST_STALL_FENCE_S,
    'ex_FRESH_REST_STALL_FENCE_S': ex.FRESH_REST_STALL_FENCE_S,
    'REST_RETURN_ONSET_S': sc.REST_RETURN_ONSET_S,
    'dt': dt,
}))
'''

# lead -> (constants of design section 5, in knots or seconds)
_TABLE = {
    0: dict(WIRE_READ_KNOTS=3, LEAD_KNOTS=9, LEAD_S=0.225, CATCH_FREEZE_S=0.250,
            HANDOFF_LEAD_KNOTS=17, HANDOFF_LEAD_S=0.425,
            FRESH_THROW_LEAD_KNOTS=13, FRESH_THROW_LEAD_S=0.325,
            BUMP_LEAD_KNOTS=5, BUMP_FREEZE_LEAD_S=0.225,
            RESEND_PINNED_BUDGET_FLOOR_S=0.150, rest_solve_budget_s_base=0.150,
            FRESH_REST_REBASE_S=0.075, FRESH_REST_STALL_FENCE_S=0.0375,
            REST_RETURN_ONSET_S=0.100),
    1: dict(WIRE_READ_KNOTS=4, LEAD_KNOTS=10, LEAD_S=0.250, CATCH_FREEZE_S=0.275,
            HANDOFF_LEAD_KNOTS=18, HANDOFF_LEAD_S=0.450,
            FRESH_THROW_LEAD_KNOTS=14, FRESH_THROW_LEAD_S=0.350,
            BUMP_LEAD_KNOTS=6, BUMP_FREEZE_LEAD_S=0.250,
            RESEND_PINNED_BUDGET_FLOOR_S=0.150, rest_solve_budget_s_base=0.150,
            FRESH_REST_REBASE_S=0.050, FRESH_REST_STALL_FENCE_S=0.0375,
            REST_RETURN_ONSET_S=0.100),
}


def _run(lead: int) -> subprocess.CompletedProcess:
    env = dict(os.environ)
    env['PYTHONPATH'] = os.pathsep.join(
        [_PKG, os.path.join(_REPO, 'config', 'generated'), _REPO]
        + ([env['PYTHONPATH']] if env.get('PYTHONPATH') else []))
    return subprocess.run([sys.executable, '-c', _SCRIPT, str(lead)],
                          capture_output=True, text=True, env=env,
                          cwd=_REPO, timeout=120)


@pytest.mark.parametrize('lead', [0, 1])
def test_the_wire_horizon_table_at_each_emit_lead(lead):
    proc = _run(lead)
    assert proc.returncode == 0, proc.stderr
    got = json.loads(proc.stdout.strip().splitlines()[-1])
    assert got['EMIT_LEAD_KNOTS'] == lead == got['ex_EMIT_LEAD_KNOTS']
    # the executor re-exports the SAME values, never a second copy
    assert got['ex_WIRE_READ_KNOTS'] == got['WIRE_READ_KNOTS']
    assert got['ex_FRESH_REST_REBASE_S'] == got['FRESH_REST_REBASE_S']
    assert got['ex_FRESH_REST_STALL_FENCE_S'] == got['FRESH_REST_STALL_FENCE_S']
    for name, want in _TABLE[lead].items():
        assert got[name] == pytest.approx(want, abs=1e-9), (lead, name)
    dt = got['dt']
    # the two derivations that must NOT follow the naive rule, restated
    # structurally so a re-tuned table cannot hide a re-introduced shortcut
    assert got['FRESH_REST_REBASE_S'] == pytest.approx(
        got['REST_RETURN_ONSET_S'] - (lead + 1) * dt, abs=1e-12)
    assert got['FRESH_REST_REBASE_S'] != pytest.approx(
        got['WIRE_READ_KNOTS'] * dt, abs=1e-9) or lead == 0
    assert got['FRESH_REST_STALL_FENCE_S'] == pytest.approx(
        0.5 * 3 * dt, abs=1e-12)


def test_an_emit_lead_past_one_is_refused_at_import():
    proc = _run(2)
    assert proc.returncode != 0
    assert 'emit_lead_knots=2' in proc.stderr
    assert 'only 0 and 1 are designed' in proc.stderr


# ---------------------------------------------------------------------------
# Consumers, not constants (P1b): the call sites run at lead 1 for real
# ---------------------------------------------------------------------------

_ENV = 'JUGGLEBOT_TEST_EMIT_LEAD_KNOTS'

#: The tests that drive the real call sites of the derived numbers. Each is an
#: ordinary per-commit test at the configured lead; the subprocess below runs
#: the same ids at lead 1 through the ``tests/conftest.py`` env hook. Each fails
#: when its call site is reverted to the old expression: the fresh gate to one
#: shared threshold, the stall fence to ``0.5*WIRE_READ_KNOTS*dt``, the
#: emitter's ``lead`` to 0.
_CONSUMER_IDS = [
    'tests/motion/test_emit_lead_derivation.py::test_the_env_hook_sets_the_lead',
    'tests/motion/test_skills_install_origin.py::'
    'test_a_60_ms_solve_rebases_a_fresh_rest_only_past_its_own_threshold',
    'tests/ros/test_install_segment.py::'
    'test_the_fresh_rest_stall_fence_is_37_5_ms_at_every_emit_lead',
    'tests/ros/test_install_segment.py::'
    'test_the_emitter_stamps_each_frame_emit_lead_knots_ahead_of_its_tick',
]


def test_the_env_hook_sets_the_lead():
    """``tests/conftest.py``'s hook: ``JUGGLEBOT_TEST_EMIT_LEAD_KNOTS`` set
    overrides the generated constant before ``schedule`` is imported; unset
    changes nothing (the configured lead, 0 today)."""
    import jugglebot.hardware_config as hw
    from jugglebot.motion.skills import schedule as sc
    import hardware_config as hw_top
    want = os.environ.get(_ENV)
    if want is not None:
        assert sc.EMIT_LEAD_KNOTS == int(want)
        assert hw.JB_TRAJ_EMIT_LEAD_KNOTS == hw_top.JB_TRAJ_EMIT_LEAD_KNOTS \
            == int(want)
    else:
        # the hook did nothing: the planner reads the configured value
        assert sc.EMIT_LEAD_KNOTS == int(hw_top.JB_TRAJ_EMIT_LEAD_KNOTS)
        assert sc.WIRE_READ_KNOTS == 3 + sc.EMIT_LEAD_KNOTS


def test_the_lead_one_consumers_run_the_real_code():
    """The consumer tests above pass at EMIT_LEAD_KNOTS = 1 as well: a
    subprocess pytest with the env hook at 1 (parallel-safe: its own process
    and its own constants)."""
    env = dict(os.environ)
    env[_ENV] = '1'
    proc = subprocess.run(
        [sys.executable, '-m', 'pytest', '-q', '-p', 'no:cacheprovider']
        + _CONSUMER_IDS,
        capture_output=True, text=True, env=env, cwd=_REPO, timeout=600)
    tail = proc.stdout[-1500:] + proc.stderr[-500:]
    assert proc.returncode == 0, tail
    assert '%d passed' % len(_CONSUMER_IDS) in proc.stdout, tail
