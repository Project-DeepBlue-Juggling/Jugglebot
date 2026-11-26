"""BUG-75 replay tooling (tools/probes/late_reaim/): chained re-aims, timing shift, record selection, row labels.

Stubs only: no sim, no solver.  Each test names the mutation that makes it fail.
"""
from __future__ import annotations

import os
import pickle
import sys

import numpy as np
import pytest

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
# The probes prepend their own directory needs to sys.path at import, including
# the repo's `sim/` directory. Left there, `sim/` makes the legacy spellings
# (`ball_butler.sim`, `viz.telemetry`) importable for every later test in this
# worker, which tests/sim/test_sim_import_style.py forbids. Import the probes,
# then put sys.path back exactly as it was.
_SYS_PATH_BEFORE = list(sys.path)
sys.path.insert(0, os.path.join(ROOT, 'tools', 'probes', 'late_reaim'))
import bump_replay as br  # noqa: E402
import label_rows as lr  # noqa: E402
sys.path[:] = _SYS_PATH_BEFORE


def test_importing_the_probes_leaves_sys_path_as_it_was():
    assert os.path.join(ROOT, 'sim') not in sys.path
    assert os.path.join(ROOT, 'tools', 'probes', 'late_reaim') not in sys.path


def _row(seq, code='INSTALLED', mm=10.0, ds=0.01, skill=1, log='d1', ang=45, apex=0.95, sep=125.0, to_land=0.5):
    return dict(log=log, run=1, skill=skill, seq=seq, n=seq, t=100.0 + seq, code=code, mm=mm, ds=ds,
                ang=ang, apex=apex, sep=sep, to_land=to_land, pat='columns_turned')


class _Term:
    def __init__(self, x=0.0, t=0.0):
        self.x, self.t_land_s = x, t


def _stub_apply(refuse_if):
    """apply_raw stub: the 'record' is a list of the rows installed so far; refuses when refuse_if(rec, row)."""
    seen = []

    def apply_raw(rec, state, row, d):
        seen.append((list(rec), state))
        if refuse_if(rec, row):
            return 'INFEASIBLE', rec + ['junk'], _Term(x=-999.0)
        st = _Term(x=(0.0 if state is None else state.x) + d[0] * row['mm'],
                   t=(0.0 if state is None else state.t_land_s) + row['ds'])
        return 'OK', rec + [row['seq']], st
    return apply_raw, seen


def test_chain_applies_in_logged_order_each_on_previous_record():
    # mutation that fails it: pass `rec` (the base record) instead of `new_rec` on to the next row
    apply_raw, seen = _stub_apply(lambda rec, row: False)
    group = [_row(2), _row(1), _row(3)]  # shuffled; seq decides
    ordered = br.group_catches(group)[0]
    out = br.replay_chain(ordered, (1, 0), lambda row: [], apply_raw)
    assert [r['seq'] for r in ordered] == [1, 2, 3]
    assert [s[0] for s in seen] == [[], [1], [1, 2]]
    assert [s[1].x if s[1] else None for s in seen] == [None, 10.0, 20.0]
    assert [o['code'] for o in out] == ['OK', 'OK', 'OK']


def test_refused_reaim_leaves_record_and_state_as_they_were():
    # mutation: update rec/state also when the code is not OK
    apply_raw, seen = _stub_apply(lambda rec, row: row['seq'] == 2)
    out = br.replay_chain([_row(1), _row(2, code='INFEASIBLE'), _row(3)], (1, 0), lambda row: [], apply_raw)
    assert [o['code'] for o in out] == ['OK', 'INFEASIBLE', 'OK']
    assert seen[2][0] == [1] and seen[2][1].x == 10.0
    # row 2 was refused in both worlds, so the chain stays intact; a flown-accepted row refused by replay breaks it
    assert [o['chain_ok'] for o in out] == [True, True, True]
    out = br.replay_chain([_row(1), _row(2)], (1, 0), lambda row: [], _stub_apply(lambda r, w: w['seq'] == 1)[0])
    assert [o['chain_ok'] for o in out] == [True, False]


def test_last_row_goes_through_apply_last_on_chained_record():
    got = []
    apply_raw, _ = _stub_apply(lambda rec, row: False)
    br.replay_chain([_row(1), _row(2)], (1, 0), lambda row: [], apply_raw,
                    lambda rec, state, row, d: got.append((list(rec), row['seq'])) or 'OK')
    assert got == [([1], 2)]


def test_moved_terminal_applies_ds_only_when_asked():
    # mutation: ignore use_ds / always apply it
    import dataclasses
    T = dataclasses.make_dataclass('T', [('landing_mm', object), ('t_land_s', float), ('then_throw', object)])
    t0 = T(np.zeros(3), 2.0, None)
    assert br.moved_terminal(t0, (3.0, -4.0), 0.02, True).t_land_s == pytest.approx(2.02)
    assert br.moved_terminal(t0, (3.0, -4.0), 0.02, False).t_land_s == 2.0
    assert list(br.moved_terminal(t0, (3.0, -4.0), 0.02, True).landing_mm) == [3.0, -4.0, 0.0]


def test_record_selection_by_row_label(tmp_path):
    # mutation: return the default/first file whatever the label
    paths = {}
    for name in ('a', 'b'):
        paths[name] = str(tmp_path / (name + '.pkl'))
        with open(paths[name], 'wb') as f:
            pickle.dump(name, f)
    spec = ['45:0.95:125=%s' % paths['a'], '90:1.0:120=%s' % paths['b']]
    rs = br.RecordSet(None, br.RecordSet.parse(spec))
    assert rs.get(_row(1, ang=45)) == 'a'
    assert rs.get(_row(1, ang=90, apex=1.0, sep=120.0)) == 'b'
    assert not rs.has(_row(1, ang=90, apex=0.95, sep=125.0))
    with pytest.raises(KeyError):
        rs.get(_row(1, ang=90, apex=0.95, sep=125.0))
    # default (no --record): one file serves every row, as in the 2026-11-25 commands
    assert br.RecordSet(paths['a']).get(_row(1, ang=90)) == 'a'


def test_default_mode_untouched_flags():
    # the 2026-11-25 commands name only --records/--rows/--raw[/--raw-ds]: no chain/record flag may be on by default
    import argparse
    src = open(os.path.join(ROOT, 'tools', 'probes', 'late_reaim', 'bump_replay.py')).read()
    assert "ap.add_argument('--chain', action='store_true'" in src
    assert "ap.add_argument('--record', action='append', default=[]" in src
    assert "if a.chain or a.record:" in src


LOG = """\
1000.0 [skill_node-10] skills/nominate 'columns_turned,apex_m=0.95,separation_mm=125,dwell_s=0.27,num_cycles=20,columns_angle_deg=45': nomination OK
1001.0 [orchestrator_node-3] Juggle requested via jugglebot/juggle_request — goal dispatched (pattern='columns_turned', apex_m=0.95, dwell_s=0.27, num_cycles=20).
1002.0 [skill_node-10] [skill_node]: 1002.000 CATCH-AIM skill 4: source=tracker landing=(-80.0, -30.0, 829.9) mm t_land=1003.000
1002.5 [skill_node-10] [skill_node]: 1002.400 RESEND skill 4: the fit moved the landing 27.8 mm / +0.023 s (re-aim 1 of 2) — spliced CATCH at knot 83
1002.7 [skill_node-10] [skill_node]: 1002.600 RESEND-REFUSED INFEASIBLE at skill 4: the fit moved the landing 17.7 mm / -0.004 s — REJECTED
1002.9 [skill_node-10] [skill_node]: 1002.700 RESEND-REFUSED LIMIT_VEL at skill 4: the fit moved the landing 9.0 mm / +0.001 s — REJECTED
1003.0 [skill_node-10] skills/nominate 'columns_turned,apex_m=1.0,separation_mm=120,dwell_s=0.27,num_cycles=20,columns_angle_deg=90': nomination OK
1004.0 [orchestrator_node-3] Juggle requested via jugglebot/juggle_request — goal dispatched (pattern='columns_turned', apex_m=1.0, dwell_s=0.27, num_cycles=20).
1005.0 [skill_node-10] [skill_node]: 1005.000 CATCH-AIM skill 4: source=tracker landing=(0.0, 0.0, 829.9) mm t_land=1006.000
1005.5 [skill_node-10] [skill_node]: 1005.500 RESEND-REFUSED INFEASIBLE at skill 4: the fit moved the landing 12.0 mm / -0.002 s — REJECTED
1006.0 [orchestrator_node-3] Juggle requested via jugglebot/juggle_request — goal dispatched (pattern='self_toss', apex_m=0.9, dwell_s=0.27, num_cycles=20).
1007.0 [skill_node-10] [skill_node]: 1007.500 RESEND-REFUSED INFEASIBLE at skill 4: the fit moved the landing 5.0 mm / -0.002 s — REJECTED
"""


def test_label_lines_orders_rows_and_labels_runs():
    # mutations: take the angle/Apex from the wrong line, count refused rows as n=1 always, or key the Catch by skill alone
    rows = lr.label_lines(LOG.splitlines(), 'dX')
    assert [(r['run'], r['seq'], r['n'], r['code']) for r in rows] == [
        (1, 1, 1, 'INSTALLED'), (1, 2, 2, 'INFEASIBLE'), (1, 3, 2, 'LIMIT_VEL'), (2, 1, 1, 'INFEASIBLE')]
    assert [(r['ang'], r['apex'], r['sep']) for r in rows] == [(45, 0.95, 125.0)] * 3 + [(90, 1.0, 120.0)]
    assert rows[0]['mm'] == 27.8 and rows[0]['ds'] == pytest.approx(0.023) and rows[1]['ds'] == pytest.approx(-0.004)
    assert rows[0]['to_land'] == pytest.approx(0.6) and rows[3]['to_land'] == pytest.approx(0.5)
    # the self_toss run's refusal is not a columns_turned row
    assert len(rows) == 4
    # group_catches keeps (log, run, skill) apart: skill 4 of run 1 and of run 2 are two Catches
    assert [len(g) for g in br.group_catches(rows)] == [3, 1]


# Real launch-log lines (temp/logs/launch_r2gate_20260918_1325.log and a 2026-11 launch): the OLD format.
_OLD_REFUSED = ("[skill_node-12] [INFO] [1789702122.955757938] [skill_node]: 1789702122.909 RESEND-REFUSED LIMIT_ACC at skill 2: "
                "the fit moved the landing 78.9 mm / -0.038 s — REJECTED_CYCLE_INFEASIBLE(LIMIT_ACC: peak leg acceleration 6462.9 mm/s² > 5000.0)")
_OLD_OK = ("[skill_node-12] [INFO] [1790083594.784561046] [skill_node]: 1790083594.720 RESEND skill 2: the fit moved the landing "
           "0.0 mm / +0.025 s (re-aim 1 of 2) — spliced CATCH at knot 44 (1.100 s on the plan clock)")
# The NEW format appends ` dx=<mm> dy=<mm>` (Robot frame, new landing minus installed) at the end of the same lines.
_NEW_REFUSED = _OLD_REFUSED + "; no further re-aim for this catch dx=-30.0 dy=+14.0"
_NEW_OK = _OLD_OK + " dx=+12.0 dy=-7.0"


def test_label_rows_and_parse_resends_read_old_and_new_lines(tmp_path):
    # mutations: anchor the old regexes at end of line (new lines stop matching), drop the dx/dy read (new rows give None),
    # or give old lines a number (old rows must be None, not 0)
    import json
    import subprocess
    # label_lines skips rows outside a Juggle run, so feed it the run header lines first
    hdr = ["1.0 [skill_node-10] skills/nominate 'columns_turned,apex_m=0.95,separation_mm=125,dwell_s=0.27,num_cycles=20,"
           "columns_angle_deg=45': nomination OK",
           "2.0 [orchestrator_node-3] Juggle requested via jugglebot/juggle_request — goal dispatched (pattern='columns_turned', "
           "apex_m=0.95, dwell_s=0.27, num_cycles=20)."]
    rows = lr.label_lines(hdr + [_OLD_REFUSED, _OLD_OK, _NEW_REFUSED, _NEW_OK], 'd')
    assert [(r['dx'], r['dy']) for r in rows] == [(None, None), (None, None), (-30.0, 14.0), (12.0, -7.0)]
    assert [r['mm'] for r in rows] == [78.9, 0.0, 78.9, 0.0]
    d = tmp_path / 'log'
    d.mkdir()
    (d / 'launch.log').write_text('\n'.join([_OLD_REFUSED, _OLD_OK, _NEW_REFUSED, _NEW_OK]) + '\n')
    out = tmp_path / 'out.json'
    subprocess.run([sys.executable, os.path.join(ROOT, 'tools', 'probes', 'late_reaim', 'parse_resends.py'),
                    str(out), 'd=%s' % d], check=True, capture_output=True)
    got = json.loads(out.read_text())
    assert [(r['dx'], r['dy']) for r in got] == [(None, None), (None, None), (-30.0, 14.0), (12.0, -7.0)]
    assert [r['code'] for r in got] == ['LIMIT_ACC', 'INSTALLED', 'LIMIT_ACC', 'INSTALLED']
