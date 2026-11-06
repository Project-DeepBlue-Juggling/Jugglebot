"""The corpus replay through the PRODUCTION Learner (plan ``plans/active/learner-generalisation.md``
test T-I1; P1 unit L5).

Research characterisation, not a contract: it MEASURES the 3276-row Learner corpus
(``temp/learn/jugglebot/memory.csv``, tracked, schema v1, append-only so ``--limit 3276`` keeps
the same rows after later Sittings add more) through ``tools/probes/learner_replay_production.py``
(production ``Learner`` defaults k = 16, w0 = 2.0, fed row by row as a predictor with the probe's
2-throw delay; definitions in that module's docstring).  Hence ``nightly`` (``pyproject.toml``
marker contract: research/characterisation measuring a corpus), one module-scoped replay.

Recipe (2026-11-06, Jetson, worktree Jugglebot-p3):
    OPENBLAS_NUM_THREADS=1 python tools/probes/learner_replay_production.py \
        --csv temp/learn/jugglebot/memory.csv --limit 3276
 -> 3276 rows, 3233 inliers, cold start (9 first appearances with earlier accepted data) apex
    74.5 mm identity vs 12.9 mm production, steady state 19.8 / 19.5 / 13.9 mm RMS, 0.49 % of
    inliers at the lateral cap, model accepted 99.1 %, K 1.0792; 8.8 s.
    OPENBLAS_NUM_THREADS=1 python -m pytest tests/motion/test_learner_replay.py -q \
        -p no:cacheprovider -m nightly   (2026-11-06: see the unit report for the result).

The old-law steady-state RMS literals (19.9 / 19.7 / 14.4 mm, axis 0 / axis 1 / apex) are the
probe's: ``OPENBLAS_NUM_THREADS=1 python tools/probes/learner_replay.py --csv
temp/probes/learner_replay/corpus_3276.csv --tag 3276 --rule band+struct --lam 0.99``
(2026-11-06, ``scratchpad/P1_PROBE_REPORT.md`` table B, "steady state", old law row).

Pin tolerances: the replay is deterministic float arithmetic; BLAS / libm differences move values
by ~1e-9 relative, EXCEPT that a borderline inlier decision flipping on one row can move a mean over
9-11 first appearances by a mm.  So RMS values are pinned to +-0.5 mm, the cold-start apex means to
+-2.0 mm, shares to +-0.5 percentage points, K to +-0.003 (the per-Sitting K spread is 0.006).
"""
from __future__ import annotations

import os
import sys

import pytest

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, os.path.join(REPO, 'tools', 'probes'))

import learner_replay_production as lrp                         # noqa: E402

CORPUS = os.path.join(REPO, 'temp', 'learn', 'jugglebot', 'memory.csv')
N_ROWS = 3276
#: Old-law steady-state RMS mm (axis 0, axis 1, apex); see the module docstring for the command.
OLD_STEADY = (19.9, 19.7, 14.4)

pytestmark = pytest.mark.nightly


@pytest.fixture(scope='module')
def replay(tmp_path_factory):
    if not os.path.exists(CORPUS):
        pytest.skip('the tracked corpus %s is absent (sparse checkout?)' % CORPUS)
    out_v2 = str(tmp_path_factory.mktemp('learner_replay') / 'memory_v2.csv')
    h, stats, _, _ = lrp.run(CORPUS, limit=N_ROWS, out_v2=out_v2)
    return h, stats


def test_replay_covers_the_whole_corpus_without_drops(replay):
    h, stats = replay
    assert h['n_rows'] == N_ROWS and stats['dropped'] == 0
    assert h['n_inliers'] == pytest.approx(3233, abs=10)


def test_cold_start_apex_meets_the_plan_gate(replay):
    h, _ = replay
    c = h['cold_start_with_earlier_accepted']
    prod, ident = c['production']['apex_mean_abs_mm'], c['identity']['apex_mean_abs_mm']
    assert c['n'] >= 5
    assert prod < 15.0, prod
    assert prod * 3.0 <= ident, (prod, ident)


def test_cold_start_lateral_no_worse_than_identity(replay):
    h, _ = replay
    for key in ('cold_start_with_earlier_accepted', 'cold_start_all_first_appearances'):
        c = h[key]
        assert c['production']['lat_mean_abs_mm'] <= c['identity']['lat_mean_abs_mm'], key
        assert c['production']['lat_rms_mm'] <= c['identity']['lat_rms_mm'], key


def test_steady_state_no_worse_than_the_old_law(replay):
    h, _ = replay
    st = h['steady_state']['production_rms_mm']
    for got, old, name in zip((st['ax0'], st['ax1'], st['apex']), OLD_STEADY, ('axis 0', 'axis 1', 'apex')):
        assert got <= old + 1.0, (name, got, old)


def test_share_at_the_cap_is_small(replay):
    h, _ = replay
    assert h['share_at_cap_inliers'] < 0.20            # the plan's T-I1 baseline
    assert h['share_at_cap_inliers'] < 0.05


def test_model_accepts_nearly_every_row_and_k_is_plausible(replay):
    h, _ = replay
    assert h['model_accepted_share'] > 0.95
    assert 1.06 <= h['K'] <= 1.10, h['K']


def test_headline_values_are_pinned(replay):
    h, _ = replay
    st = h['steady_state']['production_rms_mm']
    assert st['ax0'] == pytest.approx(19.8, abs=0.5)
    assert st['ax1'] == pytest.approx(19.5, abs=0.5)
    assert st['apex'] == pytest.approx(13.9, abs=0.5)
    c = h['cold_start_with_earlier_accepted']
    assert c['production']['apex_mean_abs_mm'] == pytest.approx(12.9, abs=2.0)
    assert c['identity']['apex_mean_abs_mm'] == pytest.approx(74.5, abs=2.0)
    assert 100 * h['share_at_cap_inliers'] == pytest.approx(0.49, abs=0.5)
    assert h['model_accepted'] == pytest.approx(3244, abs=15)
    assert h['K'] == pytest.approx(1.0792, abs=0.003)
