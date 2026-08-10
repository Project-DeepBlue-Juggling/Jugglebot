"""Contract C-TOSS-CAL-1 — the toss AIM calibration loader and its one apply seam.

Covers `jugglebot/motion/toss_cal.py` (pure, no ROS) plus the two structural
guards that make the *contract* real rather than documented:

* **D4 — one lookup, one scope.** The map is evaluated exactly once per toss
  GOAL, in ``reload_coordinator_node._build_toss_cycle``. The AST manifest below
  is the same shape as ``tests/ros/test_levelling_frame.py``'s C-LEVEL-2
  manifest, and it exists for the same reason: a second evaluation site (a 40 Hz
  emitter, a per-knot call, the reload path) is a perfectly working program whose
  only symptom is a silently different aim.
* **The disabled path is TODAY'S MACHINE, provably.** Zero aim ⇒ the virtual
  target offset is exactly ``[0.0, 0.0]`` ⇒ the tilted path reproduces
  ``compute_release_state`` bitwise. Pinned in
  ``tests/motion/test_toss_release.py``; re-stated here from the map's side.

Every file this scans is read from the source tree, so these run under the
pytest venv with no ROS 2 sourced.
"""

from __future__ import annotations

import ast
import math
import os

import numpy as np
import pytest
import yaml

from jugglebot.motion import toss_cal
from jugglebot.motion.toss_cal import (
    ESTIMATOR_VERSION,
    TOTAL_MAX_RAD,
    TossCal,
    TossCalError,
    clamp_total_aim,
    load_toss_cal,
    lookup,
    map_version,
    parse_toss_cal,
    resolve_toss_cal_path,
    toss_cal_candidates,
)

_PKG_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
    'ros_ws', 'src', 'jugglebot', 'jugglebot')


# ── Fixtures ──────────────────────────────────────────────────────────────────

def _doc(**over):
    """A minimal VALID schema-v1 document.

    The grid is deliberately **asymmetric** in both axes and in both aim
    components: a transposed ``[iy][ix]`` index is invisible on a symmetric map,
    and that is the mistake this schema most invites.
    """
    doc = {
        'version': 1,
        'captured': {'date': '2026-08-11', 'tool': 'toss_cal_fit.py'},
        'requires': {'tilt_map_version': '2026-08-10-abcd1234',
                     'estimator_version': ESTIMATOR_VERSION},
        'units': {'aim': 'rad'},
        'grid': {'z_mm': 170.0, 'orientation': 'level',
                 'x_mm': [-150.0, 0.0, 150.0],
                 'y_mm': [-100.0, 200.0]},
        'aim_rad': {
            'rx': [[0.001, 0.002, 0.003],
                   [0.004, 0.005, 0.006]],
            'ry': [[-0.001, -0.002, -0.003],
                   [-0.004, -0.005, -0.006]],
        },
    }
    doc.update(over)
    return doc


def _write(tmp_path, doc, name='toss_calibration.yaml'):
    path = tmp_path / name
    path.write_text(yaml.safe_dump(doc, sort_keys=False))
    return str(path)


# ── Parse: the happy path and the frame convention ────────────────────────────

def test_parse_builds_the_grid_with_iy_ix_indexing():
    cal = parse_toss_cal(_doc())
    assert cal.shape == (2, 3)
    # [iy][ix]: row selects y, column selects x. Asymmetric on purpose.
    assert cal.rx_rad[1][2] == pytest.approx(0.006)
    assert cal.ry_rad[0][0] == pytest.approx(-0.001)
    assert cal.z_mm == 170.0
    assert cal.requires_tilt_map_version == '2026-08-10-abcd1234'
    assert cal.requires_estimator_version == ESTIMATOR_VERSION


def test_the_arrays_are_frozen_but_the_callers_are_not():
    """Read-only because the map is a shared per-process reference every goal
    reads — one in-place write would silently re-aim every subsequent throw.
    Copied because the fit tool self-validates its own working arrays before it
    writes and must not find them frozen for the rest of its run."""
    rx = np.zeros((2, 3))
    cal = TossCal(x_mm=[-1.0, 0.0, 1.0], y_mm=[-1.0, 1.0], rx_rad=rx,
                  ry_rad=np.zeros((2, 3)), version='v',
                  requires_tilt_map_version='', requires_estimator_version='e')
    with pytest.raises(ValueError):
        cal.rx_rad[0][0] = 1.0
    rx[0][0] = 1.0            # the caller's own array is untouched
    assert cal.rx_rad[0][0] == 0.0


def test_lookup_is_exact_at_nodes_and_bilinear_between():
    cal = parse_toss_cal(_doc())
    assert lookup(cal, -150.0, -100.0) == (pytest.approx(0.001),
                                           pytest.approx(-0.001))
    assert lookup(cal, 150.0, 200.0) == (pytest.approx(0.006),
                                         pytest.approx(-0.006))
    # Midpoint of the first cell in x, at y = -100.
    rx, ry = lookup(cal, -75.0, -100.0)
    assert rx == pytest.approx(0.0015)
    assert ry == pytest.approx(-0.0015)


def test_outside_the_hull_the_query_clamps_and_never_extrapolates():
    """A wrong-signed edge extrapolation aims a throw WORSE than no map at all —
    C-LEVEL-2 states the rule and it binds harder here, because this map's whole
    output is the aim."""
    cal = parse_toss_cal(_doc())
    assert lookup(cal, -10_000.0, -10_000.0) == lookup(cal, -150.0, -100.0)
    assert lookup(cal, 10_000.0, 10_000.0) == lookup(cal, 150.0, 200.0)


def test_a_non_finite_query_raises_rather_than_returning_nan():
    """A NaN aim becomes a NaN virtual target, a NaN launch velocity and a NaN
    event_vel on the wire — surfacing only as a downstream rejection after the
    goal has already claimed the platform."""
    cal = parse_toss_cal(_doc())
    for bad in (float('nan'), float('inf')):
        with pytest.raises(TossCalError):
            lookup(cal, bad, 0.0)
        with pytest.raises(TossCalError):
            lookup(cal, 0.0, bad)


# ── Validation is all-or-nothing and loud ─────────────────────────────────────

@pytest.mark.parametrize('mutate, fragment', [
    (lambda d: d.__setitem__('version', 2), 'schema version'),
    (lambda d: d.__setitem__('version', True), 'schema version'),
    (lambda d: d.pop('version'), "'version' key"),
    (lambda d: d.pop('grid'), "'grid' block"),
    (lambda d: d.pop('aim_rad'), "'aim_rad' block"),
    (lambda d: d.pop('units'), "'units' block"),
    (lambda d: d.pop('requires'), "'requires' block"),
    (lambda d: d['requires'].pop('tilt_map_version'), 'requires.tilt_map_version'),
    (lambda d: d['requires'].pop('estimator_version'), 'requires.estimator_version'),
    (lambda d: d['units'].__setitem__('aim', 'mm'), 'units.aim'),
    (lambda d: d['grid'].__setitem__('x_mm', [0.0]), 'at least 2 nodes'),
    (lambda d: d['grid'].__setitem__('x_mm', [150.0, 0.0, -150.0]),
     'strictly increasing'),
    (lambda d: d['grid'].__setitem__('y_mm', [1.0, 1.0]), 'strictly increasing'),
    (lambda d: d['aim_rad'].__setitem__('rx', [[0.0, 0.0], [0.0, 0.0]]),
     'indexed [iy][ix]'),
    (lambda d: d['aim_rad'].__setitem__(
        'ry', [[0.0, 0.0, float('nan')], [0.0, 0.0, 0.0]]), 'non-finite'),
    (lambda d: d['grid'].__setitem__('z_mm', 'high'), 'grid.z_mm'),
])
def test_every_schema_fault_refuses_the_whole_document(mutate, fragment):
    """No partial load, no per-node repair, no silent zero-fill: a half-trusted
    aim map is indistinguishable at the machine from a correct one until a ball
    misses."""
    doc = _doc()
    mutate(doc)
    with pytest.raises(TossCalError) as exc:
        parse_toss_cal(doc)
    assert fragment in str(exc.value)


def test_the_units_key_is_validated_not_assumed():
    """SC-1 may find the residual is fixed-mm or impulse-shaped rather than
    angular. A mm-valued grid read as radians is a ~3000x error, so the build
    refuses a unit it does not know how to apply."""
    with pytest.raises(TossCalError) as exc:
        parse_toss_cal(_doc(units={'aim': 'mm'}))
    assert 'rad' in str(exc.value)
    assert '3000' in str(exc.value)


def test_the_authority_bound_is_a_magnitude_not_a_per_axis_box():
    """A per-axis box would admit hypot = 1.414 deg at the corners — 41 % past
    the authority every downstream argument (composition regime, cup swing,
    landing shift) is sized on."""
    just_under = math.radians(0.99)
    doc = _doc(aim_rad={'rx': [[just_under, 0.0, 0.0], [0.0, 0.0, 0.0]],
                        'ry': [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]})
    parse_toss_cal(doc)                                # magnitude 0.99 deg: fine

    both = math.radians(0.8)                           # hypot = 1.131 deg
    doc = _doc(aim_rad={'rx': [[both, 0.0, 0.0], [0.0, 0.0, 0.0]],
                        'ry': [[both, 0.0, 0.0], [0.0, 0.0, 0.0]]})
    with pytest.raises(TossCalError) as exc:
        parse_toss_cal(doc)
    assert 'authority bound' in str(exc.value)
    assert 'iy=0' in str(exc.value) and 'ix=0' in str(exc.value)


def test_the_authority_bound_is_one_degree():
    """1.0 deg — the design's TOTAL_MAX (§ 3.6, D7). Three independent reasons,
    each a number: it keeps the additive rotation composition inside C-LEVEL-2's
    documented 1x1 deg regime; it caps the cup swing at 1.13 mm; and it is
    ~55 mm of commanded landing shift at h = 0.78 m, 1.6x the 35 mm capture
    radius, so a saturated wrong-signed aim is guaranteed to miss."""
    assert TOTAL_MAX_RAD == pytest.approx(math.radians(1.0))
    assert 64.78 * math.sin(TOTAL_MAX_RAD) == pytest.approx(1.1306, abs=1e-3)


def test_clamp_total_aim_preserves_direction_and_names_its_hit():
    """A per-axis clamp would ROTATE the aim, which is worse than shortening it."""
    rx, ry, hits = clamp_total_aim(0.0005, -0.0005)
    assert (rx, ry, hits) == (0.0005, -0.0005, [])     # inside: untouched
    big = TOTAL_MAX_RAD
    rx, ry, hits = clamp_total_aim(big, big)
    assert hits == ['total_aim']
    assert math.hypot(rx, ry) == pytest.approx(TOTAL_MAX_RAD)
    assert rx == pytest.approx(ry)                     # direction preserved
    with pytest.raises(TossCalError):
        clamp_total_aim(float('nan'), 0.0)


def test_clamp_at_apply_is_a_no_op_for_any_valid_map():
    """Parse time bounds every node's magnitude and a bilinear blend of bounded
    vectors is a convex combination, hence bounded. The apply-time clamp exists
    because D7 requires the TOTAL to be re-clamped at apply, and phase 2e's trim
    adds to this number — one enforcement point, landed with the seam."""
    edge = math.radians(1.0)
    cal = parse_toss_cal(_doc(aim_rad={
        'rx': [[edge, -edge, edge], [-edge, edge, -edge]],
        'ry': [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]}))
    for x in np.linspace(-200.0, 200.0, 41):
        for y in np.linspace(-200.0, 300.0, 41):
            rx, ry = lookup(cal, float(x), float(y))
            assert clamp_total_aim(rx, ry)[2] == []


# ── Provenance dormancy (D3) ──────────────────────────────────────────────────

def test_a_matching_provenance_is_appliable():
    cal = parse_toss_cal(_doc())
    assert cal.provenance_mismatch('2026-08-10-abcd1234') is None


def test_a_tilt_map_mismatch_makes_the_map_dormant():
    """An aim residual fitted under tilt map A double-counts tilt map B's
    delta — the map LOADS and does nothing, loudly."""
    cal = parse_toss_cal(_doc())
    reason = cal.provenance_mismatch('2026-08-11-99999999')
    assert reason and 'tilt_map_version' in reason


def test_an_unknown_live_tilt_map_is_a_mismatch_not_a_pass():
    """"I cannot verify which layer 0 is underneath me" is not "the right one is
    underneath me". Fail closed; it costs only precision."""
    cal = parse_toss_cal(_doc())
    reason = cal.provenance_mismatch('')
    assert reason and 'UNKNOWN' in reason


def test_a_map_captured_with_no_tilt_map_is_dormant_on_a_mapped_machine():
    cal = parse_toss_cal(_doc(requires={'tilt_map_version': '',
                                        'estimator_version': ESTIMATOR_VERSION}))
    assert cal.provenance_mismatch('') is None
    reason = cal.provenance_mismatch('2026-08-10-abcd1234')
    assert reason and 'NO tilt map' in reason


def test_an_estimator_mismatch_makes_the_map_dormant():
    """The estimator version names what ``land_err_mm`` MEANS — fit plane rule,
    band width, lateral gate, minimum samples. A map fitted under one definition
    and applied under another is the R1 frame-error class with a plausible
    version string on it."""
    cal = parse_toss_cal(_doc(requires={
        'tilt_map_version': '2026-08-10-abcd1234',
        'estimator_version': 'arrival-offset/0'}))
    reason = cal.provenance_mismatch('2026-08-10-abcd1234')
    assert reason and 'estimator_version' in reason


# ── map_version ───────────────────────────────────────────────────────────────

def test_version_ignores_provenance_and_tracks_every_applied_number():
    """Two files whose applied numbers are identical report the same version; an
    edit to a single applied node changes it."""
    base = map_version(_doc())
    # Provenance churn must NOT move the version.
    for key, value in (('captured', {'date': '2026-08-11', 'tool': 'other'}),
                       ('stats', {'n_per_node': [[8, 8, 8], [8, 8, 8]]}),
                       ('jacobian', {'gain_mm_per_rad': 3120.0})):
        assert map_version(_doc(**{key: value})) == base
    doc = _doc()
    doc['grid']['orientation'] = 'whatever'
    assert map_version(doc) == base
    # Applied numbers MUST move it.
    doc = _doc()
    doc['aim_rad']['rx'][1][2] += 1e-9
    assert map_version(doc) != base
    assert map_version(_doc(units={'aim': 'mm'})) != base
    assert map_version(_doc(anchor={'aim_rad': [0.001, 0.0]})) != base
    assert map_version(_doc(speed={'k_v': 1.02})) != base


def test_the_date_prefix_is_readable_and_the_digest_identifying():
    v = map_version(_doc())
    assert v.startswith('2026-08-11-')
    assert len(v.split('-')[-1]) == 8
    assert map_version({}) .startswith('undated-')


def test_int_and_float_spellings_of_the_same_calibration_agree():
    """`170` and `170.0` are the same calibration written by two serializers.
    Without float-normalisation the fit tool's re-emit churns the version and
    sends an operator hunting a difference that does not exist — the tilt map's
    own audit finding, inherited."""
    doc = _doc()
    doc['grid']['x_mm'] = [-150, 0, 150]
    doc['aim_rad']['rx'] = [[0.001, 0.002, 0.003], [0.004, 0.005, 0.006]]
    assert map_version(doc) == map_version(_doc())


def test_ndarrays_hash_like_the_lists_they_came_from():
    """The second inherited audit finding: ``json.dumps(default=str)`` would hash
    an ndarray's REPR — which differs from the list's, and which numpy TRUNCATES
    past 1000 elements, so two genuinely different large maps could share a
    version. That is the exact silent-wrong-map failure the string exists to
    make impossible."""
    doc = _doc()
    doc['aim_rad']['rx'] = np.asarray(doc['aim_rad']['rx'])
    doc['grid']['x_mm'] = np.asarray(doc['grid']['x_mm'])
    assert map_version(doc) == map_version(_doc())

    big = {'version': 1, 'units': {'aim': 'rad'},
           'grid': {'x_mm': list(range(40)), 'y_mm': list(range(40))}}
    a = dict(big, aim_rad={'rx': np.zeros((40, 40)), 'ry': np.zeros((40, 40))})
    b_rx = np.zeros((40, 40))
    b_rx[20][20] = 1e-6
    b = dict(big, aim_rad={'rx': b_rx, 'ry': np.zeros((40, 40))})
    assert map_version(a) != map_version(b)


# ── load / resolve ────────────────────────────────────────────────────────────

def test_load_round_trips_a_written_file(tmp_path):
    cal = load_toss_cal(_write(tmp_path, _doc()))
    assert cal.version == map_version(_doc())
    assert cal.shape == (2, 3)


@pytest.mark.parametrize('body, fragment', [
    ('', 'is empty'),
    ('{{{not yaml', 'not valid YAML'),
    ('version: 1\n', "'units' block"),
])
def test_every_file_level_failure_is_one_error_type(tmp_path, body, fragment):
    """Unreadable, non-YAML, empty and schema-invalid all raise the same type:
    the caller's contract is identical in every case — do not load a map, log
    loudly, keep throwing exactly as before."""
    path = tmp_path / 'cal.yaml'
    path.write_text(body)
    with pytest.raises(TossCalError) as exc:
        load_toss_cal(str(path))
    assert fragment in str(exc.value)


def test_an_unreadable_file_is_also_a_toss_cal_error(tmp_path):
    with pytest.raises(TossCalError) as exc:
        load_toss_cal(str(tmp_path / 'does-not-exist.yaml'))
    assert 'cannot read' in str(exc.value)


def test_the_env_override_is_the_only_candidate_when_set(tmp_path):
    """Falling through would apply A DIFFERENT CALIBRATION THAN THE OPERATOR
    NAMED while reporting a plausible version — strictly worse than applying
    none, because C-TOSS-CAL-1 is non-gating and applying none costs only
    precision. A typo therefore degrades to unaimed, loudly."""
    named = str(tmp_path / 'named.yaml')
    assert toss_cal_candidates({toss_cal.TOSS_CAL_ENV: named}) == (named,)
    assert resolve_toss_cal_path({toss_cal.TOSS_CAL_ENV: named}) is None
    _write(tmp_path, _doc(), 'named.yaml')
    assert resolve_toss_cal_path({toss_cal.TOSS_CAL_ENV: named}) == named


def test_the_source_tree_is_searched_before_the_ament_share():
    """The acquisition tool REWRITES the source-tree file at runtime and then
    asks the running node to reload it. Share-first would serve the previous
    build's stale calibration until the next colcon build — silently, with
    toss_cal_loaded still true and a plausible-looking version."""
    candidates = toss_cal_candidates({})
    assert candidates, 'the repo source tree must always be a candidate here'
    assert candidates[0].endswith(
        os.path.join('config', 'toss_calibration.yaml'))
    assert os.path.isdir(os.path.join(os.path.dirname(
        os.path.dirname(candidates[0])), 'ros_ws'))


def test_absence_is_silent_not_an_error():
    """C-TOSS-CAL-1 makes an absent map produce EXACTLY the pre-2b toss, so the
    resolver returns None rather than raising — unlike friction_ff_params, whose
    YAML is a hard runtime dependency."""
    assert resolve_toss_cal_path(
        {toss_cal.TOSS_CAL_ENV: '/nonexistent/toss.yaml'}) is None


def test_the_repo_ships_no_toss_calibration_yet():
    """Phase 2b lands the plumbing APPLIED AT ZERO. A committed map here would
    mean an un-captured, un-verified aim shipped to hardware — and the first
    hardware application of a new map is gated on a deliberately DOUBLED bias on
    ONE node (§ 6 P5.4), which cannot happen if the map arrives with the code."""
    repo = os.path.dirname(os.path.dirname(
        os.path.dirname(os.path.abspath(__file__))))
    assert os.path.isdir(os.path.join(repo, 'ros_ws')), (
        'the repo root walk is wrong — this guard would pass vacuously')
    assert not os.path.exists(
        os.path.join(repo, 'config', 'toss_calibration.yaml')), (
        'config/toss_calibration.yaml exists — if a capture has genuinely run, '
        'delete this test in the same commit that lands the map, and say so in '
        'the logbook')


# ── D4: the map is looked up in exactly one scope ─────────────────────────────

def _calls_in_package(dotted_names):
    """Every (relpath, enclosing-function, dotted-callee) call site in the live
    package matching ``dotted_names``."""
    found = []
    for root, dirs, files in os.walk(_PKG_DIR):
        dirs[:] = [d for d in dirs if d not in ('__pycache__', 'archived')]
        for fname in sorted(files):
            if not fname.endswith('.py'):
                continue
            path = os.path.join(root, fname)
            rel = os.path.relpath(path, _PKG_DIR).replace(os.sep, '/')
            with open(path, 'r', encoding='utf-8') as fh:
                tree = ast.parse(fh.read(), filename=rel)
            scopes = {}
            for node in ast.walk(tree):
                if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
                    for child in ast.walk(node):
                        scopes[id(child)] = node.name
            for node in ast.walk(tree):
                if not isinstance(node, ast.Call):
                    continue
                dotted = _dotted(node.func)
                if dotted in dotted_names:
                    found.append((rel, scopes.get(id(node), '<module>'), dotted))
    return sorted(found)


def _dotted(node):
    parts = []
    while isinstance(node, ast.Attribute):
        parts.append(node.attr)
        node = node.value
    if isinstance(node, ast.Name):
        parts.append(node.id)
        return '.'.join(reversed(parts))
    return ''


def test_the_map_is_looked_up_in_exactly_one_scope():
    """D4, structurally. The aim is evaluated ONCE PER TOSS GOAL and nowhere
    else: not in the 40 Hz emitter, not per Hermite knot, not in
    ``catch_coordinator``, and never on the reload path — a BallButler ball's aim
    belongs to BallButler.

    Why a structural test and not a behavioural one: a second lookup site is a
    perfectly working program. Every behavioural assertion about "is the aim
    applied" still passes; the only symptom is that two evaluations of a
    pose-dependent field disagree, which shows up as a systematically worse aim
    on exactly the tosses that moved. That is the C-LEVEL-2 manifest's argument,
    one refinement layer up.
    """
    sites = _calls_in_package({'toss_cal.lookup', 'lookup'})
    lookups = [s for s in sites if s[2] == 'toss_cal.lookup']
    assert lookups == [('reload_coordinator_node.py', '_toss_aim_for_goal',
                        'toss_cal.lookup')], (
        f"the toss aim map must be evaluated in exactly one scope (D4); "
        f"found {lookups}")


def test_the_one_lookup_scope_is_reached_from_exactly_one_place():
    """The other half of D4: ``_toss_aim_for_goal`` is the only lookup scope, and
    ``_build_toss_cycle`` is the only caller of it — so "once per scope" really
    is "once per goal". ``_build_toss_cycle`` is itself shared verbatim by the
    single ``Toss`` and by every ``TossContinuous`` cycle, which is what stops a
    session drifting from the single toss the hardware ladder validated."""
    callers = _calls_in_package({'self._toss_aim_for_goal'})
    assert callers == [('reload_coordinator_node.py', '_build_toss_cycle',
                        'self._toss_aim_for_goal')], (
        f"_toss_aim_for_goal must be called from _build_toss_cycle alone; "
        f"found {callers}")


def test_nothing_else_imports_the_loader():
    """One owner (operator decision 7): ``reload_coordinator_node`` holds the
    map because the map rewrites a GOAL. The tilt map lives in
    ``trajectory_node`` because it rewrites POSES at ingest. A second importer
    inside the package is a second owner, and two owners of one calibration is
    how "applied" and "loaded" drift apart."""
    importers = set()
    for root, dirs, files in os.walk(_PKG_DIR):
        dirs[:] = [d for d in dirs if d not in ('__pycache__', 'archived')]
        for fname in sorted(files):
            if not fname.endswith('.py'):
                continue
            rel = os.path.relpath(os.path.join(root, fname),
                                  _PKG_DIR).replace(os.sep, '/')
            if rel == 'motion/toss_cal.py':
                continue
            with open(os.path.join(root, fname), 'r', encoding='utf-8') as fh:
                tree = ast.parse(fh.read(), filename=rel)
            # AST, not a substring scan: sibling modules legitimately NAME the
            # contract in prose, and a docstring reference is not an owner.
            for node in ast.walk(tree):
                if isinstance(node, ast.Import):
                    names = [a.name for a in node.names]
                elif isinstance(node, ast.ImportFrom):
                    names = [(node.module or '')] + [
                        (node.module or '') + '.' + a.name for a in node.names]
                else:
                    continue
                if any(n.endswith('toss_cal') or '.toss_cal' in n
                       for n in names):
                    importers.add(rel)
    assert importers == {'reload_coordinator_node.py'}, (
        f"only reload_coordinator_node may own the toss aim map; found "
        f"{sorted(importers)}")


def test_the_loader_imports_no_ros():
    """Pure Python, consumed by ROS nodes AND by tests/motion with no ROS mocking
    at all — the same rule ``tilt_map`` follows, and the reason ``ament_index``
    is imported lazily inside the resolver."""
    with open(os.path.join(_PKG_DIR, 'motion', 'toss_cal.py'),
              encoding='utf-8') as fh:
        tree = ast.parse(fh.read())
    top_level = []
    for node in tree.body:
        if isinstance(node, ast.Import):
            top_level += [a.name for a in node.names]
        elif isinstance(node, ast.ImportFrom):
            top_level.append(node.module or '')
    banned = [m for m in top_level
              if m.split('.')[0] in ('rclpy', 'std_msgs', 'std_srvs',
                                     'geometry_msgs', 'jugglebot_interfaces',
                                     'ament_index_python')]
    assert banned == [], f'toss_cal.py imports ROS at module scope: {banned}'


def test_the_repo_root_is_found_by_marker_search_not_a_file_walk():
    """A fixed ``__file__`` walk is correct only from the source checkout: colcon
    COPIES the package into ``install/jugglebot/lib/python3.8/site-packages/…``,
    where the same number of levels lands on a directory nothing ever creates.
    The source-tree candidate would then be unable to exist in production and
    every load would silently fall through to the stale share copy — the tilt-cal
    Phase-2 finding, inherited."""
    with open(os.path.join(_PKG_DIR, 'motion', 'toss_cal.py'),
              encoding='utf-8') as fh:
        source = fh.read()
    assert 'find_repo_root(__file__)' in source
    assert 'os.path.dirname(os.path.dirname(' not in source
