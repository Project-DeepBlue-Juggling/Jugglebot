"""Pin the GUI juggle panel's per-pattern apex / separation pre-fill
(``PATTERN_POINTS`` in ros_ws/gui/js/juggle-panel.js) to the committed
admissible box.

Root cause (R5 sitting 7b): the panel flew skill_node's single 0.9 m / 250 mm
default and kept a typed apex across pattern switches, but the box is swept at
a different point per pattern (columns 125 mm / apex band 0.925-0.975, hop
only 250 mm), so operator inputs were refused or silently flew the wrong
geometry.  A pre-fill that drifts out of the box recreates that refusal, so
this test fails if any row is outside it.
"""
from __future__ import annotations

import re
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]
JS = (ROOT / "ros_ws" / "gui" / "js" / "juggle-panel.js").read_text()
BOX = yaml.safe_load((ROOT / "config" / "generated" / "admissible_box.yaml").read_text())
PATTERNS = re.findall(r"^\s*\{ value: '(\w+)'", JS, re.M)


def _sep_of(cell):
    """Site separation of a two-site cell: twice the release site's radius
    (the turned cells sit off the x axis, so the x coordinate alone is not it)."""
    x, y = cell["release_site_xy_mm"]
    return 2 * (x * x + y * y) ** 0.5


def _points():
    block = re.search(r"export const PATTERN_POINTS = \{(.*?)\n\};", JS, re.S).group(1)
    out = {}
    for m in re.finditer(
            r"(\w+):\s*\{ box: '(\w+)',\s*apex_m: ([\d.]+),\s*separation_mm: (null|[\d.]+),"
            r"\s*dwell_s: ([\d.]+)(?:, \w+: [\d.]+)? \}", block):
        out[m.group(1)] = (m.group(2), float(m.group(3)),
                           None if m.group(4) == "null" else float(m.group(4)),
                           float(m.group(5)))
    return out


def test_every_panel_pattern_has_a_point():
    assert set(_points()) == set(PATTERNS) and len(PATTERNS) == 9


def test_points_sit_inside_the_committed_box():
    for pattern, (kind, apex, sep, _dwell) in _points().items():
        rows = [b for b in BOX["boxes"] if b["pattern"] == kind]
        assert rows, pattern
        if sep is None:   # one-site pattern: apex only
            hits = [b for b in rows if b["apex_band_m"][0] <= apex <= b["apex_band_m"][1]]
        else:
            hits = [b for b in rows
                    if abs(_sep_of(b) - sep) < 1e-6
                    and b["apex_band_m"][0] <= apex <= b["apex_band_m"][1]]
        assert hits, "%s: apex %.3f / sep %s is not covered by any %s box" % (
            pattern, apex, sep, kind)


def test_panel_switches_points_on_pattern_change():
    assert "applyPatternPoint(ui.pattern);" in JS
    assert JS.count("applyPatternPoint(") >= 4   # def + init + change + reset


def test_magazine_button_sends_exact_token_and_reuses_start_dispatch():
    """Throw-to-magazine is a separate hold-to-confirm button, NOT a pattern row
    (apex/separation/num_cycles do not apply); it shares Start's dispatch/latch."""
    assert "export const MAGAZINE_REQUEST = 'magazine';" in JS
    assert "dispatchJuggle(MAGAZINE_REQUEST, null)" in JS
    assert "holdToConfirm(magazine, onMagazineConfirm, HOLD_MS)" in JS
    assert "value: 'magazine'" not in JS and len(PATTERNS) == 9
    assert "msg.name === 'MAGAZINE_MISS'" in JS
    # gate: connection + ACTIVE + not busy, field errors ignored
    assert "startGateReason(null, true)" in JS and "if (ignoreFields) return null;" in JS


def test_reload_button_sends_exact_token_and_shares_the_magazine_gate():
    """The plain "Reload" button (2026-10-13): Ball Butler feeds, Jugglebot
    catches and holds — the bare `reload` token, not a pattern row, gated and
    dispatched exactly like the magazine button (hold-to-confirm, one latch)."""
    assert "export const RELOAD_REQUEST = 'reload';" in JS
    assert "dispatchJuggle(RELOAD_REQUEST, null)" in JS
    assert "holdToConfirm(reloadBtn, onReloadConfirm, HOLD_MS)" in JS
    assert "setDisabled(dom.reloadBtn, !!magReason);" in JS
    assert "value: 'reload'" not in JS and len(PATTERNS) == 9
    assert "actions.append(start, reloadBtn, magazine, stop, status);" in JS


def test_no_reload_flag_on_wire_and_cup_preview_exists():
    """P2 (columns-empty-cup-two-feed D3/D4): the skill node decides reload-first
    from the cup sensor, so the request builder never emits a `reload` token
    after the pattern (the orchestrator refuses it) and the 'Reload first'
    checkbox / lob drawing are gone.  The bare `reload` PATTERN stays."""
    for dead in ("userReload", "effectiveReload", "reloadRequired", 'juggle-reload"',
                 "m.lob", "jp-arrow-lob", "hand lob", "Reload first", "parts.push('reload')"):
        assert dead not in JS, dead
    body = re.search(r"function buildRequestData\(p\) \{(.*?)\n\}", JS, re.S).group(1)
    assert "reload" not in body.replace("No `reload` token", "")
    # the cup preview strings exist for all three states plus the fixed requirements
    for text in ("cup EMPTY: Ball Butler loads first", "cup SEATED: throw first",
                 "cup UNKNOWN: will be refused", "needs an empty cup",
                 "needs an empty cup (Ball 1 phantom)", "needs a ball in the cup"):
        assert text in JS, text
    # columns + EMPTY is accepted by the node since P3 (2026-10-17): the preview shows the two-feed start
    assert "TWO_FEED_LANDED = true" in JS  # P3 landed 2026-10-17: the node accepts columns on an EMPTY cup
    assert "cup EMPTY: will be refused (two-feed chain not landed)" in JS and "columnsEmptyRefused" in JS
    assert "TWO_FEED_LANDED" in re.search(r"function cupPreviewText\(p, cup\) \{(.*?)\n\}", JS, re.S).group(1)
    # classification is read from the shared ball-in-hand pill, not re-subscribed
    assert "flag-ball-held-pill" in JS and "ros.subscribe" not in JS


def test_point_dwell_equals_the_boxs_swept_dwell_per_pattern():
    """Owner 2026-10-21: dwell_s travels on the Juggle goal and the GUI
    pre-fills it per pattern FROM THE BOX.  The 22:53 sitting's oval refusal
    ("live dwell 0.27 but the box was swept at 0.65") was a per-pattern value
    living in a node parameter the operator had to remember.  A re-sweep at a
    new dwell now fails here until PATTERN_POINTS follows it."""
    for pattern, (kind, apex, sep, dwell) in _points().items():
        rows = [b for b in BOX["boxes"] if b["pattern"] == kind]
        assert rows, pattern
        if sep is None:
            near = rows
        else:
            near = [b for b in rows if abs(_sep_of(b) - sep) < 1e-6]
        covering = [b for b in near
                    if b["apex_band_m"][0] <= apex <= b["apex_band_m"][1]]
        assert covering, pattern
        swept = {float(b["dwell_s"]) for b in covering}
        # rung 2 (2026-10-23) swept the oval at 0.55 / 0.50 too: the GUI
        # default must be one of the swept dwells (the other families: only one).
        assert dwell in swept, "%s: GUI dwell %s not in box dwell(s) %s" % (
            pattern, dwell, sorted(swept))
        if kind not in ("oval_1ball", "oval"):
            assert swept == {dwell}


def test_dwell_is_an_editable_field_sent_on_the_wire():
    assert "dwell_s: {" in JS and "'dwell_s']" in JS
    assert "parts.push('dwell_s=' + wireNum(typed.dwell_s, false));" in JS
    assert "juggle-dwell" not in JS and "dwellVal" not in JS   # the read-only span is gone


def _admissible(cell):
    """A swept cell that admits something: a finite landing window."""
    import math
    return all(math.isfinite(v) for row in cell["landing_xy_m"] for v in row)


def test_param_backed_points_sit_in_their_own_admissible_cells():
    """The two parameter-backed goals (pattern-progression P2/P4, 2026-10-31):
    the pre-filled Angle must have an admissible turned cell (`P1@<angle>`) at
    the pre-filled apex and separation, and the pre-filled Apex 2 an admissible
    `columns_highlow` cell in its own band at that separation.  A re-sweep that
    empties either band fails here until the pre-fill follows it."""
    js_pts = re.search(r"export const PATTERN_POINTS = \{(.*?)\n\};", JS, re.S).group(1)
    angle = float(re.search(r"columns_turned:.*?columns_angle_deg: ([\d.]+)", js_pts).group(1))
    apex2 = float(re.search(r"columns_highlow:.*?apex_m_ball2: ([\d.]+)", js_pts).group(1))
    kind, apex, sep, _ = _points()["columns_turned"]
    turned = [b for b in BOX["boxes"] if b["pattern"] == kind and b["planner_mode"] == "pose"
              and b["site_pair"][0].endswith("@%g" % angle) and abs(_sep_of(b) - sep) < 1e-6
              and b["apex_band_m"][0] <= apex <= b["apex_band_m"][1]]
    assert len(turned) == 2 and all(_admissible(b) for b in turned), turned
    kind, apex, sep, _ = _points()["columns_highlow"]
    for a in (apex, apex2):
        cells = [b for b in BOX["boxes"] if b["pattern"] == kind and abs(_sep_of(b) - sep) < 1e-6
                 and b["apex_band_m"][0] <= a <= b["apex_band_m"][1]]
        assert len(cells) == 2 and all(_admissible(b) for b in cells), (a, cells)


def test_param_backed_goals_write_the_node_parameter_before_dispatch():
    """Angle / Apex 2 are node parameters, not goal fields: Start writes the
    typed value through /skill_node/set_parameters and dispatches only on a
    successful write; the value never rides the request string (the relay
    would refuse the token) and a blank field is a Start error, never 'node
    default' (a stale parameter from an earlier goal must never fly)."""
    rows = {m.group(1): m.group(2) for m in re.finditer(r"\{ value: '(\w+)'[^{}]*?param: '(\w+)'", JS, re.S)}
    assert rows == {"columns_turned": "columns_angle_deg", "columns_highlow": "apex_m_ball2"}
    assert "const SET_PARAM_SERVICE = '/skill_node/set_parameters';" in JS
    assert "if (!p.param) { dispatchJuggle(data, cc); return; }" in JS
    assert "setParamThenDispatch(p, data, cc);" in JS
    assert "if (!r || !r.successful) throw new Error" in JS
    assert "return FIELDS[p.param].label + ': required for ' + p.label;" in JS
    body = re.search(r"function buildRequestData\(p\) \{(.*?)\n\}", JS, re.S).group(1)
    assert "columns_angle_deg" not in body and "apex_m_ball2" not in body
    assert "const FIELD_KEYS = GOAL_FIELD_KEYS.concat(PARAM_FIELD_KEYS);" in JS
    # the juggle_start echo closes the stale-relay cross-check for both
    assert "f.angle_deg" in JS and "f.apex_m_ball2" in JS
