"""Pin the GUI juggle panel's per-pattern apex / separation / dwell pre-fill
(``PATTERN_POINTS`` in ros_ws/gui/js/juggle-panel.js) to what the robot accepts.

Root cause (R5 sitting 7b): the panel flew skill_node's single 0.9 m / 250 mm
default and kept a typed apex across pattern switches, but each pattern is
flyable at a different point (columns 125 mm, hop only 250 mm), so operator
inputs were refused or silently flew the wrong geometry.  Goal acceptance is a
nomination (``motion/skills/nominate.py``, since P3b 2026-11-06; the offline
Admissible box this file used to pin the pre-fills to is retired), so a
pre-fill the nomination refuses at the working point recreates that refusal:
this test fails if any row does not nominate viable.
"""
from __future__ import annotations

import re
from pathlib import Path

from jugglebot.motion.skills import nominate as nom

ROOT = Path(__file__).resolve().parents[2]
JS = (ROOT / "ros_ws" / "gui" / "js" / "juggle-panel.js").read_text()
PATTERNS = re.findall(r"^\s*\{ value: '(\w+)'", JS, re.M)


def _points():
    """pattern -> dict(apex_m, separation_mm|None, dwell_s, + the optional
    param-backed field), parsed from the PATTERN_POINTS block."""
    block = re.search(r"export const PATTERN_POINTS = \{(.*?)\n\};", JS, re.S).group(1)
    out = {}
    for m in re.finditer(r"^\s*(\w+):\s*\{(.*?)\},?\s*$", block, re.M):
        fields = dict(re.findall(r"(\w+): (null|[\d.]+)", m.group(2)))
        out[m.group(1)] = {k: (None if v == "null" else float(v))
                           for k, v in fields.items()}
    return out


def test_every_panel_pattern_has_a_point():
    assert set(_points()) == set(PATTERNS) and len(PATTERNS) == 9
    for pt in _points().values():
        assert pt["apex_m"] > 0.0 and pt["dwell_s"] > 0.0


def _nomination_config(pattern, pt):
    """The NominationConfig the node builds for this pattern's pre-fill
    (`SkillNode._nomination_config` at the node defaults: pose planner, no carry
    ZVD, feed site P1, self-toss Site (-50, 0); the two parameter-backed
    patterns add `apex_m_ball2` / the angle)."""
    return nom.NominationConfig(
        pattern=pattern, apex_m=pt["apex_m"],
        separation_mm=0.0 if pt["separation_mm"] is None else pt["separation_mm"],
        dwell_s=pt["dwell_s"], apex_m_ball2=pt.get("apex_m_ball2"),
        angle_deg=pt.get("columns_angle_deg", 0.0), planner_mode="pose",
        carry_zvd=False, feed_site="P1", self_toss_site_xy_mm=(-50.0, 0.0),
        entry="hop" if pattern in nom.COLUMNS_FED else "t0")


def test_every_point_nominates_viable_at_the_working_point():
    """A pre-fill the robot would refuse reddens the suite: each pattern's
    pre-filled apex / separation / dwell (and Apex 2 / Angle for the two
    parameter-backed goals) nominates viable at the generated launch limits."""
    for pattern, pt in _points().items():
        rep = nom.nominate(_nomination_config(pattern, pt), search=False)
        assert rep.ok, "%s pre-fill %s is refused: %s" % (
            pattern, pt, "; ".join(rep.lines()))


def test_param_backed_points_are_present():
    """The Angle and Apex 2 pre-fills the nomination test above feeds its
    config exist and are positive (a missing one would silently nominate plain
    Columns / equal-apex high-low)."""
    pts = _points()
    assert pts["columns_turned"]["columns_angle_deg"] > 0.0
    assert pts["columns_highlow"]["apex_m_ball2"] > 0.0
    assert pts["columns_highlow"]["apex_m_ball2"] != pts["columns_highlow"]["apex_m"]


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


def test_dwell_is_an_editable_field_sent_on_the_wire():
    assert "dwell_s: {" in JS and "'dwell_s']" in JS
    assert "parts.push('dwell_s=' + wireNum(typed.dwell_s, false));" in JS
    assert "juggle-dwell" not in JS and "dwellVal" not in JS   # the read-only span is gone


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
