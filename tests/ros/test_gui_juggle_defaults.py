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


def _points():
    block = re.search(r"export const PATTERN_POINTS = \{(.*?)\n\};", JS, re.S).group(1)
    out = {}
    for m in re.finditer(
            r"(\w+):\s*\{ box: '(\w+)',\s*apex_m: ([\d.]+),\s*separation_mm: (null|[\d.]+) \}", block):
        out[m.group(1)] = (m.group(2), float(m.group(3)),
                           None if m.group(4) == "null" else float(m.group(4)))
    return out


def test_every_panel_pattern_has_a_point():
    assert set(_points()) == set(PATTERNS) and len(PATTERNS) == 5


def test_points_sit_inside_the_committed_box():
    for pattern, (kind, apex, sep) in _points().items():
        rows = [b for b in BOX["boxes"] if b["pattern"] == kind]
        assert rows, pattern
        if sep is None:   # one-site pattern: apex only
            hits = [b for b in rows if b["apex_band_m"][0] <= apex <= b["apex_band_m"][1]]
        else:
            hits = [b for b in rows
                    if abs(2 * abs(b["release_site_xy_mm"][0]) - sep) < 1e-6
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
    assert "value: 'magazine'" not in JS and len(PATTERNS) == 5
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
    assert "value: 'reload'" not in JS and len(PATTERNS) == 5
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
                 "needs an empty cup (ball A phantom)", "needs a ball in the cup"):
        assert text in JS, text
    # columns + EMPTY is accepted by the node since P3 (2026-10-17): the preview shows the two-feed start
    assert "TWO_FEED_LANDED = true" in JS  # P3 landed 2026-10-17: the node accepts columns on an EMPTY cup
    assert "cup EMPTY: will be refused (two-feed chain not landed)" in JS and "columnsEmptyRefused" in JS
    assert "TWO_FEED_LANDED" in re.search(r"function cupPreviewText\(p, cup\) \{(.*?)\n\}", JS, re.S).group(1)
    # classification is read from the shared ball-in-hand pill, not re-subscribed
    assert "flag-ball-held-pill" in JS and "ros.subscribe" not in JS
