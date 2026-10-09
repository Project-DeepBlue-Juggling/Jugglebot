"""GUI: a failed BB recalibration keeps "Calibrated" and shows its reason
separately (keep-last-good, 2026-10-10).

``ros_ws/gui/js/bb-calibration-status.js`` holds the pure view logic; it is run
here UNMODIFIED under node (copied into a sandbox marked as an ESM package, as
``test_gui_bb_aim_timeout.py`` does for bb-aim.js). The wiring in main.js /
panels.js is DOM-bound, so it is pinned by string tripwires: the indicator
follows ``bb/calibration_result`` (the calibration in force), the note and the
event log follow ``bb/calibration_attempt`` (every sweep's outcome).
"""
from __future__ import annotations

import json
import os
import re
import shutil
import subprocess
from pathlib import Path

import pytest

from tests.ros.test_gui_bb_aim_timeout import _NODE, requires_node

ROOT = Path(__file__).resolve().parents[2]
GUI_JS = ROOT / 'ros_ws' / 'gui' / 'js'

_SCRIPT = r"""
import { calibrationIndicator, attemptNote, parseAcceptedAt } from './bb-calibration-status.js';
const ok = {success: true, message: 'Calibration successful (accepted 2026-10-10T12:34:56Z) · sweep · gate: vs accepted 2026-10-09T13:24:52 ok'};
const okNoTime = {success: true, message: 'Calibration successful'};
const fail = {success: false, message: 'CALIBRATION_INCONSISTENT: Δyaw -0.366° exceeds ±0.150°'};
console.log(JSON.stringify({
  ok: calibrationIndicator(ok),
  okNoTime: calibrationIndicator(okNoTime),
  fail: calibrationIndicator(fail),
  none: calibrationIndicator(null),
  noteFail: attemptNote(fail),
  noteOk: attemptNote(ok),
  noteNone: attemptNote(null),
  noteEmpty: attemptNote({success: false, message: ''}),
  parsed: parseAcceptedAt(ok.message).toISOString(),
  parsedNone: parseAcceptedAt('Calibration successful'),
}));
"""


@requires_node
def test_view_logic_under_node(tmp_path):
    shutil.copy(GUI_JS / 'bb-calibration-status.js', tmp_path / 'bb-calibration-status.js')
    (tmp_path / 'package.json').write_text('{"type": "module"}\n')
    (tmp_path / 'run.js').write_text(_SCRIPT, encoding='utf-8')
    out = subprocess.check_output([_NODE, 'run.js'], cwd=str(tmp_path),
                                  env=dict(os.environ, TZ='UTC'), timeout=30)
    r = json.loads(out.decode())
    assert r['ok'] == {'calibrated': True, 'text': 'Calibrated · 12:34'}
    assert r['okNoTime'] == {'calibrated': True, 'text': 'Calibrated'}
    assert r['fail'] == {'calibrated': False, 'text': 'Not Calibrated'}
    assert r['none'] == {'calibrated': False, 'text': 'Not Calibrated'}
    assert r['noteFail'] == ('Last attempt failed: CALIBRATION_INCONSISTENT: '
                             'Δyaw -0.366° exceeds ±0.150°')
    assert r['noteOk'] == '' and r['noteNone'] == ''
    assert r['noteEmpty'] == 'Last attempt failed'
    assert r['parsed'] == '2026-10-10T12:34:56.000Z' and r['parsedNone'] is None


def _fn_body(src, name):
    m = re.search(r'function\s+' + name + r'\b.*?\n\}', src, re.S)
    assert m, f'{name}() not found'
    return m.group(0)


def test_main_routes_the_two_topics():
    main = (GUI_JS / 'main.js').read_text()
    assert re.search(r"ros\.subscribe\('bb/calibration_attempt',\s*"
                     r"'jugglebot_interfaces/msg/BallButlerCalibrationResult',\s*"
                     r"onBBCalibrationAttempt", main)
    result = _fn_body(main, 'onBBCalibrationResult')
    assert 'updateBBCalibration(msg)' in result and 'setBallButlerCalibration(msg)' in result
    assert 'emitEvent' not in result, \
        'the event log follows bb/calibration_attempt, one event per sweep'
    attempt = _fn_body(main, 'onBBCalibrationAttempt')
    assert 'updateBBCalibrationAttempt(msg)' in attempt and 'emitEvent' in attempt
    assert "'bb/calibration_attempt'" in main.split('GUI_SUBSCRIBED_TOPICS')[1].split(']);')[0]


def test_panels_use_the_view_logic_and_reset_clears_the_note():
    panels = (GUI_JS / 'panels.js').read_text()
    assert "from './bb-calibration-status.js'" in panels
    assert 'id="bb-calib-note"' in panels
    assert 'calibrationIndicator(msg)' in _fn_body(panels, 'updateBBCalibration')
    assert 'attemptNote(msg)' in _fn_body(panels, 'updateBBCalibrationAttempt')
    assert 'updateBBCalibrationAttempt(null)' in _fn_body(panels, 'resetBBCalibration')
