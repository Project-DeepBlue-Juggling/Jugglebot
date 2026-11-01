// records_hover_place_harness.js — the REAL ros_ws/gui/js/records-model.js under node: the hover
// parameters line (JB-90) and the State Machine dodge rule (JB-103). Sandbox built by
// tests/ros/test_gui_records_hover_place.py. Prints one JSON object.
import { recordRows, paramsText, paramLabel, dodgesMinimap } from './records-model.js';

const rec = (params) => ({ when_local: '2026-10-22 18:40', throws: 33, catches: 32, end_code: 'STOPPED',
                           category: 'stopped', note: '', ...(params === undefined ? {} : { params }) });
const json = { patterns: [
    { pattern: 'columns', label: 'Columns', balls: 2, history: 1,
      record: rec({ apex_m: '0.95', separation_mm: '125', dwell_s: '0.27', planner: 'pose' }) },
    { pattern: 'hop', label: 'Hop', balls: 1, history: 1, record: rec({}) },
    { pattern: 'oval_1ball', label: 'Oval', balls: 1, history: 1, record: rec(undefined) },
] };
const rows = Object.fromEntries(recordRows(json, new Set()).map(r => [r.pattern, r.title]));
// Rects as getBoundingClientRect gives them (top/bottom/height).
const R = (top, bottom) => ({ top, bottom, height: bottom - top });

console.log(JSON.stringify({
    titles: rows,
    full: paramsText({ apex_m: '0.95', separation_mm: '125', dwell_s: '0.27', planner: 'pose',
                       turned_angle_deg: '30', apex_m_ball2: '0.85' }),
    none: [paramsText({}), paramsText(undefined), paramsText(null), paramsText(['x'])],
    labels: [paramLabel('apex_m'), paramLabel('separation_mm'), paramLabel('dwell_s'),
             paramLabel('turned_angle_deg'), paramLabel('apex_m_ball2'), paramLabel('planner'),
             paramLabel('hold_s')],
    dodge: {
        tallPane: dodgesMinimap(R(52, 200), R(500, 760), true, 12),
        reaches: dodgesMinimap(R(52, 200), R(190, 760), true, 12),
        withinGap: dodgesMinimap(R(52, 200), R(205, 760), true, 12),
        expanded: dodgesMinimap(R(52, 200), R(0, 760), false, 12),
        cardHidden: dodgesMinimap(R(52, 200), { top: 0, bottom: 0, height: 0 }, true, 12),
        panelHidden: dodgesMinimap({ top: 0, bottom: 0, height: 0 }, R(0, 760), true, 12),
        noRects: dodgesMinimap(null, null, true, 12),
    },
}));
