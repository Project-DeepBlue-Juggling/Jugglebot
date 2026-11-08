// axis_ticks_harness.js — the REAL ros_ws/gui/js/axis-ticks.js (BUG-37) under node. Sandbox built by
// tests/ros/test_gui_axis_ticks.py. Replays uPlot 1.6's findIncr (the step choice: the first ladder
// step whose on-screen spacing is >= `space`) over a grid of plot sizes, spans and offsets, and counts
// the multiples of the chosen step inside [min, max]. Prints one JSON object.
import {
    TIME_INCRS, NUM_LADDER_RATIO, TIME_LADDER_RATIO, numTickSpace, timeTickSpace, timeTickLabels,
} from './axis-ticks.js';

// uPlot's default numeric ladder: genIncrs(10, -16, 0, [1,2,2.5,5]) ++ genIncrs(10, 0, 16, ...).
const NUM_INCRS = [];
for (let e = -16; e <= 16; e++) for (const m of [1, 2, 2.5, 5]) NUM_INCRS.push(+(m * 10 ** e).toPrecision(3));

function findIncr(min, max, incrs, dim, minSpace) {          // uPlot findIncr, digit guard omitted
    const delta = max - min;
    for (const incr of incrs) if (dim * incr / delta >= minSpace) return incr;
    return 0;
}
function ticksIn(min, max, incr) {
    let n = 0;
    for (let v = Math.ceil(min / incr - 1e-9) * incr; v <= max * (1 + 1e-12) + 1e-12; v += incr) n++;
    return n;
}
function ratio(incrs) {
    let r = 0;
    for (let i = 1; i < incrs.length; i++) r = Math.max(r, incrs[i] / incrs[i - 1]);
    return r;
}
function sweep(spaceFn, incrs, spans, offsets) {
    let worst = Infinity, cases = 0, worstCase = null;
    for (let dim = 20; dim <= 1600; dim += 7) for (const span of spans) for (const off of offsets) {
        const min = off, max = off + span;
        const incr = findIncr(min, max, incrs, dim, spaceFn(null, 1, min, max, dim));
        const n = incr ? ticksIn(min, max, incr) : 0;
        cases++;
        if (n < worst) { worst = n; worstCase = { dim, span, off, incr }; }
    }
    return { worst, cases, worstCase };
}
const numSpans = [], timeSpans = [];
for (let e = -3; e <= 4; e++) for (const m of [1, 1.1, 1.37, 1.9, 2.4, 3.3, 4.9, 6.1, 7.7, 9.3]) numSpans.push(m * 10 ** e);
for (const s of [0.004, 0.03, 0.31, 0.9, 1.3, 2.2, 4.4, 6, 9, 14, 29, 44, 61, 110, 170, 290, 590, 1300, 3000, 7000]) timeSpans.push(s);

// Without the fix: the same sweep with uPlot's fixed defaults (30 px y, 50 px x on the stock ladder).
const STOCK_TIME = TIME_INCRS.filter(s => s !== 2 && s !== 120);
const before = {
    num: sweep(() => 30, NUM_INCRS, numSpans, [0, -3.7, 12.34, -345.6]).worst,
    time: sweep(() => 50, STOCK_TIME, timeSpans, [1.7e9, 1.7e9 + 0.37]).worst,
};
const t0 = new Date(2026, 10, 1, 12, 34, 56).getTime() / 1000;

console.log(JSON.stringify({
    numRatio: ratio(NUM_INCRS), timeRatio: ratio(TIME_INCRS), stockTimeRatio: ratio(STOCK_TIME),
    NUM_LADDER_RATIO, TIME_LADDER_RATIO,
    timeSorted: TIME_INCRS.every((v, i) => i === 0 || v > TIME_INCRS[i - 1]),
    num: sweep(numTickSpace(30), NUM_INCRS, numSpans, [0, -3.7, 12.34, -345.6]),
    time: sweep(timeTickSpace(50), TIME_INCRS, timeSpans, [1.7e9, 1.7e9 + 0.37]),
    before,
    bigNumSpace: numTickSpace(30)(null, 1, 0, 1, 900),
    bigTimeSpace: timeTickSpace(50)(null, 0, 0, 1, 900),
    labels: {
        whole: timeTickLabels([t0, t0 + 5]),
        tenths: timeTickLabels([t0, t0 + 0.2, t0 + 0.4]),
        hundredths: timeTickLabels([t0 + 0.05, t0 + 0.1]),
        ms: timeTickLabels([t0 + 0.001, t0 + 0.003]),
        single: timeTickLabels([t0]),
    },
}));
