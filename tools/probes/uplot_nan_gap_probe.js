// uplot_nan_gap_probe.js — regression probe: NaN runs MUST render as pen-up
// gaps (not bridging lines) in the GUI telemetry charts' vendored uPlot.
//
// Validates the SHIPPED nanGaps hook (extracted verbatim from
// ros_ws/gui/js/telemetry-charts.js at run time, so validated code == shipped
// code) against the REAL vendored ros_ws/gui/lib/uPlot.iife.min.js.
// Pins version-specific behaviour: uPlot 1.6.31's gap scanner honours strict
// null only, and the chart store's Float64Array columns can only carry NaN —
// without the hook, a stale window draws a false bridging ramp on resume.
//
// Run:      node tools/probes/uplot_nan_gap_probe.js   (exit 0 = all pass)
// Re-run whenever lib/uPlot.iife.min.js is upgraded or the nanGaps hook /
// series wiring in telemetry-charts.js changes.
//
// Logbook:  logbook/2026-07-11-gui-leg-setpoint-echo-poscmd.md (review HIGH #1)
// Guarded code: ros_ws/gui/js/telemetry-charts.js (nanGaps + gaps wiring)
//
// Validate the NaN-gap fix (telemetry-charts.js nanGaps) against the REAL
// vendored uPlot 1.6.31 linear path builder.
//
// The nanGaps function is EXTRACTED from ros_ws/gui/js/telemetry-charts.js
// (validated code == shipped code), and the series config mirrors the exact
// object buildUPlotOpts pushes for pos_commanded (dashed, points off), plus
// the internals uPlot's instance-construction would set (pxRound, spanGaps,
// sorted defaults).  The script also greps the source to assert the series
// wiring (`gaps: nanGaps`) is present.
//
// Assertions:
//   S1  interior NaN run  → gap tuple IDENTICAL to uPlot's own null-run
//       output for the same shape; clip Path2D excludes the bridge interval.
//   S2  all-finite data   → no gaps, no clip, every point drawn, no jump.
//   S3  single-point NaN  → one gap [px(k-1), px(k+1)], clip present.
//   S4  all-NaN           → no throw, no gaps, nothing rendered.
//   S5  NaN runs at both window edges → no tuples (nothing to clip) and no
//       drawn segment outside the finite middle.
//   S6  mixed null+NaN (plain Array, defensive branch) → sorted, merged,
//       non-overlapping tuples.
//   S7  dashed stroke     → builder output identical with/without dash
//       (dash applies at ctx.stroke time, after the clip).
'use strict';
const fs = require('fs');
const path = require('path');
const REPO = path.resolve(__dirname, '..', '..');

// ---- Path2D stub that records verbs (canvas spec: nonfinite lineTo/moveTo
// are no-ops at RENDER time; Path2D recording itself accepts them). ----
class Path2DStub {
    constructor() { this.ops = []; }
    moveTo(x, y) { this.ops.push(['moveTo', x, y]); }
    lineTo(x, y) { this.ops.push(['lineTo', x, y]); }
    rect(...a)   { this.ops.push(['rect', ...a]); }
    arc(...a)    { this.ops.push(['arc', ...a]); }
    bezierCurveTo(...a) { this.ops.push(['bez', ...a]); }
    addPath(p)   { this.ops.push(['addPath', p]); }
    closePath()  { this.ops.push(['closePath']); }
}
global.Path2D = Path2DStub;

const libSrc = fs.readFileSync(
    path.join(REPO, 'ros_ws/gui/lib/uPlot.iife.min.js'), 'utf8');
const uPlot = eval(libSrc + '; uPlot');

// ---- Extract the SHIPPED nanGaps from telemetry-charts.js ----
const tcSrc = fs.readFileSync(
    path.join(REPO, 'ros_ws/gui/js/telemetry-charts.js'), 'utf8');
const m = tcSrc.match(/function nanGaps\(u, sidx, i0, i1, nullGaps\) \{[\s\S]*?\n\}/);
if (!m) { console.error('FAIL: nanGaps not found in telemetry-charts.js'); process.exit(1); }
const nanGaps = eval('(' + m[0] + ')');
if (!/gaps:\s*nanGaps,/.test(tcSrc)) {
    console.error('FAIL: series wiring `gaps: nanGaps` not found in buildUPlotOpts');
    process.exit(1);
}
console.log('shipped nanGaps extracted; series wiring present');

// ---- Fake-u scaffolding (mirrors the verifier probe) ----
const N = 30;
const bbox = { left: 0, top: 0, width: 300, height: 100 };
const xs = new Float64Array(N);
for (let i = 0; i < N; i++) xs[i] = i;

function valToPosH(val, scale, dim, off) { return off + (val - scale.min) / (scale.max - scale.min) * dim; }
function valToPosV(val, scale, dim, off) { return off + dim - (val - scale.min) / (scale.max - scale.min) * dim; }
const scaleX = { ori: 0, dir: 1, min: 0, max: N - 1, distr: 1 };
const pxOf = (xVal) => Math.round(valToPosH(xVal, scaleX, bbox.width, bbox.left));

/** Exact series config buildUPlotOpts pushes for pos_commanded, plus the
 *  instance-construction internals (pxRound from pxAlign=1 default,
 *  spanGaps=false default, sorted flag). `gapsFn` defaults to the shipped
 *  nanGaps; pass uPlot's passthrough default ((u,s,i0,i1,g)=>g) to get
 *  baseline behaviour. */
function makeSeries({ dash = [5, 3], gapsFn = nanGaps } = {}) {
    return {
        label: 'Pos (cmd)',
        scale: 'y',
        stroke: '#60a5fa',
        width: 1.5,
        dash,
        points: { show: false },
        gaps: gapsFn,
        // instance internals:
        pxRound: v => Math.round(v),
        pxAlign: 1,
        sorted: 1,
        spanGaps: false,
    };
}

function makeU(ys, series) {
    return {
        mode: 1,
        series: [{ scale: 'x' }, series],
        _data: [xs, ys],
        data: [xs, ys],           // nanGaps reads u.data
        scales: { x: scaleX, y: { ori: 1, dir: 1, min: -1.2, max: 1.2, distr: 1 } },
        bbox,
        valToPosH,
        valToPosV,
        // nanGaps calls u.valToPos(val, 'x', true) — canvas px, like the real API.
        valToPos(val, scaleKey, canvas) {
            const sc = this.scales[scaleKey];
            return scaleKey === 'x'
                ? valToPosH(val, sc, bbox.width, bbox.left)
                : valToPosV(val, sc, bbox.height, bbox.top);
        },
        pxRatio: 1,
    };
}

function build(ys, seriesOpts) {
    const series = makeSeries(seriesOpts);
    const u = makeU(ys, series);
    return uPlot.paths.linear()(u, 1, 0, N - 1);
}

/** Canvas-render simulation: drop nonfinite moveTo/lineTo (spec no-ops),
 *  return surviving vertices. */
function renderedVerts(stroke) {
    return (stroke ? stroke.ops : []).filter(op =>
        (op[0] === 'lineTo' || op[0] === 'moveTo') && isFinite(op[1]) && isFinite(op[2]));
}

/** Largest x-jump actually DRAWN (consecutive rendered verts, 2nd is lineTo). */
function maxDrawnJump(stroke) {
    const v = renderedVerts(stroke);
    let maxJump = 0;
    for (let i = 1; i < v.length; i++) {
        if (v[i][0] !== 'lineTo') continue;
        maxJump = Math.max(maxJump, v[i][1] - v[i - 1][1]);
    }
    return maxJump;
}

/** True iff no clip rect overlaps the OPEN interval (a, b). */
function clipExcludesInterval(clip, a, b) {
    if (!clip) return false;
    for (const op of clip.ops) {
        if (op[0] !== 'rect') continue;
        const [x, , w] = [op[1], op[2], op[3]];
        if (x < b && x + w > a) return false;  // rect overlaps gap interior
    }
    return true;
}

let failures = 0;
function assert(cond, label) {
    console.log((cond ? 'PASS' : 'FAIL') + '  ' + label);
    if (!cond) failures++;
}

function ysWith(nanIdx) {
    const ys = new Float64Array(N);
    for (let i = 0; i < N; i++) ys[i] = nanIdx(i) ? NaN : Math.sin(i / 5);
    return ys;
}

// ---- S1: interior NaN run (10..19) ----
{
    const out = build(ysWith(i => i >= 10 && i < 20));
    // uPlot's OWN behaviour for the same shape with strict nulls:
    const ysNull = new Array(N);
    for (let i = 0; i < N; i++) ysNull[i] = (i >= 10 && i < 20) ? null : Math.sin(i / 5);
    const ref = build(ysNull, { gapsFn: (u, s, i0, i1, g) => g });  // uPlot default passthrough
    console.log(`S1 gaps=${JSON.stringify(out.gaps)}  uPlot-null-ref=${JSON.stringify(ref.gaps)}`);
    assert(JSON.stringify(out.gaps) === JSON.stringify(ref.gaps),
        'S1: NaN-run gap tuple IDENTICAL to uPlot\'s own null-run tuple');
    assert(out.gaps.length === 1 && out.gaps[0][0] === pxOf(9) && out.gaps[0][1] === pxOf(20),
        `S1: gap spans [px(lastFiniteBefore)=${pxOf(9)}, px(firstFiniteAfter)=${pxOf(20)}]`);
    assert(out.clip instanceof Path2DStub && out.clip.ops.some(o => o[0] === 'rect'),
        'S1: clip Path2D produced');
    assert(clipExcludesInterval(out.clip, out.gaps[0][0], out.gaps[0][1]),
        'S1: clip EXCLUDES the gap interior — the bridging lineTo cannot render');
    assert(renderedVerts(out.stroke).length > 0,
        'S1: finite data on both sides still draws');
}

// ---- S2: all-finite data ----
{
    const out = build(ysWith(() => false));
    console.log(`S2 gaps=${JSON.stringify(out.gaps)} clip=${String(out.clip)} drawnVerts=${renderedVerts(out.stroke).length} maxJump=${maxDrawnJump(out.stroke)}`);
    assert(out.gaps.length === 0, 'S2: no gaps for all-finite data');
    assert(out.clip == null, 'S2: no clip for all-finite data (nothing clipped away)');
    assert(renderedVerts(out.stroke).length === N, 'S2: every point drawn');
    assert(maxDrawnJump(out.stroke) <= Math.ceil(bbox.width / (N - 1)) + 1,
        'S2: no jump beyond one-sample spacing (normal data unaffected)');
}

// ---- S3: single-point NaN (k=15) ----
{
    const out = build(ysWith(i => i === 15));
    console.log(`S3 gaps=${JSON.stringify(out.gaps)} expected=[[${pxOf(14)},${pxOf(16)}]]`);
    assert(out.gaps.length === 1 && out.gaps[0][0] === pxOf(14) && out.gaps[0][1] === pxOf(16),
        'S3: single NaN → one gap [px(k-1), px(k+1)]');
    assert(out.clip instanceof Path2DStub, 'S3: clip present');
    assert(clipExcludesInterval(out.clip, out.gaps[0][0], out.gaps[0][1]),
        'S3: clip excludes the single-point gap interior');
}

// ---- S4: all-NaN (BB charts 7-8 pos_commanded) ----
{
    let out = null, threw = null;
    try { out = build(ysWith(() => true)); } catch (e) { threw = e; }
    console.log(`S4 threw=${threw ? threw.message : 'no'} gaps=${out ? JSON.stringify(out.gaps) : '-'} drawnVerts=${out ? renderedVerts(out.stroke).length : '-'}`);
    assert(threw === null, 'S4: all-NaN does not throw');
    assert(out.gaps.length === 0, 'S4: all-NaN → no gap tuples (nothing to clip)');
    assert(renderedVerts(out.stroke).length === 0, 'S4: all-NaN renders NOTHING (non-drawn series, not an error)');
}

// ---- S5: NaN runs at both window edges (0..9 and 20..29 NaN) ----
{
    const out = build(ysWith(i => i < 10 || i >= 20));
    const verts = renderedVerts(out.stroke);
    const xsDrawn = verts.map(v => v[1]);
    console.log(`S5 gaps=${JSON.stringify(out.gaps)} drawnX=[${Math.min(...xsDrawn)}..${Math.max(...xsDrawn)}] finiteRegion=[${pxOf(10)}..${pxOf(19)}] maxJump=${maxDrawnJump(out.stroke)}`);
    assert(out.gaps.length === 0, 'S5: edge runs produce no tuples (no bridging segment exists)');
    assert(Math.min(...xsDrawn) >= pxOf(10) && Math.max(...xsDrawn) <= pxOf(19),
        'S5: nothing drawn outside the finite middle');
    assert(maxDrawnJump(out.stroke) <= Math.ceil(bbox.width / (N - 1)) + 1,
        'S5: no bridge drawn across the edge runs');
}

// ---- S6: mixed null+NaN in a plain Array (defensive merge branch) ----
{
    const ys = new Array(N);
    for (let i = 0; i < N; i++) ys[i] = Math.sin(i / 5);
    ys[10] = null; ys[11] = NaN;              // overlapping null-gap + NaN-run
    ys[16] = NaN; ys[17] = NaN;               // pure NaN run
    const out = build(ys);
    const sortedNonOverlap = out.gaps.every((g, i) =>
        g[1] > g[0] && (i === 0 || g[0] > out.gaps[i - 1][1]));
    console.log(`S6 gaps=${JSON.stringify(out.gaps)}`);
    assert(out.gaps.length === 2 && sortedNonOverlap,
        'S6: null+NaN merge → sorted, non-overlapping tuples');
    assert(out.gaps[0][0] === pxOf(9) && out.gaps[0][1] === pxOf(12)
        && out.gaps[1][0] === pxOf(15) && out.gaps[1][1] === pxOf(18),
        'S6: merged tuple spans the union run; second run independent');
}

// ---- S7: dashed vs solid — identical builder output (dash applies at
//          ctx.stroke time via setLineDash, after the clip) ----
{
    const dashed = build(ysWith(i => i >= 10 && i < 20), { dash: [5, 3] });
    const solid  = build(ysWith(i => i >= 10 && i < 20), { dash: undefined });
    assert(JSON.stringify(dashed.gaps) === JSON.stringify(solid.gaps)
        && JSON.stringify(dashed.stroke.ops) === JSON.stringify(solid.stroke.ops),
        'S7: dashed stroke does not change paths/gaps (dash is a render-time property)');
}

console.log(failures === 0 ? '\nALL ASSERTIONS PASSED' : `\n${failures} ASSERTION(S) FAILED`);
process.exit(failures === 0 ? 0 : 1);
