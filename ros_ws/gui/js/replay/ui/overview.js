/**
 * overview.js — pure layout / zoom math + the once-per-(recording, zoom) overview strip.
 * Compact overview (charting decision 17): orchestrator state bands + ticks only. No imports; the
 * trackbar passes its `document`. Times are wall-clock epoch seconds; x positions are fractions 0..1
 * of the visible range [v0, v1].
 */

export const MIN_SPAN = 2;     // s: the narrowest zoom
export const ZOOM_STEP = 2;    // +/- scale factor

/** Tick kinds drawn on the overview (homed / levelled are deliberately NOT ticks). */
export const TICK_KINDS = {
    fault: { color: '#ef4444', label: 'fault' },
    skill_attempt: { color: '#a78bfa', label: 'skill attempt' },
    catch_event: { color: '#f59e0b', label: 'cone catch' },
    bb_calibration: { color: '#e2e8f0', label: 'BB calibration' },
};
export const BAND_COLORS = { IDLE: '#f59e0b', LEVELLING: '#06b6d4', ACTIVE: '#22c55e', FAULT: '#ef4444' };
const BAND_FALLBACK = '#64748b';

const p2 = (n) => (n < 10 ? '0' : '') + n;

/** Local wall-clock time of day HH:MM:SS of epoch seconds (the clock the charts label live). */
export function fmtClock(t) {
    if (!isFinite(t) || t <= 0) return '--:--:--';
    const d = new Date(t * 1000);
    return p2(d.getHours()) + ':' + p2(d.getMinutes()) + ':' + p2(d.getSeconds());
}

/** Fraction of [v0,v1] at time t (unclamped). */
export function frac(t, v0, v1) { return v1 > v0 ? (t - v0) / (v1 - v0) : 0; }

/** Time at fraction f of [v0,v1]. */
export function timeAt(f, v0, v1) { return v0 + f * (v1 - v0); }

/** Clamp a [a,b] range into the full range, keeping its span (shifted), min span MIN_SPAN. */
export function clampRange(a, b, full) {
    const fullSpan = full.t1 - full.t0;
    let span = Math.max(MIN_SPAN, b - a);
    if (span >= fullSpan) return { v0: full.t0, v1: full.t1 };
    let v0 = a;
    if (v0 < full.t0) v0 = full.t0;
    if (v0 + span > full.t1) v0 = full.t1 - span;
    return { v0, v1: v0 + span };
}

/** Zoom the view about `center` by `factor` (>1 zooms in: the span shrinks). Clamped to `full`. */
export function zoomAbout(view, factor, center, full) {
    const span = (view.v1 - view.v0) / factor;
    const f = frac(center, view.v0, view.v1);
    const a = center - f * span;
    return clampRange(a, a + span, full);
}

/** Drag-select on the overview: fractions fa, fb of the CURRENT view -> new view (clamped), or null
 *  when the drag is shorter than `minFrac` (a click). */
export function dragToRange(fa, fb, view, full, minFrac) {
    const lo = Math.min(fa, fb), hi = Math.max(fa, fb);
    if (hi - lo < (minFrac === undefined ? 0.01 : minFrac)) return null;
    return clampRange(timeAt(lo, view.v0, view.v1), timeAt(hi, view.v0, view.v1), full);
}

/** Band segments -> [{name, l, w, t0, t1}] (fractions of the view), clipped to the view. */
export function layoutBands(timeline, v0, v1) {
    const out = [];
    const bands = (timeline && timeline.bands) || [];
    for (const band of bands) {
        for (const seg of band.segments || []) {
            const a = Math.max(seg[0], v0), b = Math.min(seg[1], v1);
            if (b <= a) continue;
            out.push({ name: String(seg[2]), l: frac(a, v0, v1), w: frac(b, v0, v1) - frac(a, v0, v1), t0: seg[0], t1: seg[1] });
        }
    }
    return out;
}

/** Ticks of the drawn kinds inside the view -> [{kind, l, t, label}]. */
export function layoutTicks(timeline, v0, v1) {
    const out = [];
    for (const k of (timeline && timeline.ticks) || []) {
        if (!TICK_KINDS[k.kind] || k.t < v0 || k.t > v1) continue;
        out.push({ kind: k.kind, l: frac(k.t, v0, v1), t: k.t, label: k.label || '' });
    }
    return out;
}

const pct = (f) => (f * 100).toFixed(3) + '%';

/**
 * Draw the overview into `el` (called once per (recording, zoom range) — never per frame).
 * @returns {{bands:number, ticks:number}} counts drawn
 */
export function renderOverview(doc, el, timeline, view) {
    el.replaceChildren();
    const bands = layoutBands(timeline, view.v0, view.v1);
    const ticks = layoutTicks(timeline, view.v0, view.v1);
    for (const b of bands) {
        const d = doc.createElement('div');
        d.className = 'rp-band';
        d.style.left = pct(b.l); d.style.width = pct(b.w);
        d.style.background = BAND_COLORS[b.name] || BAND_FALLBACK;
        d.title = b.name + '  ' + fmtClock(b.t0) + ' - ' + fmtClock(b.t1);
        el.appendChild(d);
    }
    for (const k of ticks) {
        const d = doc.createElement('div');
        d.className = 'rp-tick';
        d.style.left = pct(k.l);
        d.style.background = TICK_KINDS[k.kind].color;
        d.title = TICK_KINDS[k.kind].label + (k.label ? ' (' + k.label + ')' : '') + ' @ ' + fmtClock(k.t);
        el.appendChild(d);
    }
    return { bands: bands.length, ticks: ticks.length };
}
