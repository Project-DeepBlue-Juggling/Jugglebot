/**
 * imu-charts-core.js — the DOM-free half of the IMU charts (imu-charts.js): binary capture parse, plot
 * columns with gaps, angular acceleration, min/max decimation, nearest-sample lookup, the capture list
 * labels and launch window, the capture selection / follow state machine and the "do not yank" rule.
 * No imports and no globals, so tests/ros/test_gui_imu_charts.py runs it under node as is.
 *
 * Time is epoch SECONDS throughout (the shared chart x axis); the server converts t_wall_us.
 */

// ---- Constants ----------------------------------------------------------------------------------

export const IMU_SHOWN_KEY = 'jugglebot-imu-charts-shown';
export const IMU_SERIES_KEY = 'jugglebot-imu-charts-series';
export const IMU_GYRO_MODE_KEY = 'jugglebot-imu-charts-gyro-mode';

/** Boxcar width (ms) applied to the gyro before the central difference that gives angular acceleration. */
export const ANG_ACCEL_SMOOTH_MS = 10;
/** Decimate when the samples in view exceed this many per plot pixel column. */
export const DECIMATE_ABOVE_PER_PX = 4;
/** List poll period while the group is shown in LIVE and no platform_imu/status is arriving. */
export const LIST_POLL_MS = 5000;
/** LIVE launch window fallback when the recordings API gives no start time. */
export const LIVE_FALLBACK_SEC = 12 * 3600;

export const CAPTURE_MAGIC = 'JBIMU1\0\0';
const HEADER_BYTES = 16;

/** Series definitions; `col` names the plot column. Colours: x/y/z red/green/blue as the 3D triads, |a| neutral. */
export const ACCEL_SERIES = [
    { key: 'ax', col: 'ax', label: 'x', color: '#ef4444' },
    { key: 'ay', col: 'ay', label: 'y', color: '#22c55e' },
    { key: 'az', col: 'az', label: 'z', color: '#3b82f6' },
    { key: 'amag', col: 'amag', label: '|a|', color: '#e2e8f0' },
];
export const GYRO_SERIES = [
    { key: 'gx', col: 'gx', label: 'x', color: '#ef4444' },
    { key: 'gy', col: 'gy', label: 'y', color: '#22c55e' },
    { key: 'gz', col: 'gz', label: 'z', color: '#3b82f6' },
];
export const DEFAULT_SERIES = ['ax', 'ay', 'az', 'amag', 'gx', 'gy', 'gz'];

export const ACCEL_TITLE = 'Linear acceleration (g)';
export const GYRO_TITLE = 'Angular rate (deg/s)';
export const ANG_ACCEL_TITLE = 'Angular acceleration (deg/s^2, ' + ANG_ACCEL_SMOOTH_MS + ' ms boxcar)';
export const FRAME_NOTE = 'IMU x/y/z (sensor frame)';

// ---- Binary capture --------------------------------------------------------------------------------

const LITTLE_ENDIAN_HOST = new Uint8Array(new Uint16Array([1]).buffer)[0] === 1;

/**
 * Parse GET /api/imu/captures/<stem>/data: magic "JBIMU1\0\0", uint32 n, uint32 reserved, float64 t[n]
 * (epoch s), float32 accel_g[n*3], float32 gyro_dps[n*3], uint8 valid[n]; little-endian.
 * @param {ArrayBuffer} buf
 * @returns {{n:number, t:Float64Array, accel:Float32Array, gyro:Float32Array, valid:Uint8Array}}
 */
export function parseImuCapture(buf) {
    if (!(buf instanceof ArrayBuffer)) throw new Error('capture data is not binary');
    if (buf.byteLength < HEADER_BYTES) throw new Error('capture data is truncated (' + buf.byteLength + ' bytes)');
    const head = new Uint8Array(buf, 0, 8);
    for (let i = 0; i < 8; i++) {
        if (head[i] !== CAPTURE_MAGIC.charCodeAt(i)) throw new Error('capture data has the wrong format (bad magic)');
    }
    const dv = new DataView(buf);
    const n = dv.getUint32(8, true);
    const need = HEADER_BYTES + 8 * n + 12 * n + 12 * n + n;
    if (buf.byteLength !== need) {
        throw new Error('capture data length ' + buf.byteLength + ' does not match ' + n + ' samples (' + need + ')');
    }
    const offT = HEADER_BYTES;
    const offA = offT + 8 * n;
    const offG = offA + 12 * n;
    const offV = offG + 12 * n;
    let t, accel, gyro;
    if (LITTLE_ENDIAN_HOST) {
        t = new Float64Array(buf, offT, n);            // offsets 16, 16+8n: 8- and 4-aligned
        accel = new Float32Array(buf, offA, 3 * n);
        gyro = new Float32Array(buf, offG, 3 * n);
    } else {
        t = new Float64Array(n);
        accel = new Float32Array(3 * n);
        gyro = new Float32Array(3 * n);
        for (let i = 0; i < n; i++) t[i] = dv.getFloat64(offT + 8 * i, true);
        for (let i = 0; i < 3 * n; i++) {
            accel[i] = dv.getFloat32(offA + 4 * i, true);
            gyro[i] = dv.getFloat32(offG + 4 * i, true);
        }
    }
    const valid = new Uint8Array(buf, offV, n);
    return { n, t, accel, gyro, valid };
}

/**
 * Plot columns (Float64Array, one per series, shared ascending t). Invalid samples become NaN (a gap);
 * samples whose time is non-finite or not after the previous kept time are dropped, and the drop is
 * marked by one NaN sample between its neighbours so the line breaks there instead of bridging.
 * @returns {{t:Float64Array, ax, ay, az, amag, gx, gy, gz, nGaps:number}}
 */
export function buildPlotColumns(cap) {
    const n = cap.n;
    const t = new Float64Array(2 * n);           // worst case one gap marker per sample
    const cols = {};
    for (const k of ['ax', 'ay', 'az', 'amag', 'gx', 'gy', 'gz']) cols[k] = new Float64Array(2 * n);
    let m = 0;
    let lastT = -Infinity;
    let pendingGap = false;
    let nGaps = 0;
    const put = (tt, ok, i) => {
        t[m] = tt;
        if (ok) {
            const x = cap.accel[3 * i], y = cap.accel[3 * i + 1], z = cap.accel[3 * i + 2];
            cols.ax[m] = x; cols.ay[m] = y; cols.az[m] = z;
            cols.amag[m] = Math.sqrt(x * x + y * y + z * z);
            cols.gx[m] = cap.gyro[3 * i]; cols.gy[m] = cap.gyro[3 * i + 1]; cols.gz[m] = cap.gyro[3 * i + 2];
        } else {
            for (const k in cols) cols[k][m] = NaN;
        }
        m++;
    };
    for (let i = 0; i < n; i++) {
        const ti = cap.t[i];
        if (!Number.isFinite(ti) || ti <= lastT) { pendingGap = true; continue; }
        if (pendingGap && m > 0) { put((lastT + ti) / 2, false, -1); nGaps++; }
        pendingGap = false;
        put(ti, !!cap.valid[i], i);
        lastT = ti;
    }
    const out = { t: t.slice(0, m), nGaps };
    for (const k in cols) out[k] = cols[k].slice(0, m);
    return out;
}

// ---- Angular acceleration ------------------------------------------------------------------------

/** Median sample spacing (s) of an ascending time column, or NaN. */
export function medianDt(t) {
    const n = t.length;
    if (n < 2) return NaN;
    const step = Math.max(1, Math.floor((n - 1) / 512));
    const d = [];
    for (let i = step; i < n; i += step) {
        const v = (t[i] - t[i - step]) / step;
        if (Number.isFinite(v) && v > 0) d.push(v);
    }
    if (!d.length) return NaN;
    d.sort((a, b) => a - b);
    return d[d.length >> 1];
}

/**
 * Angular acceleration (deg/s^2) from a gyro column (deg/s): a centred boxcar of `smoothMs` (rounded to an
 * odd sample count at the median rate), then a central difference over the actual times. A window or a
 * difference that touches a NaN, or that spans a time hole (> 1.5 x its nominal width), is NaN: a gap is
 * never smoothed or differenced across.
 */
export function angularAcceleration(t, y, smoothMs = ANG_ACCEL_SMOOTH_MS) {
    const n = t.length;
    const out = new Float64Array(n).fill(NaN);
    const dt = medianDt(t);
    if (!(dt > 0) || n < 3) return out;
    const h = Math.max(0, Math.round((smoothMs / 1000) / dt / 2));
    const s = new Float64Array(n).fill(NaN);
    // Running sum over [i-h, i+h]; nan = count of non-finite values in the window.
    let sum = 0, nan = 0;
    const add = (j, sign) => { const v = y[j]; if (Number.isFinite(v)) sum += sign * v; else nan += sign; };
    for (let j = 0; j <= Math.min(n - 1, 2 * h); j++) add(j, 1);
    const maxSpan = 1.5 * Math.max(2 * h, 1) * dt;
    for (let i = h; i < n - h; i++) {
        if (i > h) { add(i - h - 1, -1); add(i + h, 1); }
        if (nan === 0 && (h === 0 || t[i + h] - t[i - h] <= maxSpan)) s[i] = sum / (2 * h + 1);
    }
    for (let i = 1; i < n - 1; i++) {
        const a = s[i - 1], b = s[i + 1];
        const span = t[i + 1] - t[i - 1];
        if (Number.isFinite(a) && Number.isFinite(b) && span > 0 && span <= 3 * dt) out[i] = (b - a) / span;
    }
    return out;
}

// ---- Lookup and decimation ------------------------------------------------------------------------

/** First index i with ts[i] >= t (ts ascending). */
export function lowerBound(ts, t) {
    let lo = 0, hi = ts.length;
    while (lo < hi) {
        const mid = (lo + hi) >>> 1;
        if (ts[mid] < t) lo = mid + 1;
        else hi = mid;
    }
    return lo;
}

/**
 * Index of the sample nearest t, or -1 when t is outside the data's coverage (more than `tol` seconds
 * before the first or after the last sample) — the callout of a chart whose data does not cover the
 * hovered time stays hidden.
 */
export function nearestIndex(ts, t, tol) {
    const n = ts.length;
    if (!n || !Number.isFinite(t)) return -1;
    if (t < ts[0] - tol || t > ts[n - 1] + tol) return -1;
    const i = lowerBound(ts, t);
    if (i <= 0) return 0;
    if (i >= n) return n - 1;
    return (t - ts[i - 1] <= ts[i] - t) ? i - 1 : i;
}

/**
 * The arrays to hand uPlot for the view [xMin, xMax] on a plot `widthPx` columns wide.
 * One sample either side of the view is kept so lines reach the edges. When the samples in view are at
 * most DECIMATE_ABOVE_PER_PX x widthPx the slice is returned as zero-copy views (full resolution);
 * otherwise each pixel column becomes three points — first sample time with the column's first extreme,
 * mid time with the other extreme, last time with the column's last value — so every column's min and
 * max are drawn exactly. A column that holds any NaN of a series ends that series with NaN (the gap stays
 * visible at every zoom); a column that is all NaN is all NaN.
 * @param {Float64Array} t
 * @param {Float64Array[]} cols
 * @returns {{data: Array, decimated: boolean, i0:number, i1:number}}
 */
export function decimateMinMax(t, cols, xMin, xMax, widthPx, perPx = DECIMATE_ABOVE_PER_PX) {
    const n = t.length;
    const i0 = Math.max(0, lowerBound(t, xMin) - 1);
    let past = lowerBound(t, xMax);
    while (past < n && t[past] <= xMax) past++;               // first sample after the view
    const i1 = Math.min(n, past + 1);                           // exclusive
    const count = Math.max(0, i1 - i0);
    const W = Math.max(1, Math.floor(widthPx));
    if (count <= perPx * W || !(xMax > xMin)) {
        return { data: [t.subarray(i0, i1), ...cols.map((c) => c.subarray(i0, i1))], decimated: false, i0, i1 };
    }
    const outT = new Float64Array(3 * W + 4);
    const outC = cols.map(() => new Float64Array(3 * W + 4));
    let m = 0;
    const k = W / (xMax - xMin);
    let j = i0;
    // Edge samples outside the view pass through unchanged.
    const pass = (idx) => { outT[m] = t[idx]; for (let c = 0; c < cols.length; c++) outC[c][m] = cols[c][idx]; m++; };
    while (j < i1 && t[j] < xMin) pass(j++);
    let end = i1;
    while (end > j && t[end - 1] > xMax) end--;
    while (j < end) {
        const col = Math.min(W - 1, Math.floor((t[j] - xMin) * k));
        let e = j + 1;
        while (e < end && Math.min(W - 1, Math.floor((t[e] - xMin) * k)) === col) e++;
        const tFirst = t[j], tLast = t[e - 1];
        if (e - j <= 3) {
            for (let q = j; q < e; q++) pass(q);
        } else {
            const tMid = (tFirst + tLast) / 2;
            outT[m] = tFirst; outT[m + 1] = tMid; outT[m + 2] = tLast;
            for (let c = 0; c < cols.length; c++) {
                const y = cols[c];
                let mn = Infinity, mx = -Infinity, iMn = -1, iMx = -1, gap = false, last = NaN;
                for (let q = j; q < e; q++) {
                    const v = y[q];
                    if (!Number.isFinite(v)) { gap = true; continue; }
                    if (v < mn) { mn = v; iMn = q; }
                    if (v > mx) { mx = v; iMx = q; }
                    last = v;
                }
                const o = outC[c];
                if (iMn < 0) { o[m] = NaN; o[m + 1] = NaN; o[m + 2] = NaN; continue; }
                const minFirst = iMn <= iMx;
                o[m] = minFirst ? mn : mx;
                o[m + 1] = minFirst ? mx : mn;
                o[m + 2] = gap ? NaN : last;
            }
            m += 3;
        }
        j = e;
    }
    while (j < i1) pass(j++);
    return { data: [outT.slice(0, m), ...outC.map((c) => c.slice(0, m))], decimated: true, i0, i1 };
}

/** y range with a floor on the half-height for flat data (mirrors telemetry-charts yRangeFor). */
export function imuYRange(dataMin, dataMax, padFloor) {
    if (dataMin == null || dataMax == null || !Number.isFinite(dataMin) || !Number.isFinite(dataMax)) {
        return [-padFloor, padFloor];
    }
    const span = dataMax - dataMin;
    const pad = Math.max(span * 0.05, span === 0 ? padFloor : 0);
    return [dataMin - pad, dataMax + pad];
}

// ---- Capture list ------------------------------------------------------------------------------------

function pad2(v) { return (v < 10 ? '0' : '') + v; }

/** "HH:MM:SS · <label> · <span> s" (+ " · incomplete"), local time of the first sample. */
export function captureLabel(c) {
    const d = new Date(c.t_start * 1000);
    const hms = pad2(d.getHours()) + ':' + pad2(d.getMinutes()) + ':' + pad2(d.getSeconds());
    const span = Number.isFinite(c.t_end - c.t_start) ? (c.t_end - c.t_start).toFixed(1) : '?';
    const label = (c.label && String(c.label).trim()) || 'no label';
    return hms + ' · ' + label + ' · ' + span + ' s' + (c.complete === false ? ' · incomplete' : '');
}

/** Captures with usable times, sorted by t_start (the server sorts too; this is defensive). */
export function normaliseCaptures(listing) {
    const raw = (listing && Array.isArray(listing.captures)) ? listing.captures : [];
    return raw.filter((c) => c && typeof c.stem === 'string' && Number.isFinite(c.t_start) && Number.isFinite(c.t_end))
        .slice().sort((a, b) => a.t_start - b.t_start);
}

/** Local-time start parsed from a recording id "YYYY-MM-DD_HH-MM-SS[...]", or null. */
export function recordingIdStart(id) {
    const m = /^(\d{4})-(\d{2})-(\d{2})[_T](\d{2})[-:](\d{2})[-:](\d{2})/.exec(id || '');
    if (!m) return null;
    const d = new Date(+m[1], +m[2] - 1, +m[3], +m[4], +m[5], +m[6]);
    const s = d.getTime() / 1000;
    return Number.isFinite(s) ? s : null;
}

/** One recording row's start (epoch s): start_ns when real, else the id's local time, else null. */
export function recordingStart(r) {
    if (!r) return null;
    if (typeof r.start_ns === 'number' && r.start_ns > 0 && r.start_ns < 4e18) return r.start_ns / 1e9;
    return recordingIdStart(r.id);
}

/**
 * The LIVE launch window: from the newest recording's start (the bag starts with the launch) to open
 * end; null listing or no usable start -> the last LIVE_FALLBACK_SEC. A newest start in the future
 * (clock skew) also falls back.
 * @returns {{from:number, fallback:boolean}}
 */
export function liveLaunchWindow(listing, nowSec) {
    const recs = (listing && Array.isArray(listing.recordings)) ? listing.recordings : [];
    let best = null;
    for (const r of recs) {
        const s = recordingStart(r);
        if (s !== null && (best === null || s > best)) best = s;
    }
    if (best === null || best > nowSec + 60) return { from: nowSec - LIVE_FALLBACK_SEC, fallback: true };
    return { from: best, fallback: false };
}

/** Plain-words message for a failed list request (`status` = HTTP status, 0 = network failure). */
export function listErrorMessage(status) {
    if (status === 404) return 'IMU capture list unavailable: restart the GUI server to enable IMU charts';
    if (!status) return 'IMU capture list unavailable (GUI server not reachable)';
    return 'IMU capture list unavailable (HTTP ' + status + ')';
}

// ---- Selection / following -------------------------------------------------------------------------

function newestStem(captures) {
    return captures.length ? captures[captures.length - 1].stem : null;
}

function has(captures, stem) {
    return stem !== null && captures.some((c) => c.stem === stem);
}

/**
 * LIVE: the selection after the list (re)loads. Following (or nothing usable selected) -> the newest
 * capture; a hand-picked older capture that is still listed is kept and following stays off.
 * @param {{selected:string|null, follow:boolean}} prev
 * @returns {{selected:string|null, follow:boolean, changed:boolean}}
 */
export function liveAfterRefresh(prev, captures) {
    const newest = newestStem(captures);
    if (!prev.follow && has(captures, prev.selected)) {
        return { selected: prev.selected, follow: prev.selected === newest, changed: false };
    }
    return { selected: newest, follow: true, changed: newest !== prev.selected };
}

/** LIVE: the operator picks `stem`; picking the newest re-enables following. */
export function livePick(captures, stem) {
    return { selected: stem, follow: stem === newestStem(captures) };
}

/** The capture whose [t_start, t_end] contains p, else null. */
export function captureAt(captures, p) {
    for (const c of captures) if (p >= c.t_start && p <= c.t_end) return c;
    return null;
}

/** REPLAY default: the capture containing the playhead, else the nearest one (distance to its span). */
export function replayAuto(captures, p) {
    if (!captures.length || !Number.isFinite(p)) return captures.length ? newestStem(captures) : null;
    const inside = captureAt(captures, p);
    if (inside) return inside.stem;
    let best = null, bestD = Infinity;
    for (const c of captures) {
        const d = p < c.t_start ? c.t_start - p : p - c.t_end;
        if (d < bestD) { bestD = d; best = c.stem; }
    }
    return best;
}

/**
 * REPLAY: the operator picks `stem` with the playhead at p. The pick holds until the playhead enters a
 * capture other than the picked one and other than the one it was in at the moment of the pick.
 * @returns {{selected:string, hold:{stem:string, at:string|null}}}
 */
export function replayPick(captures, stem, p) {
    const at = captureAt(captures, p);
    return { selected: stem, hold: { stem, at: at ? at.stem : null } };
}

/**
 * REPLAY: the selection for playhead p.
 * @param {{selected:string|null, hold:{stem:string, at:string|null}|null}} prev
 * @returns {{selected:string|null, hold:object|null, changed:boolean}}
 */
export function replayFollow(prev, captures, p) {
    const hold = prev.hold;
    if (hold && has(captures, hold.stem)) {
        const at = captureAt(captures, p);
        const entered = at && at.stem !== hold.stem && at.stem !== hold.at;
        if (!entered) {
            // Leaving the pick-time capture into empty time keeps the hold; it stops being the "at" capture.
            const nextHold = (at === null && hold.at !== null) ? { stem: hold.stem, at: null } : hold;
            return { selected: hold.stem, hold: nextHold, changed: hold.stem !== prev.selected };
        }
    }
    const sel = replayAuto(captures, p);
    return { selected: sel, hold: null, changed: sel !== prev.selected };
}

// ---- The "do not yank" rule ------------------------------------------------------------------------

function sameRange(a, b) {
    if (!a || !b) return false;
    const tol = Math.max(1e-3, 1e-4 * Math.abs(b.max - b.min));
    return Math.abs(a.min - b.min) <= tol && Math.abs(a.max - b.max) <= tol;
}

/**
 * LIVE: may an AUTOMATIC selection change (following a new capture) move the charts?
 *   'jump'  — the charts scroll live, or they still show exactly the range this module last applied
 *             (the operator has not touched the view since);
 *   'offer' — the operator paused, zoomed, panned or picked a preset onto some other range: leave the view
 *             alone, change only the dropdown and offer a "jump to capture" button.
 * A HAND pick always jumps and never asks this.
 * @param {{paused:boolean, manual:boolean, min:number, max:number}} view  getSharedViewState()
 * @param {{min:number, max:number}|null} lastApplied
 */
export function autoJumpDecision(view, lastApplied) {
    if (!view) return 'offer';
    if (!view.paused && !view.manual) return 'jump';
    return sameRange(view, lastApplied) ? 'jump' : 'offer';
}

/** Whether the selected capture's span overlaps the visible range (the jump button is offered when not). */
// ---- IMU row height (the divider between the IMU row and the motor grid) ---------------------------

export const IMU_HEIGHT_KEY = 'jugglebot-imu-chart-height';
export const IMU_HEIGHT_DEFAULT = 200;
export const IMU_HEIGHT_MIN = 100;
export const IMU_HEIGHT_MAX = 500;
/** The 3D view keeps at least this many px when the panel grows. */
export const VIEWER_MIN_PX = 150;

/** A wanted IMU row height clamped to [MIN, MAX] whole px; junk (null, NaN, text) is the default. */
export function clampImuHeight(h) {
    const n = (h === null || h === undefined || h === '') ? NaN : Math.round(Number(h));
    if (!Number.isFinite(n)) return IMU_HEIGHT_DEFAULT;
    return Math.max(IMU_HEIGHT_MIN, Math.min(IMU_HEIGHT_MAX, n));
}

/**
 * The IMU row height to apply: the wanted one, shrunk (IMU row first, never below MIN) so the 3D view
 * (`viewerH` px now, with the row currently `current` px) keeps VIEWER_MIN_PX.  The panel is anchored at the
 * bottom, so every px the IMU row gains is a px the 3D view loses.
 */
export function effectiveImuHeight(wanted, current, viewerH) {
    const w = clampImuHeight(wanted);
    if (!Number.isFinite(viewerH) || !Number.isFinite(current)) return w;
    return Math.max(IMU_HEIGHT_MIN, Math.min(w, Math.round(current + viewerH - VIEWER_MIN_PX)));
}

export function captureInView(c, view) {
    return !!(c && view && c.t_end >= view.min && c.t_start <= view.max);
}

// ---- platform_imu/status -----------------------------------------------------------------------------

/** {busy, sig} from a DiagnosticStatus: sig changes whenever a finished upload changes the last_* keys. */
export function statusDigest(msg) {
    const kv = {};
    for (const e of (msg && msg.values) || []) kv[e.key] = e.value;
    const sigKeys = Object.keys(kv).filter((k) => k.indexOf('last_') === 0 || k.indexOf('capture') >= 0).sort();
    return { busy: kv.busy === '1', sig: sigKeys.map((k) => k + '=' + kv[k]).join('|') };
}

/** True when a status message means a new capture may have landed: busy fell, or the last_* keys moved. */
export function statusMeansNewCapture(prev, next) {
    if (!prev) return false;
    if (prev.busy && !next.busy) return true;
    return prev.sig !== next.sig;
}
