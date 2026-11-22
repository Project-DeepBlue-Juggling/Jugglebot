// imu_charts_harness.js — the REAL ros_ws/gui/js/imu-charts-core.js, imu-charts.js and the external-chart
// hook of telemetry-charts.js under node. Sandbox built by tests/ros/test_gui_imu_charts.py (verbatim copies
// + stub stewart-model.js / ball-butler-model.js). Usage: node imu_charts_harness.js <capture.bin>
// (a fixture written by the test with struct, per the spec's byte layout). Prints one JSON object.
import fs from 'fs';

// ---- minimal DOM / uPlot fakes (as replay_chart_harness.js) ----
function fakeEl(name) {
  const el = {
    _name: name, style: { setProperty() {}, removeProperty() {} }, children: [],
    classList: { _s: new Set(), add(c) { this._s.add(c); }, remove(c) { this._s.delete(c); },
                 toggle(c, on) { if (on === undefined ? !this._s.has(c) : on) this._s.add(c); else this._s.delete(c); },
                 contains(c) { return this._s.has(c); } },
    clientWidth: 800, clientHeight: 300, offsetWidth: 800, offsetHeight: 16, textContent: '', dataset: {}, options: [],
    value: '10', title: '', hidden: false, disabled: false, innerHTML: '',
    appendChild(c) { el.children.push(c); return c; }, insertBefore(c) { el.children.push(c); return c; },
    remove() {}, addEventListener(ev, fn) { (el._h = el._h || {})[ev] = fn; }, removeEventListener() {},
    querySelector() { return fakeEl('q'); }, querySelectorAll() { return []; }, setAttribute() {},
    getBoundingClientRect() { return { left: 0, top: 0, width: 800, height: 300 }; },
    replaceChildren() { el.children = []; }, append() {}, contains() { return false; }, blur() {},
    get firstChild() { return null; }, get parentNode() { return null; }, get nextSibling() { return null; },
  };
  return el;
}
const els = {};
globalThis.document = {
  getElementById(id) { return els[id] || (els[id] = fakeEl(id)); }, createElement(t) { return fakeEl(t); },
  addEventListener() {}, body: fakeEl('body'), documentElement: fakeEl('html'),
  querySelector() { return null; }, querySelectorAll() { return []; },
};
globalThis.window = { addEventListener() {}, devicePixelRatio: 1 };
globalThis.devicePixelRatio = 1;
const rafQ = [];
globalThis.requestAnimationFrame = (fn) => { rafQ.push(fn); return rafQ.length; };
function flushRaf() { for (let n = 0; n < 6 && rafQ.length; n++) rafQ.splice(0).forEach((f) => f()); }
globalThis.getComputedStyle = () => ({ rowGap: '0', getPropertyValue() { return ''; } });
globalThis.ResizeObserver = class { observe() {} };
const ls = {};
globalThis.localStorage = { getItem: (k) => (k in ls ? ls[k] : null), setItem: (k, v) => { ls[k] = String(v); } };
const intervals = [];
globalThis.setInterval = (fn, ms) => { intervals.push({ fn, ms }); return intervals.length; };
globalThis.clearInterval = () => {};
console.error = () => {};
const made = [];
globalThis.uPlot = class {
  constructor(o, d, c) {
    this.opts = o; this.data = d; this.scales = { x: { min: null, max: null } };
    this.over = fakeEl('over'); this.sets = []; this.destroyed = false; this.container = c;
    this.select = { left: 0, top: 0, width: 0, height: 0 }; this.cursor = { left: -10, idx: null };
    this.hooks = o.hooks; this.bbox = { left: 0, top: 0, width: 800, height: 200 };
    this.series = o.series.map((s) => ({ ...s }));
    this.ctx = { save() {}, restore() {}, beginPath() {}, moveTo() {}, lineTo() {}, closePath() {}, rect() {}, clip() {},
                 fill() {}, stroke() {}, fillRect() {}, fillText() {}, arc() {} };
    made.push(this);
  }
  setScale(k, r) { this.sets.push([k, r]); this.scales[k] = { min: r.min, max: r.max }; }
  setData(d) { this.data = d; }
  setSeries(i, o) { Object.assign(this.series[i], o); }
  setSize() {} destroy() { this.destroyed = true; } setSelect() {} redraw() {}
  posToVal(px) { const s = this.scales.x; return s.min + (px / 800) * (s.max - s.min); }
  valToPos(v, k) { if (k !== 'x') return 50; const s = this.scales.x; return (v - s.min) / (s.max - s.min) * 800; }
};
globalThis.uPlot.fmtDate = () => () => ''; globalThis.uPlot.tzDate = () => 0;

// fetch fake: recordings listing, capture listing, capture data
const fetches = [];
let captureListing = { captures: [] };
let captureBin = null;
let listStatus = 200;
globalThis.fetch = (url) => {
  fetches.push(String(url));
  const ok = (body) => Promise.resolve({ ok: true, status: 200, json: async () => body, arrayBuffer: async () => body });
  if (url.startsWith('/api/replay/recordings')) return ok({ recordings: [{ id: '2026-11-20_20-00-00', start_ns: 1795162000 * 1e9 }] });
  if (url.startsWith('/api/imu/captures?')) {
    if (listStatus !== 200) return Promise.resolve({ ok: false, status: listStatus, json: async () => ({}) });
    return ok(captureListing);
  }
  if (url.split('?')[0].endsWith('/data')) return ok(captureBin);
  return Promise.resolve({ ok: false, status: 404, json: async () => ({}) });
};
const settle = async () => { for (let k = 0; k < 20; k++) { await Promise.resolve(); flushRaf(); } };

const core = await import('./imu-charts-core.js');
const tc = await import('./telemetry-charts.js');
const out = {};
const near = (a, b, tol = 1e-9) => Math.abs(a - b) <= tol;

// =============================== core: binary parse ===============================
const binBytes = fs.readFileSync(process.argv[2]);
const bin = binBytes.buffer.slice(binBytes.byteOffset, binBytes.byteOffset + binBytes.byteLength);
captureBin = bin;
{
  const cap = core.parseImuCapture(bin);
  out.parse = { n: cap.n, t: Array.from(cap.t), accel: Array.from(cap.accel), gyro: Array.from(cap.gyro), valid: Array.from(cap.valid) };
  const bad = new Uint8Array(bin.slice(0)); bad[0] = 0x58;
  const err = (f) => { try { f(); return null; } catch (e) { return e.message; } };
  out.parse_bad_magic = err(() => core.parseImuCapture(bad.buffer));
  out.parse_truncated = err(() => core.parseImuCapture(bin.slice(0, bin.byteLength - 1)));
  out.parse_not_binary = err(() => core.parseImuCapture('x'));
  const cols = core.buildPlotColumns(cap);
  out.columns = { t: Array.from(cols.t), ax: Array.from(cols.ax).map((v) => (Number.isFinite(v) ? v : null)),
                  amag: Array.from(cols.amag).map((v) => (Number.isFinite(v) ? v : null)), nGaps: cols.nGaps };
}

// =============================== core: decimation ===============================
{
  const n = 32000, t = new Float64Array(n), y = new Float64Array(n), z = new Float64Array(n);
  for (let i = 0; i < n; i++) {
    t[i] = 1000 + i / 1000;
    y[i] = Math.sin(i * 0.37) * (1 + (i % 97) / 50) + ((i % 1009) === 0 ? 25 : 0);   // spikes the min/max must keep
    z[i] = (i >= 15000 && i < 15040) ? NaN : Math.cos(i * 0.05);                        // a 40-sample gap
  }
  const W = 400, xMin = 1002, xMax = 1030;
  const d = core.decimateMinMax(t, [y, z], xMin, xMax, W);
  const [dt, dy, dz] = d.data;
  // per pixel column: raw min/max of y over the in-view samples == min/max of the output points in that column
  const k = W / (xMax - xMin);
  const colOf = (tt) => Math.min(W - 1, Math.floor((tt - xMin) * k));
  const rawMin = new Array(W).fill(Infinity), rawMax = new Array(W).fill(-Infinity);
  for (let i = 0; i < n; i++) if (t[i] >= xMin && t[i] <= xMax) { const c = colOf(t[i]); rawMin[c] = Math.min(rawMin[c], y[i]); rawMax[c] = Math.max(rawMax[c], y[i]); }
  const outMin = new Array(W).fill(Infinity), outMax = new Array(W).fill(-Infinity);
  for (let i = 0; i < dt.length; i++) if (dt[i] >= xMin && dt[i] <= xMax) { const c = colOf(dt[i]); outMin[c] = Math.min(outMin[c], dy[i]); outMax[c] = Math.max(outMax[c], dy[i]); }
  let colsOk = 0, colsBad = 0;
  for (let c = 0; c < W; c++) { if (rawMin[c] === outMin[c] && rawMax[c] === outMax[c]) colsOk++; else colsBad++; }
  let asc = true; for (let i = 1; i < dt.length; i++) if (!(dt[i] >= dt[i - 1])) asc = false;
  let gapNaN = 0; for (let i = 0; i < dt.length; i++) if (dt[i] >= t[15000] - 0.002 && dt[i] <= t[15039] + 0.002 && Number.isNaN(dz[i])) gapNaN++;
  out.decimate = { decimated: d.decimated, points: dt.length, W, colsOk, colsBad, ascending: asc, gapNaN,
                   edgeBefore: dt[0] < xMin, edgeAfter: dt[dt.length - 1] > xMax };
  const f = core.decimateMinMax(t, [y, z], 1002, 1002.5, W);          // 500 samples <= 4 x 400: full resolution
  out.full = { decimated: f.decimated, len: f.data[0].length, zeroCopy: f.data[0].buffer === t.buffer, first: f.data[0][0], last: f.data[0][f.data[0].length - 1] };
}

// =============================== core: derivative ===============================
{
  const rate = 1000, n = 3000, A = 100, fq = 2, t = new Float64Array(n), g = new Float64Array(n);
  for (let i = 0; i < n; i++) { t[i] = 5000 + i / rate; g[i] = A * Math.sin(2 * Math.PI * fq * (t[i] - 5000)); }
  const a = core.angularAcceleration(t, g);
  let maxErr = 0, finite = 0;
  for (let i = 20; i < n - 20; i++) {
    const want = 2 * Math.PI * fq * A * Math.cos(2 * Math.PI * fq * (t[i] - 5000));
    maxErr = Math.max(maxErr, Math.abs(a[i] - want)); if (Number.isFinite(a[i])) finite++;
  }
  out.deriv = { maxErr, peak: 2 * Math.PI * fq * A, finite, interior: n - 40, edgeNaN: Number.isNaN(a[0]) && Number.isNaN(a[n - 1]) };
  const g2 = g.slice(); for (let i = 1500; i < 1510; i++) g2[i] = NaN;
  const a2 = core.angularAcceleration(t, g2);
  let nanNear = 0, finiteFar = 0;
  for (let i = 1494; i < 1516; i++) if (Number.isNaN(a2[i])) nanNear++;
  for (let i = 1400; i < 1480; i++) if (Number.isFinite(a2[i])) finiteFar++;
  out.deriv_gap = { nanNear, finiteFar };
  // a time hole (no NaN, 50 ms missing) is not differenced across either
  const t3 = Float64Array.from(t, (v, i) => (i >= 2000 ? v + 0.05 : v));
  const a3 = core.angularAcceleration(t3, g);
  out.deriv_hole = { nanAtHole: Number.isNaN(a3[2000]) && Number.isNaN(a3[1999]) };
  out.ang_title = core.ANG_ACCEL_TITLE;
}


// =============================== core: low-pass filter ===============================
{
  const mk = (fs, n, f) => { const t = new Float64Array(n), x = new Float64Array(n);
    for (let i = 0; i < n; i++) { t[i] = 7000 + i / fs; x[i] = Math.sin(2 * Math.PI * f * i / fs); } return { t, x }; };
  const gain = (fs, f, cutoff) => {
    const n = 6000, { t, x } = mk(fs, n, f), o = core.lowpassColumn(t, x, cutoff);
    let num = 0, den = 0; const lo = Math.floor(n * 0.3), hi = Math.floor(n * 0.7);
    for (let i = lo; i < hi; i++) { num += o[i] * x[i]; den += x[i] * x[i]; }
    // zero phase: the in-phase projection carries the whole output (the quadrature part is ~0)
    let q = 0; for (let i = lo; i < hi; i++) q += o[i] * Math.cos(2 * Math.PI * f * i / fs);
    return { g: num / den, quad: Math.abs(q) / den };
  };
  out.filt_gain_1k = [5, 15, 30, 100].map((f) => gain(1000, f, 15));
  out.filt_gain_2k = [5, 15, 30, 100].map((f) => gain(2000, f, 15));   // same cutoff, other rate: same response
  out.filt_gain_100 = [20, 100, 300].map((f) => gain(1000, f, 100));
  out.filt_fs_from_data = core.sampleRate(mk(994, 2000, 3).t);
  { // a pulse keeps its centre sample (no time shift) and the output is symmetric about it
    const n = 2001, t = new Float64Array(n), y = new Float64Array(n);
    for (let i = 0; i < n; i++) { t[i] = i / 1000; y[i] = i === 1000 ? 1 : 0; }
    const o = core.lowpassColumn(t, y, 15);
    let imax = 0; for (let i = 0; i < n; i++) if (o[i] > o[imax]) imax = i;
    let asym = 0; for (let k = 1; k < 200; k++) asym = Math.max(asym, Math.abs(o[1000 - k] - o[1000 + k]));
    out.filt_pulse = { imax, asym, peak: o[1000] };
  }
  { // never across a gap: a +100 step after a 10-sample gap does not leak into the samples before it
    const n = 3000, t = Float64Array.from({ length: n }, (_, i) => i / 1000), y = new Float64Array(n);
    for (let i = 0; i < n; i++) y[i] = i >= 1510 ? 100 : 0;
    for (let i = 1500; i < 1510; i++) y[i] = NaN;
    const o = core.lowpassColumn(t, y, 15);
    let before = 0, after = 0, inGap = 0;
    for (let i = 0; i < 1500; i++) before = Math.max(before, Math.abs(o[i]));
    for (let i = 1510; i < n; i++) after = Math.max(after, Math.abs(o[i] - 100));
    for (let i = 1500; i < 1510; i++) if (Number.isNaN(o[i])) inGap++;
    out.filt_gap = { before, after, inGap };
  }
  { // a run shorter than FILTER_MIN_RUN becomes a gap; a time hole (no NaN) also splits the run
    const n = 200, t = Float64Array.from({ length: n }, (_, i) => i / 1000), y = new Float64Array(n).fill(1);
    for (let i = 0; i < n; i++) if (i < 50 || (i >= 60 && i < 70) || i >= 100) y[i] = i < 50 ? 1 : (i >= 100 ? 1 : 1);
    for (const i of [50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79]) y[i] = NaN;
    // short run: 60..69 (10 samples) between NaNs
    const o = core.lowpassColumn(t, y, 15);
    out.filt_short = { shortNaN: Array.from(o.slice(60, 70)).every(Number.isNaN), longFinite: Number.isFinite(o[10]) && Number.isFinite(o[150]), minRun: core.FILTER_MIN_RUN };
    const th = Float64Array.from({ length: n }, (_, i) => i / 1000 + (i >= 100 ? 0.05 : 0));
    const yy = new Float64Array(n).fill(2);
    const oh = core.lowpassColumn(th, yy, 15);
    out.filt_hole = { finite: Array.from(oh).every(Number.isFinite), split: Math.abs(oh[99] - 2) < 1e-6 && Math.abs(oh[100] - 2) < 1e-6 };
  }
  { // off / cutoff too close to Nyquist / unusable rate: the raw array itself
    const t = Float64Array.from({ length: 100 }, (_, i) => i / 1000), y = new Float64Array(100).fill(3);
    out.filt_raw = [core.lowpassColumn(t, y, 0) === y, core.lowpassColumn(t, y, 500) === y, core.lowpassColumn(new Float64Array(1), new Float64Array(1), 15) !== undefined,
                    core.parseCutoff('15'), core.parseCutoff('77'), core.parseCutoff(null), core.filterBadge(15), core.filterBadge(0)];
  }
  { // |a| from the filtered components; the raw columns object is not modified
    const n = 400, t = Float64Array.from({ length: n }, (_, i) => i / 1000);
    const cols = { t, nGaps: 0 }; for (const k of ['ax', 'ay', 'az', 'gx', 'gy', 'gz', 'amag']) cols[k] = new Float64Array(n);
    for (let i = 0; i < n; i++) { cols.ax[i] = 3; cols.ay[i] = 4; cols.az[i] = i === 200 ? 50 : 0; cols.amag[i] = Math.hypot(cols.ax[i], cols.ay[i], cols.az[i]); }
    const f = core.lowpassColumns(cols, 15);
    out.filt_cols = { off: core.lowpassColumns(cols, 0) === cols, rawSpike: cols.amag[200], amag100: f.amag[100], want: Math.hypot(f.ax[100], f.ay[100], f.az[100]), spikeCut: f.az[200] < 10, rawKept: cols.az[200] };
  }
}

// =============================== core: FFT, Welch, peaks ===============================
{
  const sines = (fs, n, parts) => { const t = new Float64Array(n), y = new Float64Array(n);
    for (let i = 0; i < n; i++) { t[i] = 9000 + i / fs; y[i] = parts.reduce((a, [A, f]) => a + A * Math.sin(2 * Math.PI * f * i / fs + 0.3), 0); } return { t, y }; };
  const power = (w, a) => { let P = 0; for (let k = 0; k < a.length; k++) P += a[k] * a[k] * w.df; return P; };
  const run = (fs, f, A) => { const { t, y } = sines(fs, 12000, [[A, f]]); const w = core.welchSpectra(t, [y], t[0], t[t.length - 1]);
    const p = core.spectrumPeak(w.freqs, w.asd[0], 1, fs / 2); return { df: w.df, seg: w.seg, f: p.f, err: Math.abs(p.f - f), bin: Math.abs(p.f - f) / w.df, powerRatio: power(w, w.asd[0]) / (A * A / 2), nSeg: w.nSeg[0], fs: w.fs }; };
  out.spec_sine = { s994: run(994, 5.0, 2.0), g994: run(994, 78.3, 0.7), s2k: run(2000, 5.0, 2.0), s2k_hi: run(2000, 333.3, 1.0) };
  { // fft of a bin-exact cosine: energy in bins k and N-k only
    const N = 64, re = Float64Array.from({ length: N }, (_, i) => Math.cos(2 * Math.PI * 5 * i / N)), im = new Float64Array(N);
    core.fft(re, im);
    out.fft_bins = { k5: re[5], kN5: re[N - 5], other: Math.max(...Array.from(re).map((v, i) => (i === 5 || i === N - 5 ? 0 : Math.abs(v))), ...Array.from(im).map(Math.abs)) };
    out.fft_bad = (() => { try { core.fft(new Float64Array(6), new Float64Array(6)); return null; } catch (e) { return e.message; } })();
  }
  { // white noise: ASD = sigma * sqrt(2 / fs) across the band (deterministic LCG, Box-Muller)
    let st = 12345; const rnd = () => { st = (st * 1664525 + 1013904223) >>> 0; return (st + 0.5) / 4294967296; };
    const fs = 1000, n = 40000, t = new Float64Array(n), y = new Float64Array(n);
    for (let i = 0; i < n; i++) { t[i] = 9000 + i / fs; y[i] = 1.7 * Math.sqrt(-2 * Math.log(rnd())) * Math.cos(2 * Math.PI * rnd()); }
    const w = core.welchSpectra(t, [y], t[0], t[n - 1]); const a = w.asd[0];
    let m = 0, c = 0; for (let k = 0; k < a.length; k++) if (w.freqs[k] > 20 && w.freqs[k] < 480) { m += a[k] * a[k]; c++; }
    out.spec_noise = { rms: Math.sqrt(m / c), want: 1.7 * Math.sqrt(2 / fs), nSeg: w.nSeg[0], seg: w.seg };
  }
  { // range selection: only samples inside [xMin, xMax]; a sine that stops at t = 9010 is invisible after it
    const fs = 1000, n = 30000, t = new Float64Array(n), y = new Float64Array(n);
    for (let i = 0; i < n; i++) { t[i] = 9000 + i / fs; y[i] = (i < 10000 ? Math.sin(2 * Math.PI * 12.5 * i / fs) : 0) + Math.sin(2 * Math.PI * 40 * i / fs) * (i < 10000 ? 0 : 1); }
    const a = core.welchSpectra(t, [y], 9000, 9009.9), b = core.welchSpectra(t, [y], 9015, 9025);
    out.spec_range = { a: core.spectrumPeak(a.freqs, a.asd[0], 1, 100).f, b: core.spectrumPeak(b.freqs, b.asd[0], 1, 100).f,
                       idx: core.rangeIndices(t, 9001, 9001.0049), idxEmpty: core.rangeIndices(t, 5, 6), idxAll: core.rangeIndices(t, 0, 1e9) };
  }
  { // segment length: largest power of two up to 4096 that fits; a short window is one (or two) segments; < 256 -> message
    const fs = 1000, mk = (n) => { const t = Float64Array.from({ length: n }, (_, i) => i / fs), y = Float64Array.from({ length: n }, (_, i) => Math.sin(i)); return { t, y }; };
    const segFor = (n) => { const { t, y } = mk(n); const w = core.welchSpectra(t, [y], -1, 1e9); return w.error ? w.error : [w.seg, w.nSeg[0]]; };
    out.spec_seg = { n255: segFor(255), n256: segFor(256), n300: segFor(300), n5000: segFor(5000), n20000: segFor(20000) };
  }
  { // never across a gap: two 600-sample runs split by NaN -> segments tile each run (L = 512: 1 segment of 512 + 1 hop-shifted? 600 -> starts 0 only), a 10-sample gap
    const fs = 1000, n = 1210, t = Float64Array.from({ length: n }, (_, i) => i / fs), y = Float64Array.from({ length: n }, (_, i) => Math.sin(i * 0.2));
    for (let i = 600; i < 610; i++) y[i] = NaN;
    const w = core.welchSpectra(t, [y], -1, 1e9);
    out.spec_gap = { seg: w.seg, nSeg: w.nSeg[0], finite: Array.from(w.asd[0]).every(Number.isFinite) };
    const short = Float64Array.from({ length: 1210 }, (_, i) => (i % 100 < 50 ? 1 : NaN));
    out.spec_fragmented = core.welchSpectra(t, [short], -1, 1e9).error || null;
  }
  { // the peak search starts at 1 Hz: a bigger 0.4 Hz component is not "the peak"; parabola refines a between-bin tone
    const fs = 1000, n = 20000, t = new Float64Array(n), y = new Float64Array(n);
    for (let i = 0; i < n; i++) { t[i] = i / fs; y[i] = 5 * Math.sin(2 * Math.PI * 0.4 * i / fs) + Math.sin(2 * Math.PI * 7.37 * i / fs); }
    const w = core.welchSpectra(t, [y], 0, 1e9);
    const p = core.spectrumPeak(w.freqs, w.asd[0], 1, 50), full = core.spectrumPeak(w.freqs, w.asd[0], 0, 50);
    out.spec_peak = { f: p.f, df: w.df, fullF: full ? full.f : null, none: core.spectrumPeak(w.freqs, w.asd[0], 400, 401), xmax: [core.specXMax(10, 994), core.specXMax(50, 994), core.specXMax(0, 994), core.specXMax(50, 60)],
                      parse: [core.parseSpecMax('10'), core.parseSpecMax('0'), core.parseSpecMax('7'), core.parseSpecMax(null)] };
  }
}

// =============================== core: lookup, labels, launch window ===============================
{
  const ts = Float64Array.from([10, 10.001, 10.002, 10.003]);
  out.nearest = [core.nearestIndex(ts, 10.0016, 0.002), core.nearestIndex(ts, 9.0, 0.002), core.nearestIndex(ts, 10.0045, 0.002), core.nearestIndex(ts, 10.0055, 0.002)];
  out.label = core.captureLabel({ t_start: 1795168214.0, t_end: 1795168246.4, label: 'columns attempt 3', complete: false });
  out.label_complete = core.captureLabel({ t_start: 1795168214.0, t_end: 1795168246.4, label: '', complete: true });
  out.launch = core.liveLaunchWindow({ recordings: [{ id: '2026-11-20_22-53-34', start_ns: 1795175619424748297 }, { id: '2026-11-20_22-49-44', start_ns: 1795175387633562873 }] }, 1795180000);
  out.launch_inprogress = core.liveLaunchWindow({ recordings: [{ id: '2026-11-20_12-30-00', start_ns: null }, { id: '2026-11-20_22-49-44', start_ns: 1795175387633562873 }] }, 1795180000);
  out.launch_none = core.liveLaunchWindow(null, 1795180000);
  out.list_msgs = [core.listErrorMessage(404), core.listErrorMessage(0), core.listErrorMessage(500)];
  const st = (busy, label) => ({ values: [{ key: 'busy', value: busy }, { key: 'last_label', value: label }, { key: 'phase', value: 'x' }] });
  const d0 = core.statusDigest(st('1', 'a')), d1 = core.statusDigest(st('0', 'a')), d2 = core.statusDigest(st('0', 'b')), d3 = core.statusDigest(st('0', 'b'));
  out.status = [core.statusMeansNewCapture(null, d0), core.statusMeansNewCapture(d0, d1), core.statusMeansNewCapture(d1, d2), core.statusMeansNewCapture(d2, d3)];
}

// =============================== core: selection / following / yank ===============================
{
  const C = (stem, a, b) => ({ stem, t_start: a, t_end: b });
  const A = C('A', 100, 130), B = C('B', 200, 230), D = C('D', 300, 330), E = C('E', 400, 430);
  const L = [];
  let s = { selected: null, follow: true };
  const step = (caps) => { const r = core.liveAfterRefresh(s, caps); s = { selected: r.selected, follow: r.follow }; L.push([r.selected, r.follow, r.changed]); };
  step([A, B]);                                     // newest, following
  step([A, B, D]);                                  // a new capture: moves
  s = core.livePick([A, B, D], 'A'); L.push([s.selected, s.follow, 'pick']);
  step([A, B, D, E]);                               // hand-picked older: held
  s = core.livePick([A, B, D, E], 'E'); L.push([s.selected, s.follow, 'pick']);
  step([A, B, D, E, C('F', 500, 530)]);             // following again: moves
  s = { selected: 'Z', follow: false }; step([A, B]);   // a held capture that vanished -> newest, following
  out.live = L;

  const caps = [A, B, D];
  const R = [];
  R.push(core.replayAuto(caps, 110), core.replayAuto(caps, 160), core.replayAuto(caps, 190), core.replayAuto(caps, 1000), core.replayAuto([], 5));
  let rs = { selected: core.replayAuto(caps, 110), hold: null };
  const rstep = (p) => { const r = core.replayFollow(rs, caps, p); rs = { selected: r.selected, hold: r.hold }; R.push(r.selected); };
  const pk = core.replayPick(caps, 'D', 115); rs = { selected: pk.selected, hold: pk.hold }; R.push(JSON.stringify(pk.hold));
  rstep(120);          // still inside A (the pick-time capture): hold
  rstep(160);          // empty time: hold
  rstep(210);          // enters B: hold released -> B
  rstep(320);          // enters D -> D
  const pk2 = core.replayPick(caps, 'A', 320); rs = { selected: pk2.selected, hold: pk2.hold };
  rstep(260);          // leaves D into empty: still A
  rstep(310);          // re-enters D after leaving it: released -> D
  out.replay = R;

  const v = (paused, manual, min, max) => ({ paused, manual, min, max });
  out.yank = [
    core.autoJumpDecision(v(false, false, 0, 10), null),
    core.autoJumpDecision(v(true, true, 200, 230), { min: 200, max: 230 }),
    core.autoJumpDecision(v(true, true, 205, 215), { min: 200, max: 230 }),
    core.autoJumpDecision(v(true, false, 90, 100), null),
    core.autoJumpDecision(v(true, true, 200, 230), null),
  ];
  out.in_view = [core.captureInView(B, { min: 220, max: 240 }), core.captureInView(B, { min: 231, max: 240 })];
}

// =============================== telemetry-charts external hook ===============================
tc.initTelemetryCharts();
flushRaf();
const odrive = made.filter((u) => !u.destroyed);
out.hook_n_odrive = odrive.length;
const got = [];
const unsub = tc.onSharedXRange((a, b) => got.push([a, b]));
out.hook_sync_key = tc.SYNC_KEY;
out.hook_show = tc.showRangeOnAllCharts(100, 132);
out.hook_after_show = { got: got.slice(-1)[0], view: tc.getSharedViewState(), odriveX: odrive.map((u) => u.scales.x).every((x) => x.min === 100 && x.max === 132) };
tc.selectRangeOnAllCharts(110, 120);
out.hook_after_select = { got: got.slice(-1)[0], odriveX: odrive.every((u) => u.scales.x.min === 110 && u.scales.x.max === 120) };
// an external chart's wheel (cursor at mid-plot) drives every chart through the shared handlers
const ext = new globalThis.uPlot({ series: [{}], hooks: {} }, [[]], fakeEl('ext'));
ext.scales.x = { min: 110, max: 120 };
tc.attachZoomPanControls(ext);
ext.over._h.wheel({ preventDefault() {}, deltaY: 100, clientX: 400 });
out.hook_after_wheel = { got: got.slice(-1)[0], odrive: [odrive[0].scales.x.min, odrive[0].scales.x.max] };
ext.over._h.dblclick({ preventDefault() {} });
out.hook_after_reset = { got: got.slice(-1)[0], view: tc.getSharedViewState() };
out.hook_listener_count_before_unsub = got.length;
unsub();
tc.selectRangeOnAllCharts(50, 60);
out.hook_after_unsub = got.length;
// replay: the playhead broadcast; the capture jump is a seek, never a pause
const seeks = [];
tc.onSharedXRange((a, b) => got.push([a, b]));
tc.enterReplayCharts({ axes: [], onRebuild() { return () => {}; } }, { onSeek: (t) => seeks.push(t) });
tc.setReplayPlayhead(500);
out.replay_hook = { got: got.slice(-1)[0], show: tc.showRangeOnAllCharts(1, 2), seek: tc.seekReplayTo(600), seeks, last: got.slice(-1)[0],
                    view: tc.getSharedViewState() };
tc.exitReplayCharts();
out.seek_live = tc.seekReplayTo(1);

// =============================== imu-charts.js glue ===============================
// Hidden by default: nothing fetched; showing it fetches, follows the newest capture and jumps the view.
captureListing = { captures: [
  { stem: '20261120T095142Z_C33B', t_start: 1795168214.0, t_end: 1795168246.0, label: 'one', complete: true, wall_ok: true },
  { stem: '20261120T115806Z_AEDE', t_start: 1795175854.0, t_end: 1795175886.0, label: 'two', complete: true, wall_ok: true },
] };
const imu = await import('./imu-charts.js');
// return the ODrive view to live scrolling the way an operator can: wheel-zoom out past the live edge
const nowS = Date.now() / 1000;
ext.scales.x = { min: nowS - 10, max: nowS + 5 };
ext.over._h.wheel({ preventDefault() {}, deltaY: 100, clientX: 400 });
out.glue_live_before = tc.getSharedViewState();
imu.initImuCharts();
await settle();
out.glue_hidden = { fetches: fetches.length, groupHidden: els['imu-chart-group'].hidden, selectHidden: els['imu-capture-select'].hidden,
                    shownKey: ls['jugglebot-imu-charts-shown'] === undefined, intervals: intervals.length };
els['imu-charts-toggle-btn']._h.click();
await settle();
const view = tc.getSharedViewState();
out.glue_shown = { fetches: fetches.slice(), shownKey: ls['jugglebot-imu-charts-shown'], groupHidden: els['imu-chart-group'].hidden,
                   options: els['imu-capture-select'].children.map((o) => o.value), selected: els['imu-capture-select'].value,
                   view, imuCharts: made.filter((u) => !u.destroyed && u.opts.cursor && u.opts.cursor.sync && u.opts.cursor.sync.scales).length };
// the IMU charts drew the capture: x range = the capture, data = its samples
const imuU = made.filter((u) => !u.destroyed && u.opts.cursor && u.opts.cursor.sync && u.opts.cursor.sync.scales);
out.glue_draw = imuU.map((u) => ({ x: [u.scales.x.min, u.scales.x.max], n: u.data[0].length, nSeries: u.data.length - 1, sync: u.opts.cursor.sync.key }));
// series pill off -> setSeries(show false) and persisted
const pill = els['imu-accel-toggles'].children[3];
pill._h.click();
out.glue_pill = { stored: JSON.parse(ls['jugglebot-imu-charts-series']), show: imuU[0].series[4].show };

// Filter selector and Spectrum toggle (glue): persisted, badge on the titles, spectrum charts leave the shared groups
{
  const fsel = els['imu-filter-select'];
  fsel.value = '15'; fsel._h.change(); await settle();
  out.glue_filter = { stored: ls['jugglebot-imu-charts-filter'], badge: els['imu-accel-badge'].textContent, badgeGyro: els['imu-gyro-badge'].textContent };
  fsel.value = '0'; fsel._h.change(); await settle();
  out.glue_filter_off = { stored: ls['jugglebot-imu-charts-filter'], badge: els['imu-accel-badge'].textContent };
  els['imu-spectrum-btn']._h.click(); await settle();
  const live = made.filter((u) => !u.destroyed && u.container && (u.container === els['imu-accel-plot'] || u.container === els['imu-gyro-plot']));
  out.glue_spectrum = { stored: ls['jugglebot-imu-charts-spectrum'], n: live.length,
    sync: live.map((u) => !!(u.opts.cursor && u.opts.cursor.sync)), wheel: live.map((u) => !!(u.over._h && u.over._h.wheel)),
    logY: live.map((u) => u.opts.scales.asd && u.opts.scales.asd.distr), timeX: live.map((u) => u.opts.scales.x.time),
    msg: els['imu-chart-message'].textContent, msgHidden: els['imu-chart-message'].hidden, accelTitle: els['imu-accel-title'].textContent,
    gyroModeHidden: els['imu-gyro-mode-btn'].hidden, maxHidden: els['imu-spec-max-select'].hidden, filterDisabled: fsel.disabled };
  els['imu-spec-max-select'].value = '10'; els['imu-spec-max-select']._h.change();
  out.glue_spec_max = ls['jugglebot-imu-charts-spectrum-max'];
  els['imu-spectrum-btn']._h.click(); await settle();
  const live2 = made.filter((u) => !u.destroyed && u.container && (u.container === els['imu-accel-plot'] || u.container === els['imu-gyro-plot']));
  out.glue_spectrum_off = { stored: ls['jugglebot-imu-charts-spectrum'], sync: live2.map((u) => !!(u.opts.cursor && u.opts.cursor.sync)), wheel: live2.map((u) => !!(u.over._h && u.over._h.wheel)),
    markerHook: live2.map((u) => u.opts.hooks.draw.includes(tc.drawEventMarkers)), accelTitle: els['imu-accel-title'].textContent };
}
// operator zooms elsewhere; a new capture arrives by following -> no yank, jump button offered
tc.selectRangeOnAllCharts(1795175860, 1795175870);
captureListing = { captures: captureListing.captures.concat([{ stem: '20261120T120305Z_AEDF', t_start: 1795176185.0, t_end: 1795176217.0, label: 'three', complete: false, wall_ok: true }]) };
imu.imuChartsOnStatus({ values: [{ key: 'busy', value: '1' }, { key: 'last_label', value: 'two' }] });
imu.imuChartsOnStatus({ values: [{ key: 'busy', value: '0' }, { key: 'last_label', value: 'three' }] });
await settle();
out.glue_no_yank = { selected: els['imu-capture-select'].value, view: tc.getSharedViewState(), jumpHidden: els['imu-jump-btn'].hidden };
els['imu-jump-btn']._h.click();
await settle();
out.glue_jump = { view: tc.getSharedViewState(), jumpHidden: els['imu-jump-btn'].hidden };
// hiding stops everything
const nBefore = fetches.length;
els['imu-charts-toggle-btn']._h.click();
await settle();
imu.imuChartsOnStatus({ values: [{ key: 'busy', value: '0' }, { key: 'last_label', value: 'four' }] });
await settle();
out.glue_hide = { fetchesAfter: fetches.length - nBefore, groupHidden: els['imu-chart-group'].hidden, shownKey: ls['jugglebot-imu-charts-shown'] };

// Panel height model: IMU share clamp / even split, panel bounds, the both-shown default
out.imu_height = {
  clamp: [core.clampImuHeight(50), core.clampImuHeight(9999), core.clampImuHeight(333.4), core.clampImuHeight(null),
          core.clampImuHeight('abc'), core.clampImuHeight('250')],
  share: [core.clampImuShare(300, 600), core.clampImuShare(50, 600), core.clampImuShare(900, 600),
          core.clampImuShare(400, 250), core.clampImuShare(200, 150), core.clampImuShare(200, NaN)],
  even: [core.evenSplit(600), core.evenSplit(501), core.evenSplit(150)],
  both_default: core.bothHeightDefault(300),
  bounds_both: core.panelHeightBounds({ both: true, toolbarH: 38, dividerH: 5, borderH: 1, viewerTop: 40, handleH: 6, windowH: 1000 }),
  bounds_single: core.panelHeightBounds({ both: false, viewerTop: 40, handleH: 6, windowH: 1000 }),
  bounds_tiny_window: core.panelHeightBounds({ both: true, toolbarH: 38, dividerH: 5, borderH: 1, viewerTop: 40, handleH: 6, windowH: 300 }),
  clamp_panel: [core.clampPanelHeight(10, { min: 244, max: 804 }), core.clampPanelHeight(5000, { min: 244, max: 804 }),
                core.clampPanelHeight(500.4, { min: 244, max: 804 }), String(core.clampPanelHeight('x', { min: 1, max: 2 }))],
  // stored height re-clamped on restore / window resize (stored kept, applied clamped); below the stored floor or junk: NaN
  applied_stored: [core.appliedStoredHeight(1400, 150, { min: 150, max: 700 }), core.appliedStoredHeight(400, 150, { min: 150, max: 700 }),
                   core.appliedStoredHeight(400, 150, { min: 150, max: 300 }), String(core.appliedStoredHeight(100, 150, { min: 150, max: 700 })),
                   String(core.appliedStoredHeight(NaN, 150, { min: 150, max: 700 }))],
};
// CSV merge: time-sorted union of motor and IMU times, equal times share a row, NaN (gap) rows and out-of-range dropped
{
  const nan = NaN;
  const imu = { t: Float64Array.from([9.0, 10.0, 10.5, 11.0, 11.25, 12.0, 14.0]),
                ax: Float64Array.from([1, 1, 1, nan, 1, 1, 1]), ay: Float64Array.from([2, 2, 2, nan, 2, 2, 2]),
                az: Float64Array.from([3, 3, 3, nan, 3, 3, 3]), gx: Float64Array.from([4, 4, 4, nan, 4, 4, 4]),
                gy: Float64Array.from([5, 5, 5, nan, 5, 5, 5]), gz: Float64Array.from([6, 6, 6, nan, 6, 6, 6]) };
  const motor = Float64Array.from([10, 11, 12, 13, 0, 0]);   // capacity buffer: only 4 rows are real
  out.csv_merge = {
    header: core.IMU_CSV_HEADER,
    rows: core.mergeCsvRows(motor, 4, imu).map((r) => [r.t, r.k, r.j]),
    no_imu: core.mergeCsvRows(motor, 4, null).map((r) => [r.t, r.k, r.j]),
    no_motor: core.mergeCsvRows(motor, 0, imu).length,
  };
}
out.frames = {
  q_platform: core.frameQuery('platform'), q_imu: core.frameQuery('imu'), q_unknown: core.frameQuery(undefined),
  frame_row: [core.captureFrame({ frame: 'platform' }), core.captureFrame({ frame: 'imu' }), core.captureFrame({}), core.captureFrame(null)],
  label_p: core.frameSeriesLabel('x', 'platform'), label_i: core.frameSeriesLabel('x', 'imu'),
  note_p: core.frameNote('platform', '2026-11-22'), note_i: core.frameNote('imu'),
  csv_p: core.csvHeader('platform'), csv_i: core.csvHeader('imu'),
};
// REPLAY playhead on the IMU charts: the draw hook is the motor charts' own
out.imu_playhead_hook = made.filter((u) => u.opts.hooks && u.opts.hooks.draw && u.opts.hooks.draw.includes(tc.drawReplayPlayhead)).length;

console.log(JSON.stringify(out));
