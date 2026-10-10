/**
 * Replay chart store (replay design § 5).
 *
 * An immutable, chunk-fed stand-in for the live ring buffers of
 * telemetry-charts.js.  Columns come from decoded chunks through the SAME pure
 * `telemetrySample()` the live path uses, so a replayed record plots exactly
 * what the live GUI would have plotted for it.
 *
 *   - Per chunk, per axis (0..8): own timestamps + one Float64Array per signal
 *     key, derived ONCE and cached (WeakMap keyed by the chunk object; the
 *     cache is dropped by invalidate(), e.g. on a unit toggle).  Axes keep
 *     their own timestamps: a robot_state with fewer than 9 motors feeds only
 *     its first axes, exactly like the live push loop.
 *   - Joins (the live handler latches, made explicit):
 *       leg_setpoint_echo  latest-before each robot_state t, STALE after
 *                          LEG_ECHO_STALE_S (main.js legEchoTimeout, 1 s) -> null -> NaN
 *       hand_telemetry     latest-before, never stale (live never nulls it)
 *   - setResident(chunks) concatenates the resident chunks' derived columns in
 *     time order into FRESH Float64Arrays and swaps `axes`; nothing is ever
 *     shifted or written in place, so a view uPlot holds can never move under
 *     it (the 2026-09-07 drift class).  `version` increases on every rebuild.
 *   - A chunk is cached only once sealed (`t instanceof Float64Array`); the
 *     session buffer's OPEN chunk (plain `t`, growing `n`) is re-derived on
 *     every setResident from a snapshot of its first `n` rows.
 *
 * Two tiers (post-Phase-4, the 10-minute view):
 *   - FULL tier: the resident chunks above.
 *   - DIGEST tier: `digestChunk(chunk)` bins the SAME derived columns into 1 s min/max/mean bins
 *     (`{t: bin centre, n, cols:{key:{min,mean,max}}}` per axis, Float64Arrays), one digest per
 *     10 s slot; `setDigests(Map slot -> digest)` installs them (digest.js owns the ring).
 *   - axes[a].getAlignedData(keys) is the COMPOSED view: digest means of the bins wholly before the
 *     resident window (an overlapping slot is clipped per bin), the full-rate samples, digest means of the
 *     bins wholly after - monotonic t, with one all-NaN point wherever neighbours are > GAP_S apart.
 *     With no digests it is exactly the full tier (zero-copy).  axes[a].envelope(key) is the digest
 *     regions' {t, min, max} (+ `split`, the count of leading points before the resident window).
 *     `axes[a].timestamps/columns/length` stay FULL-tier only.
 *
 * Cross-chunk joins: pass `latestBefore(topic, tSec)` backed by the feed so an
 * echo that precedes a chunk's first robot_state is found in the previous
 * chunk.  Without it the store searches the chunks handed to setResident only,
 * and the head of the first resident chunk may be left unjoined.
 *
 * Pure module: no DOM, no uPlot, no telemetry-charts import (injected).
 */
import { indexLatestBefore, scrubNonFinite } from './chunk.js';

export const LEG_ECHO_STALE_S = 1.0;
export const MOTOR_COUNT = 9;
export const BIN_S = 1;   // digest bin width (s)
export const GAP_S = 1.5 * BIN_S;   // composed-view neighbours further apart than this are separated by a NaN point
export const SIGNAL_KEYS = [
  'pos_measured', 'vel_measured', 'pos_commanded', 'iq_setpoint', 'iq_measured',
  'fet_temp', 'motor_temp', 'bus_voltage', 'bus_current',
];

const ROBOT_STATE = '/robot_state';
const LEG_ECHO = '/leg_setpoint_echo';
const HAND = '/hand_telemetry';

const EMPTY = new Float64Array(0);

/** Axis store with the same read surface the live ChartDataStore exposes. */
class ReplayAxisStore {
  /**
   * @param {Float64Array} timestamps FULL-tier timestamps
   * @param {object} columns FULL-tier columns (key -> Float64Array)
   * @param {number} a axis index
   * @param {{before:object[], after:object[]}|null} view digests wholly before / after the resident window
   */
  constructor(timestamps, columns, a, view) {
    this.length = timestamps.length;
    this.timestamps = timestamps;
    this.columns = columns;
    this._a = a;
    this._view = view || null;
    this._composed = null;
    Object.freeze(this.columns);
  }

  _compose() {
    if (this._composed) return this._composed;
    const v = this._view;
    const a = this._a;
    const nb = v ? v.before.reduce((s, d) => s + d.axes[a].t.length, 0) : 0;
    const na = v ? v.after.reduce((s, d) => s + d.axes[a].t.length, 0) : 0;
    let c;
    if (nb + na === 0) {
      c = { timestamps: this.timestamps, columns: this.columns, length: this.length, split: 0,
        env: { t: EMPTY, min: {}, max: {} } };
      for (const key of SIGNAL_KEYS) { c.env.min[key] = EMPTY; c.env.max[key] = EMPTY; }
    } else {
      // Ordered segments: digest means | full samples | digest means.  Wherever two neighbours are
      // more than GAP_S apart (a slot whose digest has not arrived, a recording hole) ONE all-NaN
      // point is inserted midway so the series line breaks instead of bridging the hole.
      const segs = [];
      for (const d of v.before) segs.push({ t: d.axes[a].t, col: (key) => d.axes[a].cols[key].mean });
      segs.push({ t: this.timestamps, col: (key) => this.columns[key] });
      for (const d of v.after) segs.push({ t: d.axes[a].t, col: (key) => d.axes[a].cols[key].mean });
      let total = 0, last = NaN;
      for (const sg of segs) {
        if (sg.t.length === 0) continue;
        if (last === last && sg.t[0] - last > GAP_S) total++;
        total += sg.t.length;
        last = sg.t[sg.t.length - 1];
      }
      const ts = new Float64Array(total);
      const cols = {};
      const et = new Float64Array(nb + na);
      const emin = {}, emax = {};
      for (const key of SIGNAL_KEYS) { cols[key] = new Float64Array(total).fill(NaN); emin[key] = new Float64Array(nb + na); emax[key] = new Float64Array(nb + na); }
      let o = 0, e = 0;
      last = NaN;
      const envPut = (list) => {
        for (const d of list) {
          const ax = d.axes[a];
          et.set(ax.t, e);
          for (const key of SIGNAL_KEYS) { emin[key].set(ax.cols[key].min, e); emax[key].set(ax.cols[key].max, e); }
          e += ax.t.length;
        }
      };
      for (const sg of segs) {
        const m = sg.t.length;
        if (m === 0) continue;
        if (last === last && sg.t[0] - last > GAP_S) { ts[o] = (last + sg.t[0]) / 2; o++; }   // cols pre-filled NaN
        ts.set(sg.t, o);
        for (const key of SIGNAL_KEYS) cols[key].set(sg.col(key), o);
        o += m;
        last = sg.t[m - 1];
      }
      envPut(v.before);
      envPut(v.after);
      Object.freeze(cols);
      c = { timestamps: ts, columns: cols, length: total, split: nb, env: { t: et, min: emin, max: emax } };
    }
    this._composed = c;
    return c;
  }

  /**
   * uPlot data for the active signals: the COMPOSED view (digest means | full samples | digest means;
   * exactly the full tier when no digests are installed).  The whole buffer is returned as zero-copy
   * views (immutable, so safe); `windowStart` and `copy` are accepted for interface parity and ignored -
   * the chart scale, not the data slice, picks the visible window, so moving the playhead never exposes a gap.
   */
  getAlignedData(signalKeys /* , windowStart, copy */) {
    const c = this._compose();
    const data = [c.timestamps];
    for (const key of signalKeys) data.push(c.columns[key]);
    return data;
  }

  /** Full-tier-only uPlot data (the resident chunks, no digest regions). */
  getFullData(signalKeys) {
    const data = [this.timestamps];
    for (const key of signalKeys) data.push(this.columns[key]);
    return data;
  }

  /** Digest regions only: {t, min, max, split} (arrays shared per snapshot; empty when no digests). */
  envelope(key) {
    const c = this._compose();
    const memo = c.envObj || (c.envObj = {});
    return memo[key] || (memo[key] = { t: c.env.t, min: c.env.min[key], max: c.env.max[key], split: c.split });
  }
}

/**
 * @param {{telemetrySample:Function, latestBefore?:Function}} deps
 *   telemetrySample(m, idx, commandedLegs, hand) -> values; latestBefore(topic, tSec)
 *   -> {t, row}|null with row the hydrated message (optional, see header).
 */
export function createReplayChartStore(deps) {
  const telemetrySample = deps.telemetrySample;
  const injectedLatest = deps.latestBefore || null;
  let cache = new WeakMap();
  let resident = [];
  let digests = new Map();     // slot index -> digest (digest.js owns the ring)
  let versionNo = 0;
  const listeners = new Set();
  const invalidateListeners = new Set();

  const store = {
    axes: [],
    get version() { return versionNo; },
    get chunks() { return resident; },
    get digestCount() { return digests.size; },
    setResident, windowFor, invalidate, setDigests, digestChunk, residentRange,
    envelope(a, key) { return store.axes[a].envelope(key); },
    onRebuild(fn) { listeners.add(fn); return () => listeners.delete(fn); },
    /** Fires from invalidate() BEFORE the rebuild: derived digests are stale (units changed). */
    onInvalidate(fn) { invalidateListeners.add(fn); return () => invalidateListeners.delete(fn); },
  };

  /** Latest-before over the resident chunks (default join source). */
  function residentLatest(topic, tSec) {
    for (let c = resident.length - 1; c >= 0; c--) {
      const tp = resident[c].topics[topic];
      if (!tp || tp.n === 0) continue;
      const k = indexLatestBefore(tp.t, tSec, tp.n);
      if (k >= 0) return { t: tp.t[k], row: tp.hydrate(k) };
    }
    return null;
  }

  function derive(chunk, latestOpt) {
    const rs = chunk.topics[ROBOT_STATE];
    const n = rs ? rs.n : 0;
    const latest = latestOpt || injectedLatest || residentLatest;
    const axT = [];
    const axC = [];
    for (let a = 0; a < MOTOR_COUNT; a++) { axT.push([]); axC.push({}); for (const k of SIGNAL_KEYS) axC[a][k] = []; }
    // The join rows change slowly; remember the last lookup so a 100 Hz chunk
    // does not hydrate the same echo row 100 times.
    let echoT = NaN, echoRow = null, handT = NaN, handRow = null;
    for (let k = 0; k < n; k++) {
      const t = rs.t[k];
      const motors = scrubNonFinite(rs.cols.motor_states[k]);
      if (!motors || motors.length === 0) continue;
      const e = latest(LEG_ECHO, t);
      let legs = null;
      if (e && t - e.t < LEG_ECHO_STALE_S) {
        if (e.t !== echoT) { echoT = e.t; echoRow = e.row; }
        legs = echoRow && echoRow.data ? echoRow.data : null;
      }
      const h = latest(HAND, t);
      let hand = null;
      if (h) {
        if (h.t !== handT) { handT = h.t; handRow = h.row; }
        hand = handRow;
      }
      const cnt = Math.min(motors.length, MOTOR_COUNT);
      for (let i = 0; i < cnt; i++) {
        const v = telemetrySample(motors[i], i, legs, hand);
        axT[i].push(t);
        for (const key of SIGNAL_KEYS) axC[i][key].push(v[key] === undefined ? NaN : v[key]);
      }
    }
    const axes = [];
    for (let a = 0; a < MOTOR_COUNT; a++) {
      const cols = {};
      for (const key of SIGNAL_KEYS) cols[key] = Float64Array.from(axC[a][key]);
      axes.push({ t: Float64Array.from(axT[a]), cols });
    }
    return axes;
  }

  function derivedFor(chunk) {
    let sealed = true;
    for (const name in chunk.topics) if (!(chunk.topics[name].t instanceof Float64Array)) sealed = false;
    if (sealed) {
      let d = cache.get(chunk);
      if (!d) { d = derive(chunk); cache.set(chunk, d); }
      return d;
    }
    return derive(chunk); // open chunk: never cached, derived from a snapshot of n rows
  }

  /** [t0, t1] spanned by the resident chunks, or null. */
  function residentRange() {
    if (resident.length === 0) return null;
    let a = Infinity, b = -Infinity;
    for (const c of resident) { if (c.t0 < a) a = c.t0; if (c.t1 > b) b = c.t1; }
    return { t0: a, t1: b };
  }

  /** The bins of `d` that lie entirely before (side<0) / after (side>0) the resident window rr, or null. */
  function clipDigest(d, rr, side) {
    const half = BIN_S / 2;
    const axes = [];
    let any = false;
    for (let a = 0; a < MOTOR_COUNT; a++) {
      const ax = d.axes[a];
      const T = ax.t;
      let from = 0, to = T.length;
      if (side < 0) { to = 0; while (to < T.length && T[to] + half <= rr.t0) to++; }
      else { from = T.length; while (from > 0 && T[from - 1] - half >= rr.t1) from--; }
      if (to - from > 0) any = true;
      const cols = {};
      for (const key of SIGNAL_KEYS) {
        const c = ax.cols[key];
        cols[key] = { min: c.min.subarray(from, to), mean: c.mean.subarray(from, to), max: c.max.subarray(from, to) };
      }
      axes.push({ t: T.subarray(from, to), n: ax.n.subarray(from, to), cols });
    }
    return any ? { i: d.i, t0: d.t0, t1: d.t1, axes } : null;
  }

  /**
   * Digest bins entirely before / after the resident window, in slot order.  A digest that overlaps the
   * window is clipped PER BIN (zero-copy subarrays), so no more than one bin of gap is left at each edge.
   */
  function digestView() {
    if (digests.size === 0) return null;
    const rr = residentRange();
    const before = [], after = [];
    const all = Array.from(digests.values()).sort((x, y) => x.i - y.i);
    for (const d of all) {
      if (!rr || d.t1 <= rr.t0) before.push(d);
      else if (d.t0 >= rr.t1) after.push(d);
      else {
        const b = clipDigest(d, rr, -1), f = clipDigest(d, rr, 1);
        if (b) before.push(b);
        if (f) after.push(f);
      }
    }
    return { before, after };
  }

  function install(axesBase) {
    const view = digestView();
    const axes = [];
    for (let a = 0; a < MOTOR_COUNT; a++) axes.push(new ReplayAxisStore(axesBase[a].timestamps, axesBase[a].columns, a, view));
    store.axes = axes;
    versionNo++;
    for (const fn of Array.from(listeners)) fn(store);
    return store;
  }

  function rebuild() {
    const ordered = resident.slice().sort((a, b) => (a.t0 - b.t0) || (a.i - b.i));
    const per = ordered.map(derivedFor);
    const base = [];
    for (let a = 0; a < MOTOR_COUNT; a++) {
      let total = 0;
      for (const d of per) total += d[a].t.length;
      const ts = new Float64Array(total);
      const cols = {};
      for (const key of SIGNAL_KEYS) cols[key] = new Float64Array(total);
      let off = 0;
      for (const d of per) {
        ts.set(d[a].t, off);
        for (const key of SIGNAL_KEYS) cols[key].set(d[a].cols[key], off);
        off += d[a].t.length;
      }
      base.push({ timestamps: ts, columns: cols });
    }
    return install(base);
  }

  function isSealed(chunk) {
    for (const name in chunk.topics) if (!(chunk.topics[name].t instanceof Float64Array)) return false;
    return true;
  }

  /** latest-before that looks in the chunk itself first (a far chunk is not resident), else the store's source. */
  function ownFirstLatest(chunk) {
    const fallback = injectedLatest || residentLatest;
    const memo = {};
    return (topic, tSec) => {
      const tp = chunk.topics[topic];
      if (tp && tp.n > 0) {
        const k = indexLatestBefore(tp.t, tSec, tp.n);
        if (k >= 0) {
          const m = memo[topic];
          if (m && m.k === k) return m.v;
          const v = { t: tp.t[k], row: tp.hydrate(k) };
          memo[topic] = { k, v };
          return v;
        }
      }
      return fallback(topic, tSec);
    };
  }

  /**
   * Bin a SEALED chunk's derived columns (the exact setResident derivation, incl. the echo/hand joins)
   * into 1 s min/max/mean bins.  Returns null for an open (unsealed) chunk.  Digest shape:
   *   {i, t0, t1, axes:[{t, n, cols:{key:{min, mean, max}}} x9]}, every field a Float64Array; only bins
   *   holding at least one axis sample are emitted; NaN samples are skipped per key (all-NaN bin -> NaN).
   */
  function digestChunk(chunk) {
    if (!isSealed(chunk)) return null;
    const d = resident.indexOf(chunk) >= 0 ? derivedFor(chunk) : derive(chunk, ownFirstLatest(chunk));
    const nb = Math.max(1, Math.round((chunk.t1 - chunk.t0) / BIN_S));
    const axes = [];
    for (let a = 0; a < MOTOR_COUNT; a++) {
      const T = d[a].t;
      const cnt = new Float64Array(nb);
      const kc = {}, ksum = {}, kmin = {}, kmax = {};
      for (const key of SIGNAL_KEYS) {
        kc[key] = new Float64Array(nb); ksum[key] = new Float64Array(nb);
        kmin[key] = new Float64Array(nb).fill(Infinity); kmax[key] = new Float64Array(nb).fill(-Infinity);
      }
      for (let k = 0; k < T.length; k++) {
        let b = Math.floor((T[k] - chunk.t0) / BIN_S);
        if (b < 0) b = 0; else if (b >= nb) b = nb - 1;
        cnt[b]++;
        for (const key of SIGNAL_KEYS) {
          const v = d[a].cols[key][k];
          if (v !== v) continue;   // NaN
          kc[key][b]++; ksum[key][b] += v;
          if (v < kmin[key][b]) kmin[key][b] = v;
          if (v > kmax[key][b]) kmax[key][b] = v;
        }
      }
      let m = 0;
      for (let b = 0; b < nb; b++) if (cnt[b] > 0) m++;
      const t = new Float64Array(m), n = new Float64Array(m);
      const cols = {};
      for (const key of SIGNAL_KEYS) cols[key] = { min: new Float64Array(m), mean: new Float64Array(m), max: new Float64Array(m) };
      let o = 0;
      for (let b = 0; b < nb; b++) {
        if (cnt[b] === 0) continue;
        t[o] = chunk.t0 + (b + 0.5) * BIN_S; n[o] = cnt[b];
        for (const key of SIGNAL_KEYS) {
          const c = kc[key][b];
          cols[key].min[o] = c ? kmin[key][b] : NaN;
          cols[key].max[o] = c ? kmax[key][b] : NaN;
          cols[key].mean[o] = c ? ksum[key][b] / c : NaN;
        }
        o++;
      }
      axes.push({ t, n, cols });
    }
    return Object.freeze({ i: chunk.i, t0: chunk.t0, t1: chunk.t1, axes });
  }

  /**
   * Install the digest tier (Map slot -> digest).  Fresh axis snapshots over the SAME full-tier arrays;
   * `version` increases and onRebuild listeners fire, like a rebuild.
   */
  function setDigests(bySlot) {
    digests = new Map(bySlot || []);
    const base = store.axes.length ? store.axes : null;
    if (!base) return rebuild();
    return install(base);
  }

  /**
   * Swap the resident chunk set.  Rebuilds every axis into fresh arrays.
   * @param {object[]} chunks decoded chunks (any order)
   * @returns {object} this store (`axes[i].getAlignedData(keys)` is the uPlot data)
   */
  function setResident(chunks) {
    const seen = new Set();
    resident = [];
    for (const c of chunks) { if (!seen.has(c.i)) { seen.add(c.i); resident.push(c); } }
    return rebuild();
  }

  /** Drop the derived cache (units changed) and rebuild from the same chunks. */
  function invalidate() {
    cache = new WeakMap();
    digests = new Map();   // derived in the old units: the digester re-fills via onInvalidate
    for (const fn of Array.from(invalidateListeners)) fn(store);
    return rebuild();
  }

  /**
   * Window geometry for a playhead: span clamped to [min, maxSpan], the time
   * range centred on pSec, and the resident chunk index range overlapping it.
   */
  function windowFor(pSec, spanSec, maxSpan) {
    const cap = maxSpan === undefined ? 600 : maxSpan;
    const span = Math.min(cap, Math.max(0, spanSec));
    const min = pSec - span / 2;
    const max = pSec + span / 2;
    let first = -1, last = -1;
    for (const c of resident) {
      if (c.t1 >= min && c.t0 <= max) {
        if (first < 0 || c.i < first) first = c.i;
        if (c.i > last) last = c.i;
      }
    }
    return { min, max, span, firstChunk: first, lastChunk: last };
  }

  return store;
}
