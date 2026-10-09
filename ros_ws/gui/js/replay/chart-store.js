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
export const SIGNAL_KEYS = [
  'pos_measured', 'vel_measured', 'pos_commanded', 'iq_setpoint', 'iq_measured',
  'fet_temp', 'motor_temp', 'bus_voltage', 'bus_current',
];

const ROBOT_STATE = '/robot_state';
const LEG_ECHO = '/leg_setpoint_echo';
const HAND = '/hand_telemetry';

/** Axis store with the same read surface the live ChartDataStore exposes. */
class ReplayAxisStore {
  constructor(timestamps, columns) {
    this.length = timestamps.length;
    this.timestamps = timestamps;
    this.columns = columns;
    Object.freeze(this.columns);
  }

  /**
   * uPlot data for the active signals.  The whole resident buffer is returned
   * as zero-copy views (immutable, so safe); `windowStart` and `copy` are
   * accepted for interface parity and ignored — the chart scale, not the data
   * slice, picks the visible window, so moving the playhead never exposes a gap.
   */
  getAlignedData(signalKeys /* , windowStart, copy */) {
    const data = [this.timestamps];
    for (const key of signalKeys) data.push(this.columns[key]);
    return data;
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
  let versionNo = 0;
  const listeners = new Set();

  const store = {
    axes: [],
    get version() { return versionNo; },
    get chunks() { return resident; },
    setResident, windowFor, invalidate,
    onRebuild(fn) { listeners.add(fn); return () => listeners.delete(fn); },
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

  function derive(chunk) {
    const rs = chunk.topics[ROBOT_STATE];
    const n = rs ? rs.n : 0;
    const latest = injectedLatest || residentLatest;
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

  function rebuild() {
    const ordered = resident.slice().sort((a, b) => (a.t0 - b.t0) || (a.i - b.i));
    const per = ordered.map(derivedFor);
    const axes = [];
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
      axes.push(new ReplayAxisStore(ts, cols));
    }
    store.axes = axes;
    versionNo++;
    for (const fn of Array.from(listeners)) fn(store);
    return store;
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
    return rebuild();
  }

  /**
   * Window geometry for a playhead: span clamped to [min, maxSpan], the time
   * range centred on pSec, and the resident chunk index range overlapping it.
   */
  function windowFor(pSec, spanSec, maxSpan) {
    const cap = maxSpan === undefined ? 120 : maxSpan;
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
