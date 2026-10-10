/**
 * Replay chunk shape: index a 10 s slot, hydrate rows lazily.
 *
 * The record shape is defined by ros_ws/gui/replay/schema.py (read its docstring):
 * per topic, one `t` array and one column per flattened field. The mcap worker
 * (mcap-worker.js over mcap-decode.js) builds it in memory; this module is the
 * consumer half of that contract:
 *
 *   - chunkFromRecord() worker slot record -> {i, t0, t1, topics:{"/x": {type, n, t:Float64Array, cols, hydrate(k)}}}
 *   - makeHydrator()   column paths -> a function building row k as a nested plain object
 *                      (split on '.', nested objects; array columns assigned as-is)
 *   - flattenMessage() the exact inverse (plain objects recurse with dotted paths,
 *                      ANY array is one column value, typed arrays -> plain arrays)
 *   - indexLatestBefore() binary search on a time array
 *
 * Non-finite floats (representation rule, pinned by tests/ros/test_gui_replay_feed.py).
 * The live path is rosbridge JSON; rosbridge_library/internal/message_conversion.py
 * (_from_inst, Foxy) maps every NaN or +-Inf float to None, which JSON encodes as
 * null ("JSON does not support Inf and NaN. They are mapped to None"). msgpack
 * carries real NaN/Inf, so hydrate() mirrors rosbridge: any non-finite number
 * becomes null, including inside array columns (recursively, without mutating
 * the decoded columns). flattenMessage() leaves values untouched, so
 * hydrate(flatten(m)) deep-equals m for any m rosbridge could have produced.
 *
 * Rows are hydrated lazily, one per dispatched record; charts read `cols`
 * directly. `undefined` column entries (a field a message did not carry) are
 * omitted from the hydrated row.
 */

/** @returns {boolean} true for a plain (non-array, non-typed-array) object. */
function isPlainObject(v) {
  return v !== null && typeof v === 'object' && !Array.isArray(v) && !ArrayBuffer.isView(v);
}

// MUST equal FORMAT_VERSION in replay/schema.py (pinned by tests/ros/test_gui_replay_format_contract.py).
export const CHUNK_FORMAT = 2;

/**
 * Replace non-finite numbers with null (rosbridge's JSON rule). Returns the
 * input itself when nothing needs replacing, so clean rows share structure.
 */
export function scrubNonFinite(v) {
  if (typeof v === 'number') return Number.isFinite(v) ? v : null;
  if (Array.isArray(v)) {
    let out = null;
    for (let i = 0; i < v.length; i++) {
      const s = scrubNonFinite(v[i]);
      if (s !== v[i] && !(s === null && v[i] === null)) {
        if (out === null) out = v.slice();
        out[i] = s;
      }
    }
    return out === null ? v : out;
  }
  if (isPlainObject(v)) {
    let out = null;
    for (const k in v) {
      const s = scrubNonFinite(v[k]);
      if (s !== v[k]) {
        if (out === null) out = Object.assign({}, v);
        out[k] = s;
      }
    }
    return out === null ? v : out;
  }
  return v;
}

/**
 * Compile a row builder for a fixed list of column paths. Compiled ONCE per
 * (type, path set) by the caller's cache; the returned function is
 * `(cols, k) => row`.
 * @param {string[]} colPaths  flattened column names, e.g. "pose_offset_quat.w"
 */
export function makeHydrator(colPaths) {
  const steps = colPaths.map((p) => ({ name: p, parts: p.split('.') }));
  return function hydrateRow(cols, k) {
    const row = {};
    for (let s = 0; s < steps.length; s++) {
      const st = steps[s];
      const col = cols[st.name];
      if (col === undefined) continue;
      const v = col[k];
      if (v === undefined) continue;
      const parts = st.parts;
      let o = row;
      const last = parts.length - 1;
      for (let j = 0; j < last; j++) {
        const key = parts[j];
        let nxt = o[key];
        if (nxt === undefined) { nxt = {}; o[key] = nxt; }
        o = nxt;
      }
      o[parts[last]] = typeof v === 'number' ? (Number.isFinite(v) ? v : null) : scrubNonFinite(v);
    }
    return row;
  };
}

/**
 * Flatten a message into {dottedPath: value}. Plain objects recurse; every
 * array (primitive or message list) is ONE column value; typed arrays become
 * plain arrays.
 * @param {object} msg
 * @param {object} [out]    accumulator
 * @param {string} [prefix] path prefix (internal)
 */
export function flattenMessage(msg, out, prefix) {
  out = out || {};
  prefix = prefix || '';
  for (const key in msg) {
    const v = msg[key];
    const path = prefix + key;
    if (isPlainObject(v)) flattenMessage(v, out, path + '.');
    else if (ArrayBuffer.isView(v)) out[path] = Array.from(v);
    else out[path] = v;
  }
  return out;
}

/**
 * Index of the last entry with t[idx] <= x, or -1 when x precedes t[0].
 * @param {ArrayLike<number>} t ascending
 * @param {number} x
 * @param {number} [n] number of valid entries (default t.length)
 */
export function indexLatestBefore(t, x, n) {
  let lo = 0;
  let hi = (n === undefined ? t.length : n) - 1;
  let ans = -1;
  while (lo <= hi) {
    const mid = (lo + hi) >>> 1;
    if (t[mid] <= x) { ans = mid; lo = mid + 1; } else hi = mid - 1;
  }
  return ans;
}

const _hydrators = new Map();

/** Cached makeHydrator() keyed by type + path set. */
export function hydratorFor(type, colPaths) {
  const key = String(type) + '\u0000' + colPaths.join('\u0000');
  let h = _hydrators.get(key);
  if (h === undefined) {
    h = makeHydrator(colPaths);
    if (_hydrators.size > 256) _hydrators.clear();
    _hydrators.set(key, h);
  }
  return h;
}

/**
 * Build a topic entry (shared by decoded recording chunks and the session buffer).
 * `hydrate(k)` tracks the column set, so a session-buffer topic that gains a
 * column mid-chunk recompiles transparently.
 */
export function makeTopic(type, t, cols) {
  const topic = {
    type,
    n: t.length,
    t,
    cols,
    _h: null,
    _hn: -1,
    hydrate(k) {
      const names = Object.keys(this.cols);
      if (this._h === null || this._hn !== names.length) {
        this._h = hydratorFor(this.type, names);
        this._hn = names.length;
      }
      return this._h(this.cols, k);
    },
  };
  return topic;
}

/**
 * Build a chunk from the worker's slot record ({i, t0, t1, topics:{"/x":{type, t, cols}}}):
 * the same shape the session buffer holds, so cache/engine/chart store see one chunk type.
 * `t` arrives as a transferred Float64Array; `cols` are plain arrays (the schema.py contract).
 * @param {{i:number, t0:number, t1:number, topics:object}} rec
 */
export function chunkFromRecord(rec) {
  const topics = {};
  for (const name in rec.topics) {
    const src = rec.topics[name];
    topics[name] = makeTopic(src.type, Float64Array.from(src.t), src.cols);
  }
  return { i: rec.i, t0: rec.t0, t1: rec.t1, topics };
}
