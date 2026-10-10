/**
 * Replay chunk shape: index a 10 s slot, hydrate rows lazily.
 *
 * The record shape is defined by ros_ws/gui/replay/schema.py (read its docstring):
 * per topic, one `t` array and one column per flattened field. The mcap worker
 * (mcap-worker.js over mcap-decode.js) builds it in memory as TYPED columns (see
 * "typed columnar layout" below); this module is the consumer half of that contract:
 *
 *   - chunkFromRecord() worker slot record -> {i, t0, t1, topics:{"/x": {type, n, t:Float64Array, cols, kinds, hydrate(k)}}}
 *                      (adopts the transferred typed buffers as is: no per-row work, no copies)
 *   - typedColumns()   a topic's {cols, kinds}, building them once for a plain-column topic (test fixtures)
 *   - leafFiller()     compile a no-allocation "write row j's fields into a scratch object" reader
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
 * Rows are hydrated lazily, one per dispatched record; the chart store reads the typed
 * columns directly (leafFiller / CSR offsets), never a hydrated row. `undefined` column entries (a field a message did not carry) are
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


const fin = (v) => (Number.isFinite(v) ? v : null);

function setPath(row, parts, v) {
  let o = row;
  const last = parts.length - 1;
  for (let j = 0; j < last; j++) {
    const key = parts[j];
    let nxt = o[key];
    if (nxt === undefined) { nxt = {}; o[key] = nxt; }
    o = nxt;
  }
  o[parts[last]] = v;
}

// Value reader for one leaf/column kind at flat index i: (arr, i) -> plain value (non-finite -> null).
function readerOf(kind) {
  if (kind === 'f64') return (a, i) => fin(a[i]);
  if (kind === 'u8') return (a, i) => a[i] !== 0;
  if (kind === 'any') return (a, i) => scrubNonFinite(a[i]);
  return (a, i) => a[i];
}

/**
 * Compile a row builder for a typed topic from its `kinds` descriptor (see mcap-decode.js for the layout):
 * `(cols, k) => row`, allocating the row only when called (nothing is memoised per row).
 * @param {Object<string, *>} kinds
 */
export function makeTypedHydrator(kinds) {
  const steps = [];
  for (const name in kinds) {
    const kd = kinds[name];
    const parts = name.split('.');
    if (typeof kd === 'string') {
      const rd = readerOf(kd);
      steps.push((cols, k, row) => { const c = cols[name]; if (c !== undefined) { const v = rd(c, k); if (v !== undefined) setPath(row, parts, v); } });
    } else if (typeof kd.csr === 'string') {
      const rd = readerOf(kd.csr);
      steps.push((cols, k, row) => {
        const c = cols[name];
        if (c === undefined) return;
        const a = c.offsets[k], b = c.offsets[k + 1];
        const out = new Array(b - a);
        for (let j = a; j < b; j++) out[j - a] = rd(c.flat, j);
        setPath(row, parts, out);
      });
    } else {
      const lv = [];
      for (const p in kd.csr.leaves) lv.push({ p, parts: p.split('.'), rd: readerOf(kd.csr.leaves[p]) });
      steps.push((cols, k, row) => {
        const c = cols[name];
        if (c === undefined) return;
        const a = c.offsets[k], b = c.offsets[k + 1];
        const out = new Array(b - a);
        for (let j = a; j < b; j++) {
          const o = {};
          for (let q = 0; q < lv.length; q++) { const v = lv[q].rd(c.leaves[lv[q].p], j); if (v !== undefined) setPath(o, lv[q].parts, v); }
          out[j - a] = o;
        }
        setPath(row, parts, out);
      });
    }
  }
  return function hydrateTyped(cols, k) {
    const row = {};
    for (let s = 0; s < steps.length; s++) steps[s](cols, k, row);
    return row;
  };
}

/** True when `topic` carries a typed `kinds` descriptor (decoded recording slots); false for plain-column topics. */
export function isTyped(topic) { return topic.kinds !== undefined && topic.kinds !== null; }

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
 * Build a topic entry (decoded recording chunks; plain-column topics only in test fixtures).
 * `hydrate(k)` tracks the column set, so a plain topic that gains a column
 * recompiles transparently.
 */
export function makeTopic(type, t, cols, kinds) {
  const topic = {
    type,
    n: t.length,
    t,
    cols,
    kinds: kinds || null,
    _h: null,
    _hn: -1,
    hydrate(k) {
      if (this.kinds !== null) {
        if (this._h === null) this._h = makeTypedHydrator(this.kinds);
        return this._h(this.cols, k);
      }
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
 * Build a chunk from the worker's slot record ({i, t0, t1, topics:{"/x":{type, t, cols, kinds}}}):
 * `t` and the typed `cols` buffers are the ones the worker transferred and are adopted as is.
 * @param {{i:number, t0:number, t1:number, topics:object}} rec
 */
export function chunkFromRecord(rec) {
  const topics = {};
  for (const name in rec.topics) {
    const src = rec.topics[name];
    topics[name] = makeTopic(src.type, src.t instanceof Float64Array ? src.t : Float64Array.from(src.t), src.cols, src.kinds);
  }
  return { i: rec.i, t0: rec.t0, t1: rec.t1, topics };
}

// ---- typed columnar layout (M2a) ---------------------------------------------------------------
// One column per flattened field, shape-driven by a per-topic `kinds` descriptor that travels with the
// record. Every numeric/bool column is a typed array so the worker can TRANSFER it (no structured-clone
// copy) and the main thread holds 8 bytes per number instead of a boxed object per array element.
//   kinds[col] === 'f64'  cols[col]: Float64Array(n)         (NaN/Inf kept; hydration maps them to null)
//              === 'u8'   cols[col]: Uint8Array(n)           (booleans)
//              === 'str'  cols[col]: Array(n) of strings     (cloned, not transferred)
//              === 'any'  cols[col]: Array(n) of anything    (fallback for irregular columns; cloned)
//   {csr: 'f64'|'u8'|'str'}                 array of primitives per row:
//                                            cols[col] = {offsets: Uint32Array(n+1), flat: Float64Array|Uint8Array|Array(total)}
//   {csr: {leaves: {path: kind}}}           array of OBJECTS per row (kind in f64|u8|str|any):
//                                            cols[col] = {offsets: Uint32Array(n+1), leaves: {path: Float64Array(total)|...}}
//                                            (nested objects flatten by the dotted rule; a nested array inside an element is an 'any' leaf)
// Row k of a CSR group is the half-open range offsets[k]..offsets[k+1] of its flat / leaf columns.
const isObj = (v) => v !== null && typeof v === 'object' && !Array.isArray(v);

function primKind(a) {
  if (!a.length) return null;
  const t = typeof a[0];
  const k = t === 'number' ? 'f64' : t === 'boolean' ? 'u8' : t === 'string' ? 'str' : null;
  if (k === null) return null;
  for (let i = 1; i < a.length; i++) if (typeof a[i] !== t) return null;
  return k;
}

function typedOf(kind, a) {
  if (kind === 'f64') return Float64Array.from(a);
  if (kind === 'u8') { const o = new Uint8Array(a.length); for (let i = 0; i < a.length; i++) o[i] = a[i] ? 1 : 0; return o; }
  return a;
}

function hasKeys(o) { for (const _ in o) return true; return false; }

// Shape signature: key count at every depth (an empty object is one leaf key, as leavesOf emits it).
function shapeCount(o) {
  let c = 0;
  for (const k in o) { c++; const v = o[k]; if (isObj(v)) c += shapeCount(v); }
  return c;
}

function leavesOf(o, prefix, out) {
  for (const k in o) { const v = o[k]; if (isObj(v) && hasKeys(v)) leavesOf(v, prefix + k + '.', out); else out[prefix + k] = v; }
  return out;
}

function csrOffsets(a) {
  const off = new Uint32Array(a.length + 1);
  let p = 0;
  for (let i = 0; i < a.length; i++) { off[i] = p; p += a[i].length; }
  off[a.length] = p;
  return off;
}

// Classify one column's per-row values (length n) -> [kind, columnValue].
export function buildColumn(a) {
  const pk = primKind(a);
  if (pk) return [pk, typedOf(pk, a)];
  if (a.length && a.every((v) => Array.isArray(v))) {
    const offsets = csrOffsets(a);
    const total = offsets[a.length];
    const flat = new Array(total);
    let p = 0;
    for (let i = 0; i < a.length; i++) { const v = a[i]; for (let j = 0; j < v.length; j++) flat[p++] = v[j]; }
    if (total === 0) return [{ csr: 'f64' }, { offsets, flat: new Float64Array(0) }];
    const ek = primKind(flat);
    if (ek) return [{ csr: ek }, { offsets, flat: typedOf(ek, flat) }];
    if (flat.every(isObj)) {
      // Leaf paths come from the first element; every element of a ROS message array has the same fields by
      // construction (same .msg type), so a leaf is read by walking its path - a missing one demotes the column to 'any'.
      const first = leavesOf(flat[0], '', {});
      const names = Object.keys(first);
      const nShape = shapeCount(flat[0]);
      const lk = {}, leaves = {};
      let ok = true;
      for (let j = 1; j < total && ok; j++) { if (shapeCount(flat[j]) !== nShape) ok = false; }
      for (let q = 0; q < names.length && ok; q++) {
        const nm = names[q], parts = nm.split('.'), last = parts.length - 1;
        const get = (o) => { for (let j = 0; j < last && o !== undefined; j++) o = o[parts[j]]; return o === undefined ? undefined : o[parts[last]]; };
        const t = typeof first[nm];
        const arr = t === 'number' ? new Float64Array(total) : t === 'boolean' ? new Uint8Array(total) : new Array(total);
        let kind = t === 'number' ? 'f64' : t === 'boolean' ? 'u8' : t === 'string' ? 'str' : 'any';
        for (let j = 0; j < total; j++) {
          const el = flat[j];
          const v = get(el);
          if (v === undefined) { ok = false; break; }
          if (kind === 'f64') { if (typeof v !== 'number') { kind = null; break; } arr[j] = v; }
          else if (kind === 'u8') { if (typeof v !== 'boolean') { kind = null; break; } arr[j] = v ? 1 : 0; }
          else if (kind === 'str') { if (typeof v !== 'string') { kind = null; break; } arr[j] = v; }
          else arr[j] = v;
        }
        if (!ok) break;
        if (kind === null) {   // mixed types in one leaf: keep the values as they are
          const vals = new Array(total);
          for (let j = 0; j < total; j++) vals[j] = get(flat[j]);
          lk[nm] = 'any'; leaves[nm] = vals;
        } else { lk[nm] = kind; leaves[nm] = arr; }
      }
      if (ok) return [{ csr: { leaves: lk } }, { offsets, leaves }];
    }
  }
  return ['any', a];
}

// {col: [values per row]} -> {cols, kinds}
export function buildColumns(plainCols) {
  const cols = {}, kinds = {};
  for (const k of Object.keys(plainCols)) { const [kind, val] = buildColumn(plainCols[k]); kinds[k] = kind; cols[k] = val; }
  return { cols, kinds };
}


/**
 * A topic's typed `{cols, kinds}`: its own when decoded (typed), else built once from its plain
 * columns and memoised on the topic (re-built if `n` changed).  Test fixtures use plain topics.
 */
export function typedColumns(topic) {
  if (topic.kinds) return topic;
  if (topic._typed && topic._typedN === topic.n) return topic._typed;
  const plain = {};
  for (const name in topic.cols) plain[name] = topic.cols[name].length === topic.n ? topic.cols[name] : Array.prototype.slice.call(topic.cols[name], 0, topic.n);
  topic._typed = buildColumns(plain);
  topic._typedN = topic.n;
  return topic._typed;
}

/**
 * Compile a reader that writes row `j`'s fields into a reusable scratch object: `(o, j) => void`.
 * `arrs` maps leaf path -> column array (a topic's scalar `cols`, or a CSR group's `leaves`);
 * `kinds` maps path -> 'f64'|'u8'|'str'|'any' (entries with another kind are skipped).  Non-finite
 * numbers become null (the rosbridge rule hydrate() applies).  Dotted paths write nested objects.
 * No allocation per call once the scratch's nested objects exist.
 */
export function leafFiller(kinds, arrs) {
  const steps = [];
  for (const p in kinds) {
    const kd = kinds[p];
    const a = arrs[p];
    if (typeof kd !== 'string' || a === undefined) continue;
    const parts = p.split('.');
    steps.push({ a, kd, parts, last: parts[parts.length - 1], depth: parts.length - 1 });
  }
  return function fill(o, j) {
    for (let s = 0; s < steps.length; s++) {
      const st = steps[s];
      let v = st.a[j];
      if (st.kd === 'f64') v = Number.isFinite(v) ? v : null;
      else if (st.kd === 'u8') v = v !== 0;
      else if (st.kd === 'any') v = scrubNonFinite(v);
      let t = o;
      for (let q = 0; q < st.depth; q++) { const key = st.parts[q]; t = t[key] || (t[key] = {}); }
      t[st.last] = v;
    }
  };
}
