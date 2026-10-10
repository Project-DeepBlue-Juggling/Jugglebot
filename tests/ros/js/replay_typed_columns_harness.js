// Node harness for tests/ros/test_replay_typed_columns.py (M2a): the typed columnar layout end to end.
//   - buildColumns/hydrate round trip on a synthetic topic: scalar, bool, string, non-finite, array of numbers
//     (fixed and variable), array of objects with nested leaves + a string leaf + a nested array (any leaf), empty rows;
//   - topicBuffers + a real structuredClone with transfer: sender buffers detach, the received record hydrates identically;
//   - chunkFromRecord adopts the transferred buffers without copying.
import { buildColumns, topicBuffers } from './js/replay/mcap-decode.js';
import { makeTopic, chunkFromRecord } from './js/replay/chunk.js';

const out = {};
const rowsIn = [
  { a: 1.5, b: true, s: 'x', nan: NaN, arr: [1, 2, 3], var: [], objs: [{ id: 1, p: { x: 1, y: 2 }, tag: 'u', inner: [1] }, { id: 2, p: { x: NaN, y: 4 }, tag: 'v', inner: [] }] },
  { a: -2, b: false, s: 'y', nan: 7, arr: [4, 5, 6], var: [9], objs: [] },
  { a: Infinity, b: true, s: 'z', nan: -Infinity, arr: [7, Infinity, 9], var: [8, 7], objs: [{ id: 3, p: { x: 5, y: 6 }, tag: 'w', inner: [2, 3] }] },
];
const plain = {};
for (const r of rowsIn) for (const k of Object.keys(r)) (plain[k] ??= []).push(r[k]);
const { cols, kinds } = buildColumns(plain);
out.kinds = kinds;
out.types = Object.fromEntries(Object.entries(cols).map(([k, v]) => [k, ArrayBuffer.isView(v) ? v.constructor.name : Array.isArray(v) ? 'Array' : Object.keys(v)]));
const t = Float64Array.from([10, 11, 12]);
const tp = makeTopic('T', t, cols, kinds);
out.hydrated = rowsIn.map((_, k) => tp.hydrate(k));
out.hydrate_twice_distinct = tp.hydrate(0) !== tp.hydrate(0) && tp.hydrate(2).objs !== tp.hydrate(2).objs;
// the shared 'tp' shows nothing is memoised: mutate a hydrated row, hydrate again, get a clean row
const h = tp.hydrate(0); h.a = 99; h.objs[0].id = 99;
out.after_mutation = tp.hydrate(0);

// irregular column falls back to 'any'
const irr = buildColumns({ m: [1, 'a', null], o: [[{ x: 1 }], [{ y: 2 }], []] });
out.irregular = irr.kinds;
const irrTp = makeTopic('I', Float64Array.from([0, 1, 2]), irr.cols, irr.kinds);
out.irregular_rows = [0, 1, 2].map((k) => irrTp.hydrate(k));

// N3 / W1: nested-shape mismatch demotes to 'any'; empty-object leaf, null element, bool leaf in a CSR object
const rt = (name, col) => {
  const b = buildColumns({ [name]: col });
  const tp2 = makeTopic('R', Float64Array.from(col.map((_, i) => i)), b.cols, b.kinds);
  return { kind: b.kinds[name], rows: col.map((_, i) => tp2.hydrate(i)[name]) };
};
out.shape_nested = rt('n', [[{ a: { x: 1 } }], [{ a: { x: 2, y: 3 } }]]);
out.shape_empty = rt('e', [[{ a: {}, b: 1 }]]);
out.shape_null = rt('z', [[{ x: 1 }, null]]);
out.shape_bool = rt('q', [[{ f: true, x: 1 }, { f: false, x: 2 }]]);
// N1: a hydrated row omits keys whose column value is undefined
const un = buildColumns({ u: [[1], [2]], k: [1, 2] });
un.cols.k = Object.assign([undefined, 2]);
const unTp = makeTopic('U', Float64Array.from([0, 1]), un.cols, { u: un.kinds.u, k: 'any' });
out.undefined_omitted = { has_k0: 'k' in unTp.hydrate(0), has_k1: 'k' in unTp.hydrate(1) };

const bufs = topicBuffers({ t, cols }, new Set());
out.buffers = {
  count: bufs.size,
  expect: 1 + 3 + 2 + 2 + 1 + 3,   // t; a,b,nan; arr offsets+flat; var offsets+flat; objs offsets; id,p.x,p.y (tag and inner are cloned)
};
const msg = { op: 'slot', i: 0, topics: { '/x': { type: 'T', n: 3, t, cols, kinds } } };
const clone = structuredClone(msg, { transfer: [...bufs] });
out.detached = { t: t.byteLength === 0, a: cols.a.byteLength === 0, offs: cols.arr.offsets.byteLength === 0, leaf: cols.objs.leaves['p.x'].byteLength === 0 };
const ch = chunkFromRecord({ i: 0, t0: 0, t1: 10, topics: clone.topics });
out.adopted = ch.topics['/x'].t === clone.topics['/x'].t && ch.topics['/x'].cols === clone.topics['/x'].cols;
out.after_transfer = [0, 1, 2].map((k) => ch.topics['/x'].hydrate(k));
out.strings_cloned = Array.isArray(clone.topics['/x'].cols.s) && clone.topics['/x'].cols.s !== cols.s;
console.log(JSON.stringify(out));
