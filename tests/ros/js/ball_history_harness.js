// ball_history_harness.js - drives the REAL replay/ball-history.js over typed /balls chunks (chunk.js buildColumns, the
// shape the mcap worker delivers) with a fake source / engine / cache / layer. Prints one JSON object.
// Sandbox: replay/{chunk,ball-history}.js + this file + package.json {"type":"module"}.
const { buildColumns, chunkFromRecord } = await import('./replay/chunk.js');
const { createBallHistory } = await import('./replay/ball-history.js');

const T0 = 1000, NSLOT = 20;
// slot i: /balls at 200 Hz, ids 1 and 2 (ball 2 only in the second half) -> after 10 ms decimation 100 Hz per id
function ballsChunk(i, only) {
    const t = [], balls = [];
    for (let k = 0; k < 2000; k++) {
        const tk = T0 + i * 10 + k * 0.005 + 0.001;
        const bs = [{ id: 1, status: 1, position: { x: k * 0.1, y: 7, z: 8 } }];
        if (k >= 1000) bs.push({ id: 2, status: 1, position: { x: k * 0.1, y: 9, z: 10 } });
        t.push(tk); balls.push(bs);
    }
    const c = buildColumns({ balls });
    const topics = { '/balls': { type: 'T', n: t.length, t: Float64Array.from(t), cols: c.cols, kinds: c.kinds } };
    if (!only) topics['/other'] = { type: 'O', n: 1, t: Float64Array.from([T0 + i * 10 + 1]), ...buildColumns({ v: [1] }) };
    return chunkFromRecord({ i, t0: T0 + i * 10, t1: T0 + (i + 1) * 10, topics });
}
function fakeSource() {
    const log = [];
    const src = {
        log, gate: null,
        range: () => ({ t0: T0, t1: T0 + NSLOT * 10, frontier: T0 + NSLOT * 10 }),
        chunkIndex: (t) => Math.floor((t - T0) / 10),
        bounds: (i) => [T0 + i * 10, T0 + (i + 1) * 10],
        peek: () => null, onChange: () => () => {},
        async load(i, o) {
            log.push({ i, lite: !!(o && o.lite), topics: o && o.topics ? o.topics.slice() : null });
            if (src.gate) await src.gate;
            return ballsChunk(i, !!(o && o.topics));
        },
    };
    return src;
}
function fakeEngine(p) {
    const l = { playhead: new Set(), buffering: new Set() };
    const e = {
        mode: 'playing', p,
        state: () => ({ mode: e.mode, playhead: e.p }),
        on: (evt, cb) => { l[evt].add(cb); return () => l[evt].delete(cb); },
        move(x) { e.p = x; if (e.hook) e.hook(x); for (const cb of Array.from(l.playhead)) cb(x); },
        setBuffering(b) { e.mode = b ? 'buffering' : 'playing'; for (const cb of Array.from(l.buffering)) cb(b); },
    };
    return e;
}
function fakeCache() {
    const ls = new Set();
    const c = { res: new Map(), busy: false, peek: (i) => c.res.get(i) || null, loading: () => c.busy,
        onChange: (cb) => { ls.add(cb); return () => ls.delete(cb); }, fire() { for (const cb of Array.from(ls)) cb({ reason: 'residency' }); } };
    return c;
}
function fakeLayer() {
    const l = { calls: [], clears: 0, last: null, setSlots(rows) { l.calls.push(rows.map((r) => r.i)); l.last = rows; }, clear() { l.clears++; l.calls.push('clear'); } };
    return l;
}
const mkQueue = () => { const q = []; return { q, fn: (f) => { q.push(f); } }; };
async function settle(d) {
    for (let n = 0; n < 600; n++) {
        await Promise.resolve(); await Promise.resolve();
        if (d.q.length) { const f = d.q.shift(); f(); } else { await new Promise((r) => setTimeout(r, 0)); if (!d.q.length) break; }
    }
}
function make(p, tail, over) {
    over = over || {};
    const src = over.src || fakeSource(), eng = fakeEngine(p), cache = fakeCache(), layer = fakeLayer();
    const d = mkQueue(), fr = mkQueue();
    const tl = { ms: tail, cbs: new Set(), set(ms) { tl.ms = ms; for (const cb of Array.from(tl.cbs)) cb({}); } };
    const h = createBallHistory({ source: src, cache, engine: eng, layer, getTailMs: () => tl.ms,
        onTailChange: (cb) => { tl.cbs.add(cb); return () => tl.cbs.delete(cb); }, defer: d.fn, frame: fr.fn });
    eng.hook = (p) => h.onPlayhead(p);   // replay/mode.js forwards the engine's onPlayhead hook
    const flush = () => { while (fr.q.length) fr.q.shift()(); };
    return { src, eng, cache, layer, d, fr, h, tl, flush };
}
const out = {};

// 1. fill order: playhead slot, then backward nearest-first, then the forward slack; lite + /balls filter on every load
{
    const x = make(T0 + 150, 60000);
    await settle(x.d);
    out.fill = { order: x.src.log.map((e) => e.i), all_lite: x.src.log.every((e) => e.lite),
        all_filtered: x.src.log.every((e) => e.topics && e.topics.length === 1 && e.topics[0] === '/balls'), status: x.h.status() };
    out.batched = { frames_queued: x.fr.q.length, calls_before_frame: x.layer.calls.length };
    x.flush();
    out.batched.calls = x.layer.calls.slice();
    const rows = x.layer.last;
    const r0 = rows.find((r) => r.i === 15);
    out.decimation = { n: r0.n, ids: Array.from(new Set(r0.id)).sort(), mono: r0.t.every((t, k) => k === 0 || t >= r0.t[k - 1]),
        min_dt_per_id: (() => { const last = {}; let m = 1e9; for (let k = 0; k < r0.n; k++) { const id = r0.id[k]; if (id in last) m = Math.min(m, r0.t[k] - last[id]); last[id] = r0.t[k]; } return m; })(),
        types: [r0.t.constructor.name, r0.id.constructor.name, r0.x.constructor.name] };
    out.fill.sorted_publish = x.layer.calls[0].every((v, k, a) => k === 0 || v > a[k - 1]);

    // 2. retarget threshold: <= 10 s no new work; > 10 s retargets (and evicts only outside [p-tail-10, p+10])
    const n0 = x.src.log.length;
    x.eng.move(T0 + 155); await settle(x.d);
    const small = { loads: x.src.log.length - n0, centre: x.h.status().centre };
    x.eng.move(T0 + 165); await settle(x.d);
    const big = { new: x.src.log.slice(n0).map((e) => e.i), centre: x.h.status().centre };
    x.flush();
    x.eng.move(T0 + 20); await settle(x.d); x.flush();
    const far = x.layer.calls[x.layer.calls.length - 1];
    out.retarget = { small, big, far, status: x.h.status() };

    // 4. tail change: shrinking evicts, going <= 5 s clears and goes inactive, growing refills
    x.tl.set(5000); await settle(x.d); x.flush();
    const inactive = { active: x.h.status().active, slots: x.h.status().slots, last: x.layer.calls[x.layer.calls.length - 1] };
    const n1 = x.src.log.length;
    x.eng.move(T0 + 100); await settle(x.d);
    inactive.loads_while_inactive = x.src.log.length - n1;
    x.tl.set(30000); await settle(x.d); x.flush();
    out.tail = { inactive, regrown: x.layer.calls[x.layer.calls.length - 1], regrown_loads: x.src.log.slice(n1).map((e) => e.i) };
    x.h.dispose();
    out.dispose = { clears: x.layer.clears, last: x.layer.calls[x.layer.calls.length - 1] };
    x.eng.move(T0 + 150); await settle(x.d);
    out.dispose.after_move_loads = x.src.log.length - n1 - out.tail.regrown_loads.length;
}

// 3. paused while the engine buffers or the cache has a load in flight; a resident full chunk is read in place
{
    const x = make(T0 + 150, 20000);
    x.eng.setBuffering(true);
    await settle(x.d);
    const idle = x.src.log.length;
    x.eng.setBuffering(false);
    await settle(x.d);
    const afterResume = x.src.log.length;
    const y = make(T0 + 150, 20000);
    y.cache.busy = true; await settle(y.d);
    const busyLoads = y.src.log.length;
    y.cache.busy = false; y.cache.fire(); await settle(y.d);
    const z = make(T0 + 150, 20000);
    z.cache.res.set(15, ballsChunk(15, false));
    z.cache.res.set(14, ballsChunk(14, false));
    await settle(z.d); z.flush();
    out.paused = { idle_loads: idle, resumed: afterResume > 0, busy_loads: busyLoads, busy_resumed: y.src.log.length > 0 };
    out.resident = { loaded: z.src.log.map((e) => e.i).sort((a, b) => a - b), slots: z.layer.calls[0] };
}

// 5. a failed/aborted load does not hot-loop; an in-flight load cancelled by a retarget is ignored
{
    const x = make(T0 + 150, 20000);
    let release; x.src.gate = new Promise((r) => { release = r; });
    await settle(x.d);
    const inflight = x.h.status().inflight;
    x.eng.move(T0 + 20);       // far retarget while slot 15 is in flight
    await settle(x.d);
    x.src.gate = null; release(); await settle(x.d); x.flush();
    out.cancel = { inflight, slots: x.h.status().slots, last: x.layer.calls[x.layer.calls.length - 1] };
}
process.stdout.write(JSON.stringify(out));
