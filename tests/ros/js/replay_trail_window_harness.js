// replay_trail_window_harness.js - drives the REAL replay/trail-window.js over typed chunk records (chunk.js
// buildColumns, the shape the mcap worker delivers) with a FAKE feed that logs every call. Prints one JSON object
// that tests/ros/test_gui_replay_trail_window.py asserts on.
// Sandbox: replay/{chunk,trail-window}.js + this file + package.json {"type":"module"}.
// Fixture: synthetic (the oracle bag has no /balls): /mocap_data 200 Hz with 3 markers (2 labelled, 1 unlabelled ''),
// /balls 200 Hz while a ball exists (t in [3.0, 9.0], two ids in [5, 7]); 10 s slots, 3 slots.
const { buildColumns, chunkFromRecord } = await import('./replay/chunk.js');
const { createTrailWindow } = await import('./replay/trail-window.js');

const T0 = 1000, NSLOT = 3;
function slotRecord(i) {
    const mt = [], markers = [], bt = [], balls = [];
    for (let k = 0; k < 2000; k++) {
        const t = T0 + i * 10 + k * 0.005 + 0.001;
        mt.push(t);
        markers.push([
            { position: { x: k, y: 1, z: 2 }, label: 'a' },
            { position: { x: k, y: 3, z: 4 }, label: 'b' },
            { position: { x: k, y: 5, z: 6 }, label: '' },
        ]);
        const rel = t - T0;
        if (rel >= 3 && rel <= 9) {
            const bs = [{ id: 1, status: 1, position: { x: rel, y: 7, z: 8 } }];
            if (rel >= 5 && rel <= 7) bs.push({ id: 2, status: 2, position: { x: rel, y: 9, z: 10 } });
            bt.push(t); balls.push(bs);
        }
    }
    const mk = (t, cols) => { const c = buildColumns(cols); return { type: 'T', n: t.length, t: Float64Array.from(t), cols: c.cols, kinds: c.kinds }; };
    return { i, t0: T0 + i * 10, t1: T0 + (i + 1) * 10, topics: { '/mocap_data': mk(mt, { markers }), '/balls': mk(bt, { balls }) } };
}
const chunks = [];
for (let i = 0; i < NSLOT; i++) chunks.push(chunkFromRecord(slotRecord(i)));
const out = { kinds: chunks[0].topics['/mocap_data'].kinds, ballKinds: chunks[0].topics['/balls'].kinds };

const resident = new Map(chunks.map((c) => [c.i, c]));
const source = { chunkIndex: (t) => Math.max(0, Math.min(NSLOT - 1, Math.floor((t - T0) / 10))), range: () => ({ t0: T0, t1: T0 + 30 }) };
function mkCache() {
    const ls = new Set();
    return { peek: (i) => resident.get(i) || null, onChange(cb) { ls.add(cb); return () => ls.delete(cb); }, fire(reason) { for (const f of [...ls]) f({ reason }); }, size: () => ls.size };
}
function mkFeed() {
    const log = [];
    return {
        log,
        reset() { log.push('R'); }, setRenderTime(v) { log.push('S' + v); }, tick(t) { log.push('T'); },
        beginMocap(t) { log.push('bm' + t); }, marker(t, l, x, y, z) { log.push(['m', t, l, x, y, z].join()); }, endMocap(t) { log.push('em' + t); },
        beginBalls(t) { log.push('bb' + t); }, ball(t, id, st, x, y, z) { log.push(['b', t, id, st, x, y, z].join()); }, endBalls(t) { log.push('eb' + t); },
    };
}
const isRec = (s) => /^(bm|em|bb|eb|m,|b,)/.test(s);
const recT = (s) => parseFloat(s.startsWith('m,') || s.startsWith('b,') ? s.split(',')[1] : s.slice(2));
const recs = (log) => log.filter(isRec);
let tail = 1000;
const mk = (extra) => {
    const cache = mkCache(), feed = mkFeed();
    const w = createTrailWindow(Object.assign({ source, cache, feed, getTailMs: () => tail }, extra || {}));
    return { w, cache, feed };
};

// (1) a seek rebuild at p == playing from p - tail in small steps (records with t >= p - tail), same calls same order
{
    tail = 1000;
    const p = T0 + 6.2;
    const a = mk(); a.w.onResetForSeek(); a.w.onPlayhead(p);
    const b = mk();
    for (let t = p - 1.0 - 0.0; t <= p + 1e-9; t += 0.037) b.w.onPlayhead(t);
    b.w.onPlayhead(p);
    const A = recs(a.feed.log), B = recs(b.feed.log).filter((s) => recT(s) >= p - 1.0 - 1e-12);
    out.seek_eq = { equal: JSON.stringify(A) === JSON.stringify(B), nA: A.length, nB: B.length, hasBall: A.some((s) => s.startsWith('b,')), hasTwoIds: A.some((s) => s.startsWith('b,') && s.split(',')[2] === '2'),
        sorted: A.every((s, i) => i === 0 || recT(A[i - 1]) <= recT(s)), unlabelled: A.some((s) => s.startsWith('m,') && s.split(',')[2] === ''),
        endsWith: b.feed.log.slice(-2), aStats: a.w.stats(), bStats: b.w.stats() };
}
// (2) reverse step = rebuild of the shorter window
{
    const a = mk(); a.w.onPlayhead(T0 + 6.2); a.feed.log.length = 0;
    a.w.onPlayhead(T0 + 6.0);
    const L = a.feed.log;
    const r = mk(); r.w.onResetForSeek(); r.w.onPlayhead(T0 + 6.0);
    out.reverse = { startsWithReset: L[0] === 'R', equalsFresh: JSON.stringify(recs(L)) === JSON.stringify(recs(r.feed.log)), maxT: Math.max.apply(null, recs(L).map(recT)) <= T0 + 6.0 + 1e-9,
        minT: Math.min.apply(null, recs(L).map(recT)) >= T0 + 5.0 - 1e-9, rebuilds: a.w.stats().rebuilds };
}
// forward append within the tail: no reset, exactly (lastP, p]
{
    const a = mk(); a.w.onPlayhead(T0 + 6.0); a.feed.log.length = 0;
    a.w.onPlayhead(T0 + 6.1);
    const R = recs(a.feed.log).map(recT);
    out.append = { noReset: !a.feed.log.includes('R'), min: Math.min.apply(null, R) > T0 + 6.0, max: Math.max.apply(null, R) <= T0 + 6.1 + 1e-9, n: R.length, stats: a.w.stats(), tail: a.feed.log.slice(-2) };
}
// (3) a forward jump larger than the tail rebuilds
{
    const a = mk(); a.w.onPlayhead(T0 + 4.0); a.feed.log.length = 0; a.w.onPlayhead(T0 + 8.0);
    const R = recs(a.feed.log).map(recT);
    out.jump = { startsWithReset: a.feed.log[0] === 'R', min: Math.min.apply(null, R) >= T0 + 7.0 - 1e-9, rebuilds: a.w.stats().rebuilds };
}
// (4) tail 0 still feeds the ball-staleness span (spheres stay; the layer draws no trail at tail 0)
{
    tail = 0;
    const a = mk(); a.w.onPlayhead(T0 + 6.0); a.w.onPlayhead(T0 + 6.1); a.w.onResetForSeek(); a.w.onPlayhead(T0 + 2);
    out.tail0 = { recT: recs(a.feed.log).map(recT), resets: a.feed.log.filter((s) => s === 'R').length, renderTimes: a.feed.log.filter((s) => s.startsWith('S')) };
    tail = 1000;
}
// (5) a missing chunk marks the window incomplete; the residency event rebuilds; a non-residency event does not
{
    resident.delete(1);
    const a = mk(); a.w.onResetForSeek(); a.w.onPlayhead(T0 + 10.4);   // window [9.4, 10.4] straddles slots 0 and 1
    const before = recs(a.feed.log).map(recT);
    const inc1 = a.w.stats().incomplete;
    a.cache.fire('frontier'); const afterOther = a.w.stats().rebuilds;
    resident.set(1, chunks[1]);
    a.feed.log.length = 0;
    a.cache.fire('residency');
    const R = recs(a.feed.log).map(recT);
    const full = mk(); full.w.onResetForSeek(); full.w.onPlayhead(T0 + 10.4);
    out.incomplete = { inc1, inc2: a.w.stats().incomplete, onlyBefore: before.every((t) => t < T0 + 10), afterOther, rebuiltEqualsFull: JSON.stringify(recs(a.feed.log)) === JSON.stringify(recs(full.feed.log)),
        nAfter: R.length, startsWithReset: a.feed.log[0] === 'R', endsRender: a.feed.log[a.feed.log.length - 1] };
    a.cache.fire('residency'); out.incomplete.noRebuildWhenComplete = a.w.stats().rebuilds;
}
// (6) dispose resets the feed, clears render time, unsubscribes
{
    const a = mk(); a.w.onPlayhead(T0 + 6); a.feed.log.length = 0; a.w.dispose();
    const n = a.cache.size(); a.w.onPlayhead(T0 + 6.1);
    out.dispose = { log: a.feed.log, listeners: n };
}
// (7) onResetForSeek forces a rebuild even for a small forward step
{
    const a = mk(); a.w.onPlayhead(T0 + 6.0); a.feed.log.length = 0;
    a.w.onResetForSeek(); a.w.onPlayhead(T0 + 6.01);
    const R = recs(a.feed.log).map(recT);
    out.forced = { startsWithReset: a.feed.log[0] === 'R', min: Math.min.apply(null, R) >= T0 + 5.01 - 1e-9 };
}
// tail change -> rebuild at lastP; tail grows window
{
    let cb = null; let unsub = 0;
    const a = mk({ onTailChange: (f) => { cb = f; return () => { unsub++; }; } });
    a.w.onPlayhead(T0 + 6.0); a.feed.log.length = 0; tail = 2000; cb();
    const R = recs(a.feed.log).map(recT);
    a.w.dispose();
    out.tailChange = { startsWithReset: a.feed.log[0] === 'R', min: Math.min.apply(null, R) >= T0 + 4.0 - 1e-9 && Math.min.apply(null, R) < T0 + 4.1, unsub };
    tail = 1000;
}
// unlabelled/untyped topic skipped: plain-column topic without kinds
{
    const saved = chunks[0].topics['/balls'].kinds; chunks[0].topics['/balls'].kinds = null;
    const a = mk(); a.w.onResetForSeek(); a.w.onPlayhead(T0 + 4.0);
    out.untyped = { ballCalls: recs(a.feed.log).filter((s) => s.startsWith('b,')).length, mocapCalls: recs(a.feed.log).filter((s) => s.startsWith('m,')).length };
    chunks[0].topics['/balls'].kinds = saved;
}
// timing: a 5 s window rebuild
{
    tail = 5000;
    const a = mk(); a.w.onResetForSeek(); a.w.onPlayhead(T0 + 8.0); a.w.onResetForSeek(); a.w.onPlayhead(T0 + 8.0);
    const ts = [];
    for (let k = 0; k < 20; k++) { a.w.onResetForSeek(); a.w.onPlayhead(T0 + 8.0 + k * 0.01); ts.push(a.w.stats().lastRebuildMs); }
    ts.sort((x, y) => x - y);
    out.timing = { records: a.w.stats().lastRecords, medianMs: ts[10], maxMs: ts[19] };
    tail = 1000;
}
console.log(JSON.stringify(out));
