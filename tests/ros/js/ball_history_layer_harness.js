// ball_history_layer_harness.js - real js/ball-history-layer.js + real Three (sandbox node_modules/three) + marker-palette.js.
// Sandbox: ./ball-history-layer.js ./marker-palette.js ./node_modules/three/. Prints one JSON object.
import * as THREE from 'three';
import { createBallHistoryLayer } from './ball-history-layer.js';
import { ballColour, labelToColour } from './marker-palette.js';

const out = {};
const row = (i, samples) => {   // samples: [t, id, x, y, z]
    const n = samples.length;
    const r = { i, n, t: new Float64Array(n), id: new Int32Array(n), x: new Float32Array(n), y: new Float32Array(n), z: new Float32Array(n) };
    samples.forEach((s, k) => { r.t[k] = s[0]; r.id[k] = s[1]; r.x[k] = s[2]; r.y[k] = s[3]; r.z[k] = s[4]; });
    return r;
};
const mk = (maxVertices) => { const parent = new THREE.Group(); const layer = createBallHistoryLayer(parent, { maxVertices }); return [layer, parent]; };
const mesh = (parent) => parent.children[0].children[0];
const attr = (parent, name) => mesh(parent).geometry.attributes[name];

{   // segments join consecutive same-id samples with dt <= 0.15; interleaved ids do not join each other; gap breaks
    const [layer, parent] = mk(64);
    layer.setSlots([row(0, [
        [100.00, 1, 0, 0, 0], [100.00, 2, 5, 0, 0],
        [100.05, 1, 1, 0, 0], [100.05, 2, 6, 0, 0],
        [100.10, 1, 2, 0, 0],
        [100.50, 1, 3, 0, 0],                 // gap 0.40 > 0.15: no segment
        [100.55, 1, 4, 0, 0],
        [100.55, 2, 7, 0, 0],                 // id 2 gap 0.50: no segment
    ])]);
    const s = layer.stats();
    out.join = { vertices: s.vertices, segments: s.segments, draw: mesh(parent).geometry.drawRange.count };
    const pos = attr(parent, 'position').array;
    const xs = []; for (let v = 0; v < s.vertices; v++) xs.push(Math.round(pos[3 * v] * 1000));
    out.join.xs = xs;   // pairs (start,end) in mm: id1 0->1, id2 5->6, id1 1->2, id1 3->4
}
{   // robot mm (x,y,z) -> three (x, z, -y) m; colour = ballColour(id); time relative to the earliest sample
    const [layer, parent] = mk(16);
    layer.setSlots([row(0, [[50, 3, 1000, 2000, 3000], [50.01, 3, 1000, 2000, 3000]])]);
    const p = attr(parent, 'position').array, c = attr(parent, 'aColor').array, t = attr(parent, 'aT').array;
    const col = new THREE.Color().setHex(ballColour(3));
    out.map = { p: [p[0], p[1], p[2]], colour_ok: Math.abs(c[0] - col.r) < 1e-6 && Math.abs(c[1] - col.g) < 1e-6 && Math.abs(c[2] - col.b) < 1e-6,
        t: [t[0], t[1]], epoch: layer.stats().epoch };
}
{   // a join across a slot boundary; slots may arrive in any order
    const [layer] = mk(32);
    layer.setSlots([row(1, [[10.01, 1, 2, 0, 0], [10.02, 1, 3, 0, 0]]), row(0, [[9.98, 1, 0, 0, 0], [9.99, 1, 1, 0, 0]])]);
    out.cross = { segments: layer.stats().segments, slots: layer.stats().slots, epoch: layer.stats().epoch };
}
{   // render sets uniforms only: no buffer upload flag, no attribute write, no allocation of attributes
    const [layer, parent] = mk(64);
    layer.setSlots([row(0, [[100, 1, 0, 0, 0], [100.05, 1, 1, 0, 0], [100.10, 1, 2, 0, 0]])]);
    const names = ['position', 'aT', 'aColor'];
    const before = names.map((n) => attr(parent, n).version);
    const snap = names.map((n) => Array.from(attr(parent, n).array.subarray(0, 12)));
    const rb = layer.stats().rebuilds;
    const v1 = layer.render(100.2, 5000), v2 = layer.render(100.3, 8000);
    const u = mesh(parent).material.uniforms;
    out.render = { versions_same: names.every((n, k) => attr(parent, n).version === before[k]),
        data_same: names.every((n, k) => JSON.stringify(Array.from(attr(parent, n).array.subarray(0, 12))) === JSON.stringify(snap[k])),
        rebuilds_same: layer.stats().rebuilds === rb, v1, v2, uNow: u.uNow.value, uTail: u.uTail.value, visible: mesh(parent).visible };
    layer.render(100.2, 0); out.render.hidden_at_tail0 = mesh(parent).visible === false;
    layer.render(100.2, 5000); layer.clear(); out.render.cleared = [layer.render(100.2, 5000), mesh(parent).visible, mesh(parent).geometry.drawRange.count];
    out.render.shader_discards = /discard/.test(mesh(parent).material.fragmentShader) && /uNow/.test(mesh(parent).material.fragmentShader);
}
{   // overflow drops the OLDEST slots first and counts them
    const [layer] = mk(8);   // 4 segments
    const slot = (i, t0) => row(i, [[t0, 1, 0, 0, 0], [t0 + 0.05, 1, 1, 0, 0], [t0 + 0.10, 1, 2, 0, 0]]);   // 2 segments each
    layer.setSlots([slot(0, 10), slot(1, 20), slot(2, 30)]);   // 6 segments > 4
    const s = layer.stats();
    out.overflow = { vertices: s.vertices, segments: s.segments, slots: s.slots, droppedSlots: s.droppedSlots, droppedSegments: s.droppedSegments, epoch: s.epoch };
}
out.ctor = { odd_max: (() => { const [l] = mk(7); return l.stats().maxVertices; })() };
{   // lazy buffers: nothing before data, power-of-two >= 4096 sized to the data, released on clear (review 2026-10-11)
    const [layer, parent] = mk(1000000);
    const before = layer.stats().capacity;
    const samples = [];
    for (let k = 0; k < 3000; k++) samples.push([k * 0.01, 1, k, 0, 0]);   // one id, 2999 segments = 5998 vertices
    layer.setSlots([row(0, samples)]);
    const after = layer.stats().capacity, posLen = attr(parent, 'position').array.length;
    layer.clear();
    out.lazy = { before, after, posLen, cleared: layer.stats().capacity, hasPos: !!attr(parent, 'position') };
}
const mrow = (i, ball, mk) => {   // ball: [t,id,x,y,z][], mk: [t,labelIdx,x,y,z][]
    const r = row(i, ball), n = mk.length;
    Object.assign(r, { mn: n, mt: new Float64Array(n), mid: new Int32Array(n), mx: new Float32Array(n), my: new Float32Array(n), mz: new Float32Array(n) });
    mk.forEach((s, k) => { r.mt[k] = s[0]; r.mid[k] = s[1]; r.mx[k] = s[2]; r.my[k] = s[3]; r.mz[k] = s[4]; });
    return r;
};
{   // markers: own key space (label 1 never joins ball id 1), per-label palette colour, mapping like balls
    const [layer, parent] = mk(64);
    layer.setSlots([mrow(0, [[10.00, 1, 0, 0, 0], [10.05, 1, 1, 0, 0]], [[10.00, 1, 100, 0, 0], [10.05, 1, 200, 0, 0], [10.05, 0, 5, 0, 0]])], ['Platform 1', 'Ball Butler 2']);
    const s = layer.stats();
    const c = attr(parent, 'aColor').array, p = attr(parent, 'position').array;
    const eq = (v, hex) => { const k = new THREE.Color().setHex(hex); return Math.abs(v[0] - k.r) < 1e-6 && Math.abs(v[1] - k.g) < 1e-6 && Math.abs(v[2] - k.b) < 1e-6; };
    out.markers = { vertices: s.vertices, ball_colour: eq([c[0], c[1], c[2]], ballColour(1)), marker_colour: eq([c[6], c[7], c[8]], labelToColour('Ball Butler 2')),
        marker_x: [Math.round(p[6] * 1000), Math.round(p[9] * 1000)], palette_differs: labelToColour('Ball Butler 2') !== ballColour(1) };
}
{   // marker-only slot works; markers sort/epoch with the balls; render hides at tail 0
    const [layer] = mk(64);
    layer.setSlots([mrow(0, [], [[5, 0, 0, 0, 0], [5.05, 0, 1, 0, 0]])], ['Base 1']);
    out.marker_only = { segments: layer.stats().segments, epoch: layer.stats().epoch, r0: layer.render(5.1, 0), r5: layer.render(5.1, 5000) };
}
{   // cap policy: over the cap the OLDEST slots' MARKER rows go first, balls stay; only then whole slots
    const ball = (t0) => [[t0, 1, 0, 0, 0], [t0 + 0.05, 1, 1, 0, 0]];                       // 1 segment
    const mkr = (t0) => [[t0, 0, 0, 0, 0], [t0 + 0.05, 0, 1, 0, 0], [t0 + 0.10, 0, 2, 0, 0]];   // 2 segments
    const slots = () => [mrow(0, ball(10), mkr(10)), mrow(1, ball(20), mkr(20)), mrow(2, ball(30), mkr(30))];   // 9 segments
    const [a] = mk(14);   // cap 7 segments -> drop slot 0 + slot 1 marker rows (4) -> 5
    a.setSlots(slots(), ['Base 1']);
    const sa = a.stats();
    const [b] = mk(6);    // cap 3: all marker rows dropped (3 balls segs = 3) fits
    b.setSlots(slots(), ['Base 1']);
    const sb = b.stats();
    const [c] = mk(4);    // cap 2: markers all dropped, then the oldest ball slot too
    c.setSlots(slots(), ['Base 1']);
    const sc = c.stats();
    out.cap = { a: [sa.segments, sa.slots, sa.droppedMarkerSlots, sa.droppedMarkerSegments, sa.droppedSlots], b: [sb.segments, sb.droppedMarkerSlots, sb.droppedSlots],
        c: [sc.segments, sc.slots, sc.droppedMarkerSlots, sc.droppedSlots] };
}
{   // append-only (2026-10-11): rows matched by object; a new slot writes + uploads only itself (+ the small joins geometry);
    // the drawn set equals a from-scratch build; an eviction rebuilds
    const seq = (i, t0, x0) => row(i, [0, 1, 2, 3].map((k) => [t0 + 0.05 * k, 1, x0 + k, 0, 0]));   // 3 intra segments
    const A = seq(1, 10.0, 10), B = seq(2, 10.2, 20), C = seq(0, 9.75, 0);   // C ends at 9.90: joins A (0.10); A ends 10.15 -> B 10.20 joins
    const segsOf = (parent, layer) => {   // multiset of segments as "x0>x1@t0" (t absolute, ms)
        const out2 = []; const ep = layer.stats().epoch;
        for (const m of parent.children[0].children) {
            const g = m.geometry, n = g.drawRange.count; if (!g.attributes.position) continue;
            const P = g.attributes.position.array, T = g.attributes.aT.array;
            for (let v = 0; v < n; v += 2) out2.push(`${Math.round(P[3 * v] * 1000)}>${Math.round(P[3 * v + 3] * 1000)}@${Math.round((T[v] + ep) * 1000)}`);
        }
        return out2.sort();
    };
    const [layer, parent] = mk(1000000);
    layer.setSlots([A]); const s1 = layer.stats();
    layer.setSlots([A, B]); const s2 = layer.stats();
    const ranges = (name) => JSON.stringify(parent.children[0].children[0].geometry.attributes[name].updateRanges);
    const r2 = { pos: ranges('position'), aT: ranges('aT') };   // nothing has rendered: pending ranges for this append only
    layer.setSlots([B, C, A]); const s3 = layer.stats();
    const [ref, refParent] = mk(1000000);
    ref.setSlots([C, A, B]);
    out.append = { full: [s1.fullUploads, s2.fullUploads, s3.fullUploads], seg: [s1.segments, s2.segments, s3.segments],
        up: [s1.uploadedVertices, s2.uploadedVertices - s1.uploadedVertices, s3.uploadedVertices - s2.uploadedVertices],
        epoch: [s1.epoch, s3.epoch], same_as_scratch: JSON.stringify(segsOf(parent, layer)) === JSON.stringify(segsOf(refParent, ref)),
        ref_segments: ref.stats().segments, ranges: r2 };
    layer.setSlots([C, B]);   // A evicted: rebuild, epoch re-based to C
    const s4 = layer.stats();
    out.append.evict = [s4.fullUploads, s4.segments, s4.epoch];
}
process.stdout.write(JSON.stringify(out));
