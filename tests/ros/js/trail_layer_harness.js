// trail_layer_harness.js — real trails.js + real Three (sandbox node_modules/three) + trail-settings.js.
// Prints one JSON object. Sandbox: ./trails.js ./trail-settings.js ./node_modules/three/
import * as THREE from 'three';
import { createTrailLayer } from './trails.js';
import * as settings from './trail-settings.js';

const out = {};
const mkLayer = (o) => {
    const parent = new THREE.Group();
    return [createTrailLayer(parent, Object.assign({ capacity: 64, maxTracks: 4 }, o)), parent];
};
const meshes = (parent) => parent.children[0].children;   // parent -> group 'trails' -> ribbon meshes
const drawn = (parent) => meshes(parent).map((m) => m.geometry.drawRange.count);
const live = (layer) => layer.stats().active;

{   // decimation: 200 Hz pushes -> ~100 Hz kept
    const [layer] = mkLayer({ capacity: 640 });
    layer.beginMessage('m');
    for (let i = 0; i < 400; i++) { layer.push('m', 'a', i * 0.005, i, 0, 0); }
    const s = layer.stats();
    out.dec_decimated = s.decimated;
    const v = layer.render(2.0, 5000);
    out.dec_vertices = v;   // 2 * k, k kept samples
    out.dec_active = s.active;
}
{   // ring wrap keeps the newest contiguous range
    const [layer, parent] = mkLayer({ capacity: 64, minDtSec: 0 });
    layer.beginMessage('m');
    const N = 200;
    for (let i = 0; i < N; i++) layer.push('m', 'a', i * 0.01, i, 0, 0);
    const v = layer.render((N - 1) * 0.01, 100000);
    out.wrap_vertices = v;
    const mesh = meshes(parent)[0], g = mesh.geometry;
    out.wrap_range = [g.drawRange.start, g.drawRange.count];
    // read back the x positions of the drawn range: indices -> vertices
    const idx = g.index.array, pos = g.attributes.position.array;
    const first = idx[g.drawRange.start], last = idx[g.drawRange.start + g.drawRange.count - 1];
    const xs = [];
    for (let j = g.drawRange.start / 6; j < (g.drawRange.start + g.drawRange.count) / 6 + 1; j++) xs.push(pos[(2 * j) * 3] * 1000);
    out.wrap_first_x = xs[0]; out.wrap_last_x = xs[xs.length - 1]; out.wrap_xs_n = xs.length;
    out.wrap_contiguous = xs.every((x, i) => i === 0 || Math.abs(x - xs[i - 1] - 1) < 1e-3);
    out.wrap_max_end = (g.drawRange.start + g.drawRange.count) <= 6 * (2 * 64 - 1);
}
{   // ended track released after now - endT > tail; before that it still draws (fade)
    const [layer, parent] = mkLayer({ minDtSec: 0 });
    layer.beginMessage('m');
    for (let i = 0; i < 10; i++) layer.push('m', 'a', i * 0.01, i, 0, 0);
    layer.end('a', 0.09);
    layer.render(0.5, 1000); out.end_active_in_tail = live(layer); out.end_drawing_in_tail = drawn(parent)[0] > 0;
    layer.render(1.2, 1000); out.end_active_after_tail = live(layer);
    out.end_free_after = layer.stats().free;
    // endMessage ends absent tracks of the tag only
    layer.beginMessage('m'); layer.push('m', 'a', 5, 0, 0, 0); layer.push('m', 'b', 5, 0, 0, 0);
    layer.beginMessage('o'); layer.push('o', 'c', 5, 0, 0, 0);
    layer.beginMessage('m'); layer.push('m', 'a', 5.02, 0, 0, 0); layer.endMessage('m', 5.02);
    layer.render(5.02 + 2, 1000);     // b ended at 5.02 and expired; a, c ended? only b
    out.endmsg_active = live(layer);  // a (alive, not ended) + c (other tag)
    // revived key starts fresh (no line across the gap)
    layer.reset();
    layer.beginMessage('m');
    layer.push('m', 'r', 0, 0, 0, 0); layer.push('m', 'r', 0.02, 1, 0, 0); layer.end('r', 0.02);
    // revive INSIDE the tail: without the reset the old samples would still be in [now - tail, now]
    layer.push('m', 'r', 0.5, 5, 0, 0); layer.push('m', 'r', 0.52, 6, 0, 0);
    layer.render(0.52, 1000);
    out.revive_indices = drawn(parent)[0];
    out.revive_keep_all = (() => {   // control: the same four samples with no end() draw three segments
        const [l2, p2] = mkLayer({ minDtSec: 0 });
        l2.beginMessage('m');
        for (const [t, x] of [[0, 0], [0.02, 1], [0.5, 5], [0.52, 6]]) l2.push('m', 'r', t, x, 0, 0);
        l2.render(0.52, 1000);
        return drawn(p2)[0];
    })();
}
{   // pool exhaustion drops without growing
    const [layer, parent] = mkLayer({ maxTracks: 3 });
    layer.beginMessage('m');
    const kids0 = meshes(parent).length;
    for (let k = 0; k < 10; k++) layer.push('m', 'k' + k, 1, k, 0, 0);
    const s = layer.stats();
    out.pool_active = s.active; out.pool_dropped = s.dropped; out.pool_meshes_same = meshes(parent).length === kids0;
}
{   // reset releases all; time going backwards resets a track
    const [layer] = mkLayer({ minDtSec: 0 });
    layer.beginMessage('m');
    for (let k = 0; k < 4; k++) layer.push('m', 'k' + k, 1, k, 0, 0);
    out.reset_before = live(layer); layer.reset();
    out.reset_after = live(layer); out.reset_free = layer.stats().free;
    layer.beginMessage('m');
    for (let i = 0; i < 5; i++) layer.push('m', 'a', 10 + i * 0.01, i, 0, 0);
    layer.push('m', 'a', 1.0, 0, 0, 0);
    out.backwards_vertices = layer.render(1.0, 1000);   // restarted: 1 sample -> nothing to draw
    layer.push('m', 'a', 1.02, 1, 0, 0);
    out.backwards_vertices2 = layer.render(1.02, 1000);
    // tail 0 draws nothing
    out.tail0 = layer.render(1.02, 0);
    // coordinate map (x,y,z) mm -> (x, z, -y) m
    layer.reset(); layer.beginMessage('m'); layer.push('m', 'p', 0, 1000, 2000, 3000);
    out.pos_first = Array.from(layer.group.children[0].geometry.attributes.position.array.slice(0, 3));
}
{   // settings
    const r = {};
    r.default = settings.getTailMs();
    const seen = [];
    const un = settings.onTailChange((v) => seen.push(v.effectiveMs));
    r.set_1234 = settings.setTailMs(1234);
    r.set_neg = settings.setTailMs(-50);
    r.set_big = settings.setTailMs(99999);
    r.set_nan = settings.setTailMs('abc');
    r.set_2500 = settings.setTailMs(2500);
    un(); settings.setTailMs(300);
    r.seen = seen;
    r.no_storage = typeof globalThis.localStorage === 'undefined';
    // storage that throws, and one that works
    let store = {};
    globalThis.localStorage = { getItem: (k) => { throw new Error('no'); }, setItem: () => { throw new Error('no'); } };
    r.throwing_set = settings.setTailMs(700);
    globalThis.localStorage = { getItem: (k) => (k in store ? store[k] : null), setItem: (k, v) => { store[k] = v; } };
    settings.setTailMs(1700); r.stored = store['jugglebot-trail-tail-ms'];
    out.settings = r;
}
{   // range / effective / near / slider mapping
    settings._resetForTest();
    delete globalThis.localStorage;
    const r = {};
    const ev = [];
    settings.onTailChange((e) => ev.push([e.effectiveMs, e.maxMs, e.storedMs]));
    r.max0 = settings.getTailMaxMs();
    settings.setTailMs(60000);
    r.live = [settings.getTailMs(), settings.getEffectiveTailMs(), settings.getNearTailMs()];
    settings.setTailMaxMs(1086000);                       // an 18:06 recording
    r.long = [settings.getTailMs(), settings.getEffectiveTailMs(), settings.getNearTailMs()];
    settings.setTailMaxMs(30000);                          // shrink: stored survives
    r.shrunk = [settings.getTailMs(), settings.getEffectiveTailMs(), settings.getNearTailMs()];
    settings.setTailMaxMs(30000);                          // no-op: no event
    settings.setTailMaxMs(5000);
    r.back = [settings.getTailMs(), settings.getEffectiveTailMs(), settings.getNearTailMs()];
    settings.setTailMaxMs(1086000);                       // re-grow: request is back
    r.regrown = settings.getEffectiveTailMs();
    r.ev = ev.slice();
    r.snap = [settings.snapMs(4949), settings.snapMs(5400), settings.snapMs(61200), settings.snapMs(-3), settings.snapMs('x')];
    settings.setTailMaxMs(0); r.max_bad = settings.getTailMaxMs();
    // stored persists across a shrink through storage
    const store = {};
    globalThis.localStorage = { getItem: (k) => (k in store ? store[k] : null), setItem: (k, v) => { store[k] = v; } };
    settings.setTailMaxMs(1086000); settings.setTailMs(120000); settings.setTailMaxMs(5000);
    r.persisted = store['jugglebot-trail-tail-ms'];
    // slider mapping
    const maps = {};
    for (const max of [3000, 5000, 20000, 1086000, 3600000]) {
        const pos = []; for (let i = 0; i <= 1000; i++) pos.push(settings.positionToMs(i / 1000, max));
        let mono = true; for (let i = 1; i < pos.length; i++) if (pos[i] < pos[i - 1]) mono = false;
        let worst = 0;
        for (const ms of [0, 100, 800, 1000, 2500, 5000, 6000, 12000, 20000, 60000, 120000, 600000, max]) {
            if (ms > max) continue;
            const back = settings.positionToMs(settings.msToPosition(ms, max), max);
            const rel = ms === 0 ? back : Math.abs(back - ms) / ms;
            worst = Math.max(worst, rel);
        }
        maps[max] = { mono, first: pos[0], last: pos[1000], at600: settings.positionToMs(0.6, max), worst,
                      posMax: settings.msToPosition(max, max), pos5: settings.msToPosition(5000, max) };
    }
    r.maps = maps;
    r.fmt = [0, 800, 5000, 12000, 59600, 60000, 200000, 1086000, 7320000].map(settings.formatTail);
    r.fmtMax = [5000, 1086000, 30000].map(settings.formatMax);
    // the slider's right end on a non-round max reaches the whole max (no re-snap); a longer request is kept as asked
    settings._resetForTest();
    settings.setTailMaxMs(1081300);
    settings.setTailMs(settings.getTailMaxMs());
    r.wholeMax = [settings.getEffectiveTailMs(), settings.getTailMaxMs(), settings.msToPosition(settings.getEffectiveTailMs(), 1081300)];
    settings.setTailMs(2000000);
    r.longer = [settings.getTailMs(), settings.getEffectiveTailMs()];
    settings._resetForTest();
    out.settings2 = r;
}
console.log(JSON.stringify(out));
