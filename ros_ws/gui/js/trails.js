/**
 * trails.js — ribbon trails for mocap markers and tracked balls (Phase 5).
 * Pure Three.js, no DOM. Lifted from the ticket-04 prototype
 * (test_replay_trails.html): RIBBON renderer only, age shown by dimming AND
 * thinning, ended tracks fade out over the tail.
 *
 *   const layer = createTrailLayer(parent, {capacity, maxTracks, getResolution, minDtSec});
 *   layer.setColorFn((key, tag) => 0xRRGGBB)   // colour chosen on acquire (and on setColorFn)
 *   layer.beginMessage(tag)                    // one per /mocap_data or /balls message
 *   layer.push(tag, key, tSec, xMm, yMm, zMm)  // robot frame mm; key = label | number
 *   layer.endMessage(tag, tSec)                // tracks of `tag` absent from the message end at t
 *   layer.end(key, tSec)                       // explicit end
 *   layer.render(nowSec, tailMs) -> vertices   // once per frame
 *   layer.reset()                              // seek / disconnect: release every track
 *   layer.stats()                              // readout only (may allocate)
 *
 * Storage per track: a CPU ring (Float32 xyz + Float64 t, `capacity` samples)
 * and a DOUBLED GPU ring (each sample written at slot s and s+capacity) so the
 * newest k samples are always one contiguous draw range. Age fade is computed
 * in the shader from a per-vertex time attribute + uNow/uTail, so per frame the
 * CPU does two binary searches and sets uniforms per live track.
 *
 * Time is seconds on ONE clock for push and render. A push with t < last t
 * resets the track (a seek the caller missed); a push to an ENDED key starts a
 * fresh trail; a push closer than `minDtSec` after the last accepted sample is
 * skipped (D2). The pool is allocated once; an exhausted pool drops (and
 * counts) new keys. Nothing on push/render allocates.
 * Known limit (audit 2026-10-11): sample times are float32 relative to the track's first sample; a track that
 * never ends (a labelled marker continuously visible) loses fade resolution after about a day (~8 ms at 1e5 s).
 */
import * as THREE from 'three';

const NOT_ENDED = Infinity;

function makeRibbonRenderer(cap, getRes) {
    const nv = 2 * cap * 2;                  // doubled ring x 2 vertices per sample (side -1/+1)
    const g = new THREE.BufferGeometry();
    const pos = new THREE.BufferAttribute(new Float32Array(nv * 3), 3).setUsage(THREE.DynamicDrawUsage);
    const prev = new THREE.BufferAttribute(new Float32Array(nv * 3), 3).setUsage(THREE.DynamicDrawUsage);
    const at = new THREE.BufferAttribute(new Float32Array(nv), 1).setUsage(THREE.DynamicDrawUsage);
    const side = new Float32Array(nv);
    for (let i = 0; i < nv; i++) side[i] = (i & 1) ? 1 : -1;
    const idx = new Uint16Array(6 * (2 * cap - 1));
    for (let j = 0; j < 2 * cap - 1; j++) {
        const a = 2 * j;
        idx.set([a, a + 1, a + 2, a + 1, a + 3, a + 2], 6 * j);
    }
    g.setAttribute('position', pos);
    g.setAttribute('aPrev', prev);
    g.setAttribute('aT', at);
    g.setAttribute('aSide', new THREE.BufferAttribute(side, 1));
    g.setIndex(new THREE.BufferAttribute(idx, 1));
    g.setDrawRange(0, 0);
    const m = new THREE.ShaderMaterial({
        transparent: true, depthWrite: false, side: THREE.DoubleSide,
        uniforms: {
            uNow: { value: 0 }, uTail: { value: 1 }, uColor: { value: new THREE.Color() },
            uWidth: { value: 4 }, uRes: { value: new THREE.Vector2(1, 1) },
        },
        vertexShader: `attribute vec3 aPrev; attribute float aT, aSide;
      uniform float uNow, uTail, uWidth; uniform vec2 uRes; varying float vAge;
      void main(){
        vec4 c = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
        vec4 p = projectionMatrix * modelViewMatrix * vec4(aPrev, 1.0);
        vec2 d = (c.xy / c.w - p.xy / p.w) * uRes;
        float L = length(d); d = L > 1e-4 ? d / L : vec2(1.0, 0.0);
        vec2 n = vec2(-d.y, d.x);
        float age = (uNow - aT) / uTail; vAge = age;
        float w = uWidth * clamp(1.0 - age, 0.0, 1.0);
        c.xy += n * aSide * w / uRes * c.w;
        gl_Position = c; }`,
        fragmentShader: `uniform vec3 uColor; varying float vAge;
      void main(){ if (vAge > 1.0 || vAge < -1e-4) discard;
        gl_FragColor = vec4(uColor, 1.0 - vAge); }`,
    });
    const obj = new THREE.Mesh(g, m);
    obj.frustumCulled = false;
    const wv = (o, x, y, z, px, py, pz, tRel) => {
        for (let v = 2 * o; v < 2 * o + 2; v++) {
            pos.array[v * 3] = x; pos.array[v * 3 + 1] = y; pos.array[v * 3 + 2] = z;
            prev.array[v * 3] = px; prev.array[v * 3 + 1] = py; prev.array[v * 3 + 2] = pz;
            at.array[v] = tRel;
        }
    };
    return {
        obj,
        write(s, x, y, z, px, py, pz, tRel) {
            wv(s, x, y, z, px, py, pz, tRel);
            wv(s + cap, x, y, z, px, py, pz, tRel);
            // Whole-buffer upload (flag only): per-sample addUpdateRange would push a
            // range object per sample, and the doubled ring makes the union ~whole anyway.
            pos.needsUpdate = true; prev.needsUpdate = true; at.needsUpdate = true;
        },
        draw(start, k, nowRel, tail) {
            if (k < 2) { g.setDrawRange(0, 0); return 0; }
            g.setDrawRange(6 * start, 6 * (k - 1));
            const u = m.uniforms;
            u.uNow.value = nowRel; u.uTail.value = tail;
            getRes(u.uRes.value);
            return 2 * k;
        },
        setColor(c) { m.uniforms.uColor.value.setHex(c); },
        dispose() { g.dispose(); m.dispose(); },
    };
}

class Trail {
    constructor(cap, getRes, minDt) {
        this.cap = cap; this.minDt = minDt;
        this.px = new Float32Array(cap * 3); this.tt = new Float64Array(cap);
        this.r = makeRibbonRenderer(cap, getRes);
        this.key = null; this.tag = null; this.seq = 0; this.color = 0;
        this.skipped = 0;
        this.reset();
    }
    reset() { this.n = 0; this.epoch = 0; this.lastT = -Infinity; this.endT = NOT_ENDED; }
    push(t, x, y, z) {                        // Three.js metres
        if (t < this.lastT) this.reset();     // time went backwards: start over
        else if (this.endT !== NOT_ENDED) this.reset();   // revived key: fresh trail
        else if (t - this.lastT < this.minDt - 1e-9) { this.skipped++; return; }   // D2
        if (this.n === 0) this.epoch = t;
        const cap = this.cap, s = this.n % cap, q = (this.n + cap - 1) % cap;
        const has = this.n > 0;
        const px = has ? this.px[3 * q] : x, py = has ? this.px[3 * q + 1] : y, pz = has ? this.px[3 * q + 2] : z;
        this.px[3 * s] = x; this.px[3 * s + 1] = y; this.px[3 * s + 2] = z; this.tt[s] = t;
        this.r.write(s, x, y, z, px, py, pz, t - this.epoch);
        this.n++; this.lastT = t;
    }
    end(t) { if (this.endT === NOT_ENDED) this.endT = t; }
    firstAtOrAfter(t) {                       // smallest sample number i in [n-m, n) with tt >= t
        const m = Math.min(this.n, this.cap);
        let lo = this.n - m, hi = this.n;
        while (lo < hi) {
            const mid = (lo + hi) >> 1;
            if (this.tt[mid % this.cap] >= t) hi = mid; else lo = mid + 1;
        }
        return lo;
    }
    render(now, tail) {                       // returns drawn vertices
        if (this.n === 0 || tail <= 0) return this.r.draw(0, 0, 0, 1);
        const lo = this.firstAtOrAfter(now - tail);
        const hi = this.firstAtOrAfter(now + 1e-9);   // exclude samples after the playhead
        const k = hi - lo;
        return this.r.draw(k > 0 ? lo % this.cap : 0, k > 0 ? k : 0, now - this.epoch, tail);
    }
}

export function createTrailLayer(parent, opts = {}) {
    const capacity = opts.capacity || 640;
    const maxTracks = opts.maxTracks || 32;
    const minDt = opts.minDtSec === undefined ? 0.01 : opts.minDtSec;
    const getRes = opts.getResolution || ((v) => v.set(1, 1));
    const group = new THREE.Group();
    group.name = 'trails';
    parent.add(group);
    const pool = [];
    for (let i = 0; i < maxTracks; i++) {
        const tr = new Trail(capacity, getRes, minDt);
        group.add(tr.r.obj);
        pool.push(tr);
    }
    const free = pool.slice().reverse();              // stack of idle trails
    const live = new Map();                   // key -> Trail (lookup only; never iterated per frame)
    const act = new Array(maxTracks);         // dense array of live trails (iteration)
    let nAct = 0;
    const seq = {};                           // tag -> message counter
    let colorFn = () => 0x06b6d4, dropped = 0;

    function release(tr, i) {
        tr.reset(); live.delete(tr.key); tr.key = null;
        tr.r.draw(0, 0, 0, 1);
        free.push(tr);
        act[i] = act[nAct - 1]; act[nAct - 1] = null; nAct--;   // swap-remove
    }
    const layer = {
        group,
        setColorFn(fn) {
            colorFn = fn;
            for (let i = 0; i < nAct; i++) {
                const tr = act[i]; tr.color = fn(tr.key, tr.tag); tr.r.setColor(tr.color);
            }
        },
        beginMessage(tag) { seq[tag] = (seq[tag] | 0) + 1; },
        push(tag, key, t, xMm, yMm, zMm) {
            let tr = live.get(key);
            if (tr === undefined) {
                tr = free.pop();
                if (tr === undefined) { dropped++; return; }   // pool exhausted: drop, never allocate
                tr.key = key; tr.tag = tag; live.set(key, tr);
                act[nAct++] = tr;
                tr.color = colorFn(key, tag); tr.r.setColor(tr.color);
            }
            tr.seq = seq[tag] | 0;
            tr.push(t, xMm * 0.001, zMm * 0.001, -yMm * 0.001);   // robot (x,y,z) mm -> Three (x,z,-y) m
        },
        endMessage(tag, t) {
            const s = seq[tag] | 0;
            for (let i = 0; i < nAct; i++) {
                const tr = act[i];
                if (tr.tag === tag && tr.seq !== s) tr.end(t);
            }
        },
        end(key, t) {
            const tr = live.get(key);
            if (tr !== undefined) tr.end(t);
        },
        render(now, tailMs) {
            const tail = tailMs / 1000;
            let v = 0;
            for (let i = nAct - 1; i >= 0; i--) {
                const tr = act[i];
                if (tr.endT !== NOT_ENDED && now - tr.endT > tail) { release(tr, i); continue; }
                v += tr.render(now, tail);
            }
            return v;
        },
        reset() { for (let i = nAct - 1; i >= 0; i--) release(act[i], i); },
        stats() {
            let skipped = 0;
            for (const tr of pool) skipped += tr.skipped;
            return { active: nAct, free: free.length, dropped, decimated: skipped, capacity, maxTracks };
        },
        dispose() { for (const tr of pool) tr.r.dispose(); parent.remove(group); },
    };
    return layer;
}
