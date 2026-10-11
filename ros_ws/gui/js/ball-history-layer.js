/**
 * ball-history-layer.js - the far-tier ball flights (Phase 5b): thin lines for every loaded ball sample pair,
 * faded by age over the whole tail.  Pure Three.js, no DOM.
 *
 *   const layer = createBallHistoryLayer(parent, {maxVertices, colourFn?});
 *   layer.setSlots(rows[])      rows = replay/ball-history.js slot rows {i, n, t:Float64Array(s), id, x, y, z (robot mm)}, any order;
 *                               REBUILDS the buffers (allowed to cost; called at most once per frame, only when the set changed)
 *   layer.render(nowSec, tailMs)  uniforms ONLY (no buffer write, no allocation); hides the line when tailMs <= 0 or empty
 *   layer.clear()               drop everything
 *   layer.stats()               {vertices, maxVertices, segments, slots, droppedSlots, droppedSegments, epoch, rebuilds}
 *   layer.dispose()
 *
 * ONE LineSegments geometry.  A segment joins two consecutive samples of the SAME id (across slot boundaries) when their
 * dt <= MAX_GAP_S (0.15 s, the trail feed's ball-staleness D1).  Per-vertex attributes: position (Three metres,
 * robot (x, y, z) mm -> (x, z, -y) * 0.001), aT (float32 seconds relative to `epoch` = the earliest loaded sample, re-based
 * on every setSlots) and aColor (ballColour(id), linear-agnostic like the ribbons).  The shader discards samples after the
 * playhead or older than the tail and fades alpha linearly with age down to ALPHA_FLOOR so the oldest flights stay legible.
 * maxVertices is a CAP, not an allocation: the buffers are sized to the loaded flights (power of two, >= 4096
 * vertices, doubling as slots arrive) and released by clear(), so a live session or a short tail holds nothing and a
 * rebuild uploads only what is used (review 2026-10-11: an eager 28 MB allocation re-uploaded whole on every arrival).
 * Over the cap the OLDEST slots are dropped first and counted.
 */
import * as THREE from 'three';
import { ballColour } from './marker-palette.js';

export const MAX_GAP_S = 0.15;
export const ALPHA_FLOOR = 0.15;
const MM_TO_M = 0.001;

export function createBallHistoryLayer(parent, opts = {}) {
  const maxVertices = (opts.maxVertices | 0) & ~1 || 2;
  const colourFn = opts.colourFn || ballColour;
  const group = new THREE.Group();
  group.name = 'ball-history';
  const MIN_CAP = 4096;
  let g = new THREE.BufferGeometry(), pos = null, at = null, col = null, capV = 0;
  // (Re)allocate the three attributes for `c` vertices on a fresh geometry (the old one is disposed: GL buffers can't grow).
  function allocate(c) {
    const ng = new THREE.BufferGeometry();
    if (c > 0) {
      pos = new THREE.BufferAttribute(new Float32Array(c * 3), 3).setUsage(THREE.DynamicDrawUsage);
      at = new THREE.BufferAttribute(new Float32Array(c), 1).setUsage(THREE.DynamicDrawUsage);
      col = new THREE.BufferAttribute(new Float32Array(c * 3), 3).setUsage(THREE.DynamicDrawUsage);
      ng.setAttribute('position', pos); ng.setAttribute('aT', at); ng.setAttribute('aColor', col);
    } else { pos = at = col = null; }
    ng.setDrawRange(0, 0);
    g.dispose(); g = ng; capV = c;
    if (obj) obj.geometry = g;
  }
  function ensure(n) {
    if (n <= capV) return;
    let c = Math.max(MIN_CAP, capV);
    while (c < n) c *= 2;
    allocate(Math.min(c, maxVertices));
  }
  let obj = null;
  const mat = new THREE.ShaderMaterial({
    transparent: true, depthWrite: false,
    uniforms: { uNow: { value: 0 }, uTail: { value: 1 }, uFloor: { value: ALPHA_FLOOR } },
    vertexShader: `attribute float aT; attribute vec3 aColor; varying float vT; varying vec3 vC;
      void main(){ vT = aT; vC = aColor; gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0); }`,
    fragmentShader: `uniform float uNow, uTail, uFloor; varying float vT; varying vec3 vC;
      void main(){ float age = uNow - vT; if (age < -1e-4 || age > uTail) discard;
        gl_FragColor = vec4(vC, max(uFloor, 1.0 - (1.0 - uFloor) * age / uTail)); }`,
  });
  obj = new THREE.LineSegments(g, mat);
  obj.frustumCulled = false;
  obj.visible = false;
  group.add(obj);
  parent.add(group);

  const tmpColor = new THREE.Color();
  let epoch = 0, vertices = 0, segments = 0, slotsN = 0, droppedSlots = 0, droppedSegments = 0, rebuilds = 0;

  // Segment count of one slot given the previous sample per id (Map id -> t); updates `lastT`.
  function countSlot(r, lastT) {
    let s = 0;
    for (let k = 0; k < r.n; k++) {
      const id = r.id[k], p = lastT.get(id);
      if (p !== undefined && r.t[k] - p <= MAX_GAP_S) s++;
      lastT.set(id, r.t[k]);
    }
    return s;
  }

  const layer = {
    group,
    setSlots(rowsIn) {
      rebuilds++;
      const rows = Array.from(rowsIn).filter((r) => r && r.n > 0).sort((a, b) => a.t[0] - b.t[0]);
      // pass 1: per-slot segment counts (a join across a slot boundary belongs to the later slot)
      const lastT = new Map();
      const cnt = rows.map((r) => countSlot(r, lastT));
      let total = 0;
      for (const c of cnt) total += c;
      let first = 0;
      const cap = maxVertices >> 1;
      droppedSlots = 0; droppedSegments = 0;
      while (total > cap && first < rows.length) { total -= cnt[first]; droppedSegments += cnt[first]; first++; droppedSlots++; }
      const kept = rows.slice(first);
      epoch = kept.length ? kept[0].t[0] : 0;
      if (total === 0) { layer.clear(); return; }
      ensure(total * 2);
      // pass 2: write
      const P = pos.array, T = at.array, C = col.array;
      const prev = new Map();   // id -> [t, x, y, z] (Three metres) of the previous sample
      let v = 0;
      for (const r of kept) {
        for (let k = 0; k < r.n; k++) {
          const id = r.id[k], t = r.t[k];
          const x = r.x[k] * MM_TO_M, y = r.z[k] * MM_TO_M, z = -r.y[k] * MM_TO_M;
          const p = prev.get(id);
          if (p !== undefined && t - p[0] <= MAX_GAP_S && v + 2 <= capV) {
            tmpColor.setHex(colourFn(id));
            for (let e = 0; e < 2; e++) {
              const j = v + e;
              P[3 * j] = e ? x : p[1]; P[3 * j + 1] = e ? y : p[2]; P[3 * j + 2] = e ? z : p[3];
              T[j] = (e ? t : p[0]) - epoch;
              C[3 * j] = tmpColor.r; C[3 * j + 1] = tmpColor.g; C[3 * j + 2] = tmpColor.b;
            }
            v += 2;
          }
          if (p === undefined) prev.set(id, [t, x, y, z]);
          else { p[0] = t; p[1] = x; p[2] = y; p[3] = z; }
        }
      }
      vertices = v; segments = v >> 1; slotsN = kept.length;
      g.setDrawRange(0, v);
      pos.needsUpdate = true; at.needsUpdate = true; col.needsUpdate = true;
      if (v === 0) obj.visible = false;
    },
    render(nowSec, tailMs) {
      if (!(tailMs > 0) || vertices === 0) { obj.visible = false; return 0; }
      obj.visible = true;
      const u = mat.uniforms;
      u.uNow.value = nowSec - epoch; u.uTail.value = tailMs / 1000;
      return vertices;
    },
    clear() {
      vertices = 0; segments = 0; slotsN = 0; droppedSlots = 0; droppedSegments = 0;
      if (capV > 0) allocate(0);           // release the buffers (CPU and GL)
      obj.visible = false;
    },
    stats() { return { vertices, maxVertices, capacity: capV, segments, slots: slotsN, droppedSlots, droppedSegments, epoch, rebuilds }; },
    dispose() { g.dispose(); mat.dispose(); parent.remove(group); },
  };
  return layer;
}
