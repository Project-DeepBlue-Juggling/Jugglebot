/**
 * ball-history-layer.js - the far-tier ball flights (Phase 5b) and labelled marker tracks (Phase 6): thin lines for every
 * loaded ball / labelled-marker sample pair, faded by age over the whole tail.  Pure Three.js, no DOM.
 *
 *   const layer = createBallHistoryLayer(parent, {maxVertices, colourFn?, markerColourFn?});
 *   layer.setSlots(rows[], labels?)  rows = replay/ball-history.js slot rows {i, n, t:Float64Array(s), id, x, y, z (robot mm)} plus
 *                               the optional marker section {mn, mt, mid, mx, my, mz} (mid indexes `labels`, the interned label
 *                               strings; colour = markerColourFn(label), default labelToColour), any order;
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
 * Marker tracks join exactly like balls (own key space: a label index never joins a ball id) and are coloured per label by
 * labelToColour(label), the same palette as their near-tier ribbons.
 * Cap policy (the cap is shared by balls and markers; busy bag, 18 min: ~63k ball vertices + ~16 persistent labels x 20 Hz
 * x 1080 s x 2 = ~690k marker vertices = ~755k < 1M, so the whole recording fits): over the cap the OLDEST slots' MARKER
 * rows are dropped first (counted in droppedMarkerSlots / droppedMarkerSegments), keeping the balls, which are the throw-
 * consistency signal; only if the balls alone still overflow are whole oldest slots dropped (droppedSlots), as before.
 */
import * as THREE from 'three';
import { ballColour, labelToColour } from './marker-palette.js';

export const MAX_GAP_S = 0.15;
export const ALPHA_FLOOR = 0.15;
const MM_TO_M = 0.001;

export function createBallHistoryLayer(parent, opts = {}) {
  const maxVertices = (opts.maxVertices | 0) & ~1 || 2;
  const colourFn = opts.colourFn || ballColour;
  const markerColourFn = opts.markerColourFn || labelToColour;
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
  let epoch = 0, vertices = 0, segments = 0, slotsN = 0, droppedSlots = 0, droppedSegments = 0, droppedMarkerSlots = 0, droppedMarkerSegments = 0, rebuilds = 0;

  // Segment count of one stream (n samples of keys `ids` at times `ts`) given the previous sample per key (Map key -> t); updates `lastT`.
  function countStream(n, ts, ids, lastT) {
    let s = 0;
    for (let k = 0; k < n; k++) {
      const id = ids[k], p = lastT.get(id);
      if (p !== undefined && ts[k] - p <= MAX_GAP_S) s++;
      lastT.set(id, ts[k]);
    }
    return s;
  }
  const t0Of = (r) => (r.n > 0 ? (r.mn > 0 ? Math.min(r.t[0], r.mt[0]) : r.t[0]) : r.mt[0]);

  // Write one stream's segments from index v; returns the new v.  colourOf(key) -> 0xRRGGBB (cached per key by the caller).
  function writeStream(n, ts, ids, xs, ys, zs, prev, colourCache, colourOf, v) {
    const P = pos.array, T = at.array, C = col.array;
    for (let k = 0; k < n; k++) {
      const id = ids[k], t = ts[k];
      const x = xs[k] * MM_TO_M, y = zs[k] * MM_TO_M, z = -ys[k] * MM_TO_M;
      const p = prev.get(id);
      if (p !== undefined && t - p[0] <= MAX_GAP_S && v + 2 <= capV) {
        let c = colourCache.get(id);
        if (c === undefined) { tmpColor.setHex(colourOf(id)); c = [tmpColor.r, tmpColor.g, tmpColor.b]; colourCache.set(id, c); }
        for (let e = 0; e < 2; e++) {
          const j = v + e;
          P[3 * j] = e ? x : p[1]; P[3 * j + 1] = e ? y : p[2]; P[3 * j + 2] = e ? z : p[3];
          T[j] = (e ? t : p[0]) - epoch;
          C[3 * j] = c[0]; C[3 * j + 1] = c[1]; C[3 * j + 2] = c[2];
        }
        v += 2;
      }
      if (p === undefined) prev.set(id, [t, x, y, z]);
      else { p[0] = t; p[1] = x; p[2] = y; p[3] = z; }
    }
    return v;
  }

  const layer = {
    group,
    setSlots(rowsIn, labels) {
      rebuilds++;
      const lab = labels || [];
      const rows = Array.from(rowsIn).filter((r) => r && ((r.n | 0) > 0 || (r.mn | 0) > 0)).map((r) => (r.mn > 0 ? r : Object.assign({}, r, { mn: 0 })))
        .sort((a, b) => t0Of(a) - t0Of(b));
      // pass 1: per-slot segment counts per stream (a join across a slot boundary belongs to the later slot)
      const lastB = new Map(), lastM = new Map();
      const cntB = rows.map((r) => (r.n > 0 ? countStream(r.n, r.t, r.id, lastB) : 0));
      const cntM = rows.map((r) => (r.mn > 0 ? countStream(r.mn, r.mt, r.mid, lastM) : 0));
      let total = 0;
      for (let i = 0; i < rows.length; i++) total += cntB[i] + cntM[i];
      const cap = maxVertices >> 1;
      droppedSlots = 0; droppedSegments = 0; droppedMarkerSlots = 0; droppedMarkerSegments = 0;
      let firstM = 0;   // marker rows of slots below firstM are dropped first
      while (total > cap && firstM < rows.length) {
        if (cntM[firstM] > 0) { total -= cntM[firstM]; droppedMarkerSegments += cntM[firstM]; droppedMarkerSlots++; cntM[firstM] = 0; }
        firstM++;
      }
      let first = 0;
      while (total > cap && first < rows.length) { total -= cntB[first] + cntM[first]; droppedSegments += cntB[first] + cntM[first]; first++; droppedSlots++; }
      const kept = rows.slice(first);
      epoch = kept.length ? t0Of(kept[0]) : 0;
      if (total === 0) { const dm = droppedMarkerSlots, dms = droppedMarkerSegments, ds = droppedSlots, dg = droppedSegments; layer.clear(); droppedMarkerSlots = dm; droppedMarkerSegments = dms; droppedSlots = ds; droppedSegments = dg; return; }
      ensure(total * 2);
      // pass 2: write
      const prevB = new Map(), prevM = new Map();   // key -> [t, x, y, z] (Three metres) of the previous sample
      const colB = new Map(), colM = new Map();
      const mColour = (li) => markerColourFn(lab[li]);
      let v = 0;
      for (let ri = 0; ri < kept.length; ri++) {
        const r = kept[ri];
        if (r.n > 0) v = writeStream(r.n, r.t, r.id, r.x, r.y, r.z, prevB, colB, colourFn, v);
        if (r.mn > 0 && first + ri >= firstM) v = writeStream(r.mn, r.mt, r.mid, r.mx, r.my, r.mz, prevM, colM, mColour, v);
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
      vertices = 0; segments = 0; slotsN = 0; droppedSlots = 0; droppedSegments = 0; droppedMarkerSlots = 0; droppedMarkerSegments = 0;
      if (capV > 0) allocate(0);           // release the buffers (CPU and GL)
      obj.visible = false;
    },
    stats() { return { vertices, maxVertices, capacity: capV, segments, slots: slotsN, droppedSlots, droppedSegments, droppedMarkerSlots, droppedMarkerSegments, epoch, rebuilds }; },
    dispose() { g.dispose(); mat.dispose(); parent.remove(group); },
  };
  return layer;
}
