/**
 * ball-history-layer.js - the far-tier ball flights (Phase 5b) and labelled marker tracks (Phase 6): thin lines for every
 * loaded ball / labelled-marker sample pair, faded by age over the whole tail.  Pure Three.js, no DOM.
 *
 *   const layer = createBallHistoryLayer(parent, {maxVertices, colourFn?, markerColourFn?});
 *   layer.setSlots(rows[], labels?)  rows = replay/ball-history.js slot rows {i, n, t:Float64Array(s), id, x, y, z (robot mm)} plus
 *                               the optional marker section {mn, mt, mid, mx, my, mz} (mid indexes `labels`, the interned label
 *                               strings; colour = markerColourFn(label), default labelToColour), any order;
 *                               rows are matched by OBJECT across calls: called with the previous rows plus new ones it
 *                               APPENDS (writes and uploads only the new slots); anything else (an eviction, a cap drop, a
 *                               buffer grow) rebuilds.  Called at most once per frame, only when the set changed.
 *   layer.render(nowSec, tailMs)  uniforms ONLY (no buffer write, no allocation); hides the line when tailMs <= 0 or empty
 *   layer.clear()               drop everything
 *   layer.stats()               {vertices, maxVertices, capacity, segments, slots, droppedSlots, droppedSegments, droppedMarker*,
 *                               epoch, rebuilds (setSlots calls), fullUploads, uploadedVertices (cumulative, both geometries)}
 *   layer.dispose()
 *
 * Two LineSegments geometries sharing one material: the MAIN buffer holds each slot's intra-slot segments in arrival
 * order, append-only, and uploads only the appended range (Three re-uploads the whole CAPACITY when no update range is
 * set); a small JOINS geometry holds the segments across slot boundaries and is rewritten on every call.  Measured
 * 2026-10-11 (busy bag, tail = whole recording, tools/probes/replay_lane_trace.mjs): the old rebuild-and-re-upload-all
 * moved 1.5 GB through bufferSubData over one fill, O(slots^2).  A segment joins two consecutive samples of the SAME id (across slot boundaries) when their
 * dt <= MAX_GAP_S (0.15 s, the trail feed's ball-staleness D1).  Per-vertex attributes: position (Three metres,
 * robot (x, y, z) mm -> (x, z, -y) * 0.001), aT (float32 seconds relative to `epoch` = the earliest loaded sample, re-based
 * on every full rebuild; an append keeps it, so an older slot's aT is negative) and aColor (ballColour(id), linear-agnostic like the ribbons).  The shader discards samples after the
 * playhead or older than the tail and fades alpha linearly with age down to ALPHA_FLOOR so the oldest flights stay legible.
 * maxVertices is a CAP, not an allocation: the buffers are sized to the loaded flights (power of two, >= 4096
 * vertices, doubling as slots arrive) and released by clear(), so a live session or a short tail holds nothing and a
 * rebuild uploads only the used prefix (review 2026-10-11: an eager 28 MB allocation re-uploaded whole on every arrival).
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
  // Boundary joins (segments across a slot boundary) live in their own small geometry, rewritten on every setSlots, so
  // the main buffer stays append-only while slots arrive in any order.  Same material, same uniforms.
  let jg = new THREE.BufferGeometry(), jPos = null, jAt = null, jCol = null, jCap = 0, jv = 0;
  const jObj = new THREE.LineSegments(jg, mat);
  jObj.frustumCulled = false;
  jObj.visible = false;
  function allocJoins(c) {
    const ng = new THREE.BufferGeometry();
    if (c > 0) {
      jPos = new THREE.BufferAttribute(new Float32Array(c * 3), 3).setUsage(THREE.DynamicDrawUsage);
      jAt = new THREE.BufferAttribute(new Float32Array(c), 1).setUsage(THREE.DynamicDrawUsage);
      jCol = new THREE.BufferAttribute(new Float32Array(c * 3), 3).setUsage(THREE.DynamicDrawUsage);
      ng.setAttribute('position', jPos); ng.setAttribute('aT', jAt); ng.setAttribute('aColor', jCol);
    } else { jPos = jAt = jCol = null; }
    ng.setDrawRange(0, 0);
    jg.dispose(); jg = ng; jCap = c; jObj.geometry = jg;
  }
  group.add(jObj);
  parent.add(group);

  const tmpColor = new THREE.Color();
  let epoch = 0, vertices = 0, segments = 0, slotsN = 0, droppedSlots = 0, droppedSegments = 0, droppedMarkerSlots = 0, droppedMarkerSegments = 0, rebuilds = 0;
  let mainV = 0, fullUploads = 0, uploadedVertices = 0;
  const drawn = new Map();               // row object -> its marker rows were drawn (main buffer, in arrival order)
  const colB = new Map(), colM = new Map();   // colour caches (ball id / label index -> [r, g, b])

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

  // Per-row facts, cached by row OBJECT (replay/ball-history.js keeps one row object per slot for as long as the slot is
  // loaded): intra-slot segment counts per stream and each key's first / last sample index, so an arrival costs its own
  // slot plus the boundary joins, never a re-scan of the slots already drawn.
  const rowInfo = new WeakMap();
  function endsOf(n, ids) {
    const first = new Map(), last = new Map();
    for (let k = 0; k < n; k++) { if (!first.has(ids[k])) first.set(ids[k], k); last.set(ids[k], k); }
    return { first, last };
  }
  function infoOf(r) {
    let f = rowInfo.get(r);
    if (!f) {
      const nb = r.n | 0, nm = r.mn | 0;
      f = { cB: nb > 0 ? countStream(nb, r.t, r.id, new Map()) : 0, cM: nm > 0 ? countStream(nm, r.mt, r.mid, new Map()) : 0,
        eB: endsOf(nb, r.id), eM: endsOf(nm, r.mid) };
      rowInfo.set(r, f);
    }
    return f;
  }
  // Boundary joins: for every row (in time order) and key, its first sample against the key's last sample in an earlier
  // row (dt <= MAX_GAP_S).  emit(rowIndex, kind, colourKey, [t, x, y, z] earlier, [t, x, y, z] later) per join.
  function walkJoins(rows, useM, emit) {
    const lastB = new Map(), lastM = new Map();   // key -> [row, k]
    const at3 = (xs, ys, zs, ts, k) => [ts[k], xs[k] * MM_TO_M, zs[k] * MM_TO_M, -ys[k] * MM_TO_M];
    for (let ri = 0; ri < rows.length; ri++) {
      const r = rows[ri], f = infoOf(r);
      for (const [id, k] of f.eB.first) {
        const p = lastB.get(id);
        if (p && r.t[k] - p[0].t[p[1]] <= MAX_GAP_S) emit(ri, 0, id, at3(p[0].x, p[0].y, p[0].z, p[0].t, p[1]), at3(r.x, r.y, r.z, r.t, k));
      }
      for (const [id, k] of f.eB.last) lastB.set(id, [r, k]);
      if (!useM(ri)) continue;
      for (const [id, k] of f.eM.first) {
        const p = lastM.get(id);
        if (p && r.mt[k] - p[0].mt[p[1]] <= MAX_GAP_S) emit(ri, 1, id, at3(p[0].mx, p[0].my, p[0].mz, p[0].mt, p[1]), at3(r.mx, r.my, r.mz, r.mt, k));
      }
      for (const [id, k] of f.eM.last) lastM.set(id, [r, k]);
    }
  }
  // Attribute ranges (elements) the next render uploads; with none set Three re-uploads the WHOLE capacity.
  function markRange(v0, nV) {
    pos.addUpdateRange(3 * v0, 3 * nV); at.addUpdateRange(v0, nV); col.addUpdateRange(3 * v0, 3 * nV);
    pos.needsUpdate = true; at.needsUpdate = true; col.needsUpdate = true;
    uploadedVertices += nV;
  }
  function writeJoins(rows, useM, lab) {
    const segs = [];
    walkJoins(rows, useM, (ri, kind, id, a, b) => segs.push(kind, id, a, b));
    const n = (segs.length / 4) * 2;
    const fresh = n > jCap;
    if (fresh) allocJoins(Math.max(256, 1 << Math.ceil(Math.log2(n))));   // allocJoins disposes the old geometry
    if (jCap === 0) { jv = 0; return; }
    const P = jPos.array, T = jAt.array, C = jCol.array;
    const colB = new Map(), colM = new Map();
    let v = 0;
    for (let s4 = 0; s4 < segs.length; s4 += 4) {
      const kind = segs[s4], id = segs[s4 + 1];
      const cache = kind ? colM : colB;
      let c = cache.get(id);
      if (c === undefined) { tmpColor.setHex(kind ? markerColourFn(lab[id]) : colourFn(id)); c = [tmpColor.r, tmpColor.g, tmpColor.b]; cache.set(id, c); }
      for (let e = 0; e < 2; e++) {
        const q = segs[s4 + 2 + e], j = v + e;
        P[3 * j] = q[1]; P[3 * j + 1] = q[2]; P[3 * j + 2] = q[3]; T[j] = q[0] - epoch;
        C[3 * j] = c[0]; C[3 * j + 1] = c[1]; C[3 * j + 2] = c[2];
      }
      v += 2;
    }
    jv = v;
    jg.setDrawRange(0, v);
    if (v > 0) {
      if (!fresh) { jPos.addUpdateRange(0, 3 * v); jAt.addUpdateRange(0, v); jCol.addUpdateRange(0, 3 * v); }   // fresh: see the main rebuild
      jPos.needsUpdate = true; jAt.needsUpdate = true; jCol.needsUpdate = true;
      uploadedVertices += fresh ? jCap : v;
    }
  }
  // Write one kept row's intra-slot segments (balls, then its marker rows if kept) at mainV; returns vertices written.
  function writeRow(r, withM, mColour) {
    const v0 = mainV;
    if ((r.n | 0) > 0) mainV = writeStream(r.n, r.t, r.id, r.x, r.y, r.z, new Map(), colB, colourFn, mainV);
    if (withM && (r.mn | 0) > 0) mainV = writeStream(r.mn, r.mt, r.mid, r.mx, r.my, r.mz, new Map(), colM, mColour, mainV);
    return mainV - v0;
  }

  const layer = {
    group,
    setSlots(rowsIn, labels) {
      rebuilds++;
      const lab = labels || [];
      const rows = Array.from(rowsIn).filter((r) => r && ((r.n | 0) > 0 || (r.mn | 0) > 0)).sort((a, b) => t0Of(a) - t0Of(b));
      // per-slot segment counts per stream: intra-slot plus the joins INTO the slot (a join across a slot boundary
      // belongs to the later slot)
      const cntB = rows.map((r) => infoOf(r).cB), cntM = rows.map((r) => infoOf(r).cM);
      walkJoins(rows, () => true, (ri, kind) => { if (kind) cntM[ri]++; else cntB[ri]++; });
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
      if (total === 0) { const dm = droppedMarkerSlots, dms = droppedMarkerSegments, ds = droppedSlots, dg = droppedSegments; layer.clear(); droppedMarkerSlots = dm; droppedMarkerSegments = dms; droppedSlots = ds; droppedSegments = dg; return; }
      const withM = (ri) => first + ri >= firstM;
      const mColour = (li) => markerColourFn(lab[li]);
      // Append-only when every drawn row is still kept with the same marker choice and the new rows fit the buffer;
      // otherwise rebuild (an eviction, a cap drop, a buffer grow).  An append keeps the old epoch: an older slot's aT
      // is simply negative.
      let added = [], append = capV > 0 && drawn.size > 0, need = 0;
      if (append) {
        let still = 0;
        for (let ri = 0; ri < kept.length; ri++) {
          const r = kept[ri], d = drawn.get(r);
          if (d === undefined) { added.push(ri); need += 2 * (infoOf(r).cB + (withM(ri) ? infoOf(r).cM : 0)); } else if (d !== withM(ri)) { append = false; break; } else still++;
        }
        if (still !== drawn.size || mainV + need > capV) append = false;
      }
      if (append) {
        for (const ri of added) {
          const v0 = mainV, n = writeRow(kept[ri], withM(ri), mColour);
          drawn.set(kept[ri], withM(ri));
          if (n > 0) markRange(v0, n);
        }
      } else {
        fullUploads++;
        drawn.clear(); colB.clear(); colM.clear();
        epoch = kept.length ? t0Of(kept[0]) : 0;
        let intra = 0;
        for (let ri = 0; ri < kept.length; ri++) intra += infoOf(kept[ri]).cB + (withM(ri) ? infoOf(kept[ri]).cM : 0);
        const capWas = capV;
        ensure(intra * 2);
        mainV = 0;
        if (capV > 0) { pos.clearUpdateRanges(); at.clearUpdateRanges(); col.clearUpdateRanges(); }
        for (let ri = 0; ri < kept.length; ri++) { writeRow(kept[ri], withM(ri), mColour); drawn.set(kept[ri], withM(ri)); }
        // A fresh allocation's first upload is a whole-capacity bufferData and r170 does not clear pending ranges there,
        // so a range set now would be re-sent merged with the next append: mark it dirty without a range instead.
        if (mainV > 0 && capV !== capWas) { pos.needsUpdate = true; at.needsUpdate = true; col.needsUpdate = true; uploadedVertices += capV; }
        else if (mainV > 0) markRange(0, mainV);
      }
      g.setDrawRange(0, mainV);
      writeJoins(kept, withM, lab);
      vertices = mainV + jv; segments = vertices >> 1; slotsN = kept.length;
      if (vertices === 0) { obj.visible = false; jObj.visible = false; }
    },
    render(nowSec, tailMs) {
      if (!(tailMs > 0) || vertices === 0) { obj.visible = false; jObj.visible = false; return 0; }
      obj.visible = mainV > 0; jObj.visible = jv > 0;
      const u = mat.uniforms;
      u.uNow.value = nowSec - epoch; u.uTail.value = tailMs / 1000;
      return vertices;
    },
    clear() {
      vertices = 0; segments = 0; slotsN = 0; droppedSlots = 0; droppedSegments = 0; droppedMarkerSlots = 0; droppedMarkerSegments = 0;
      mainV = 0; jv = 0; drawn.clear(); colB.clear(); colM.clear();
      if (capV > 0) allocate(0);           // release the buffers (CPU and GL)
      if (jCap > 0) allocJoins(0);
      obj.visible = false; jObj.visible = false;
    },
    stats() { return { vertices, maxVertices, capacity: capV, segments, slots: slotsN, droppedSlots, droppedSegments, droppedMarkerSlots, droppedMarkerSegments, epoch, rebuilds, fullUploads, uploadedVertices }; },
    dispose() { g.dispose(); jg.dispose(); mat.dispose(); parent.remove(group); },
  };
  return layer;
}
