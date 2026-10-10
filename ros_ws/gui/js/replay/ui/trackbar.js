/**
 * trackbar.js — replay trackbar + transport + hotkeys (layout A, docked in #replay-dock).
 *
 * createTrackbar({document, mode, dock, raf?}) -> {mount, unmount, mounted, frame, handleKey, ...}
 *
 * Mounted while the mode is OPENING/REPLAY (mode 'state' events), unmounted on exit. One rAF loop reads
 * `engine.state()` and moves the playhead / frontier / readouts, writing a style or text only when its
 * value changed (no per-frame allocation in steady state). The overview strip is drawn once per
 * (timeline, zoom range). The engine is looked up lazily (`mode.engine()`): it does not exist while OPENING.
 */
import { h } from './dom.js';
import {
    fmtClock, frac, timeAt, clampRange, zoomAbout, dragToRange, renderOverview, MIN_SPAN, ZOOM_STEP,
} from './overview.js';

/** Signed speed ladder, slowest reverse .. fastest forward (the engine snaps to the same magnitudes). */
export const LADDER = [0.25, 0.5, 1, 2, 4, 8];

/** FF / RW ladder stepping. Forward: faster up to x8. RW from forward enters reverse at x1, then faster
 *  reverse; FF from reverse slows the reverse down, and from -0.25 returns to forward x1. */
export function nextSpeed(cur, dir) {
    const mag = Math.abs(cur) || 1;
    let i = LADDER.indexOf(mag);
    if (i < 0) i = LADDER.indexOf(1);
    const up = Math.min(LADDER.length - 1, i + 1), dn = i - 1;
    if (dir > 0) {                                  // FF
        if (cur >= 0) return LADDER[up];
        return dn >= 0 ? -LADDER[dn] : 1;
    }
    if (cur > 0) return -1;                         // RW from forward: reverse at x1
    return -LADDER[up];
}

/** A failing / partial /overview is retried no more often than every this many frames. */
const TL_RETRY_FRAMES = 200;
/** A bar drag whose pointer rests this many frames (~150 ms at 60 Hz) runs the full seek once. */
const REST_FRAMES = 9;

const fmtSpeed = (s) => (s < 0 ? '-' : '') + '×' + Math.abs(s);
const p2 = (n) => (n < 10 ? '0' : '') + n;
function fmtMMSS(sec) {
    const s = Math.max(0, Math.floor(sec + 1e-9));
    const hh = Math.floor(s / 3600), m = Math.floor((s % 3600) / 60), r = s % 60;
    return hh > 0 ? hh + ':' + p2(m) + ':' + p2(r) : p2(m) + ':' + p2(r);
}
function fmtDate(t) {
    if (!(t > 1e8)) return '';
    const d = new Date(t * 1000);
    return d.getFullYear() + '-' + p2(d.getMonth() + 1) + '-' + p2(d.getDate());
}
function isTextTarget(t) {
    if (!t) return false;
    const tag = String(t.tag || t.tagName || '').toLowerCase();
    return tag === 'input' || tag === 'textarea' || tag === 'select' || !!t.isContentEditable;
}

export function createTrackbar(deps) {
    const doc = deps.document;
    const mode = deps.mode;
    const dock = deps.dock;
    const raf = deps.raf || null;                 // {request(cb)->id, cancel(id)}

    let root = null, els = null;
    let isMounted = false;
    let rafId = null;
    let view = null;                              // null = full range, else {v0, v1}
    let timeline = null, tlToken = 0, tlFetching = false;
    let unsubSrc = null;                          // source.onChange -> re-read a partial timeline at once (no 200-frame wait)
    let frameNo = 0, tlRetryAt = 0;               // frame-count backoff for /overview retries (no wall clock)
    let drawnKey = '';
    const cache = { head: '', play: '', el: '', speed: '', mode: '', zoom: '', date: '', pbtn: '' };
    let dragging = null;                          // {kind:'bar', rest, rested, t} | {kind:'ov', fa}
    let pendingScrub = null;                      // latest bar-drag time, flushed once per frame (latest wins)
    let lastFull = { t0: 0, t1: 0 };
    let unsubMode = null;

    const engine = () => (mode.engine ? mode.engine() : null);

    function fullRange(r) { return { t0: r.t0, t1: Math.max(r.t1, r.frontier) }; }
    function curView(full) { return view || { v0: full.t0, v1: full.t1 }; }

    // ---------------- engine commands ----------------
    function clampT(t, r) { return Math.max(r.t0, Math.min(r.frontier > r.t0 ? r.frontier : r.t1, t)); }
    function seekBy(dt) {
        const e = engine(); if (!e) return;
        const s = e.state();
        e.seek(clampT(s.playhead + dt, s.range));
    }
    function togglePlay() {
        const e = engine(); if (!e) return;
        const m = e.state().mode;
        if (m === 'playing' || m === 'buffering') e.pause(); else e.play();
    }
    function ladder(dir) {
        const e = engine(); if (!e) return;
        e.setSpeed(nextSpeed(e.state().speed, dir));
        e.play();
    }
    function stepRec(n) { const e = engine(); if (e) e.step(n); }
    function toStart() { const e = engine(); if (e) e.seek(e.state().range.t0); }
    function toEnd() { const e = engine(); if (e) { const r = e.state().range; e.seek(clampT(r.t1, r)); } }

    function setView(v) {
        view = v; drawnKey = '';
    }
    function zoom(dir) {
        const e = engine(); if (!e) return;
        const s = e.state(); const full = fullRange(s.range);
        const next = zoomAbout(curView(full), dir > 0 ? ZOOM_STEP : 1 / ZOOM_STEP, s.playhead, full);
        setView(next.v1 - next.v0 >= full.t1 - full.t0 - 1e-9 ? null : next);
    }
    function resetZoom() { setView(null); }

    /** Capture-phase document listener: the live GUI's own shortcuts (chart-pause on Space) are registered on
     *  the same document in the bubble phase, so a key the trackbar handles must not reach them. */
    function onKeyCapture(ev) {
        if (handleKey(ev) && ev.stopImmediatePropagation) ev.stopImmediatePropagation();
    }

    /** One keydown. Returns true when handled. */
    function handleKey(ev) {
        if (!isMounted || !mode.isActive()) return false;
        if (isTextTarget(ev.target) || ev.ctrlKey || ev.metaKey || ev.altKey) return false;
        const k = ev.key;
        switch (k) {
            case ' ': case 'Spacebar': togglePlay(); break;
            case 'ArrowLeft': seekBy(ev.shiftKey ? -10 : -1); break;
            case 'ArrowRight': seekBy(ev.shiftKey ? 10 : 1); break;
            case ',': case '<': stepRec(-1); break;
            case '.': case '>': stepRec(1); break;
            case 'Home': toStart(); break;
            case 'End': toEnd(); break;
            case '+': case '=': zoom(1); break;
            case '-': case '_': zoom(-1); break;
            case 'Escape': resetZoom(); break;
            default: return false;
        }
        if (ev.preventDefault) ev.preventDefault();
        return true;
    }

    // ---------------- pointer interaction ----------------
    function fracOf(el, ev) {
        const r = el.getBoundingClientRect();
        return r.width > 0 ? Math.max(0, Math.min(1, (ev.clientX - r.left) / r.width)) : 0;
    }
    function barTime(ev) {
        const e = engine(); if (!e) return null;
        const s = e.state(); const full = fullRange(s.range); const v = curView(full);
        return clampT(timeAt(fracOf(els.bar, ev), v.v0, v.v1), s.range);
    }
    function onBarDown(ev) {
        const e = engine(); if (!e) return;
        dragging = { kind: 'bar', rest: 0, rested: false, t: null };
        queueScrub(barTime(ev));
        if (ev.preventDefault) ev.preventDefault();
    }
    /** Drag positions coalesce to the latest per animation frame (frame() flushes one engine.scrub). */
    function queueScrub(t) {
        if (t === null || !dragging) return;
        pendingScrub = t; dragging.t = t; dragging.rest = 0; dragging.rested = false;
    }
    function flushScrub(e) {
        if (pendingScrub === null) return;
        const t = pendingScrub; pendingScrub = null;
        e.scrub(t);
    }
    function onOvDown(ev) {
        if (!engine()) return;
        dragging = { kind: 'ov', fa: fracOf(els.ov, ev) };
        if (ev.preventDefault) ev.preventDefault();
    }
    function onMove(ev) {
        if (!dragging) return;
        if (dragging.kind === 'bar') { queueScrub(barTime(ev)); return; }
        const fb = fracOf(els.ov, ev);
        els.sel.hidden = false;
        els.sel.style.left = (Math.min(dragging.fa, fb) * 100).toFixed(2) + '%';
        els.sel.style.width = (Math.abs(fb - dragging.fa) * 100).toFixed(2) + '%';
    }
    function onUp(ev) {
        if (!dragging) return;
        const d = dragging; dragging = null; pendingScrub = null;
        const e = engine(); if (!e) return;
        if (d.kind === 'bar') { e.seek(barTime(ev)); return; }
        els.sel.hidden = true;
        const s = e.state(); const full = fullRange(s.range); const v = curView(full);
        const fb = fracOf(els.ov, ev);
        const nv = dragToRange(d.fa, fb, v, full, 0.01);
        if (nv) setView(nv);
        else e.seek(clampT(timeAt(fb, v.v0, v.v1), s.range));   // a click on the overview seeks
    }

    // ---------------- DOM ----------------
    function build() {
        const btn = (id, text, title, fn) => h(doc, 'button', { cls: 'replay-btn rp-btn', id, text, title, on: { click: fn } });
        const bPlay = btn('rp-play', 'Play', 'Play / pause (Space)', togglePlay);
        els = {
            date: h(doc, 'span', { cls: 'rp-date' }),
            play: h(doc, 'span', { cls: 'rp-clock', id: 'rp-clock' }),
            el: h(doc, 'span', { cls: 'rp-elapsed', id: 'rp-elapsed' }),
            speed: h(doc, 'span', { cls: 'rp-speed', id: 'rp-speed' }),
            buf: h(doc, 'span', { cls: 'rp-buffering', id: 'rp-buffering', text: 'buffering', hidden: true }),
            zoomtag: h(doc, 'span', { cls: 'rp-zoomtag', id: 'rp-zoomtag' }),
            pbtn: bPlay,
            head: h(doc, 'div', { cls: 'rp-playhead' }),
            sel: h(doc, 'div', { cls: 'rp-sel', hidden: true }),
        };
        els.bar = h(doc, 'div', { cls: 'rp-bar', id: 'rp-bar', on: { mousedown: onBarDown } }, [els.head]);
        els.ovStrip = h(doc, 'div', { cls: 'rp-ov-strip' });
        els.ov = h(doc, 'div', { cls: 'rp-ov', id: 'rp-ov', on: { mousedown: onOvDown, dblclick: resetZoom } }, [els.ovStrip, els.sel]);
        const head = h(doc, 'div', { cls: 'rp-head' }, [els.date, els.play, els.el, els.speed, els.buf, els.zoomtag]);
        const ctl = h(doc, 'div', { cls: 'rp-ctl' }, [
            btn('rp-home', '|◀', 'Start (Home)', toStart),
            btn('rp-rw', '◀◀', 'Rewind: reverse speed ladder', () => ladder(-1)),
            btn('rp-stepb', '◀|', 'Back one sample (,)', () => stepRec(-1)),
            bPlay,
            btn('rp-stepf', '|▶', 'Forward one sample (.)', () => stepRec(1)),
            btn('rp-ff', '▶▶', 'Fast-forward: speed ladder', () => ladder(1)),
            btn('rp-end', '▶|', 'End (End)', toEnd),
            btn('rp-exit', 'Exit replay', 'Leave replay', () => { mode.exitReplay('closed'); }),
            h(doc, 'span', { cls: 'rp-keys', text: 'Space play  ←/→ ±1 s  Shift ±10 s  , . sample  + − zoom  Esc reset' }),
        ]);
        root = h(doc, 'div', { cls: 'rp-tb', id: 'rp-tb' }, [head, els.bar, els.ov, ctl]);
    }

    /** The trackbar mounts at OPENING, before the source exists; (re)subscribe as soon as it does. */
    function subscribeSource() {
        if (unsubSrc) return;
        const src = mode.source && mode.source();
        if (!src || typeof src.onChange !== 'function') return;
        unsubSrc = src.onChange(() => { if (!timeline || timeline.partial) fetchTimeline(); });
        fetchTimeline();
    }

    function fetchTimeline() {
        const src = mode.source && mode.source();
        if (!src || !src.timeline || tlFetching) return;
        tlFetching = true;
        tlRetryAt = frameNo + TL_RETRY_FRAMES;
        const tok = tlToken;
        src.timeline().then((tl) => {
            tlFetching = false;
            if (tok !== tlToken || !isMounted) return;
            timeline = tl; drawnKey = '';
        }, () => { tlFetching = false; });
    }

    const pctS = (f) => (Math.max(0, Math.min(1, f)) * 100).toFixed(2) + '%';

    /** The rAF body (also callable directly by tests). Reads engine.state(); touches the DOM only on change. */
    function frame() {
        if (!isMounted) return;
        const e = engine();
        if (!e) return;
        frameNo++;
        flushScrub(e);
        if (dragging && dragging.kind === 'bar' && pendingScrub === null && !dragging.rested && dragging.t !== null
            && ++dragging.rest >= REST_FRAMES) {
            dragging.rested = true;           // pointer rests mid-drag: full pipeline once, drag continues
            e.seek(dragging.t);
        }
        subscribeSource();
        const s = e.state();
        const r = s.range;
        const full = fullRange(r);
        lastFull = full;
        const v = curView(full);
        const px = pctS(frac(s.playhead, v.v0, v.v1));
        if (px !== cache.head) { cache.head = px; els.head.style.left = px; }
        const clock = fmtClock(s.playhead);
        if (clock !== cache.play) { cache.play = clock; els.play.textContent = clock; }
        const elapsed = fmtMMSS(s.playhead - r.t0) + ' / ' + (r.t1 > r.t0 ? fmtMMSS(r.t1 - r.t0) : '?');
        if (elapsed !== cache.el) { cache.el = elapsed; els.el.textContent = elapsed; }
        const sp = fmtSpeed(s.speed);
        if (sp !== cache.speed) { cache.speed = sp; els.speed.textContent = sp; }
        const m = s.mode;
        if (m !== cache.mode) {
            cache.mode = m;
            els.buf.hidden = m !== 'buffering';
            const pb = m === 'paused' ? 'Play' : 'Pause';
            if (pb !== cache.pbtn) { cache.pbtn = pb; els.pbtn.textContent = pb; }
        }
        const date = fmtDate(r.t0);
        if (date !== cache.date) { cache.date = date; els.date.textContent = date; }
        const zt = view ? fmtClock(view.v0) + ' – ' + fmtClock(view.v1) + '  (Esc resets)' : '';
        if (zt !== cache.zoom) { cache.zoom = zt; els.zoomtag.textContent = zt; }
        // overview: once per (timeline, view, full range); a partial timeline (the source is still polling /overview) is re-read, rate-limited
        const key = (timeline ? 1 : 0) + '|' + v.v0 + '|' + v.v1;
        if (key !== drawnKey) {
            drawnKey = key;
            if (timeline) renderOverview(doc, els.ovStrip, timeline, v);
        }
        if ((!timeline || timeline.partial) && frameNo >= tlRetryAt) fetchTimeline();
    }

    function loop() {
        rafId = null;
        if (!isMounted) return;
        frame();
        if (raf) rafId = raf.request(loop);
    }

    // ---------------- mount / unmount (driven by the mode's state events) ----------------
    function mount() {
        if (isMounted) return;
        if (!root) build();
        dock.replaceChildren();
        dock.appendChild(root);
        isMounted = true;
        timeline = null; drawnKey = ''; view = null; tlToken++; frameNo = 0; tlRetryAt = 0;
        for (const k of Object.keys(cache)) cache[k] = '';
        doc.addEventListener('keydown', onKeyCapture, true);
        doc.addEventListener('mousemove', onMove);
        doc.addEventListener('mouseup', onUp);
        subscribeSource();
        fetchTimeline();
        frame();
        if (raf && rafId === null) rafId = raf.request(loop);
    }
    function unmount() {
        if (!isMounted) return;
        isMounted = false;
        tlToken++;
        if (unsubSrc) { unsubSrc(); unsubSrc = null; }
        dragging = null; pendingScrub = null;
        doc.removeEventListener('keydown', onKeyCapture, true);
        doc.removeEventListener('mousemove', onMove);
        doc.removeEventListener('mouseup', onUp);
        if (rafId !== null && raf) raf.cancel(rafId);
        rafId = null;
        dock.replaceChildren();
    }
    function sync() { if (mode.isActive()) mount(); else unmount(); }

    unsubMode = mode.on('state', sync);
    sync();

    return {
        mount, unmount, frame, handleKey, sync,
        mounted: () => isMounted,
        getView: () => view,
        dispose() { unmount(); if (unsubMode) unsubMode(); },
    };
}
