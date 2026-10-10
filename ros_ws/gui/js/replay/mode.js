/**
 * mode.js — the replay mode state machine (replay design § 8, § 3 pre-notify exit).
 *
 *   LIVE (connected; replay unreachable) -> disconnect -> LOBBY -> enterReplay -> OPENING
 *     -> REPLAY{paused|playing|buffering} -> exitReplay -> LOBBY
 *   a 'connected' edge in OPENING/REPLAY -> exit BEFORE main's connection listener runs -> LIVE + notice
 *
 * Every collaborator is injected (`createReplayMode(deps)`), so a node harness can spy on the
 * ordering; replay/wiring.js builds the production deps. Dependency-free on purpose.
 *
 * ENTRY order (one function, sync after the source opens): event-store snapshot; clock._enterReplay;
 * setReplayFence(true); enterReplayCharts; set{Can,Udp,HardwareVersions}RosLink(true); resetForSeek;
 * create cache + engine; seek to the start, paused. The source is opened FIRST (network, async) so a
 * refusal (409) or an aborted entry leaves the live GUI untouched.
 *
 * EXIT is the reverse. Deviation, on purpose: the three RosLink(false) calls run AFTER clock._exitReplay
 * (setCanTrafficRosLink(false) stamps its freeze anchor from clock.now(), which must be wall time again),
 * and the live disconnect blanking is the very last step so no recorded value survives.
 *
 * deps: {
 *   ros: {getConnectionState, dispatchLocal, setBeforeConnectedHook, setStateSuppressor?},
 *   clock: {_enterReplay, _exitReplay, ...} (also handed to the engine),
 *   fence: {setReplayFence}, events: {snapshotAndBeginReplayEvents, restoreEvents, setEventsMuted, trimEventsAfter},
 *   charts: {createStore(storeDeps), enter(store, {onSeek}), exit(), setPlayhead(p), telemetrySample},
 *   links: {can(isUp), udp(isUp), hw(isUp)},
 *   resetForSeek(), blankDisconnectedState(), resetTrafficRings?(),
 *   makeRecordingSource(id), getSessionBuffer(), createCache(source), createEngine(opts), indexLatestBefore,
 *   visibleSpanSec?() (default 30), raf?: {request, cancel, hidden}, perfNow?()
 * }
 */

const DEFAULT_SPAN = 30;

/** @param {object} deps */
export function createReplayMode(deps) {
    const D = deps;
    const listeners = { state: new Set(), notice: new Set(), buffered: new Set() };
    let phase = 'idle';           // idle | opening | replay
    let kind = null;
    let source = null;
    let cache = null;
    let eng = null;
    let store = null;
    let evSnap = null;
    let latchSnap = null;         // main.js latches saved at entry, restored at exit
    let entered = false;          // the ordered entry sequence ran (exit must undo it)
    let token = 0;
    let unsubCache = null;
    let unsubEng = [];
    let rafId = null;
    let lastTs = null;
    let lastP = null;
    let abortReason = null;       // why an in-flight entry was cancelled (surfaced as its rejection)

    function emit(evt, v) {
        for (const cb of Array.from(listeners[evt])) {
            try { cb(v); } catch (e) { console.error('replay mode listener', evt, e); }
        }
    }
    const span = () => (D.visibleSpanSec ? +D.visibleSpanSec() || DEFAULT_SPAN : DEFAULT_SPAN);

    function topState() {
        if (phase === 'opening') return 'OPENING';
        if (phase === 'replay') return 'REPLAY';
        return D.ros.getConnectionState() === 'connected' ? 'LIVE' : 'LOBBY';
    }
    function snapshotState() {
        const es = eng ? eng.state() : null;
        return {
            mode: topState(),
            sub: phase === 'replay' && es ? es.mode : null,
            kind, playhead: es ? es.playhead : null, speed: es ? es.speed : null,
            range: es ? es.range : null,
        };
    }

    /** Sync back-scan latestBefore for the chart store join: resident chunks, then the source memo. */
    function latestBefore(topic, tSec) {
        if (!source) return null;
        const i0 = source.chunkIndex(tSec);
        for (let i = i0; i >= Math.max(0, i0 - 6); i--) {
            const ch = (cache && cache.peek(i)) || (source.peek && source.peek(i));
            const tp = ch && ch.topics && ch.topics[topic];
            if (!tp || tp.n < 1) continue;
            const k = D.indexLatestBefore(tp.t, tSec, tp.n);
            if (k >= 0) return { t: tp.t[k], row: tp.hydrate(k) };
        }
        return null;
    }

    function residentChunks() {
        const out = [];
        if (!cache || !source) return out;
        for (const r of cache.bufferedRanges()) {
            if (r.state !== 'resident') continue;
            const a = source.chunkIndex(r.t0 + 1e-6);
            const b = source.chunkIndex(r.t1 - 1e-6);
            for (let i = a; i <= b; i++) {
                const ch = cache.peek(i);
                if (ch && ch.topics) out.push(ch);
            }
        }
        return out;
    }
    function syncResident() {
        if (store) store.setResident(residentChunks());
        emit('buffered', cache ? cache.bufferedRanges() : []);
    }

    function dispatch(topic, msg, t, o) {
        D.events.setEventsMuted(!!(o && o.muted));
        try { D.ros.dispatchLocal(topic, msg); } finally { D.events.setEventsMuted(false); }
    }

    // ---- rAF loop: ticks the engine only while it is not paused ----
    function frame(ts) {
        rafId = null;
        if (phase !== 'replay' || !eng) return;
        const m = eng.state().mode;
        if (m === 'paused') { lastTs = null; return; }
        try {
            if (lastTs !== null) eng.tick(Math.max(0, ts - lastTs));
            lastTs = ts;
        } finally { schedule(); }
    }
    function schedule() {
        if (rafId !== null || phase !== 'replay' || !D.raf) return;
        if (D.raf.hidden && D.raf.hidden()) { lastTs = null; }   // hidden: rAF is throttled anyway; dt is clamped by the engine
        rafId = D.raf.request(frame);
    }
    function stopLoop() {
        if (rafId !== null && D.raf) D.raf.cancel(rafId);
        rafId = null;
        lastTs = null;
    }

    function waitFirstChunk(src, myToken) {
        const ready = () => {
            const st = src.status();
            if (st.state === 'failed') return 'failed';
            if (src.kind === 'session') return 'ok';
            return (st.chunksDone || 0) >= 1 ? 'ok' : null;
        };
        return new Promise((resolve, reject) => {
            const check = () => {
                if (myToken !== token) { reject(new Error('replay entry aborted')); return true; }
                const r = ready();
                if (r === 'failed') { reject(new Error('replay conversion failed')); return true; }
                if (r === 'ok') { resolve(); return true; }
                return false;
            };
            if (check()) return;
            const un = src.onChange(() => { if (check()) un(); });
        });
    }

    /**
     * @param {{kind:'recording', id:string}|{kind:'session'}} spec
     * @returns {Promise<void>} resolves paused at the start once the first chunk is resident;
     *   rejects with the 409 reason string-bearing Error (`.reason`) / 'no session buffer' / 'not disconnected'.
     */
    async function enterReplay(spec) {
        if (phase !== 'idle') throw new Error('replay already active');
        if (D.ros.getConnectionState() === 'connected') {
            const e = new Error('replay unavailable while connected'); e.reason = 'connected'; throw e;
        }
        const myToken = ++token;
        abortReason = null;
        phase = 'opening';
        kind = spec.kind;
        emit('state', snapshotState());
        try {
            if (spec.kind === 'session') {
                const snap = D.getSessionBuffer().snapshot();
                const r = snap.range();
                if (!(snap.chunkCount() > 0 && r.frontier > r.t0)) {
                    const e = new Error('no session buffer'); e.reason = 'no session buffer'; throw e;
                }
                source = snap;
            } else {
                source = D.makeRecordingSource(spec.id);
                await source.open();
                await waitFirstChunk(source, myToken);
            }
            if (myToken !== token) throw new Error('replay entry aborted');
            if (D.ros.getConnectionState() === 'connected') {
                const e = new Error('rosbridge connected'); e.reason = 'connected'; throw e;
            }

            // ---- ordered entry (§ 8) ----
            evSnap = D.events.snapshotAndBeginReplayEvents();
            entered = true;     // from here a throw in any later step reaches a full exit
            if (D.getReplayLatches) latchSnap = D.getReplayLatches();
            D.clock._enterReplay();
            D.fence.setReplayFence(true);
            cache = D.createCache(source);
            store = D.charts.createStore({ telemetrySample: D.charts.telemetrySample, latestBefore });
            D.charts.enter(store, { onSeek: (t) => { if (eng) eng.seek(t); } });
            D.links.can(true); D.links.udp(true); D.links.hw(true);
            D.resetForSeek();
            lastP = null;
            eng = D.createEngine({
                source, cache, dispatch, clock: D.clock, preRollSec: span,
                hooks: {
                    onPlayhead(p) {
                        D.charts.setPlayhead(p);
                        if (lastP !== null && p < lastP) D.events.trimEventsAfter(p);
                        lastP = p;
                    },
                    onResetForSeek() {
                        D.resetForSeek();
                        if (D.resetTrafficRings) D.resetTrafficRings();
                    },
                },
            });
            unsubCache = cache.onChange(syncResident);
            unsubEng = [
                eng.on('state', () => { emit('state', snapshotState()); if (eng.state().mode !== 'paused') schedule(); }),
                eng.on('buffering', () => emit('state', snapshotState())),
            ];
            const r = source.range();
            const start = spec.kind === 'session' ? Math.max(r.t0, r.frontier - span()) : r.t0;
            await eng.seek(start);
            if (myToken !== token) throw new Error('replay entry aborted');
            phase = 'replay';
            syncResident();
            emit('state', snapshotState());
        } catch (err) {
            if (myToken === token) doExit(null, true);
            else if (abortReason) { const e = new Error(abortReason); e.reason = abortReason; throw e; }
            throw err;
        }
    }

    function doExit(reason, silent) {
        const wasEntered = entered;
        token++;
        abortReason = reason || null;
        stopLoop();
        unsubEng.forEach((u) => { try { u(); } catch (e) { /* ignore */ } });
        unsubEng = [];
        if (unsubCache) { unsubCache(); unsubCache = null; }
        if (eng) { try { eng.dispose(); } catch (e) { console.error(e); } eng = null; }
        if (cache) { try { cache.dispose(); } catch (e) { console.error(e); } cache = null; }
        if (source && typeof source.close === 'function') { try { source.close(); } catch (e) { /* ignore */ } }
        if (wasEntered) {
            const step = (fn) => { try { fn(); } catch (e) { console.error(e); } };
            try {
                step(() => { if (D.resetTrafficRings) D.resetTrafficRings(); });
                step(() => D.charts.exit());
            } finally {
                // unconditional: a throwing undo step must never leave the fence on or the clock virtual
                step(() => D.fence.setReplayFence(false));
                step(() => D.clock._exitReplay());
            }
            step(() => { if (evSnap) D.events.restoreEvents(evSnap); });
            step(() => { if (D.restoreReplayLatches && latchSnap) D.restoreReplayLatches(latchSnap); });
            step(() => { D.links.can(false); D.links.udp(false); D.links.hw(false); });
            step(() => D.blankDisconnectedState());
        }
        store = null; source = null; evSnap = null; latchSnap = null; entered = false;
        phase = 'idle'; kind = null; lastP = null;
        emit('state', snapshotState());
        if (reason && !silent) emit('notice', { kind: 'toast', text: 'Replay ended: ' + reason });
    }

    /** @param {string} [reason] */
    function exitReplay(reason) {
        if (phase === 'idle') return Promise.resolve();
        doExit(reason || 'closed', false);
        return Promise.resolve();
    }

    /** The pre-notify hook body: exits synchronously enough to finish before main's listener. */
    function exitIfActive(reason) {
        if (phase === 'idle') return;
        // doExit is fully synchronous: the exit has completed (and main's blanking run) on return.
        doExit(reason, false);
    }

    D.ros.setBeforeConnectedHook(() => exitIfActive('rosbridge connected'));
    // Hold back connecting/disconnected edges from every live listener while OPENING/REPLAY (the
    // failed-reconnect loop would otherwise blank the panels every ~2 s). The exit sequence runs
    // main's blanking itself, as its last step.
    if (D.ros.setStateSuppressor) D.ros.setStateSuppressor(() => phase !== 'idle');

    return {
        enterReplay, exitReplay, exitIfActive,
        state: snapshotState,
        isActive: () => phase !== 'idle',
        engine: () => eng,
        source: () => source,
        on(evt, cb) { listeners[evt].add(cb); return () => listeners[evt].delete(cb); },
    };
}
