/**
 * camera-panel.js — the Wave camera record panel (s8 of
 * ros_ws/docs/wave_camera_can_adapter.md). Not to be confused with camera-presets.js,
 * which is the 3D view's camera presets: this panel drives the physical
 * camera's REC line through the CAN adapter.
 *
 * Two halves:
 *   1. A DOM-free core (every export above the "DOM / ROS glue" banner):
 *      the heartbeat reducer, the enable rule, the elapsed interpolation and
 *      the status text. Pure functions of their arguments; node-tested by
 *      tests/ros/test_gui_record_panel.py through
 *      tests/ros/js/record_panel_harness.js.
 *   2. The glue: initCameraPanel() wires #camera-panel's markup, a 4 Hz
 *      repaint, the REC/STOP click and the warning glyph's click-to-clear
 *      (camera/clear_faults) while faults are latched; cameraPanelOnHeartbeat() is the
 *      camera/heartbeat handler main.js subscribes; setCameraPanelDisconnected()
 *      is called from main.js blankDisconnectedState().
 *
 * Message: jugglebot_interfaces/msg/WaveCameraHeartbeat (10 Hz). wave_camera_can_adapter.md s7
 * names the fault byte `fault_bits`; the reducer also accepts `faults` so the
 * panel reads either spelling.
 */

import * as ros from './ros-bridge.js';
import { isReplayFenced } from './replay/fence.js';
import { fmtDuration } from './replay/ui/format.js';

// ---------------------------------------------------------------------------
// Core (DOM-free)
// ---------------------------------------------------------------------------

/** WaveCameraHeartbeat.state values (wave_camera_can_adapter.md s4.2 byte 0). */
export const STATE = Object.freeze({ BOOT: 0, IDLE: 1, STARTING: 2, RECORDING: 3, STOPPING: 4, FAULT: 5 });

/** Latched fault bits, index = bit number (wave_camera_can_adapter.md s4.2 byte 2). */
export const FAULT_NAMES = Object.freeze([
    'GPI2_REFUSED', 'TICK_LOST_MID_REC', 'CLIP_MISSING', 'INTENT_TIMEOUT_AUTOSTOP',
    'BUS_OFF_SEEN', 'SYNC_STEP', 'EDGE_RING_OVERFLOW',
]);

/** A heartbeat older than this freezes and greys the panel. */
export const STALE_MS = 1000;
/** Interpolation never runs further than this past the last heartbeat. */
export const MAX_EXTRAPOLATE_MS = 1000;
/** How long a failed service call's message replaces the status line. */
export const FLASH_MS = 2000;
/** Local give-up for a record_start/record_stop call (the node bounds itself at ~2.5 s). */
export const SERVICE_TIMEOUT_MS = 5000;

const BLANK_ELAPSED = '--:--';

/** @param {number} bits @returns {string[]} names of the set bits, low bit first. */
export function faultNames(bits) {
    const out = [];
    const b = Number(bits) || 0;
    for (let i = 0; i < FAULT_NAMES.length; i++) if (b & (1 << i)) out.push(FAULT_NAMES[i]);
    if (b & 0x80) out.push('BIT7');
    return out;
}

/**
 * Normalise a raw heartbeat message into the fields the panel reads.
 * @param {object} msg
 */
export function normalizeHeartbeat(msg) {
    const m = msg || {};
    const faults = m.fault_bits !== undefined ? m.fault_bits : m.faults;
    return {
        connected: !!m.connected,
        state: Number(m.state) || 0,
        camera_on: !!m.camera_on,
        recording: !!m.recording,
        elapsed_ms: Math.max(0, Number(m.elapsed_ms) || 0),
        recordings_this_session: Number(m.recordings_this_session) || 0,
        time_synced: !!m.time_synced,
        sync_age_ms: Number(m.sync_age_ms) || 0,
        faults: Number(faults) || 0,
        console_owns: !!m.console_owns,
        intent_fresh: !!m.intent_fresh,
    };
}

/**
 * The reducer: a heartbeat replaces the model and stamps its local arrival
 * time (monotonic ms from the glue's nowMono()). A null model is
 * the blank state (no heartbeat since load or since the last disconnect).
 * @param {object|null} _prev
 * @param {object} msg
 * @param {number} nowMs
 * @returns {{hb: object, lastHbMs: number}}
 */
export function reduceHeartbeat(_prev, msg, nowMs) {
    return { hb: normalizeHeartbeat(msg), lastHbMs: nowMs };
}

/** @returns {boolean} true when there is no heartbeat or it is older than STALE_MS. */
export function isStale(model, nowMs) {
    if (!model || !model.hb) return true;
    return (nowMs - model.lastHbMs) > STALE_MS;
}

/** @returns {boolean} the adapter reports a take in progress. */
export function isRecording(hb) {
    return !!hb && (hb.recording || hb.state === STATE.RECORDING);
}

/**
 * Displayed elapsed ms: the last heartbeat's elapsed_ms plus local time since
 * it arrived, clamped to MAX_EXTRAPOLATE_MS past it (so a stale heartbeat
 * freezes the readout). 0 when not recording; null with no heartbeat.
 */
export function interpolatedElapsedMs(model, nowMs) {
    if (!model || !model.hb) return null;
    if (!isRecording(model.hb)) return 0;
    const age = Math.min(Math.max(0, nowMs - model.lastHbMs), MAX_EXTRAPOLATE_MS);
    return model.hb.elapsed_ms + age;
}

/** @param {number|null} ms @returns {string} mm:ss (h:mm:ss from an hour), '--:--' for null. */
export function formatElapsed(ms) {
    if (ms === null || ms === undefined || !isFinite(ms)) return BLANK_ELAPSED;
    return fmtDuration(Math.floor(ms / 1000));
}

/**
 * The enable rule for the REC/STOP button.
 * @param {object|null} model
 * @param {number} nowMs
 * @param {{rosConnected:boolean, fenced:boolean, pending?:boolean}} ctx
 */
export function canPress(model, nowMs, ctx) {
    if (!ctx || !ctx.rosConnected || ctx.fenced || ctx.pending) return false;
    if (isStale(model, nowMs)) return false;
    const hb = model.hb;
    if (!hb.connected || !hb.camera_on || hb.console_owns) return false;
    return hb.state === STATE.IDLE || hb.state === STATE.RECORDING;
}

/**
 * The warning glyph clears the latched fault byte on click (camera/clear_faults)
 * when this holds: the same ROS/fence/pending/freshness gates as the button, a
 * connected adapter, and at least one fault latched. The camera need not be on.
 * @param {object|null} model
 * @param {number} nowMs
 * @param {{rosConnected:boolean, fenced:boolean, pending?:boolean}} ctx
 */
export function canClearFaults(model, nowMs, ctx) {
    if (!ctx || !ctx.rosConnected || ctx.fenced || ctx.pending) return false;
    if (isStale(model, nowMs)) return false;
    const hb = model.hb;
    return !!hb.connected && hb.faults !== 0;
}

/** @returns {string} the one-line status for a heartbeat (no ROS/flash context). */
export function statusText(hb) {
    if (!hb) return 'No heartbeat';
    if (!hb.connected) return 'Adapter disconnected';
    if (!hb.camera_on) return 'Camera off';
    if (hb.state === STATE.FAULT) {
        const names = faultNames(hb.faults);
        return 'Fault: ' + (names.length ? names.join(', ') : 'unknown');
    }
    if (isRecording(hb)) return 'Recording';
    switch (hb.state) {
        case STATE.BOOT: return 'Booting';
        case STATE.STARTING: return 'Starting…';
        case STATE.STOPPING: return 'Stopping…';
        case STATE.IDLE: {
            const n = hb.recordings_this_session;
            return 'Idle · ' + n + (n === 1 ? ' clip' : ' clips');
        }
        default: return 'State ' + hb.state;
    }
}

/** @returns {string[]} reasons for the amber warning glyph (empty = no glyph). */
export function warnings(hb) {
    if (!hb || !hb.connected) return [];
    const out = [];
    if (hb.faults) out.push('Faults: ' + faultNames(hb.faults).join(', '));
    if (!hb.time_synced) out.push('Clock not synced (sync age ' + hb.sync_age_ms + ' ms)');
    if (hb.console_owns) out.push('Console owns the camera (CAN commands refused)');
    return out;
}

/**
 * Everything the DOM needs, from the model and the context.
 * @param {object|null} model
 * @param {number} nowMs
 * @param {{rosConnected:boolean, fenced:boolean, pending?:boolean, flash?:string|null}} ctx
 */
export function cameraPanelView(model, nowMs, ctx) {
    const c = ctx || {};
    const hb = model ? model.hb : null;
    const blank = !c.rosConnected || !hb;
    const stale = !blank && isStale(model, nowMs);
    const recording = !blank && isRecording(hb);
    const warn = blank ? [] : warnings(hb);
    const clearable = !blank && canClearFaults(model, nowMs, c);
    let status = blank ? (c.rosConnected ? 'No heartbeat' : 'ROS disconnected') : statusText(hb);
    if (stale) status += ' (stale)';
    if (c.flash) status = c.flash;
    return {
        enabled: canPress(model, nowMs, c),
        label: c.pending ? '…' : (recording ? 'STOP' : 'REC'),
        recording,
        blank,
        stale,
        elapsedText: blank ? BLANK_ELAPSED : formatElapsed(interpolatedElapsedMs(model, nowMs)),
        statusText: status,
        warn: warn.length > 0,
        warnText: warn.join('\n') + (clearable ? '\n(click to clear faults)' : ''),
        clearable,
    };
}

// ---------------------------------------------------------------------------
// DOM / ROS glue
// ---------------------------------------------------------------------------

// Monotonic on purpose (owner decision): heartbeat age and the elapsed interpolation must not jump on an
// NTP step. In replay a paused playhead therefore greys the panel after STALE_MS, which is harmless (fenced).
const nowMono = () => (globalThis.performance ? globalThis.performance.now() : Date.now()); // wall-clock: monotonic heartbeat age, see above

let model = null;
let pending = false;
let flash = null;
let flashTimer = null;
let els = null;

function ctxNow() {
    return {
        rosConnected: ros.getConnectionState() === 'connected',
        fenced: isReplayFenced(),
        pending,
        flash,
    };
}

function render() {
    if (!els) return;
    const v = cameraPanelView(model, nowMono(), ctxNow());
    els.btn.disabled = !v.enabled;
    els.btn.textContent = v.label;
    els.btn.classList.toggle('rec-active', v.recording);
    els.dot.classList.toggle('recording', v.recording && !v.stale);
    els.elapsed.textContent = v.elapsedText;
    els.status.textContent = v.statusText;
    els.status.title = v.statusText;
    els.warn.hidden = !v.warn;
    els.warn.title = v.warnText;
    els.warn.classList.toggle('clearable', v.clearable);
    els.root.classList.toggle('stale', v.stale);
    els.root.classList.toggle('blank', v.blank);
}

function showFlash(text) {
    flash = text;
    if (flashTimer) clearTimeout(flashTimer);
    flashTimer = setTimeout(() => { flash = null; flashTimer = null; render(); }, FLASH_MS);
    render();
}

function onRecClick() {
    if (isReplayFenced()) return;  // replay fence (callService also refuses in replay)
    if (pending || !els || els.btn.disabled) return;
    const stop = isRecording(model && model.hb);
    const service = stop ? 'camera/record_stop' : 'camera/record_start';
    pending = true;
    render();
    ros.withTimeout(ros.callService(service, 'std_srvs/srv/Trigger'), SERVICE_TIMEOUT_MS, service)
        .then((result) => {
            if (!result || !result.success) {
                const msg = (result && result.message) || 'refused';
                console.warn(service + ' failed:', msg);
                showFlash((stop ? 'Stop failed: ' : 'Start failed: ') + msg);
            }
        })
        .catch((err) => {
            console.error(service + ' service error:', err);
            showFlash('Error: ' + ((err && err.message) || err));
        })
        .finally(() => { pending = false; render(); });
}

function onWarnClick() {
    if (isReplayFenced()) return;  // replay fence (callService also refuses in replay)
    if (pending || !els || !canClearFaults(model, nowMono(), ctxNow())) return;
    const service = 'camera/clear_faults';
    pending = true;
    render();
    ros.withTimeout(ros.callService(service, 'std_srvs/srv/Trigger'), SERVICE_TIMEOUT_MS, service)
        .then((result) => {
            if (result && result.success) {
                showFlash('Faults cleared');
            } else {
                const msg = (result && result.message) || 'refused';
                console.warn(service + ' failed:', msg);
                showFlash('Clear failed: ' + msg);
            }
        })
        .catch((err) => {
            console.error(service + ' service error:', err);
            showFlash('Error: ' + ((err && err.message) || err));
        })
        .finally(() => { pending = false; render(); });
}

/** camera/heartbeat handler (main.js subscribeAll). Snaps the readout. */
export function cameraPanelOnHeartbeat(msg) {
    model = reduceHeartbeat(model, msg, nowMono());
    render();
}

/** ROS disconnect / replay seek (main.js blankDisconnectedState): blank everything. */
export function setCameraPanelDisconnected() {
    model = null;
    render();
}

/** Bind #camera-panel's markup (index.html) and start the 4 Hz repaint. */
export function initCameraPanel() {
    const root = document.getElementById('camera-panel');
    if (!root) return;
    els = {
        root,
        btn: document.getElementById('camera-rec-btn'),
        dot: document.getElementById('camera-rec-dot'),
        elapsed: document.getElementById('camera-rec-elapsed'),
        warn: document.getElementById('camera-rec-warn'),
        status: document.getElementById('camera-rec-status'),
    };
    if (Object.values(els).some((e) => !e)) { els = null; return; }
    els.btn.addEventListener('click', onRecClick);
    els.warn.addEventListener('click', onWarnClick);
    render();
    setInterval(render, 250);
}
