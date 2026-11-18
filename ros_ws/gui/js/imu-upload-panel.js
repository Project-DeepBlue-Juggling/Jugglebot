/**
 * imu-upload-panel.js — the Platform IMU capture controls in Platform Control: the "Upload IMU data after
 * each Attempt" toggle, the upload progress and the last capture's result (IDEA-20, P3c). The Platform
 * Teensy records; teensy_bridge_node uploads and publishes platform_imu/status (2 Hz); this only shows that
 * topic and calls platform_imu/set_upload_after_attempt. The toggle shows what the node reports, never what
 * was last clicked, so a reload shows the truth. While an upload runs with the toggle ON the Jetson refuses
 * commands that start motion; the refusal itself appears where refusals already do (the Juggle panel line,
 * the console), this panel only says an upload is running and how long remains.
 *
 * Two halves as hand-fan-panel.js: a DOM-free core node-tested by tests/ros/test_gui_imu_upload_panel.py,
 * then the glue.
 */

import * as ros from './ros-bridge.js';
import { isReplayFenced } from './replay/fence.js';

// ---------------------------------------------------------------------------
// Core (DOM-free)
// ---------------------------------------------------------------------------

export const IMU_TOGGLE_SERVICE = 'platform_imu/set_upload_after_attempt';
export const IMU_TOGGLE_SERVICE_TYPE = 'std_srvs/srv/SetBool';

/** The status topic is 2 Hz, so this is about four missed messages. */
export const STATUS_STALE_MS = 2500;

const SERVICE_TIMEOUT_MS = 3000;
const FLASH_MS = 4000;

export function toggleRequest(on) {
    return { data: !!on };
}

/** DiagnosticStatus.values is [{key, value}] (rosbridge JSON). */
export function reduceStatus(msg, nowMs) {
    const kv = {};
    for (const e of (msg && msg.values) || []) kv[e.key] = e.value;
    const num = (k) => {
        const v = parseFloat(kv[k]);
        return Number.isFinite(v) ? v : null;
    };
    return {
        busy: kv.busy === '1',
        toggleOn: kv.upload_after_attempt === '1',
        phase: kv.phase || '',
        framesDone: num('frames_done') || 0,
        framesTotal: num('frames_total') || 0,
        etaS: num('eta_s'),
        platformState: kv.platform_state || '',
        lastLabel: kv.last_label || '',
        lastComplete: kv.last_complete,       // '1' | '0' | ''
        lastCrcOk: kv.last_crc_ok,            // 'True' | 'False' | 'None' | ''
        lastFailure: kv.last_failure || '',
        lastSkip: kv.last_skip || '',
        atMs: nowMs,
    };
}

function imuStateText(m) {
    if (m.busy) {
        const pct = m.framesTotal > 0 ? Math.min(100, Math.floor((100 * m.framesDone) / m.framesTotal)) : null;
        const left = m.etaS !== null && m.etaS >= 0 ? ', ' + Math.max(1, Math.round(m.etaS)) + ' s left' : '';
        return 'uploading ' + (m.phase || '') + (pct !== null ? ' ' + pct + '%' : '') + left;
    }
    switch (m.platformState) {
        case 'RUNNING': return 'recording';
        case 'FROZEN': case 'UPLOADING': return 'capture held';
        case 'DISABLED': return 'IMU disabled';
        case 'FW_HALTED': return 'IMU halted';
        default: return 'no IMU detected';
    }
}

function lastResult(m) {
    if (m.lastFailure) return { text: 'last capture failed: ' + m.lastFailure, cls: 'failed' };
    if (m.lastComplete === '') return { text: '', cls: '' };
    if (m.lastCrcOk === 'False') return { text: 'last capture: CRC FAILED', cls: 'failed' };
    if (m.lastComplete === '0') return { text: 'last capture: INCOMPLETE', cls: 'warn' };
    return { text: 'last capture: complete', cls: '' };
}

/**
 * Everything the DOM shows, from the status model and ctx { rosConnected, fenced, pending, flash }.
 * A replay is fenced (ctx.fenced): it shows nothing live and never offers the toggle, although
 * /platform_imu/status is recorded in the bag and allowlisted for replay (jugglebot_launch.py, replay/allowlist.js).
 */
export function imuView(model, nowMs, ctx) {
    const fresh = !!model && !ctx.fenced && nowMs - model.atMs <= STATUS_STALE_MS;
    if (!fresh) {
        return {
            known: false, toggleChecked: false, toggleEnabled: false, progress: null,
            stateText: ctx.flash || '--', stateClass: ctx.flash ? 'failed' : '', lastText: '', lastClass: '',
            title: 'No Platform IMU status',
        };
    }
    const last = lastResult(model);
    const pct = model.busy && model.framesTotal > 0
        ? Math.min(100, (100 * model.framesDone) / model.framesTotal) : null;
    return {
        known: true,
        toggleChecked: model.toggleOn,
        toggleEnabled: ctx.rosConnected && !ctx.pending,
        progress: model.busy ? (pct === null ? 0 : pct) : null,
        stateText: ctx.flash || imuStateText(model),
        stateClass: ctx.flash ? 'failed' : (model.busy ? 'busy' : ''),
        lastText: last.text,
        lastClass: last.cls,
        title: model.busy && model.toggleOn
            ? 'Commands that start motion are refused until the upload ends; deactivate stops it.'
            : (model.lastSkip || 'Platform IMU capture'),
    };
}

// ---------------------------------------------------------------------------
// DOM / ROS glue
// ---------------------------------------------------------------------------

const nowMono = () => (globalThis.performance ? globalThis.performance.now() : Date.now()); // live status age only
let model = null;
let pending = false;
let flash = null;
let flashTimer = null;
let els = null;

function ctxNow() {
    return { rosConnected: ros.getConnectionState() === 'connected', fenced: isReplayFenced(), pending, flash };
}

function render() {
    if (!els) return;
    const v = imuView(model, nowMono(), ctxNow());
    els.toggle.checked = v.toggleChecked;
    els.toggle.disabled = !v.toggleEnabled;
    els.state.textContent = v.stateText;
    els.state.title = v.title;
    els.state.classList.toggle('busy', v.stateClass === 'busy');
    els.state.classList.toggle('failed', v.stateClass === 'failed');
    els.bar.hidden = v.progress === null;
    if (v.progress !== null) els.bar.value = v.progress;
    els.last.textContent = v.lastText;
    els.last.classList.toggle('warn', v.lastClass === 'warn');
    els.last.classList.toggle('failed', v.lastClass === 'failed');
}

function showFlash(text) {
    flash = text;
    if (flashTimer) clearTimeout(flashTimer);
    flashTimer = setTimeout(() => { flash = null; flashTimer = null; render(); }, FLASH_MS);
    render();
}

function setToggle(on) {
    if (isReplayFenced() || pending) { render(); return; }
    pending = true;
    render();
    ros.withTimeout(ros.callService(IMU_TOGGLE_SERVICE, IMU_TOGGLE_SERVICE_TYPE, toggleRequest(on)),
                    SERVICE_TIMEOUT_MS, IMU_TOGGLE_SERVICE)
        .then((result) => {
            if (!result || !result.success) {
                console.warn(IMU_TOGGLE_SERVICE + ' failed:', result && result.message);
                showFlash('REFUSED');
            }
        })
        .catch((err) => {
            console.error(IMU_TOGGLE_SERVICE + ' service error:', err);
            showFlash('NO REPLY');
        })
        .finally(() => { pending = false; render(); });
}

/** platform_imu/status handler (main.js subscribeAll). */
export function imuUploadPanelOnStatus(msg) {
    model = reduceStatus(msg, nowMono());
    render();
}

/** ROS disconnect / replay seek (main.js blankDisconnectedState). */
export function setImuUploadPanelDisconnected() {
    model = null;
    render();
}

/** Bind the row in index.html and start the repaint that notices a stale status. */
export function initImuUploadPanel() {
    const found = {
        toggle: document.getElementById('imu-upload-toggle'),
        state: document.getElementById('imu-upload-state'),
        bar: document.getElementById('imu-upload-bar'),
        last: document.getElementById('imu-upload-last'),
    };
    if (Object.values(found).some((e) => !e)) return;
    els = found;
    els.toggle.addEventListener('change', () => setToggle(els.toggle.checked));
    render();
    setInterval(render, 500);
}
