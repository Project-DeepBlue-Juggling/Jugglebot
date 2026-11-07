/**
 * hand-fan-panel.js — the Hand motor fan's two GUI controls: the read-only ON/OFF
 * pill in the System panel and the toggle in Platform Control. The Platform Teensy
 * owns the fan (Teensy_code_platform/hand_fan.h); this only shows hand_fan/status
 * and calls hand_fan/set.
 *
 * Two halves, as camera-panel.js: a DOM-free core node-tested by
 * tests/ros/test_gui_hand_fan_panel.py, then the glue.
 *
 * WHAT THE TOGGLE MEANS. ON and OFF force the fan, so the temperature rule stops
 * applying until auto is restored. Auto is offered only while forced, which keeps the
 * row a single toggle in normal running. The toggle lights the side the fan is
 * actually on, from the status frame, never the side last clicked. Thresholds stay in
 * config/hardware_config.yaml and the service: these calls send 0, meaning keep them.
 */

import * as ros from './ros-bridge.js';
import { isReplayFenced } from './replay/fence.js';

// ---------------------------------------------------------------------------
// Core (DOM-free)
// ---------------------------------------------------------------------------

export const HAND_FAN_SERVICE = 'hand_fan/set';
export const HAND_FAN_SERVICE_TYPE = 'jugglebot_interfaces/srv/SetHandFan';

/** The status frame is 1 Hz, so this is about three missed frames. */
export const STATUS_STALE_MS = 3500;

/** The node's own wait is bounded at 0.5 s plus the RPC's retries. */
const SERVICE_TIMEOUT_MS = 3000;
const FLASH_MS = 4000;

export function handFanRequest(mode) {
    return { mode, on_above_c: 0.0, off_below_c: 0.0 };
}

export function reduceStatus(msg, nowMs) {
    return {
        mode: msg.mode,
        fanOn: !!msg.fan_on,
        readingValid: !!msg.reading_valid,
        onAboveC: msg.on_above_c,
        offBelowC: msg.off_below_c,
        motorTempC: msg.motor_temp_c,
        atMs: nowMs,
    };
}

function statusTitle(m) {
    const reading = m.readingValid ? m.motorTempC.toFixed(1) + ' °C' : 'no reading';
    return 'Hand motor ' + reading + '. Auto: on above ' + m.onAboveC.toFixed(1)
        + ' °C, off below ' + m.offBelowC.toFixed(1) + ' °C';
}

/**
 * Everything the DOM shows, from the status model and ctx
 * { rosConnected, fenced, pending, flash }. A replay shows the recorded state, so
 * only a live view goes stale; a paused replay must not read as UNKNOWN.
 */
export function handFanView(model, nowMs, ctx) {
    const fresh = !!model && (ctx.fenced || nowMs - model.atMs <= STATUS_STALE_MS);
    if (!fresh) {
        return {
            known: false, pillText: 'UNKNOWN', pillClass: 'unknown',
            offActive: false, onActive: false, offEnabled: false, onEnabled: false,
            autoHidden: true, autoEnabled: false,
            modeText: ctx.flash || '--', modeClass: ctx.flash ? 'failed' : '',
            title: 'No Hand fan status',
        };
    }
    const canCommand = ctx.rosConnected && !ctx.fenced && !ctx.pending;
    const forced = model.mode !== 'auto';
    return {
        known: true,
        pillText: model.fanOn ? 'ON' : 'OFF',
        pillClass: model.fanOn ? 'held' : 'empty',
        offActive: !model.fanOn,
        onActive: model.fanOn,
        offEnabled: canCommand && model.mode !== 'off',
        onEnabled: canCommand && model.mode !== 'on',
        autoHidden: !forced,
        autoEnabled: canCommand,
        modeText: ctx.flash || (forced ? 'FORCED' : 'AUTO'),
        modeClass: ctx.flash ? 'failed' : (forced ? 'forced' : ''),
        title: statusTitle(model),
    };
}

// ---------------------------------------------------------------------------
// DOM / ROS glue
// ---------------------------------------------------------------------------

const nowMono = () => (globalThis.performance ? globalThis.performance.now() : Date.now()); // wall-clock: live status age; a replay never goes stale, see handFanView
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
    const v = handFanView(model, nowMono(), ctxNow());
    // The pill is built by panels.js initFlagsGrid, so it is looked up each time.
    const pill = document.getElementById('flag-hand-fan-pill');
    if (pill) {
        pill.textContent = v.pillText;
        pill.className = 'flag-ball-pill ' + v.pillClass;
        pill.title = v.title;
    }
    if (!els) return;
    els.off.disabled = !v.offEnabled;
    els.on.disabled = !v.onEnabled;
    els.off.classList.toggle('active', v.offActive);
    els.on.classList.toggle('active', v.onActive);
    els.auto.hidden = v.autoHidden;
    els.auto.disabled = !v.autoEnabled;
    els.mode.textContent = v.modeText;
    els.mode.title = v.title;
    els.mode.classList.toggle('forced', v.modeClass === 'forced');
    els.mode.classList.toggle('failed', v.modeClass === 'failed');
}

function showFlash(text) {
    flash = text;
    if (flashTimer) clearTimeout(flashTimer);
    flashTimer = setTimeout(() => { flash = null; flashTimer = null; render(); }, FLASH_MS);
    render();
}

function setMode(mode) {
    if (isReplayFenced()) return;  // replay fence (callService also refuses in replay)
    if (pending) return;
    pending = true;
    render();
    ros.withTimeout(ros.callService(HAND_FAN_SERVICE, HAND_FAN_SERVICE_TYPE, handFanRequest(mode)),
                    SERVICE_TIMEOUT_MS, HAND_FAN_SERVICE)
        .then((result) => {
            if (!result || !result.success) {
                console.warn(HAND_FAN_SERVICE + ' failed:', result && result.message);
                showFlash('REFUSED');
            }
        })
        .catch((err) => {
            console.error(HAND_FAN_SERVICE + ' service error:', err);
            showFlash('NO REPLY');
        })
        .finally(() => { pending = false; render(); });
}

/** hand_fan/status handler (main.js subscribeAll). */
export function handFanPanelOnStatus(msg) {
    model = reduceStatus(msg, nowMono());
    render();
}

/** ROS disconnect / replay seek (main.js blankDisconnectedState). */
export function setHandFanPanelDisconnected() {
    model = null;
    render();
}

/** Bind the row in index.html and start the repaint that notices a stale status. */
export function initHandFanPanel() {
    const found = {
        off: document.getElementById('hand-fan-off'),
        on: document.getElementById('hand-fan-on'),
        auto: document.getElementById('hand-fan-auto'),
        mode: document.getElementById('hand-fan-mode'),
    };
    if (Object.values(found).some((e) => !e)) return;
    els = found;
    els.off.addEventListener('click', () => setMode('off'));
    els.on.addEventListener('click', () => setMode('on'));
    els.auto.addEventListener('click', () => setMode('auto'));
    render();
    setInterval(render, 500);
}
