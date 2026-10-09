/**
 * bb-aim.js — Manual Ball Butler aim from the BB panel's Yaw / Pitch readouts.
 *
 * While the orchestrator is IDLE and Ball Butler is connected and IDLE or
 * TRACKING, clicking the Yaw or Pitch readout turns it into an input: Enter
 * sends, Esc (or clicking away) cancels.  Yaw is in BB's own frame (as the
 * heartbeat reports it); pitch is degrees from horizontal.  An out-of-range
 * entry is refused in place and nothing is sent — never clamped.
 *
 * A send starts a HOLD.  BB drops out of TRACKING, and returns pitch to its
 * rest angle, 5 s after its last aim, so the hold is renewed by re-calling
 * bb/aim every RENEW_PERIOD_MS.  That renewal IS the lease: close the tab or
 * lose rosbridge and the calls stop, and BB returns pitch on its own — no pose
 * outlives the operator.  The hold ends on Release, on the orchestrator leaving IDLE (ACTIVATE
 * included), on BB leaving IDLE/TRACKING or disconnecting, and on any refused
 * or failed renewal.  ball_butler_node re-checks the same gate on every call;
 * this module's copy only decides what the operator is offered.
 *
 * bb/aim goes through `ros-bridge.js::callService`, which sets NO client-side
 * timeout — a call that never gets a reply (ball_butler_node wedged, or just
 * slow) would otherwise sit pending until rosbridge's own 50 s server-side
 * bound (`rosbridge_websocket_lean.py::CALL_SERVICE_TIMEOUT_S`) gives up on
 * our behalf. Until 2026-09-28 rosbridge also ran every service call on the
 * tab's ONE queue thread, so an unanswered call stalled every other GUI action
 * on the tab — the "whole GUI freezes" failure (2026-09-28 sitting). That half
 * is fixed server-side (`rosbridge_websocket_lean.py` item 5: each call on its
 * own thread); this module's half is not stacking renewals behind a call that
 * has not answered, and telling the operator within a few seconds.  `AIM_TIMEOUT_MS` below gives up on OUR side
 * in a few seconds instead — well under both BB's 5 s TRACKING-drop and
 * rosbridge's 50 s bound — and `renewInFlight` guards both submit() and the
 * renewal timer so at most one bb/aim call is ever considered in flight from
 * this module.  Giving up locally cannot cancel the request already sitting
 * in rosbridge's queue (no correlation id in the service contract to key a
 * cancel on — see the service definition), so a late reply, if one ever
 * arrives, is still discarded here (the `withTimeout` wrapper below settles
 * exactly once, so a late real response after our timeout has already fired
 * is inert to this module's state) — but the physical aim it carries is
 * still whatever ball_butler_node eventually does with it; that residual is
 * a ROS-side property of the shared connection, not something this module
 * can close from here.
 */

import { callService, withTimeout } from './ros-bridge.js';
import { emitEvent, EVENT_TYPES } from './event-store.js';
import { isReplayFenced } from './replay/fence.js';
import { BB_YAW_LIM_MIN_DEG, BB_YAW_LIM_MAX_DEG,
         BB_PITCH_DEG_MIN, BB_PITCH_DEG_MAX } from './geometry-config.js';

const SERVICE = 'bb/aim';
const SERVICE_TYPE = 'jugglebot_interfaces/srv/BallButlerAim';
/** Well inside BB's 5 s TRACKING timeout, so one late renewal never drops the pose. */
const RENEW_PERIOD_MS = 1000;
/**
 * Client-side bound on a single bb/aim call. Comfortably above a healthy
 * round trip (tens of ms) but far below BB's 5 s TRACKING-drop and
 * rosbridge's 50 s server-side bound — see this file's docstring.
 */
const AIM_TIMEOUT_MS = 3000;

const BB_IDLE = 1;
const BB_TRACKING = 2;

const RANGE = {
    yaw:   { min: BB_YAW_LIM_MIN_DEG, max: BB_YAW_LIM_MAX_DEG, label: 'Yaw' },
    pitch: { min: BB_PITCH_DEG_MIN,   max: BB_PITCH_DEG_MAX,   label: 'Pitch' },
};

let orchIdle = false;
let bbReady = false;                     // connected AND IDLE/TRACKING
const measured = { yaw: null, pitch: null };

/** @type {{yaw: number, pitch: number} | null} */
let hold = null;
let renewTimer = null;
let renewInFlight = false;

/** Axis whose readout is currently an <input>, or null. */
let editingAxis = null;

export function isBBAimEditing(axis) {
    return editingAxis === axis;
}

function allowed() {
    if (isReplayFenced()) return false;  // replay fence: refreshAffordance drops drag-aim
    return orchIdle && bbReady;
}

// ---- DOM ----

export function initBBAim() {
    const readouts = document.querySelector('#bb-content .bb-readouts');
    if (!readouts) return;

    for (const axis of ['yaw', 'pitch']) {
        const el = document.getElementById(`bb-${axis}`);
        if (el) el.addEventListener('click', () => beginEdit(axis));
    }

    const row = document.createElement('div');
    row.className = 'bb-aim-row';
    row.innerHTML = `
        <span class="bb-aim-status" id="bb-aim-status"></span>
        <button class="bb-aim-release" id="bb-aim-release" hidden>Release</button>
    `;
    readouts.after(row);
    document.getElementById('bb-aim-release')
        .addEventListener('click', () => endHold('Released'));

    refreshAffordance();
}

function setStatus(text, isError = false) {
    const el = document.getElementById('bb-aim-status');
    if (!el) return;
    el.textContent = text;
    el.classList.toggle('error', isError);
}

/** Mark the readouts editable (or not) and keep the hold row in step. */
function refreshAffordance() {
    const ok = allowed();
    for (const axis of ['yaw', 'pitch']) {
        const el = document.getElementById(`bb-${axis}`);
        if (!el) continue;
        el.classList.toggle('bb-editable', ok);
        const r = RANGE[axis];
        el.title = ok
            ? `Click to aim ${r.label.toLowerCase()} (${r.min}–${r.max}°)`
            : '';
    }
    if (!ok && editingAxis) cancelEdit();

    const rel = document.getElementById('bb-aim-release');
    if (rel) rel.hidden = !hold;
    if (hold) {
        setStatus(`Holding yaw ${hold.yaw.toFixed(1)}°, pitch ${hold.pitch.toFixed(1)}°`);
    }
}

function beginEdit(axis) {
    if (!allowed() || editingAxis === axis) return;
    if (editingAxis) cancelEdit();
    const el = document.getElementById(`bb-${axis}`);
    if (!el) return;

    const start = hold ? hold[axis] : measured[axis];
    if (!hold) setStatus('');
    editingAxis = axis;
    el.textContent = '';
    const input = document.createElement('input');
    input.type = 'number';
    input.step = '0.1';
    input.className = 'bb-aim-input';
    input.value = start == null ? '' : start.toFixed(1);
    el.appendChild(input);
    input.focus();
    input.select();

    input.addEventListener('keydown', (e) => {
        if (e.key === 'Enter') { e.preventDefault(); submit(axis, input); }
        else if (e.key === 'Escape') { e.preventDefault(); cancelEdit(); }
    });
    // Clicking away cancels; a pending Enter has already run by then.
    input.addEventListener('blur', () => {
        if (editingAxis === axis) cancelEdit();
    });
}

function cancelEdit() {
    const axis = editingAxis;
    editingAxis = null;
    const el = axis && document.getElementById(`bb-${axis}`);
    if (el) {
        el.textContent = measured[axis] == null
            ? '--' : measured[axis].toFixed(1) + '°';
    }
}

function inRange(axis, v) {
    const r = RANGE[axis];
    return Number.isFinite(v) && v >= r.min && v <= r.max;
}

function submit(axis, input) {
    const r = RANGE[axis];
    const v = parseFloat(input.value);
    if (!inRange(axis, v)) {
        input.classList.add('invalid');
        setStatus(`${r.label} must be ${r.min}–${r.max}° — nothing sent`, true);
        return;
    }

    // The command carries both axes: the one not being edited keeps its held
    // target, else stays where it measures.  A measured value a hair outside
    // the range (a backdriven or rounding-edge reading) is pulled onto it —
    // this is the axis the operator did NOT edit, so "stay put" is the intent.
    const other = axis === 'yaw' ? 'pitch' : 'yaw';
    let otherV = hold ? hold[other] : measured[other];
    if (otherV == null) {
        setStatus('No Ball Butler position yet — nothing sent', true);
        return;
    }
    otherV = Math.min(RANGE[other].max, Math.max(RANGE[other].min, otherV));

    const target = axis === 'yaw'
        ? { yaw: v, pitch: otherV }
        : { yaw: otherV, pitch: v };

    editingAxis = null;
    input.remove();
    sendAim(target, true);
}

// ---- Service calls / hold lifecycle ----

function sendAim(target, isNew) {
    if (renewInFlight) {
        // A previous call (submit or renewal) hasn't settled yet. Never
        // stack a second one behind it — see this file's docstring on why
        // that would extend, not shorten, how long the tab stays blocked.
        if (isNew) setStatus('A previous aim request is still pending — try again shortly', true);
        return Promise.resolve();
    }
    renewInFlight = true;
    return withTimeout(
        callService(SERVICE, SERVICE_TYPE,
                    { yaw_deg: target.yaw, pitch_deg: target.pitch }),
        AIM_TIMEOUT_MS, SERVICE)
        .then((res) => {
            renewInFlight = false;
            if (!res.success) {
                // Node-side gate refused — the hold (if any) is over.
                if (hold) endHold(res.message, true);
                else setStatus(res.message, true);
                return;
            }
            // A gate change while the first send was in flight must not
            // start a hold the operator is no longer allowed.
            if (isNew && allowed()) {
                hold = { ...target };
                startRenewal();
                emitEvent({
                    type: EVENT_TYPES.COMMAND,
                    label: 'BB aim',
                    detail: `bb/aim: yaw ${target.yaw.toFixed(1)}°, pitch ${target.pitch.toFixed(1)}°`,
                });
            }
            refreshAffordance();
        })
        .catch((err) => {
            renewInFlight = false;
            if (hold) endHold(`bb/aim failed: ${err.message}`, true);
            else setStatus(`bb/aim failed: ${err.message}`, true);
        });
}

function startRenewal() {
    if (renewTimer) return;
    renewTimer = setInterval(() => {
        // Skip a tick rather than stack calls behind a slow one.
        if (hold && !renewInFlight) sendAim(hold, false);
    }, RENEW_PERIOD_MS);
}

function endHold(reason, isError = false) {
    if (renewTimer) { clearInterval(renewTimer); renewTimer = null; }
    const had = hold !== null;
    hold = null;
    if (had) {
        setStatus(`${reason} — pitch returns to rest within 5 s`, isError);
        emitEvent({ type: EVENT_TYPES.COMMAND, label: 'BB aim released', detail: reason });
    }
    refreshAffordance();
}

// ---- Inputs from main.js / panels.js ----

export function bbAimOnOrchestratorState(stateStr) {
    orchIdle = stateStr.split(':')[0].trim().toUpperCase() === 'IDLE';
    if (!orchIdle && hold) endHold('Orchestrator left IDLE');
    refreshAffordance();
}

export function bbAimOnHeartbeat(hb) {
    if (!hb.connected) { bbAimOnDisconnect(); return; }
    measured.yaw = hb.yaw_deg;
    measured.pitch = hb.pitch_deg;
    const was = bbReady;
    bbReady = hb.state === BB_IDLE || hb.state === BB_TRACKING;
    if (!bbReady && hold) endHold('Ball Butler left IDLE/TRACKING');
    if (was !== bbReady) refreshAffordance();
}

export function bbAimOnDisconnect() {
    bbReady = false;
    measured.yaw = null;
    measured.pitch = null;
    if (hold) endHold('Ball Butler disconnected', true);
    refreshAffordance();
}
