/**
 * commands.js — Command button handlers.
 *
 * Creates command buttons in the overlay bar at the bottom of the 3D viewer, inside
 * #command-buttons (index.html): one group so CSS can centre the buttons as a unit between
 * the Wave camera panel (left end) and the Trail control (right end).
 * Publishes to orchestrator_command topic. Context-sensitive enable/disable.
 */

import * as ros from './ros-bridge.js';
import { currentOrchestratorState } from './panels.js';
import { emitEvent, EVENT_TYPES } from './event-store.js';
import { holdToConfirm } from './hold-to-confirm.js';
import { isReplayFenced } from './replay/fence.js';

/** Commands gated by hold-to-confirm.  Clear Errors can re-arm a faulted
 *  motor under load, so it's worth a deliberate gesture; everything else
 *  responds to a single click. */
const DESTRUCTIVE_COMMANDS = new Set(['clear_errors']);

/** In-flight guard (BUG-104).  One publish latches the command buttons inert until the orchestrator
 *  state CHANGES from the one the click saw (orchestrator_state re-publishes continuously, so "the next
 *  message" would release on the very next tick and guard nothing) or this cooldown expires, so a double
 *  click inside one state-update latency publishes once.  Shorter than the Juggle Start cooldown (4 s):
 *  these are state-changing commands whose result shows in the state within a tick or two, and a stuck
 *  latch on a command that is refused (state never changes) must clear quickly. */
const COMMAND_COOLDOWN_MS = 1500;

/** Commands exempt from the SHARED latch (owner, TKT-78 option 1).  Deactivate is the operator's way out of
 *  Active and must never wait on another button's latch; Clear Errors is the fault way out.  Each still guards
 *  against its own double click (a private latch of the same kind), and a click on one of them does not latch
 *  the others. */
const LATCH_EXEMPT = new Set(['deactivate', 'clear_errors']);

const BUTTON_IDS = ['cmd-home', 'cmd-level', 'cmd-activate', 'cmd-deactivate', 'cmd-clear'];

/** @type {{ publish: function } | null} */
let cmdPublisher = null;
/** Latches by key: 'shared' (every non-exempt button) or the command name (an exempt button's own).
 *  @type {Map<string, {state: string|null, timer: any}>} */
const latches = new Map();

const latchKey = (command) => (LATCH_EXEMPT.has(command) ? command : 'shared');

function clearLatch(key) {
    const l = latches.get(key);
    if (l) clearTimeout(l.timer);
    latches.delete(key);
}

function releaseInFlight(key) {
    clearLatch(key);
    updateCommandStates();
}

// The Standby / SpaceMouse / Shell / GUI command buttons were removed — the
// ACTIVE sub-mode is now changed from the state-machine minimap panel. Those
// buttons were the ONLY firer of the old mode-change callback; jog / speed-
// limit panel visibility is still driven by onControlMode (control_mode_topic)
// and onOrchestratorState (sub-mode) in main.js, and the speed-limit slider
// reset that the callback used to perform now lives in onControlMode, so
// nothing here needs the callback any more.
const COMMANDS = [
    { id: 'cmd-home',       label: 'Home',           command: 'home',         cssClass: 'btn-home' },
    { id: 'cmd-level',      label: 'Level',          command: 'level',        cssClass: 'btn-home' },
    { id: 'cmd-activate',   label: 'Activate',       command: 'activate',     cssClass: 'btn-activate' },
    { id: 'cmd-deactivate', label: 'Deactivate',     command: 'deactivate',   cssClass: 'btn-deactivate' },
    { id: 'cmd-clear',      label: 'Clear Errors',   command: 'clear_errors', cssClass: 'btn-fault' },
];

/**
 * Initialise command overlay.
 */
export function initCommands() {
    const overlay = document.getElementById('command-buttons') || document.getElementById('command-overlay');
    if (!overlay) return;

    cmdPublisher = ros.advertise('orchestrator_command', 'std_msgs/msg/String');

    for (const cmd of COMMANDS) {
        const btn = document.createElement('button');
        btn.id = cmd.id;
        btn.className = `cmd-btn ${cmd.cssClass}`;
        btn.textContent = cmd.label;
        btn.disabled = true;

        const dispatch = () => {
            if (isReplayFenced()) return;   // replay fence: never publish or log a COMMAND
            const key = latchKey(cmd.command);
            if (latches.has(key)) return;   // BUG-104: one publish per state-update latency
            latches.set(key, {
                state: currentOrchestratorState,
                timer: setTimeout(() => releaseInFlight(key), COMMAND_COOLDOWN_MS),
            });
            updateCommandStates();
            if (cmdPublisher) {
                cmdPublisher.publish({ data: cmd.command });
            }
            emitEvent({
                type: EVENT_TYPES.COMMAND,
                label: cmd.label,
                detail: `orchestrator_command: ${cmd.command}`,
            });
        };

        if (DESTRUCTIVE_COMMANDS.has(cmd.command)) {
            btn.classList.add('hold-fillable');
            btn.title = `${cmd.label} — hold to confirm`;
            holdToConfirm(btn, dispatch);
        } else {
            btn.addEventListener('click', dispatch);
        }

        overlay.appendChild(btn);
    }
}

/**
 * Update button enabled/disabled states based on current orchestrator state.
 * Called after each orchestrator_state message.
 */
export function updateCommandStates() {
    if (isReplayFenced()) {
        // Replay fence: a recorded IDLE must not light the command buttons.
        for (const id of BUTTON_IDS) setEnabled(id, false);
        return;
    }
    const state = currentOrchestratorState;
    for (const [key, l] of [...latches]) {
        if (state !== l.state) clearLatch(key);   // the command took effect: release
    }
    const shared = latches.has('shared');
    const active = state === 'ACTIVE';

    setEnabled('cmd-home', !shared && (state === 'IDLE' || state === 'BOOT'));
    setEnabled('cmd-level', !shared && state === 'IDLE');
    setEnabled('cmd-activate', !shared && state === 'IDLE');
    setEnabled('cmd-deactivate', active && !latches.has('deactivate'));   // exempt from the shared latch
    setEnabled('cmd-clear', !latches.has('clear_errors'));                // always available, own latch only
}

function setEnabled(id, enabled) {
    const btn = document.getElementById(id);
    if (btn) btn.disabled = !enabled;
}
