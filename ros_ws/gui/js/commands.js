/**
 * commands.js — Command button handlers.
 *
 * Creates command buttons in the overlay bar at the bottom of the 3D viewer.
 * Publishes to orchestrator_command topic. Context-sensitive enable/disable.
 */

import * as ros from './ros-bridge.js';
import { currentOrchestratorState } from './panels.js';
import { emitEvent, EVENT_TYPES } from './event-store.js';
import { holdToConfirm } from './hold-to-confirm.js';

/** Commands gated by hold-to-confirm.  Clear Errors can re-arm a faulted
 *  motor under load, so it's worth a deliberate gesture; everything else
 *  responds to a single click. */
const DESTRUCTIVE_COMMANDS = new Set(['clear_errors']);

/** @type {{ publish: function } | null} */
let cmdPublisher = null;

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
    const overlay = document.getElementById('command-overlay');
    if (!overlay) return;

    cmdPublisher = ros.advertise('orchestrator_command', 'std_msgs/msg/String');

    for (const cmd of COMMANDS) {
        const btn = document.createElement('button');
        btn.id = cmd.id;
        btn.className = `cmd-btn ${cmd.cssClass}`;
        btn.textContent = cmd.label;
        btn.disabled = true;

        const dispatch = () => {
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
    const state = currentOrchestratorState;
    const active = state === 'ACTIVE';

    setEnabled('cmd-home', state === 'IDLE' || state === 'BOOT');
    setEnabled('cmd-level', state === 'IDLE');
    setEnabled('cmd-activate', state === 'IDLE');
    setEnabled('cmd-deactivate', active);
    setEnabled('cmd-clear', true); // always available
}

function setEnabled(id, enabled) {
    const btn = document.getElementById(id);
    if (btn) btn.disabled = !enabled;
}
