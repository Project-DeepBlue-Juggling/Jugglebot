/**
 * commands.js — Command button handlers.
 *
 * Creates command buttons in the overlay bar at the bottom of the 3D viewer.
 * Publishes to orchestrator_command topic. Context-sensitive enable/disable.
 */

import * as ros from './ros-bridge.js';
import { currentOrchestratorState, currentSubMode } from './panels.js';
import { emitEvent, EVENT_TYPES } from './event-store.js';
import { holdToConfirm } from './hold-to-confirm.js';

/** Commands gated by hold-to-confirm.  Clear Errors can re-arm a faulted
 *  motor under load, so it's worth a deliberate gesture; everything else
 *  responds to a single click. */
const DESTRUCTIVE_COMMANDS = new Set(['clear_errors']);

/** @type {{ publish: function } | null} */
let cmdPublisher = null;

/** Callback for mode-button clicks (GUI, SpaceMouse, Shell). Set by main.js. */
let onModeChangeCallback = null;

/**
 * Register a callback that fires when a mode button is clicked.
 * @param {function} cb - Called with the command string (e.g. 'gui', 'spacemouse')
 */
export function onModeButtonClick(cb) {
    onModeChangeCallback = cb;
}

const COMMANDS = [
    { id: 'cmd-home',       label: 'Home',           command: 'home',         cssClass: 'btn-home' },
    { id: 'cmd-level',      label: 'Level',          command: 'level',        cssClass: 'btn-home' },
    { id: 'cmd-activate',   label: 'Activate',       command: 'activate',     cssClass: 'btn-activate' },
    { id: 'cmd-deactivate', label: 'Deactivate',     command: 'deactivate',   cssClass: 'btn-deactivate' },
    { id: 'cmd-standby',    label: 'Standby',        command: 'standby',      cssClass: '' },
    { id: 'cmd-spacemouse', label: 'SpaceMouse',     command: 'spacemouse',   cssClass: '' },
    { id: 'cmd-shell',      label: 'Shell',          command: 'shell',        cssClass: '' },
    { id: 'cmd-gui',        label: 'GUI',            command: 'gui',          cssClass: '' },
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
            // Notify mode change listener for immediate UI response
            if (onModeChangeCallback && ['standby', 'gui', 'spacemouse', 'shell'].includes(cmd.command)) {
                onModeChangeCallback(cmd.command);
            }
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
    const sub = currentSubMode;
    const active = state === 'ACTIVE';

    setEnabled('cmd-home', state === 'IDLE' || state === 'BOOT');
    setEnabled('cmd-level', state === 'IDLE');
    setEnabled('cmd-activate', state === 'IDLE');
    setEnabled('cmd-deactivate', active);
    setEnabled('cmd-standby', active && sub !== 'STANDBY');
    setEnabled('cmd-spacemouse', active && sub !== 'SPACEMOUSE');
    setEnabled('cmd-shell', active && sub !== 'SHELL');
    setEnabled('cmd-gui', active && sub !== 'GUI');
    setEnabled('cmd-clear', true); // always available
}

function setEnabled(id, enabled) {
    const btn = document.getElementById(id);
    if (btn) btn.disabled = !enabled;
}
