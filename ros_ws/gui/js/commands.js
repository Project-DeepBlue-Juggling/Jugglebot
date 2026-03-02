/**
 * commands.js — Command button handlers.
 *
 * Creates command buttons in the overlay bar at the bottom of the 3D viewer.
 * Publishes to orchestrator_command topic. Context-sensitive enable/disable.
 */

import * as ros from './ros-bridge.js';
import { currentOrchestratorState } from './panels.js';

/** @type {{ publish: function } | null} */
let cmdPublisher = null;

const COMMANDS = [
    { id: 'cmd-home',       label: 'Home',           command: 'home',         cssClass: 'btn-home' },
    { id: 'cmd-level',      label: 'Level',          command: 'level',        cssClass: 'btn-home' },
    { id: 'cmd-activate',   label: 'Activate',       command: 'activate',     cssClass: 'btn-activate' },
    { id: 'cmd-deactivate', label: 'Deactivate',     command: 'deactivate',   cssClass: 'btn-deactivate' },
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

        btn.addEventListener('click', () => {
            if (cmdPublisher) {
                cmdPublisher.publish({ data: cmd.command });
            }
        });

        overlay.appendChild(btn);
    }
}

/**
 * Update button enabled/disabled states based on current orchestrator state.
 * Called after each orchestrator_state message.
 */
export function updateCommandStates() {
    const state = currentOrchestratorState;

    setEnabled('cmd-home', state === 'IDLE' || state === 'BOOT');
    setEnabled('cmd-level', state === 'IDLE');
    setEnabled('cmd-activate', state === 'IDLE');
    setEnabled('cmd-deactivate', state === 'ACTIVE');
    setEnabled('cmd-spacemouse', state === 'ACTIVE');
    setEnabled('cmd-shell', state === 'ACTIVE');
    setEnabled('cmd-gui', state === 'ACTIVE');
    setEnabled('cmd-clear', true); // always available
}

function setEnabled(id, enabled) {
    const btn = document.getElementById(id);
    if (btn) btn.disabled = !enabled;
}
