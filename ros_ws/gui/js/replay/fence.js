/**
 * fence.js — the replay affordance fence (replay design § 3, layer 2).
 *
 * One switch: the replay mode module calls setReplayFence(true) on entry and
 * (false) on exit. It toggles `body.replay` (CSS is Phase 3) and every
 * command affordance consults isReplayFenced() through an early return:
 * commands.js updateCommandStates, jog-panel.js setJogPanelVisible /
 * setSpeedLimitsPanelVisible, bb-aim.js allowed(), and the panels.js Ball
 * Butler calibrate / reset / throw click handlers. Layer 1 (the transport
 * refusal in ros-bridge.js) is keyed on clock.isReplay() instead, so either
 * layer alone still blocks a live publish. Dependency-free (node harnesses).
 */

let active = false;

/** @param {boolean} on */
export function setReplayFence(on) {
    active = !!on;
    const doc = globalThis.document;
    if (doc && doc.body && doc.body.classList) doc.body.classList.toggle('replay', active);
}

/** @returns {boolean} true while a replay owns the screen. */
export function isReplayFenced() { return active; }
