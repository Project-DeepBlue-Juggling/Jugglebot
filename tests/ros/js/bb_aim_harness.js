/**
 * bb_aim_harness.js — drives the REAL ros_ws/gui/js/bb-aim.js under node
 * against a minimal fake DOM and a controllable fake `./ros-bridge.js`
 * whose `callService` can be told to hang forever (never resolve/reject).
 *
 * Driven by tests/ros/test_gui_bb_aim_timeout.py, which assembles a sandbox
 * exactly like tests/ros/test_gui_fk_golden.py does for stewart-fk.js: the
 * real GUI sources are copied VERBATIM (bb-aim.js, event-store.js,
 * geometry-config.js, and ros-bridge.js itself renamed to
 * ros-bridge-real.js so its real, unmodified `withTimeout` is what's under
 * test) next to a hand-written fake `ros-bridge.js` that re-exports that
 * real `withTimeout` but replaces `callService` with a controllable stub.
 * No bb-aim.js source is rewritten for this harness.
 *
 * Usage: node bb_aim_harness.js <scenario>
 *   concurrent — submit yaw, then (while it's still pending) submit pitch;
 *                asserts only ONE real callService call was made.
 *   timeout    — submit yaw against a call that never answers; waits past
 *                the client-side timeout and asserts the GUI's own status
 *                text recovers (does not stay stuck/blank forever).
 *
 * Nothing is asserted here; the scenario's observations are printed as JSON
 * on stdout and all comparison happens in Python, per the existing
 * fk_harness.js convention in this directory.
 */

import { __control } from './ros-bridge.js';
import {
    initBBAim,
    bbAimOnOrchestratorState,
    bbAimOnHeartbeat,
} from './bb-aim.js';

// ---- minimal fake DOM -------------------------------------------------

const idRegistry = new Map();

function makeClassList() {
    const set = new Set();
    return {
        add: (c) => set.add(c),
        remove: (c) => set.delete(c),
        toggle: (c, force) => {
            if (force === undefined) {
                if (set.has(c)) { set.delete(c); return false; }
                set.add(c);
                return true;
            }
            if (force) set.add(c); else set.delete(c);
            return force;
        },
        contains: (c) => set.has(c),
    };
}

function makeElement(tag) {
    const listeners = {};
    let text = '';
    let idVal = '';
    const el = {
        tagName: String(tag).toUpperCase(),
        children: [],
        parentNode: null,
        classList: makeClassList(),
        hidden: false,
        title: '',
        value: '',
        type: '',
        step: '',
        className: '',
        get id() { return idVal; },
        set id(v) { idVal = v; idRegistry.set(v, el); },
        get textContent() { return text; },
        set textContent(v) {
            text = String(v);
            for (const c of el.children) c.parentNode = null;
            el.children = [];
        },
        set innerHTML(html) { parseFragmentInto(el, html); },
        addEventListener(type, fn) { (listeners[type] = listeners[type] || []).push(fn); },
        // Test-only: invoke every listener registered for `type`.
        _fire(type, evt) { for (const fn of (listeners[type] || [])) fn(evt); },
        appendChild(child) { child.parentNode = el; el.children.push(child); return child; },
        remove() {
            if (el.parentNode) {
                el.parentNode.children = el.parentNode.children.filter((c) => c !== el);
                el.parentNode = null;
            }
        },
        after() { /* ordering is irrelevant to the logic under test */ },
        focus() {},
        select() {},
    };
    return el;
}

/** Parses the flat, non-nested markup bb-aim.js's initBBAim() assigns via
 *  innerHTML (a <span> and a <button>, each on one line, no children) into
 *  real fake elements registered by id — enough to resolve
 *  document.getElementById('bb-aim-status') / ('bb-aim-release')
 *  afterwards. Not a general HTML parser. */
function parseFragmentInto(parent, html) {
    const tagRe = /<(\w+)([^>]*)>([^<]*)<\/\1>/g;
    let m;
    while ((m = tagRe.exec(html))) {
        const [, tag, attrsStr, text] = m;
        const child = makeElement(tag);
        const attrRe = /([\w-]+)(?:="([^"]*)")?/g;
        let a;
        while ((a = attrRe.exec(attrsStr))) {
            const [, name, val] = a;
            if (name === 'id') child.id = val;
            else if (name === 'class') child.className = val;
            else if (name === 'hidden') child.hidden = true;
        }
        child.textContent = text;
        parent.appendChild(child);
    }
}

const readoutsEl = makeElement('div');

globalThis.document = {
    getElementById: (id) => idRegistry.get(id) || null,
    querySelector: (sel) => (sel === '#bb-content .bb-readouts' ? readoutsEl : null),
    createElement: (tag) => makeElement(tag),
};

// ---- scenario plumbing --------------------------------------------------

const BB_IDLE = 1;

function setup() {
    const bbYaw = makeElement('span');
    bbYaw.id = 'bb-yaw';
    const bbPitch = makeElement('span');
    bbPitch.id = 'bb-pitch';

    initBBAim();
    bbAimOnOrchestratorState('IDLE: standby');
    bbAimOnHeartbeat({ connected: true, yaw_deg: 10, pitch_deg: 45, state: BB_IDLE });

    return { bbYaw, bbPitch };
}

/** Click a readout, type `value` into the input it becomes, press Enter —
 *  the exact sequence bb-aim.js wires up for a real click + keystroke. */
function editAndSubmit(el, axis, value) {
    el._fire('click');
    const input = el.children[0];
    input.value = String(value);
    input._fire('keydown', { key: 'Enter', preventDefault() {} });
}

function statusText() {
    const el = idRegistry.get('bb-aim-status');
    return el ? el.textContent : null;
}

async function scenarioConcurrent() {
    __control.defaultBehavior = 'hang';
    const { bbYaw, bbPitch } = setup();

    editAndSubmit(bbYaw, 'yaw', 30);       // call #1 — hangs forever
    const callsAfterFirst = __control.calls.length;
    editAndSubmit(bbPitch, 'pitch', 50);   // must NOT fire a second real call

    return {
        scenario: 'concurrent',
        calls_after_first_submit: callsAfterFirst,
        calls_after_second_submit: __control.calls.length,
        status_after_second_submit: statusText(),
    };
}

async function scenarioTimeout() {
    __control.defaultBehavior = 'hang';
    const { bbYaw } = setup();

    editAndSubmit(bbYaw, 'yaw', 30);        // call #1 — hangs forever
    const statusImmediately = statusText();

    // AIM_TIMEOUT_MS is 3000 in the shipped bb-aim.js (not exported); wait
    // comfortably past it under real timers.
    await new Promise((resolve) => setTimeout(resolve, 3500));

    return {
        scenario: 'timeout',
        calls_total: __control.calls.length,
        status_immediately: statusImmediately,
        status_after_wait: statusText(),
    };
}

async function main() {
    const scenario = process.argv[2];
    let result;
    if (scenario === 'concurrent') result = await scenarioConcurrent();
    else if (scenario === 'timeout') result = await scenarioTimeout();
    else {
        process.stderr.write(`usage: node bb_aim_harness.js <concurrent|timeout>\n`);
        process.exit(2);
    }
    process.stdout.write(JSON.stringify(result));
}

main();
