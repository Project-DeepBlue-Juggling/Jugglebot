// command_inflight_harness.js — the REAL commands.js under node with a fake DOM, a MUTABLE stub panels.js and
// captured timers. Asserts the BUG-104 in-flight guard: a double click publishes once; the latch releases on an
// orchestrator state CHANGE (not on a same-state re-publish) or on cooldown expiry. The clear-errors hold-to-confirm path is not exercised here.
globalThis.window = { location: { hostname: 'localhost' }, addEventListener() {} };
class Ros { constructor() { this.isConnected = false; } on() {} connect() {} close() {} }
class Topic { constructor(o) { this.name = o && o.name; } subscribe() {} unsubscribe() {} publish() {} }
globalThis.ROSLIB = { Ros, Topic, Service: function () {}, Message: (m) => m, ServiceRequest: (r) => r };

let timers = [];
globalThis.setTimeout = (f, ms) => { const t = { f, ms, live: true }; timers.push(t); return t; };
globalThis.clearTimeout = (t) => { if (t) t.live = false; };
const fireTimers = () => { const t = timers.filter((x) => x.live); timers = []; t.forEach((x) => { x.live = false; x.f(); }); return t.length; };

const buttons = new Map();
const mkBtn = () => {
    const l = {};
    return {
        id: '', className: '', textContent: '', disabled: false, title: '', listeners: l,
        classList: { add() {}, remove() {} }, style: { setProperty() {} },
        addEventListener(t, f) { (l[t] = l[t] || []).push(f); },
    };
};
const overlay = { appendChild(b) { buttons.set(b.id, b); } };
globalThis.document = {
    body: { classList: { toggle() {} } },
    createElement: () => mkBtn(),
    getElementById: (id) => (id === 'command-overlay' ? overlay : (buttons.get(id) || null)),
};

const panels = await import('./panels.js');
const { initCommands, updateCommandStates } = await import('./commands.js');
const ev = await import('./event-store.js');
const published = { get length() { return list().length; }, slice: () => list() };
const list = () => ev.getRecentEvents(null, 500).filter((e) => e.type === 'command').map((e) => e.detail.split(': ')[1]).reverse();
const ids = ['cmd-home', 'cmd-level', 'cmd-activate', 'cmd-deactivate', 'cmd-clear'];
const disabled = () => ids.filter((i) => buttons.get(i).disabled);
const click = (id) => buttons.get(id).listeners.click[0]();
const out = {};

initCommands();
updateCommandStates();                                   // IDLE: only deactivate disabled
out.idle_disabled = disabled();

click('cmd-level'); click('cmd-level'); click('cmd-activate');   // double click + a different button
out.double_click_published = published.slice();
out.latched_disabled = disabled();

updateCommandStates();                                   // same-state re-publish: must NOT release
out.same_state_release = disabled().length === 0;
click('cmd-level');
out.after_same_state_published = published.length;

panels.__setState('LEVELLING'); updateCommandStates();   // state changed: release
out.after_change_disabled = disabled();
panels.__setState('IDLE'); updateCommandStates();
click('cmd-level');
out.after_release_published = published.slice();

fireTimers();                                            // cooldown expiry releases without any state change
click('cmd-home');
out.after_cooldown_published = published.slice();
console.log(JSON.stringify(out));
process.exit(0);
