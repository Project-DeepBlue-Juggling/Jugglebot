// replay_fence_harness.js — the REAL commands.js (+ ros-bridge, event-store, fence, clock, hold-to-confirm) under
// node with a fake DOM and a STUB panels.js (currentOrchestratorState = 'IDLE'). Asserts the command fence:
// (c1) fenced updateCommandStates disables every cmd-* button even though the live state is IDLE;
// (c2) a click while fenced emits no COMMAND event; (c3) the same click unfenced does (control).
globalThis.window = { location: { hostname: 'localhost' }, addEventListener() {} };
class Ros { constructor() { this.isConnected = false; } on() {} connect() {} close() {} }
class Topic { constructor() {} subscribe() {} unsubscribe() {} publish() {} }
globalThis.ROSLIB = { Ros, Topic, Service: function () {}, Message: (m) => m, ServiceRequest: (r) => r };

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

const { initCommands, updateCommandStates } = await import('./commands.js');
const { setReplayFence } = await import('./replay/fence.js');
const ev = await import('./event-store.js');
const ids = ['cmd-home', 'cmd-level', 'cmd-activate', 'cmd-deactivate', 'cmd-clear'];
const out = {};
const nCommands = () => ev.getRecentEvents(null, 500).filter((e) => e.type === 'command').length;

initCommands();
updateCommandStates();                       // live IDLE: home/level/activate enabled
out.live_disabled = ids.filter((i) => buttons.get(i).disabled);
setReplayFence(true);
updateCommandStates();                       // what resetForSeek now ends with
out.fenced_disabled = ids.filter((i) => buttons.get(i).disabled);
const n0 = nCommands();
buttons.get('cmd-home').listeners.click[0]();
out.fenced_click_events = nCommands() - n0;
setReplayFence(false);
buttons.get('cmd-home').listeners.click[0]();
out.unfenced_click_events = nCommands() - n0;
console.log(JSON.stringify(out));
process.exit(0);
