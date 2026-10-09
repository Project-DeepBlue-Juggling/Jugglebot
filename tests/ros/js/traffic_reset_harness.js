/**
 * traffic_reset_harness.js — drives the REAL can-traffic.js and udp-traffic.js
 * under node against a fake DOM + a recording fake uPlot, and prints one JSON
 * object. Assertions live in test_gui_traffic_reset.py (wip_ until green).
 * Sandbox: real can-traffic.js, udp-traffic.js, clock.js, geometry-config.js
 * plus a stub telemetry-charts.js exporting nanGaps.
 */
const els = new Map();
function mk(id) {
    const e = {
        id, innerHTML: '', textContent: '', title: '', style: { setProperty() {} }, dataset: {},
        clientWidth: 400, clientHeight: 200, scrollTop: 0, append() {}, replaceChildren() {},
        classList: { add() {}, remove() {}, toggle() {}, contains: () => false },
        addEventListener() {}, setAttribute() {}, appendChild() {}, remove() {},
        querySelectorAll: () => [], querySelector: () => null,
    };
    return e;
}
globalThis.document = {
    getElementById: (id) => { if (!els.has(id)) els.set(id, mk(id)); return els.get(id); },
    createElement: (t) => mk(t),
    documentElement: {},
};
globalThis.window = { devicePixelRatio: 1 };
globalThis.getComputedStyle = () => ({ getPropertyValue: () => '' });
const store = new Map();
globalThis.localStorage = {
    getItem: (k) => (store.has(k) ? store.get(k) : null),
    setItem: (k, v) => store.set(k, String(v)),
};
let lastData = null;
globalThis.uPlot = class {
    constructor(opts, data) { this.data = data; this.series = opts.series; this.over = mk('over'); lastData = data; }
    setData(d) { this.data = d; lastData = d; }
    setScale() {} setSize() {} redraw() {} destroy() {}
};

const can = await import('./can-traffic.js');
const udp = await import('./udp-traffic.js');
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
const kvs = (o) => ({ values: Object.entries(o).map(([key, value]) => ({ key, value: String(value) })) });
const out = {};

// ---- CAN ----
els.set('can-chart', mk('can-chart'));
can.initCanTrafficPanel();
const prof = (n) => kvs({ can1_rx: n, can1_tx: n, can2_rx: n, can2_tx: 0, can3_rx: 0, can3_tx: 0 });
for (const n of [100, 200, 300]) can.canTrafficOnProfile(prof(n));
out.can_before = { cols: lastData[0].length, rate2: lastData[2].slice() };
can.resetTrafficRing();
out.can_after_reset = { cols: lastData[0].length, rates: lastData.slice(1).map((a) => a.length) };
can.canTrafficOnProfile(prof(50));
out.can_next = { cols: lastData[0].length, rate2: lastData[2].slice() };

// ---- UDP ----
store.set('jugglebot-topics-mode', 'udp');
els.set('udp-table-container', mk('udp-table-container'));
udp.initUdpTrafficPanel();
udp.udpTrafficOnLinkStatus(kvs({ bridge_link: 'UP', latency_monitor: 'OK' }));
udp.udpTrafficOnDiag(kvs({ rx_frames: 1000, tx_frames: 500, crc_errors: 0, decode_errors: 0, drain_capped: 0, seq_gaps: 0, rx_SETPOINT: 1000 }));
await sleep(1100);
udp.udpTrafficOnDiag(kvs({ rx_frames: 1100, tx_frames: 540, crc_errors: 0, decode_errors: 0, drain_capped: 0, seq_gaps: 0, rx_SETPOINT: 1100 }));
out.udp_before = els.get('udp-rows').innerHTML;
udp.resetTrafficRing();
out.udp_after_reset = els.get('udp-rows').innerHTML;
udp.udpTrafficOnDiag(kvs({ rx_frames: 5, tx_frames: 2, crc_errors: 0, decode_errors: 0, drain_capped: 0, seq_gaps: 0, rx_SETPOINT: 5 }));
out.udp_next = els.get('udp-rows').innerHTML;
console.log(JSON.stringify(out));
process.exit(0);
