// replay_bridge_harness.js — loads the REAL ros_ws/gui/js/ros-bridge.js (and
// clock.js) under node with a minimal ROSLIB / window stub and prints JSON;
// tests/ros/test_gui_replay_bridge.py asserts on it.
const rosInstances = [];
const published = [];
const services = [];
class Ros {
    constructor() { this.handlers = {}; this.isConnected = false; rosInstances.push(this); }
    on(evt, cb) { this.handlers[evt] = cb; }
    connect() {}
    close() {}
    fire(evt) { this.isConnected = evt === 'connection'; if (this.handlers[evt]) this.handlers[evt](); }
}
class Topic {
    constructor(o) { this.o = o; }
    subscribe(cb) { this.cb = cb; }
    unsubscribe() {}
    publish(m) { published.push({ name: this.o.name, msg: m }); }
}
class Service {
    constructor(o) { this.o = o; services.push(o.name); }
    callService(req, ok) { ok({ success: true }); }
}
globalThis.ROSLIB = { Ros, Topic, Service, Message: function (m) { return m; }, ServiceRequest: function (r) { return r; } };
globalThis.window = { location: { hostname: 'localhost' }, addEventListener() {} };

const clock = await import('./clock.js');
const ros = await import('./ros-bridge.js');

const out = {};
const got = [];
ros.subscribe('robot_state', 'T', (m) => got.push(['robot_state', m.v]), 50);
ros.subscribe('bb/heartbeat', 'T', (m) => got.push(['bb/heartbeat', m.v]), 0);
ros.subscribe('robot_state', 'T', (m) => got.push(['robot_state#2', m.v]), 0);
ros.subscribe('thrower', 'T', () => { throw new Error('boom'); }, 0);
const origErr = console.error;
console.error = () => {};
out.dispatch = {
    slash: ros.dispatchLocal('/robot_state', { v: 1 }),
    bare: ros.dispatchLocal('bb/heartbeat', { v: 2 }),
    unknown: ros.dispatchLocal('/nope', { v: 3 }),
    thrower: ros.dispatchLocal('/thrower', { v: 4 }),
    state_after: ros.getConnectionState(),
};
console.error = origErr;
out.got = got;

const order = [];
let hookCalls = 0;
ros.setBeforeConnectedHook(() => { hookCalls++; order.push('hook'); });
ros.onConnectionStateChange((s) => order.push('listener:' + s));
ros.init('ws://localhost:9090');
const r = rosInstances[rosInstances.length - 1];
r.fire('connection');
r.fire('close');
r.fire('connection');
out.connect = { order, hookCalls };

const pub = ros.advertise('cmd', 'std_msgs/msg/String');
pub.publish({ data: 'live' });
clock._enterReplay();
pub.publish({ data: 'replay' });
let replayErr = null;
try { await ros.callService('bb/calibrate', 'std_srvs/srv/Trigger'); } catch (e) { replayErr = e.message; }
const servicesDuringReplay = services.length;
clock._exitReplay();
let liveRes = null;
try { liveRes = await ros.callService('bb/calibrate', 'std_srvs/srv/Trigger'); } catch (e) { liveRes = 'err ' + e.message; }
out.fence = { published: published.map((p) => p.msg.data), replayErr, servicesDuringReplay, liveRes, servicesAfter: services.length };

process.stdout.write(JSON.stringify(out));
process.exit(0);  // the stale-check interval would keep node alive
