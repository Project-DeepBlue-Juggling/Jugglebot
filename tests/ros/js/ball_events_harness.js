// ball_events_harness.js - the REAL ros_ws/gui/js/ball-events.js under node. Sandbox built by tests/ros/test_gui_ball_events.py.
import { createBallEventMapper, formatClock } from './ball-events.js';

const T0 = 1795175860;
const ts = (s) => ({ sec: Math.floor(T0 + s), nanosec: Math.round(((T0 + s) % 1) * 1e9) });
const ball = (o) => Object.assign({
  id: 12, status: 1, source: 'jugglebot', destination: 'jugglebot', landing_from_fit: true,
  header: { stamp: ts(0) }, throw_time: ts(-0.5), time_at_land: ts(0.4),
}, o);
const out = {};

// 1. Hand Throw while in flight: one 'throw' at throw_time, once; replay keeps recorded times
let m = createBallEventMapper();
out.throw = m.map([ball({})], { nowSec: T0, replay: true });
out.throwAgain = m.map([ball({})], { nowSec: T0 + 0.1, replay: true });
// 2. not yet thrown (status 0), Ball Butler feed (source), unannounced throw_time: no Throw
m = createBallEventMapper();
out.pending = m.map([ball({ status: 0 })], { nowSec: T0, replay: true });
out.bb = m.map([ball({ id: 11, source: 'ball_butler' })], { nowSec: T0, replay: true });
out.noTime = m.map([ball({ id: 13, throw_time: { sec: 0, nanosec: 0 } })], { nowSec: T0, replay: true });
// 3. Catch at the fit landing time, not at CAUGHT arrival; predicted label without a fit; dropped / foreign destination: none
m = createBallEventMapper();
out.catchFit = m.map([ball({ status: 2 })], { nowSec: T0 + 0.9, replay: true }).filter((e) => e.type === 'catch');
m = createBallEventMapper();
out.catchPred = m.map([ball({ id: 14, status: 2, landing_from_fit: false })], { nowSec: T0 + 0.9, replay: true }).filter((e) => e.type === 'catch');
m = createBallEventMapper();
out.dropped = m.map([ball({ id: 15, status: 3 })], { nowSec: T0, replay: true });
out.foreign = m.map([ball({ id: 16, status: 2, destination: 'cone' })], { nowSec: T0, replay: true }).filter((e) => e.type === 'catch');
// 4. live: header.stamp maps ROS time onto the browser clock (browser 100 s ahead of the Jetson)
m = createBallEventMapper();
out.live = m.map([ball({})], { nowSec: T0 + 100 + 0.02, replay: false });
// 5. muted baseline does not mark done; backward seek re-emits; forward replay does not duplicate
m = createBallEventMapper();
out.muted1 = m.map([ball({})], { nowSec: T0, replay: true, muted: true }).length;
out.muted2 = m.map([ball({})], { nowSec: T0 + 0.05, replay: true }).length;
out.muted3 = m.map([ball({})], { nowSec: T0 + 0.1, replay: true }).length;
out.afterSeek = m.map([ball({})], { nowSec: T0 - 20, replay: true }).length;
// 6. formatting
out.clock = /^\d\d:\d\d:\d\d\.\d\d$/.test(formatClock(T0 + 0.426));
process.stdout.write(JSON.stringify(out));
