/**
 * clock_harness.js — drives the real ros_ws/gui/js/clock.js under node and
 * prints one JSON object of observations. Assertions live in test_gui_clock.py.
 * Usage: node clock_harness.js   (clock.js must sit next to this file)
 */
import * as clock from './clock.js';

const out = {};
const fired = [];
const rec = (name) => () => fired.push(name);
const near = (a, b) => Math.abs(a - b) < 1000;

// wall mode delegates to Date.now
const w0 = Date.now(); const wn = clock.now(); const w1 = Date.now();
out.wall_now_in_range = wn >= w0 && wn <= w1;
out.wall_isReplay = clock.isReplay();
out.wall_settimeout_is_real = typeof clock.setTimeout(() => {}, 0) === 'object'
    || typeof clock.setTimeout(() => {}, 0) === 'number';
out.wall_pending = clock._pending();

// replay now follows _setNow
clock._enterReplay();
out.replay_isReplay = clock.isReplay();
clock._setNow(123456);
out.replay_now_a = clock.now();
clock._setNow(999000);
out.replay_now_b = clock.now();

// speed 1: 1000 ms timer
fired.length = 0;
clock.setTimeout(rec('a'), 1000);
clock._travel(999, 1);
out.s1_before = fired.slice();
clock._travel(1, 1);
out.s1_at = fired.slice();

// speed 4: fires at tau = 4000 not before
fired.length = 0;
clock.setTimeout(rec('b'), 1000);
clock._travel(3999, 4);
out.s4_before = fired.slice();
clock._travel(1, 4);
out.s4_at = fired.slice();

// speed 0.25 is clamped to 1 (no stretching below 1x)
fired.length = 0;
clock.setTimeout(rec('c'), 1000);
clock._travel(1000, 0.25);
out.slow_at = fired.slice();

// reverse: engine passes |delta|, negative speed still ages timers
fired.length = 0;
clock.setTimeout(rec('r'), 1000);
clock._travel(1000, -1);
out.reverse_at = fired.slice();

// frozen when _travel is not called (paused)
fired.length = 0;
clock.setTimeout(rec('p'), 1000);
clock._setNow(5e9);
out.frozen_fired = fired.slice();
out.frozen_pending = clock._pending();
clock._travel(0, 1);
out.zero_travel_fired = fired.slice();

// _clearTimers drops pending
clock._clearTimers();
clock._travel(10000, 1);
out.cleared_fired = fired.slice();
out.cleared_pending = clock._pending();

// clearTimeout cancels one
fired.length = 0;
const idx = clock.setTimeout(rec('x'), 100);
clock.setTimeout(rec('y'), 100);
clock.clearTimeout(idx);
clock._travel(100, 1);
out.clear_one = fired.slice();

// deadline order, once; arming order differs from deadline order
fired.length = 0;
clock.setTimeout(rec('t300'), 300);
clock.setTimeout(rec('t100'), 100);
clock.setTimeout(rec('t200'), 200);
clock._travel(500, 1);
clock._travel(500, 1);
out.order = fired.slice();

// timer armed after partial travel is relative to arming time
fired.length = 0;
clock._travel(700, 1);
clock.setTimeout(rec('late'), 1000);
clock._travel(999, 1);
out.rel_before = fired.slice();
clock._travel(1, 1);
out.rel_at = fired.slice();

// speed change mid-wait: 1000 ms timer, 2000 of travel at 4x (=500 wall) + 500 at 1x
fired.length = 0;
clock.setTimeout(rec('mix'), 1000);
clock._travel(2000, 4);
out.mix_before = fired.slice();
clock._travel(500, 1);
out.mix_at = fired.slice();

// _exitReplay cancels and restores wall time
fired.length = 0;
clock.setTimeout(rec('z'), 10);
clock._exitReplay();
out.exit_pending = clock._pending();
out.exit_isReplay = clock.isReplay();
clock._travel(10000, 1);
out.exit_fired = fired.slice();
const e0 = Date.now(); const en = clock.now(); const e1 = Date.now();
out.exit_now_wall = en >= e0 && en <= e1;

// ---- hardening: ids, throwing callbacks, fuse ----
{
    const realClear = globalThis.clearTimeout;
    const cleared = [];
    globalThis.clearTimeout = (id) => { cleared.push(id); return realClear(id); };
    const origErr = console.error; const errs = [];
    console.error = (...a) => errs.push(a.map(String).join(' '));

    // (d1) a virtual id cleared after _exitReplay never reaches the real clearTimeout
    clock._enterReplay();
    const vid = clock.setTimeout(() => {}, 100);
    out.virtual_id_negative = typeof vid === 'number' && vid < 0;
    clock._exitReplay();
    clock.clearTimeout(vid);
    out.stale_virtual_forwarded = cleared.includes(vid);

    // (d2) a REAL id cleared during replay is really cancelled
    let realFired = false;
    const rid = clock.setTimeout(() => { realFired = true; }, 30);   // wall mode: real timer
    clock._enterReplay();
    clock.clearTimeout(rid);
    out.real_id_forwarded = cleared.includes(rid);
    clock._exitReplay();
    await new Promise((r) => setTimeout(r, 80));
    out.real_timer_fired = realFired;

    // (b1) a throwing callback is logged and the remaining timers still fire
    clock._enterReplay();
    const f2 = [];
    clock.setTimeout(() => { throw new Error('boom-timer'); }, 10);
    clock.setTimeout(() => f2.push('after'), 20);
    let threw = false;
    try { clock._travel(50, 1); } catch (e) { threw = true; }
    out.throw_timer = { propagated: threw, later_fired: f2.slice(), logged: errs.some((x) => x.includes('boom-timer')) };

    // (e) a zero-delay re-arming timer is stopped by the fuse
    let count = 0;
    const loop = () => { count++; clock.setTimeout(loop, 0); };
    clock.setTimeout(loop, 0);
    clock._travel(1, 1);
    out.fuse = { count, pending: clock._pending(), logged: errs.some((x) => x.includes('fuse')) };
    clock._exitReplay();

    console.error = origErr;
    globalThis.clearTimeout = realClear;
}

console.log(JSON.stringify(out));
