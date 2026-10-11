// trail_feed_harness.js — drives the real trail-feed.js against a FAKE layer that records calls.
// Sandbox layout: ./clock.js ./marker-palette.js ./trail-feed.js beside this file. Prints one JSON object.
import { createTrailFeed, BALL_KEY } from './trail-feed.js';

function fakeLayer() {
    const calls = [];
    return {
        calls,
        colorFn: null,
        setColorFn(fn) { this.colorFn = fn; },
        beginMessage(tag) { calls.push(['begin', tag]); },
        push(tag, key, t, x, y, z) { calls.push(['push', tag, key, t, x, y, z]); },
        endMessage(tag, t) { calls.push(['endMsg', tag, t]); },
        end(key, t) { calls.push(['end', key, t]); },
        render(now, tail) { calls.push(['render', now, tail]); return 0; },
        reset() { calls.push(['reset']); },
    };
}
const pushes = (l, tag) => l.calls.filter((c) => c[0] === 'push' && c[1] === tag).map((c) => c[2]);
const out = {};
let nowV = 0;
const mk = () => { const l = fakeLayer(); return [l, createTrailFeed(l, { now: () => nowV })]; };
const mocap = (f, t, ms) => { f.beginMocap(t); for (const m of ms) f.marker(t, m[0], m[1], m[2], m[3]); f.endMocap(t); };

{   // label keying
    const [l, f] = mk();
    mocap(f, 1, [['Platform1', 0, 0, 0], ['Base2', 1, 1, 1]]);
    out.label_keys = pushes(l, 'mocap');
    out.colour_fn = [l.colorFn('Platform1', 'mocap'), l.colorFn('Base2', 'mocap'), l.colorFn(-3, 'mocap'),
        l.colorFn(BALL_KEY + 0, 'balls'), l.colorFn(BALL_KEY + 1, 'balls')];
}
{   // NN continuity + new identity
    const [l, f] = mk();
    mocap(f, 1.00, [['', 0, 0, 0]]);
    mocap(f, 1.01, [['', 10, 0, 0]]);          // within gate -> same key
    mocap(f, 1.02, [['', 500, 0, 0]]);         // far -> new identity
    mocap(f, 1.03, [['', 505, 0, 0]]);         // follows the new one
    out.nn_keys = pushes(l, 'mocap');
    // memory expiry: same spot after 0.6 s is a new identity
    const [l2, f2] = mk();
    mocap(f2, 0, [['', 0, 0, 0]]); mocap(f2, 0.6, [['', 0, 0, 0]]);
    out.nn_memory_keys = pushes(l2, 'mocap');
    // two unlabelled in one message take two slots, stable across messages
    const [l3, f3] = mk();
    mocap(f3, 0, [['', 0, 0, 0], ['', 300, 0, 0]]); mocap(f3, 0.01, [['', 301, 0, 0], ['', 1, 0, 0]]);
    out.nn_two_keys = pushes(l3, 'mocap');
}
{   // D3 suppression + far unlabelled still trails + no NN slot consumed
    const [l, f] = mk();
    f.beginBalls(1); f.ball(1, 0, 1, 100, 100, 500); f.endBalls(1);
    mocap(f, 1.01, [['', 110, 100, 500], ['', 400, 400, 400], ['Platform1', 110, 100, 500]]);
    out.d3_keys = pushes(l, 'mocap');
    // a suppressed marker takes no NN slot: next far marker still gets key -1
    out.d3_far_key_first_slot = out.d3_keys[0] === -1;
    // after the ball goes stale the same marker trails again
    mocap(f, 1.5, [['', 110, 100, 500]]);
    out.d3_after_stale = pushes(l, 'mocap').length;
}
{   // D1 stale end + balls().n
    const [l, f] = mk();
    f.beginBalls(2.0); f.ball(2.0, 3, 1, 0, 0, 0); f.endBalls(2.0);
    nowV = 2.1; f.setRenderTime(null);
    const b1 = f.balls(); out.balls_n_fresh = b1.n; out.balls_id0 = b1.id[0];
    out.balls_same_object = f.balls() === b1;
    f.tick(2.1); out.ends_before = l.calls.filter((c) => c[0] === 'end').length;
    f.tick(2.2);                                  // 200 ms > 150
    out.d1_ends = l.calls.filter((c) => c[0] === 'end');
    out.balls_n_stale = f.balls().n;
    f.tick(2.3); out.d1_ends_after_second_tick = l.calls.filter((c) => c[0] === 'end').length;
    // render in live mode ticks first
    const [l2, f2] = mk();
    f2.beginBalls(5); f2.ball(5, 0, 1, 0, 0, 0); f2.endBalls(5);
    nowV = 5.5; f2.render(1000);
    out.render_live = l2.calls.filter((c) => c[0] === 'end' || c[0] === 'render');
    // render with a render time does NOT tick, uses it
    const [l3, f3] = mk();
    f3.beginBalls(5); f3.ball(5, 0, 1, 0, 0, 0); f3.endBalls(5);
    f3.setRenderTime(9); f3.render(1000);
    out.render_replay = l3.calls.filter((c) => c[0] === 'end' || c[0] === 'render');
    out.render_replay_balls_n = f3.balls().n;
    f3.render(0); out.render_tail0 = l3.calls[l3.calls.length - 1];
}
{   // absence end: id gone from a later message
    const [l, f] = mk();
    f.beginBalls(1); f.ball(1, 0, 1, 0, 0, 0); f.ball(1, 1, 1, 9, 9, 9); f.endBalls(1);
    f.beginBalls(1.005); f.ball(1.005, 0, 1, 0, 0, 0); f.endBalls(1.005);
    out.absence = l.calls.filter((c) => c[0] === 'endMsg' || c[0] === 'begin').map((c) => c.join(':'));
    f.setRenderTime(1.005);
    const b = f.balls(); out.absence_n = b.n; out.absence_ids = [b.id[0]];
}
{   // reset clears everything
    const [l, f] = mk();
    mocap(f, 1, [['', 0, 0, 0]]);
    f.beginBalls(1); f.ball(1, 0, 1, 0, 0, 0); f.endBalls(1);
    f.setRenderTime(1.0);
    f.reset();
    out.reset_called = l.calls.filter((c) => c[0] === 'reset').length;
    out.reset_balls_n = f.balls().n;
    mocap(f, 1.01, [['', 0, 0, 0]]);              // NN slot state cleared: key restarts at -1
    out.reset_nn_key = pushes(l, 'mocap').slice(-1)[0];
    f.tick(99); out.reset_no_end = l.calls.filter((c) => c[0] === 'end').length;
}
{   // D1 across a gap with no tick between messages (a rebuilt replay window): the old track ends at ITS last time
    const [l, f] = mk();
    f.beginBalls(1.0); f.ball(1.0, 5, 1, 0, 0, 0); f.endBalls(1.0);
    f.beginBalls(1.5); f.ball(1.5, 6, 1, 500, 0, 0); f.endBalls(1.5);
    out.gap_end = l.calls.filter((c) => c[0] === 'end');
    nowV = 1.5; f.setRenderTime(null);
    out.gap_balls = [f.balls().n, f.balls().id[0]];
}
console.log(JSON.stringify(out));
