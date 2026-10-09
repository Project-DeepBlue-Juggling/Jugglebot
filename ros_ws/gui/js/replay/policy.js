/**
 * policy.js — the replay dispatch policy: one row per converted topic
 * (replay design § 4 topic classes, § 7 gates). Pure data + pure helpers;
 * pinned against ros_ws/gui/replay/schema.py SUBSCRIBED + PLANNED by
 * tests/ros/test_gui_replay_policy.py.
 *
 * Classes:
 *   state-render  latest-before suffices (seek: baseline + one record at p)
 *   state-edge    handler emits events on change (seek: baseline muted, then
 *                 the pre-roll window forward at the 1x gates, events on)
 *   history-ring  panel differences consecutive samples (pre-roll only; never
 *                 dispatched in reverse)
 *   event         every record is dispatched (never gated, never in reverse)
 *   columns-only  never dispatched; charts / trails read chunk columns
 *
 * throttleMs is the live rosbridge throttle from main.js subscribeAll()
 * (0 = unthrottled). Gate rule (§ 7, playhead time):
 *   dispatch a record if  t - t_last >= gate * max(1, |speed|)
 *   gate = throttleMs/1000, or for an UNTHROTTLED PERIODIC topic above 1x the
 *   chunk's recorded mean period ((t1 - t0) / n); 0 at or below 1x.
 * onChange: also dispatch any record whose `data` differs from the last
 * dispatched one (orchestrator_state, control_mode_topic).
 * every: dispatch every record (event topics; bb/calibration_result, bb/calibration_attempt, a rare
 * latched edge whose second publish must not be gated away at 8x).
 */

export const W_PREROLL_SEC = 30;
export const SPEED_LADDER = Object.freeze([0.25, 0.5, 1, 2, 4, 8]);

export const CLASSES = Object.freeze({
    STATE_RENDER: 'state-render',
    STATE_EDGE: 'state-edge',
    HISTORY: 'history-ring',
    EVENT: 'event',
    COLUMNS: 'columns-only',
});

const R = CLASSES.STATE_RENDER;
const E = CLASSES.STATE_EDGE;
const H = CLASSES.HISTORY;
const V = CLASSES.EVENT;
const C = CLASSES.COLUMNS;

/** topic -> {cls, throttleMs, onChange?, every?} — keys are chunk names ('/x'). */
export const TOPIC_POLICY = Object.freeze({
    '/robot_state':           { cls: E, throttleMs: 50 },
    '/orchestrator_state':    { cls: E, throttleMs: 0, onChange: true },
    '/bb/calibration_result': { cls: E, throttleMs: 0, every: true },
    '/bb/calibration_attempt': { cls: E, throttleMs: 0, every: true },
    '/mocap_data':            { cls: R, throttleMs: 50 },
    '/rigid_body_poses':      { cls: R, throttleMs: 50 },
    '/hand_telemetry':        { cls: R, throttleMs: 100 },
    '/leg_setpoint_echo':     { cls: R, throttleMs: 50 },
    '/motion/diagnostics':    { cls: R, throttleMs: 200 },
    '/bb/heartbeat':          { cls: R, throttleMs: 0 },
    '/cone/heartbeat':        { cls: R, throttleMs: 0 },
    '/control_mode_topic':    { cls: R, throttleMs: 0, onChange: true },
    '/profile':               { cls: H, throttleMs: 0 },
    '/udp_diag':              { cls: H, throttleMs: 0 },
    '/clock_diag':            { cls: H, throttleMs: 0 },
    '/link_status':           { cls: H, throttleMs: 0 },
    '/skills/attempt':        { cls: V, throttleMs: 0 },
    '/cone/timing_result':    { cls: V, throttleMs: 0 },
    '/cone/catch_event':      { cls: V, throttleMs: 0 },
    '/balls':                 { cls: C, throttleMs: 0 },
});

/** @returns {string|null} the topic's class, null for an unknown topic. */
export function classOf(topic) {
    const p = TOPIC_POLICY[topic.charAt(0) === '/' ? topic : '/' + topic];
    return p ? p.cls : null;
}

/** @returns {object|null} the policy row. */
export function policyOf(topic) {
    return TOPIC_POLICY[topic.charAt(0) === '/' ? topic : '/' + topic] || null;
}

/** True for the two state classes (baseline / settle / reverse candidates). */
export function isStateClass(cls) { return cls === R || cls === E; }

/**
 * Minimum playhead seconds between two dispatched records of `topic`.
 * @param {string} topic
 * @param {number} speed signed playback speed
 * @param {{t0:number, t1:number, topics:object}} [chunk] the chunk the record is in
 * @returns {number} seconds (0 = every record); Infinity for columns-only/unknown
 */
export function gateFor(topic, speed, chunk) {
    const p = policyOf(topic);
    if (!p || p.cls === C) return Infinity;
    if (p.cls === V || p.every) return 0;
    const s = Math.max(1, Math.abs(+speed || 0));
    if (p.throttleMs > 0) return (p.throttleMs / 1000) * s;
    if (s <= 1) return 0;
    const tp = chunk && chunk.topics ? chunk.topics[topic.charAt(0) === '/' ? topic : '/' + topic] : null;
    if (!tp || !(tp.n > 0)) return 0;
    const span = (chunk.t1 - chunk.t0) > 0 ? (chunk.t1 - chunk.t0) : 10;
    return (span / tp.n) * s;
}

/** Snap a speed to the ladder by magnitude, keeping its sign (0 -> 1). */
export function snapSpeed(x) {
    const v = +x;
    if (!Number.isFinite(v) || v === 0) return 1;
    const a = Math.abs(v);
    let best = SPEED_LADDER[0];
    for (const r of SPEED_LADDER) if (Math.abs(Math.log(r / a)) < Math.abs(Math.log(best / a))) best = r;
    return v < 0 ? -best : best;
}
