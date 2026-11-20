/**
 * ball-events.js - Throw and Catch chart markers from the tracker's /balls stream (pure; no DOM, no ROS).
 *
 *   const m = createBallEventMapper();
 *   for (const ev of m.map(balls, {nowSec, replay, muted})) emitEvent(ev);
 *
 * Source choice (2026-11-21 investigation, bag 2026-11-20_22-53-34, an oval_1ball_fed Attempt): /balls is on the replay allowlist
 * and subscribed live, and one BallState carries both instants:
 *   Throw  = BallState.throw_time, the ANNOUNCED release instant of the track (the schedule's planned release, identical to the
 *            ThrowAnnouncement). Only Hand Throws (source 'jugglebot') get a marker; Ball Butler Feeds do not. It is a PLANNED
 *            time, so the label says so. The Hand's ball sensor flips about 0.1 s after the release (debounce plus poll) and
 *            /skills/attempt is one message per Attempt, so neither is a better per-Throw source.
 *   Catch  = BallState.time_at_land once status is CAUGHT: the landing-plane crossing of the converged gravity-fixed fit
 *            (landing_from_fit), a measured flight time; the CAUGHT status itself arrives 0.2-0.5 s later and is not used as a
 *            time. Without a fit the predicted time is used and the label says "predicted".
 * Live, ROS times are mapped onto the browser clock by the message's own header.stamp (now - (stamp - eventTime)), so clock skew
 * between the Jetson and the browser cancels and only the ~2-30 ms rosbridge latency remains. Replay uses the recorded times as they are.
 *
 * Replay re-delivers a span after every seek, and the event store dedups on (t, type, label), so a replay mapping stays idempotent:
 * labels are deterministic (the ball id), and a MUTED pass (a seek baseline) never marks a ball as done.
 */

export const BALL_STATUS = Object.freeze({ TO_BE_THROWN: 0, IN_FLIGHT: 1, CAUGHT: 2, DROPPED: 3, UNKNOWN: 4 });

const SOURCE_HAND = 'jugglebot';
const MAX_TRACKED = 256;

function toSec(t) {
    return t && typeof t.sec === 'number' ? t.sec + (t.nanosec || 0) * 1e-9 : 0;
}

/** HH:MM:SS.cc, local time of the machine rendering it. */
export function formatClock(t) {
    const d = new Date(t * 1000);
    const p = (n, w) => String(n).padStart(w, '0');
    return `${p(d.getHours(), 2)}:${p(d.getMinutes(), 2)}:${p(d.getSeconds(), 2)}.${p(Math.floor(d.getMilliseconds() / 10), 2)}`;
}

export function createBallEventMapper() {
    /** @type {Map<number, {thrown: boolean, caught: boolean}>} */
    let seen = new Map();
    let lastNow = -Infinity;

    function reset() { seen = new Map(); lastNow = -Infinity; }

    /**
     * @param {Array<object>} balls  BallState[] of one /balls message
     * @param {{nowSec: number, replay?: boolean, muted?: boolean}} ctx
     * @returns {Array<{type: string, label: string, detail: string, t: number}>}
     */
    function map(balls, ctx) {
        const out = [];
        if (!balls || !balls.length) return out;
        const { nowSec, replay = false, muted = false } = ctx;
        if (replay && nowSec < lastNow - 1) reset();   // a backward seek: the store was trimmed, so re-emit
        lastNow = nowSec;
        for (const b of balls) {
            if (!b || typeof b.id !== 'number') continue;
            let rec = seen.get(b.id);
            if (!rec) {
                if (seen.size >= MAX_TRACKED) seen.delete(seen.keys().next().value);
                rec = { thrown: false, caught: false };
                seen.set(b.id, rec);
            }
            const stamp = toSec(b.header && b.header.stamp);
            // eventTime (ROS clock) -> the event-store axis
            const place = (rosT) => (replay || !stamp ? rosT : nowSec - (stamp - rosT));

            if (!rec.thrown && b.source === SOURCE_HAND
                && (b.status === BALL_STATUS.IN_FLIGHT || b.status === BALL_STATUS.CAUGHT)) {
                const rel = toSec(b.throw_time);
                if (rel > 0) {
                    const t = place(rel);
                    const label = `Throw ${b.id} · release ${formatClock(t)} (planned)`;
                    out.push({ type: 'throw', label, detail: label, t });
                    if (!muted) rec.thrown = true;
                }
            }
            if (!rec.caught && b.status === BALL_STATUS.CAUGHT
                && (b.destination === 'jugglebot' || !b.destination)) {
                const land = toSec(b.time_at_land);
                if (land > 0) {
                    const t = place(land);
                    const label = `Catch ${b.id} · ${formatClock(t)}${b.landing_from_fit ? '' : ' (predicted)'}`;
                    out.push({ type: 'catch', label, detail: label, t });
                    if (!muted) rec.caught = true;
                }
            }
        }
        return out;
    }

    return { map, reset };
}
