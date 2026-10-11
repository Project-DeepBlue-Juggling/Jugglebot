/**
 * trail-feed.js — the keying and policy shared by live and replay trails
 * (Phase 5). Primitive-argument API so the replay columns path never hydrates.
 *
 *   const feed = createTrailFeed(layer, opts);
 *   feed.beginMocap(t); feed.marker(t, label, x, y, z); feed.endMocap(t);
 *   feed.beginBalls(t); feed.ball(t, id, status, x, y, z); feed.endBalls(t);
 *   feed.tick(t);  feed.setRenderTime(tSec | null);  feed.render(tailMs);
 *   feed.balls();  feed.reset();
 *
 * Keys: marker = QTM label string; unlabelled marker = nearest-neighbour key
 * (negative int); ball = BALL_KEY + id.
 *
 *  D1  a ball track ends, at the last /balls message time, when no /balls
 *      message arrived for `ballStaleMs`; the ball state then empties.
 *  D3  an unlabelled marker within `dedupMm` of a live ball gets no trail and
 *      no nearest-neighbour slot (its sphere is still drawn by the caller).
 * Nothing here allocates after construction (balls() reuses one object).
 */
import * as clock from './clock.js';
import { labelToColour, ballColour, DEFAULT_COLOUR } from './marker-palette.js';

export const BALL_KEY = 1e9;
/** D1 default (s); the replay window feeds at least this much so a still-fresh ball keeps its sphere. */
export const BALL_STALE_SEC = 0.15;
const NN_SLOTS = 8;

export function createTrailFeed(layer, opts = {}) {
    const now = opts.now || (() => clock.now() / 1000);
    const ballStaleMs = opts.ballStaleMs === undefined ? 150 : opts.ballStaleMs;
    const dedupMm = opts.dedupMm === undefined ? 40 : opts.dedupMm;
    const nnGateMm = opts.nnGateMm === undefined ? 80 : opts.nnGateMm;
    const nnMemorySec = opts.nnMemorySec === undefined ? 0.5 : opts.nnMemorySec;
    const maxBalls = opts.maxBalls === undefined ? 8 : opts.maxBalls;
    const staleSec = ballStaleMs / 1000;
    const dedup2 = dedupMm * dedupMm, gate2 = nnGateMm * nnGateMm;

    layer.setColorFn((key) => {
        if (typeof key === 'string') return labelToColour(key);
        if (key >= BALL_KEY) return ballColour(key - BALL_KEY);
        return DEFAULT_COLOUR;
    });

    // nearest-neighbour slots for unlabelled markers
    const nnKey = new Float64Array(NN_SLOTS), nnX = new Float64Array(NN_SLOTS),
        nnY = new Float64Array(NN_SLOTS), nnZ = new Float64Array(NN_SLOTS),
        nnT = new Float64Array(NN_SLOTS), nnUsed = new Uint8Array(NN_SLOTS);
    function resetNN() {
        for (let i = 0; i < NN_SLOTS; i++) {
            nnKey[i] = -(i + 1); nnX[i] = 0; nnY[i] = 0; nnZ[i] = 0; nnT[i] = -Infinity; nnUsed[i] = 0;
        }
    }
    resetNN();

    // current ball list (the last /balls message); one reused result object
    const out = {
        n: 0, id: new Int32Array(maxBalls), status: new Int32Array(maxBalls),
        x: new Float64Array(maxBalls), y: new Float64Array(maxBalls), z: new Float64Array(maxBalls),
    };
    let nb = 0, lastBallsT = -Infinity, renderTime = null;

    function ballsFresh(t) { return nb > 0 && t - lastBallsT <= staleSec; }

    function unlabelledKey(t, x, y, z) {
        let best = -1, bd = gate2;
        for (let i = 0; i < NN_SLOTS; i++) {
            if (nnUsed[i] || t - nnT[i] > nnMemorySec) continue;
            const dx = nnX[i] - x, dy = nnY[i] - y, dz = nnZ[i] - z, d = dx * dx + dy * dy + dz * dz;
            if (d < bd) { bd = d; best = i; }
        }
        if (best < 0) {                       // new identity in the stalest slot not already used by this message
            for (let i = 0; i < NN_SLOTS; i++) if (!nnUsed[i] && (best < 0 || nnT[i] < nnT[best])) best = i;
            if (best < 0) return null;        // more unlabelled markers than slots in one message: no trail
            if (nnT[best] !== -Infinity) nnKey[best] -= NN_SLOTS;   // a never-used slot keeps its seed key
        }
        nnUsed[best] = 1; nnX[best] = x; nnY[best] = y; nnZ[best] = z; nnT[best] = t;
        return nnKey[best];
    }

    const feed = {
        beginMocap(t) {
            for (let i = 0; i < NN_SLOTS; i++) nnUsed[i] = 0;
            layer.beginMessage('mocap');
        },
        marker(t, label, x, y, z) {
            if (label) { layer.push('mocap', label, t, x, y, z); return; }
            if (ballsFresh(t)) {              // D3
                for (let i = 0; i < nb; i++) {
                    const dx = out.x[i] - x, dy = out.y[i] - y, dz = out.z[i] - z;
                    if (dx * dx + dy * dy + dz * dz < dedup2) return;
                }
            }
            const key = unlabelledKey(t, x, y, z);
            if (key !== null) layer.push('mocap', key, t, x, y, z);
        },
        endMocap(t) { layer.endMessage('mocap', t); },
        beginBalls(t) {
            // D1 before the new message: a gap > ballStaleMs ends the previous tracks at THEIR last
            // message time, not at this one (a rebuilt replay window spans gaps between ticks).
            feed.tick(t);
            layer.beginMessage('balls');
            nb = 0; lastBallsT = t;
        },
        ball(t, id, status, x, y, z) {
            layer.push('balls', BALL_KEY + id, t, x, y, z);
            if (nb < maxBalls) {
                out.id[nb] = id; out.status[nb] = status;
                out.x[nb] = x; out.y[nb] = y; out.z[nb] = z; nb++;
            }
        },
        endBalls(t) { layer.endMessage('balls', t); },   // ids absent from this message end
        tick(t) {                             // D1
            if (nb > 0 && t - lastBallsT > staleSec) {
                for (let i = 0; i < nb; i++) layer.end(BALL_KEY + out.id[i], lastBallsT);
                nb = 0;
            }
        },
        setRenderTime(t) { renderTime = t === null || t === undefined ? null : t; },
        render(tailMs) {
            const t = renderTime === null ? now() : renderTime;
            if (renderTime === null) feed.tick(t);
            return layer.render(t, tailMs);
        },
        balls() {
            const t = renderTime === null ? now() : renderTime;
            out.n = ballsFresh(t) ? nb : 0;
            return out;
        },
        reset() {
            layer.reset(); resetNN();
            nb = 0; out.n = 0; lastBallsT = -Infinity;
        },
    };
    return feed;
}
