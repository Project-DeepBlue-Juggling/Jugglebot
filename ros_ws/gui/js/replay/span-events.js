/**
 * span-events.js - every Throw / Catch marker of a time span of a recording, independent of the playhead (pure; no DOM).
 *
 *   const evs = await scanBallEvents(source, t0, t1);   // [{type, label, detail, t}] with t in [t0, t1], time order
 *
 * The replay engine feeds /balls to main.js only as the playhead advances, so the event store (and the ODrive charts,
 * whose data also stops at the playhead) hold the PAST only. The IMU charts show a whole capture, data on both sides of
 * the playhead, and want the markers on both sides too. This reads /balls of the span straight from the recording:
 * source.load(i, {lite: true, topics: ['/balls']}) decodes only that topic on the worker's low-priority lane and is never
 * memoised, so a ~32 s capture costs a handful of filtered chunk decodes and leaves nothing resident. The messages go
 * through a FRESH ball-events.js mapper (replay mode, recorded times as they are), so the markers are the very ones the
 * playhead-driven path would have produced; the result never touches the event store's ring, hence the Event Log gains
 * nothing.
 *
 * MARGIN_S: an event is stamped before the message that reports it (a Throw is the planned release, a Catch the landing
 * crossing 0.2-0.5 s earlier than its CAUGHT message), so messages are read a little beyond the span.
 */
import { createBallEventMapper } from '../ball-events.js';

const BALLS = '/balls';
export const MARGIN_S = 3;

/**
 * @param {{range:function, chunkIndex:function, load:function}} source  a replay recording source
 * @param {number} t0 span start, wall-clock seconds
 * @param {number} t1 span end
 * @param {{isCancelled?:function, yieldFn?:function}} [opts]
 * @returns {Promise<Array<{type:string,label:string,detail:string,t:number}>|null>} null when cancelled
 */
export async function scanBallEvents(source, t0, t1, opts = {}) {
  const isCancelled = opts.isCancelled || (() => false);
  const yieldFn = opts.yieldFn || (() => new Promise((r) => setTimeout(r, 0)));
  if (!source || !(t1 >= t0)) return [];
  const r = source.range();
  const a = Math.max(t0 - MARGIN_S, r.t0);
  const b = Math.min(t1 + MARGIN_S, r.t1);
  if (!(b >= a)) return [];
  const mapper = createBallEventMapper();
  const out = [];
  const seen = new Set();
  for (let i = source.chunkIndex(a), last = source.chunkIndex(b); i <= last; i++) {
    const ch = await source.load(i, { lite: true, topics: [BALLS] });
    if (isCancelled()) return null;
    const tp = ch && ch.topics && ch.topics[BALLS];
    if (tp && tp.n > 0 && typeof tp.hydrate === 'function') {
      for (let k = 0; k < tp.n; k++) {
        const tk = tp.t[k];
        if (tk < a || tk > b) continue;
        const msg = tp.hydrate(k);
        for (const ev of mapper.map((msg && msg.balls) || [], { nowSec: tk, replay: true, muted: false })) {
          if (ev.t < t0 || ev.t > t1) continue;
          const key = ev.t + '|' + ev.type + '|' + ev.label;
          if (seen.has(key)) continue;
          seen.add(key);
          out.push(ev);
        }
      }
    }
    await yieldFn();
    if (isCancelled()) return null;
  }
  out.sort((p, q) => p.t - q.t);
  return out;
}
