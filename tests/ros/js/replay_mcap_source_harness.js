// Node harness for McapSource (ros_ws/gui/js/replay/sources.js) over a scripted fake module worker that
// serves the Python oracle's chunks (replay_test_support.js). Prints one JSON object.
// Usage: node replay_mcap_source_harness.js <chunkJsonDir>
const { McapSource } = await import('./sources.js');
const { loadRecords, fakeWorkerFactory } = await import('./replay_test_support.js');

const { manifest, records, overview } = loadRecords(process.argv[2]);
const out = {};
const T0 = manifest.t0;
const noNet = async () => ({ status: 503, json: async () => ({ status: 'unavailable', reason: 'worker_unavailable' }) });

function manualTimers() {
  const q = [];
  return {
    q, cleared: 0,
    setTimeout: (fn, ms) => { q.push({ fn, ms }); return q.length; },
    clearTimeout: () => { out._cleared = (out._cleared || 0) + 1; },
    async fire() { const t = q.shift(); if (t) { t.fn(); await new Promise((r) => setTimeout(r, 5)); } return !!t; },
  };
}
const mk = (o) => McapSource(Object.assign({ id: 'rec', fetch: noNet, fileUrl: 'x', overviewPollMs: 0 }, o || {}));

// ---- open / range / status / topicSet / manifest / slot math ----
{
  const log = [];
  const src = mk({ makeWorker: fakeWorkerFactory(manifest, records, { log }) });
  const events = [];
  src.onChange((s) => events.push(s.state));
  const before = src.range();
  await src.open();
  out.open = {
    before, range: src.range(), t0: T0, t1: manifest.t1, status: src.status(),
    topic_set: src.topicSet().sort(), manifest_topics: Object.keys(manifest.topics).filter((k) => manifest.topics[k].count > 0).sort(),
    slots: records.length, events, log: log.slice(),
    bounds_ok: records.every((r, i) => { const b = src.bounds(i); return Math.abs(b[0] - r.t0) < 1e-9 && Math.abs(b[1] - r.t1) < 1e-9; }),
    index_ok: records.every((r, i) => src.chunkIndex(r.t0) === i && src.chunkIndex(r.t1 - 1e-6) === i),
  };

  // ---- load / peek / dedupe / RangeError ----
  const [a, b] = await Promise.all([src.load(1), src.load(1)]);
  const ch = await src.load(1);
  const tp = ch.topics['/robot_state'];
  const want = records[1].topics['/robot_state'];
  let rangeErr = null;
  try { await src.load(records.length); } catch (e) { rangeErr = e.constructor.name; }
  let negErr = null;
  try { await src.load(-1); } catch (e) { negErr = e.constructor.name; }
  out.load = {
    same_object: a === b && b === ch, loads_posted: log.filter((x) => x === 'load:1').length,
    peek_hit: src.peek(1) === ch, peek_miss: src.peek(3),
    n: tp.n, want_n: want.n, t_is_f64: tp.t instanceof Float64Array,
    t_equal: JSON.stringify(Array.from(tp.t)) === JSON.stringify(want.t),
    cols_equal: JSON.stringify(tp.cols) === JSON.stringify(want.cols),
    hydrate0: tp.hydrate(0), range_error: rangeErr, neg_error: negErr,
    window: (await src.window(T0 + 5, T0 + 15, ['/robot_state'])).map((c) => ({ i: c.i, topics: Object.keys(c.topics) })),
  };
  src.close();
}

// ---- latestBefore: worker `latest` beyond the memo, memo shortcut inside it, >60 s back ----
{
  const log = [];
  const src = mk({ makeWorker: fakeWorkerFactory(manifest, records, { log }) });
  await src.open();
  const last = records.length - 1;
  const lastSkill = (() => {
    let best = null;
    for (const r of records) { const tp = r.topics['/skills/attempt']; if (tp) best = tp.t[tp.t.length - 1]; }
    return best;
  })();
  const far = T0 + (last * 10) + 5;
  const r1 = await src.latestBefore(far, '/skills/attempt');
  out.latest = {
    far_t: r1 ? r1.t : null, far_dist: far - lastSkill, last_skill: lastSkill,
    posted_latest: log.filter((x) => x === 'latest').length,
    msg_keys: r1 ? Object.keys(r1.msg).slice(0, 3) : null,
    unknown_topic: await src.latestBefore(far, '/nope'),
    before_t0: await src.latestBefore(T0 - 5, '/robot_state'),
  };
  await src.load(1);
  const n0 = log.filter((x) => x === 'latest').length;
  const m = await src.latestBefore(T0 + 15, '/robot_state');
  out.latest.memo_served = log.filter((x) => x === 'latest').length === n0;
  out.latest.memo_t = m ? m.t : null;
  src.close();
}

// ---- timeline(): 202, 202, 200 from a fake /overview; onChange once; no network from timeline() ----
{
  const timers = manualTimers();
  const seq = [202, 202, 200];
  let calls = 0;
  const fetchOv = async (url) => {
    calls++;
    const st = seq.shift();
    return { status: st, json: async () => (st === 200 ? overview : { status: 'computing' }) };
  };
  const src = mk({ makeWorker: fakeWorkerFactory(manifest, records), fetch: fetchOv, timers, overviewPollMs: 1234 });
  const changes = [];
  src.onChange((s) => changes.push(s.state));
  await src.open();
  await new Promise((r) => setTimeout(r, 5));
  const calls_after_open = calls;
  const t1 = await src.timeline();
  const calls_after_timeline = calls;
  const ms = timers.q.map((x) => x.ms);
  await timers.fire();
  await timers.fire();
  const t2 = await src.timeline();
  const done_changes = changes.length;
  await src.timeline(); await src.timeline();
  out.timeline = {
    calls_after_open, calls_after_timeline, partial_first: t1.partial === true, poll_ms: ms,
    calls_final: calls, timeline_is_overview: JSON.stringify(t2) === JSON.stringify(overview),
    on_change_during_poll: done_changes - 1, // one emit came from open(); the overview adds exactly one
    queue_left: timers.q.length,
  };
  src.close();
}
{
  // terminal refusal: 409 no_index -> unavailable, no further polling
  const timers = manualTimers();
  const src = mk({ makeWorker: fakeWorkerFactory(manifest, records), timers,
    fetch: async () => ({ status: 409, json: async () => ({ status: 'refused', reason: 'no_index' }) }) });
  await src.open();
  await new Promise((r) => setTimeout(r, 5));
  const tl = await src.timeline();
  out.timeline_refused = { partial: tl.partial, unavailable: tl.unavailable, queued: timers.q.length };
  src.close();
}
{
  // network error backs off then recovers; close() clears the pending timer
  const timers = manualTimers();
  let n = 0;
  const src = mk({ makeWorker: fakeWorkerFactory(manifest, records), timers, overviewPollMs: 100,
    fetch: async () => { n++; if (n < 3) throw new Error('net'); return { status: 200, json: async () => overview }; } });
  await src.open();
  await new Promise((r) => setTimeout(r, 5));
  const backoff = [timers.q[0] && timers.q[0].ms];
  await timers.fire();
  backoff.push(timers.q[0] && timers.q[0].ms);
  await timers.fire();
  out.timeline_backoff = { backoff, got_overview: !(await src.timeline()).partial, calls: n };
  src.close();
}

// ---- errors -> .reason ----
{
  const r = {};
  for (const code of ['no_index', 'compressed', 'in_progress', 'changed', 'empty']) {
    const src = mk({ makeWorker: fakeWorkerFactory(manifest, records, { openError: code }) });
    try { await src.open(); r[code] = 'resolved'; } catch (e) { r[code] = { reason: e.reason, status: e.status }; }
    src.close();
  }
  // a mid-play slot failure rejects load() with the worker's code and does not poison the memo
  const src = mk({ makeWorker: fakeWorkerFactory(manifest, records, { loadError: { 2: 'changed' } }) });
  await src.open();
  try { await src.load(2); r.load = 'resolved'; } catch (e) { r.load = { reason: e.reason }; }
  r.load_ok_after = (await src.load(0)).i;
  src.close();
  out.errors = r;
}

// ---- close() rejects pending requests and terminates the worker ----
{
  let release;
  const hold = new Promise((r) => { release = r; });
  let worker = null;
  const f = fakeWorkerFactory(manifest, records, { hold });
  const src = mk({ makeWorker: () => (worker = f()) });
  await src.open();
  const pending = src.load(0).then(() => 'resolved', (e) => e.reason);
  await new Promise((r) => setTimeout(r, 5));
  src.close();
  const reason = await pending;
  release();
  let after = null;
  try { await src.load(1); } catch (e) { after = e.reason || e.constructor.name; }
  out.close = { pending_reason: reason, terminated: worker.terminated, load_after_close: after };
}

process.stdout.write(JSON.stringify(out));
