// replay_rename_live_harness.js — renaming the OPEN recording: McapSource.retarget, the worker's 'retarget' op and
// httpReadable.setUrl (the REAL sources.js / mcap-worker.js / mcap-decode.js), and the trackbar's rename control.
// Sandbox layout: js/replay/ (copy), lib/ (copy of gui/lib), js/<this file>, package.json {"type":"module"}.
const { McapSource } = await import('./replay/sources.js');
const { createWorkerHandler } = await import('./replay/mcap-worker.js');
const { httpReadable } = await import('./replay/mcap-decode.js');
const { createTrackbar } = await import('./replay/ui/trackbar.js');
const out = {};
const settle = async () => { for (let i = 0; i < 30; i++) await new Promise((r) => setImmediate(r)); };

// ---- 1. McapSource.retarget: id, worker URL, overview poll URL ----
{
    const posted = [], fetched = [];
    const worker = {
        postMessage(m) {
            posted.push(m);
            if (m.op === 'open') setImmediate(() => worker.onmessage({ data: { op: 'opened', req: m.req, t0: 100, t1: 130, slots: 3, topics: {}, skipped: {} } }));
        },
        terminate() {},
    };
    const timers = { setTimeout: (f) => { timers.f = f; return 1; }, clearTimeout() {} };
    const src = McapSource({ id: '2026-10-12_23-24-02', fetch: (u) => { fetched.push(u); return Promise.resolve({ status: 202, json: () => Promise.resolve({}) }); },
        makeWorker: () => worker, timers });
    await src.open(); await settle();
    let changes = 0; src.onChange(() => { changes++; });
    src.retarget('2026-10-12_23-24-02_three-ball-PB');
    timers.f(); await settle();   // the next overview poll
    out.source = {
        id: src.id, open_url: posted[0].url, retarget: posted.filter((m) => m.op === 'retarget').map((m) => m.url),
        polls: fetched, changes,
    };
    src.retarget(src.id);   // same id: no message
    out.source.retarget_count_after_noop = posted.filter((m) => m.op === 'retarget').length;
    src.close();
}

// ---- 2. worker 'retarget' reaches the readable, out of band (no reply) ----
{
    const made = [], posts = [];
    const h = createWorkerHandler({
        makeReadable: (url) => { const r = { url, setUrl(u) { r.url = u; } }; made.push(r); return r; },
        post: (m) => posts.push(m),
    });
    await h.handle({ op: 'retarget', url: '/early' });        // before open: ignored, no throw
    await h.handle({ op: 'open', req: 1, url: '/api/replay/recordings/A/file' });   // fails to decode the fake, but keeps the readable
    await h.handle({ op: 'retarget', url: '/api/replay/recordings/B/file' });
    out.worker = { url: made[0].url, replies: posts.map((p) => p.op) };
}

// ---- 3. httpReadable.setUrl: later range requests go to the new URL with the same If-Match ----
{
    const calls = [];
    const f = (url, opt) => {
        calls.push({ url, method: (opt && opt.method) || 'GET', ifMatch: opt && opt.headers ? opt.headers['If-Match'] || null : null });
        const headers = { get: (k) => ({ ETag: '"10-5"', 'Content-Length': '10' })[k] || null };
        return Promise.resolve({ ok: true, status: opt && opt.method === 'HEAD' ? 200 : 206, headers, arrayBuffer: () => Promise.resolve(new ArrayBuffer(2)) });
    };
    const r = httpReadable('/old/file', { fetch: f });
    await r.size(); await r.read(0n, 2n);
    r.setUrl('/new/file');
    await r.read(4n, 2n);
    out.readable = calls;
}

// ---- 4. trackbar: name beside Exit replay, rename -> POST -> source.retarget, failure -> toast ----
{
    class El {
        constructor(tag) { this.tagName = tag; this.children = []; this.listeners = {}; this.attrs = {}; this.style = {}; this.className = ''; this.id = ''; this.hidden = false; this.disabled = false; this.value = ''; this._text = ''; }
        get textContent() { return this.children.length ? this.children.map((c) => c.textContent).join('') : this._text; }
        set textContent(v) { this.children = []; this._text = String(v); }
        appendChild(c) { c.parentNode = this; this.children.push(c); return c; }
        replaceChildren() { this.children = []; }
        setAttribute(k, v) { this.attrs[k] = v; }
        addEventListener(t, fn) { (this.listeners[t] = this.listeners[t] || []).push(fn); }
        removeEventListener() {}
        getBoundingClientRect() { return { left: 0, width: 100 }; }
        find(p, o = []) { if (p(this)) o.push(this); for (const c of this.children) c.find(p, o); return o; }
    }
    const body = new El('body');
    const doc = { body, createElement: (t) => new El(t), addEventListener() {}, removeEventListener() {} };
    const q = (id) => body.find((e) => e.id === id)[0];
    const dock = new El('div'); body.appendChild(dock);
    const src = { id: '2026-10-12_23-24-02', range: () => ({ t0: 100, t1: 130, frontier: 130 }), status: () => ({ state: 'complete' }),
        timeline: () => Promise.resolve({ partial: true }), onChange: () => () => {}, retarget(id) { src.retargeted = id; src.id = id; } };
    const engine = { state: () => ({ playhead: 110, speed: 1, mode: 'paused', range: { t0: 100, t1: 130, frontier: 130 } }) };
    const mode = { isActive: () => true, engine: () => engine, source: () => src, on: () => () => {}, exitReplay() {} };
    const toasts = [], posts = [];
    let reply = null;
    const fetchFn = (url, opt) => {
        posts.push({ url, body: JSON.parse(opt.body) });
        const j = reply || { id: '2026-10-12_23-24-02_' + JSON.parse(opt.body).name.replace(/ /g, '-') };
        return Promise.resolve({ ok: !reply, status: reply ? 409 : 200, json: () => Promise.resolve(j) });
    };
    const tb = createTrackbar({ document: doc, mode, dock, toaster: { show: (t) => toasts.push(t) }, fetch: fetchFn });
    tb.frame();
    const ctlIds = body.find((e) => e.className === 'rp-ctl')[0].children.map((c) => c.id || c.className);
    out.tb = { ctl: ctlIds, name0: q('rp-name').textContent, name0_cls: q('rp-name').className };
    q('rp-rename').listeners.click[0]({});
    out.tb.editing = { view_hidden: body.find((e) => e.className === 'rp-name-view')[0].hidden, input_value: q('rp-name-input').value };
    q('rp-name-input').value = 'three ball PB';
    q('rp-name-input').listeners.keydown[0]({ key: 'Enter' });
    await settle();
    out.tb.after = { name: q('rp-name').textContent, cls: q('rp-name').className, retargeted: src.retargeted, post: posts[0],
        edit_hidden: body.find((e) => e.className === 'rp-name-edit')[0].hidden };
    q('rp-rename').listeners.click[0]({});
    out.tb.prefill = q('rp-name-input').value;
    reply = { error: 'a recording named x already exists' };
    out.tb.fail = await tb.rename('x');
    out.tb.fail_toast = toasts[toasts.length - 1];
    out.tb.fail_still_editing = !body.find((e) => e.className === 'rp-name-edit')[0].hidden;
    q('rp-name-input').listeners.keydown[0]({ key: 'Escape', stopPropagation() {} });
    out.tb.esc_closed = body.find((e) => e.className === 'rp-name-edit')[0].hidden;
}

console.log(JSON.stringify(out));
process.exit(0);
