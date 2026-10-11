/**
 * deeplink.js — replay deep links (`?recording=<id>&t=<t>`), Phase 6 unit A. Dependency-free.
 *
 *   parseDeepLink(search)  -> null | {recording, t:{kind:'none'} | {kind:'offset', sec} | {kind:'clock', sod} | {kind:'invalid', raw}}
 *   resolveT(t, t0, durationSec) -> {sec, clamped, invalid}   sec = seconds from the recording start
 *   buildLink(origin, pathname, id, offsetSec) -> 'origin/path?recording=<id>&t=<offset, 1 decimal>'
 *   copyText(doc, text, nav?) -> Promise<boolean>   navigator.clipboard when available, else textarea + execCommand
 *   createDeepLink({search, ros, mode, toaster, list, refusalMessage}) -> {state}
 *
 * `t` is a bare number (seconds from the recording start) or `HH:MM:SS(.s)` (LOCAL wall-clock time of day, the
 * trackbar's axis). A wall-clock time is resolved against the recording's local date; a recording that crosses
 * midnight takes the occurrence inside [t0, t1] (none inside: the nearer end, reported as clamped).
 *
 * The controller honours a link ONCE per page load, only from a settled disconnected state: the page's initial
 * 'disconnected' default and 'connecting' are not settled; 'connected' REFUSES (notice, never queued). A recording
 * missing from the listing -> notice (the lobby stays); `t` outside the recording clamps, an unparseable `t` lands
 * at the start, each with a notice. The recording opens paused (enterReplay) and is then seeked to `t`, still paused.
 * Links are produced only by the transport's "copy link" button; nothing here ever touches the address bar.
 */

const NUM = /^\d+(?:\.\d+)?$/;
const CLOCK = /^(\d{1,2}):(\d{2}):(\d{2}(?:\.\d+)?)$/;

/** @param {string} search location.search @returns {null|{recording:string, t:object}} */
export function parseDeepLink(search) {
    const s = String(search || '').replace(/^\?/, '');
    if (!s) return null;
    const q = new Map();
    for (const part of s.split('&')) {
        if (!part) continue;
        const i = part.indexOf('=');
        const dec = (x) => { try { return decodeURIComponent(x.replace(/\+/g, ' ')); } catch (e) { return x; } };
        const k = dec(i < 0 ? part : part.slice(0, i));
        if (!q.has(k)) q.set(k, i < 0 ? '' : dec(part.slice(i + 1)));
    }
    const recording = (q.get('recording') || '').trim();
    if (!recording) return null;
    if (!q.has('t')) return { recording, t: { kind: 'none' } };
    const raw = q.get('t').trim();
    if (NUM.test(raw)) return { recording, t: { kind: 'offset', sec: parseFloat(raw) } };
    const m = CLOCK.exec(raw);
    if (m) {
        const hh = +m[1], mm = +m[2], ss = parseFloat(m[3]);
        if (hh < 24 && mm < 60 && ss < 60) return { recording, t: { kind: 'clock', sod: hh * 3600 + mm * 60 + ss } };
    }
    return { recording, t: { kind: 'invalid', raw } };
}

/** @param {number} t0 epoch seconds @param {number} sod seconds into the local day of day+dayOff @returns {number} epoch seconds */
function localEpoch(t0, dayOff, sod) {
    const d = new Date(t0 * 1000);
    const h = Math.floor(sod / 3600), m = Math.floor((sod % 3600) / 60), s = sod - h * 3600 - m * 60;
    const base = new Date(d.getFullYear(), d.getMonth(), d.getDate() + dayOff, h, m, 0, 0).getTime() / 1000;
    return base + s;
}

/**
 * @param {object} t parseDeepLink().t @param {number} t0 recording start (epoch s) @param {number} durationSec
 * @returns {{sec:number, clamped:boolean, invalid:boolean}}
 */
export function resolveT(t, t0, durationSec) {
    const dur = Math.max(0, +durationSec || 0);
    if (!t || t.kind === 'none') return { sec: 0, clamped: false, invalid: false };
    if (t.kind === 'invalid') return { sec: 0, clamped: false, invalid: true };
    let sec;
    if (t.kind === 'offset') sec = t.sec;
    else {
        const cands = [0, 1].map((o) => localEpoch(t0, o, t.sod) - t0);   // the start's day, then the next (midnight crossing)
        const inside = cands.filter((c) => c >= -1e-6 && c <= dur + 1e-6);
        if (inside.length) sec = inside[0];
        else sec = Math.abs(cands[0] - (cands[0] < 0 ? 0 : dur)) <= Math.abs(cands[1] - dur) ? cands[0] : cands[1];
    }
    const c = Math.max(0, Math.min(dur, sec));
    return { sec: c, clamped: Math.abs(c - sec) > 1e-9, invalid: false };
}

/** @returns {string} */
export function buildLink(origin, pathname, id, offsetSec) {
    // Rounded DOWN to 0.1 s (1e-9 absorbs x*10 landing a hair under an integer): toFixed would round a playhead at the
    // very end up past it, and the receiver would show a false "clamped" notice on a link the app itself produced.
    const t = Math.floor(Math.max(0, +offsetSec || 0) * 10 + 1e-9) / 10;
    return origin + pathname + '?recording=' + encodeURIComponent(id) + '&t=' + t.toFixed(1);
}

/** Clipboard write with a plain-http fallback (navigator.clipboard is undefined off secure origins). @returns {Promise<boolean>} */
export async function copyText(doc, text, nav) {
    try {
        if (nav && nav.clipboard && nav.clipboard.writeText) { await nav.clipboard.writeText(text); return true; }
    } catch (e) { /* fall through to the textarea path */ }
    try {
        const ta = doc.createElement('textarea');
        ta.value = text;
        if (ta.setAttribute) ta.setAttribute('readonly', '');
        if (ta.style) ta.style.cssText = 'position:fixed;left:-9999px;top:0;opacity:0';
        doc.body.appendChild(ta);
        if (ta.select) ta.select();
        const ok = !!(doc.execCommand && doc.execCommand('copy'));
        if (ta.remove) ta.remove(); else doc.body.removeChild(ta);
        return ok;
    } catch (e) { return false; }
}

/**
 * @param {{search:string, ros:object, mode:object, toaster:{show:function}, list:function():Promise<string[]>,
 *   refusalMessage?:function(string):string}} deps  `list` resolves the recording ids the backend knows.
 */
export function createDeepLink(deps) {
    const { ros, mode, toaster } = deps;
    const link = parseDeepLink(deps.search);
    let consumed = false;       // one shot per page load: exiting replay must not re-trigger it
    let outcome = link ? 'pending' : 'none';
    const note = (text) => toaster.show(text, 9000);

    function finish(o) { consumed = true; outcome = o; }

    async function honour() {
        finish('honouring');
        let ids;
        try { ids = await deps.list(); } catch (e) { ids = null; }
        if (ids && ids.indexOf(link.recording) < 0) {
            note('Recording ' + link.recording + ' not found');
            outcome = 'not_found'; return;
        }
        try {
            await mode.enterReplay({ kind: 'recording', id: link.recording });
        } catch (e) {
            const why = e && (e.reason || e.message);
            note(deps.refusalMessage ? deps.refusalMessage(why) : 'Could not open ' + link.recording);
            outcome = 'refused'; return;
        }
        const eng = mode.engine && mode.engine();
        const src = mode.source && mode.source();
        if (!eng || !src) { outcome = 'gone'; return; }
        const r = src.range();
        const res = resolveT(link.t, r.t0, r.t1 - r.t0);
        if (res.invalid) note('Replay link: could not read t="' + link.t.raw + '"; starting at the beginning');
        else if (res.clamped) note('Replay link: t is outside the recording; clamped to ' + res.sec.toFixed(1) + ' s');
        outcome = 'opened';
        if (res.sec > 0) { try { await eng.seek(r.t0 + res.sec); } catch (e) { console.error('deep link seek', e); } }
    }

    function onState(st) {
        if (!link || consumed) return;
        if (st === 'connected') {
            finish('refused_connected');
            note('Replay link ignored: connected to the robot. Disconnect and reopen the link.');
        } else if (st === 'disconnected' && settled) {
            if (mode.isActive()) { finish('active'); return; }
            honour();
        } else if (st === 'connecting') settled = true;
    }
    let settled = false;         // the initial 'disconnected' default is not a failed connect (same rule as the lobby)
    if (link) ros.onConnectionStateChange(onState);
    return { state: () => ({ link, consumed, outcome }) };
}
