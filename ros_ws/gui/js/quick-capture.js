/**
 * quick-capture.js - "+" in the Robot | Map bar: file a bug/idea/feature in the map.
 *
 * POSTs to the map server (same host, port 8765 - derived like app-tabs.js derives the
 * Map iframe URL) at /api/v1/projects/{pid}/tasks with status "inbox"; `X-Dev-User`
 * sets filed_by.  The map server must list this GUI's origin in MAP_CORS_ORIGINS (no
 * credentials are sent).  The map app live-updates from its own event stream, so the
 * Map iframe is not touched.
 *
 * Focus rules: opening moves focus into the title field, closing returns it to the "+"
 * button; every keydown inside the form stops propagation so no key reaches the Robot
 * tab's motion controls.  The draft (kind/title/note) is kept in localStorage under
 * DRAFT_KEY on every keystroke, restored on open, cleared on a successful filing.
 * The same keyboard isolation covers the "+" button itself (Space on it must not reach the
 * chart pause or the replay trackbar).  While a request is in flight Esc, Cancel and the "+"
 * toggle leave the form open, so a late failure is visible; if the form is hidden anyway the
 * error goes to the fading confirmation element.  A timeout after the POST was sent says so:
 * the server has no idempotency key, so the Item may exist.
 */

const MAP_PORT = 8765;
const API = (() => {
    const u = new URL(location.href);
    u.port = String(MAP_PORT);
    u.pathname = '/api/v1';
    u.search = '';
    u.hash = '';
    return u.href.replace(/\/$/, '');
})();
const FILED_BY = 'harrison';
const DRAFT_KEY = 'jugglebot.quickCapture.draft';
const TIMEOUT_MS = 8000;

const btn = document.getElementById('quick-capture-btn');
const form = document.getElementById('quick-capture-form');
const titleEl = document.getElementById('qc-title');
const noteEl = document.getElementById('qc-note');
const errEl = document.getElementById('qc-error');
const toast = document.getElementById('quick-capture-toast');
const fileBtn = document.getElementById('qc-file');
const cancelBtn = document.getElementById('qc-cancel');
let busy = false;
let toastTimer = null;

function kindEl() { return form.querySelector('input[name="qc-kind"]:checked'); }

function saveDraft() {
    try {
        localStorage.setItem(DRAFT_KEY, JSON.stringify({
            kind: kindEl() ? kindEl().value : 'bug', title: titleEl.value, note: noteEl.value,
        }));
    } catch (e) { /* storage blocked: the form still works */ }
}
function clearDraft() { try { localStorage.removeItem(DRAFT_KEY); } catch (e) { /* ignore */ } }
function restoreDraft() {
    try {
        const d = JSON.parse(localStorage.getItem(DRAFT_KEY) || 'null');
        if (!d) return;
        titleEl.value = d.title || '';
        noteEl.value = d.note || '';
        const r = form.querySelector(`input[name="qc-kind"][value="${d.kind}"]`);
        if (r) r.checked = true;
    } catch (e) { /* corrupt draft: ignore */ }
}

function openForm() {
    restoreDraft();
    errEl.textContent = '';
    // Drop the popover under the button, kept on-screen at narrow widths.
    const left = Math.round(btn.getBoundingClientRect().left);
    form.style.left = Math.max(4, Math.min(left, window.innerWidth - 284)) + 'px';
    form.hidden = false;
    btn.setAttribute('aria-expanded', 'true');
    titleEl.focus();
}
function closeForm() {
    form.hidden = true;
    btn.setAttribute('aria-expanded', 'false');
    btn.focus();
}

/** Close on the user's behalf: never while a request is in flight (the failure must stay visible). */
function dismiss() { if (!busy) closeForm(); }

function pad(n) { return String(n).padStart(2, '0'); }
/** Local wall-clock ISO-8601 with UTC offset, e.g. 2026-10-25T18:05:19+11:00. */
function localIso(d) {
    const off = -d.getTimezoneOffset();
    const sign = off >= 0 ? '+' : '-';
    const a = Math.abs(off);
    return `${d.getFullYear()}-${pad(d.getMonth() + 1)}-${pad(d.getDate())}T` +
        `${pad(d.getHours())}:${pad(d.getMinutes())}:${pad(d.getSeconds())}` +
        `${sign}${pad(Math.floor(a / 60))}:${pad(a % 60)}`;
}

// The GUI does not expose the rosbag recording name (only the camera-recorder state),
// so the context block carries the time alone.
function buildBody(note) {
    const ctx = `Filed from the Robot tab at ${localIso(new Date())}.`; // wall-clock: filing time is real time, never replay time
    return note ? `${note}\n\n${ctx}` : ctx;
}

async function api(path, opts) {
    const ctl = new AbortController();
    const t = setTimeout(() => ctl.abort(), TIMEOUT_MS);
    try {
        return await fetch(`${API}${path}`, { ...opts, signal: ctl.signal });
    } finally {
        clearTimeout(t);
    }
}

async function errorText(resp) {
    try {
        const j = await resp.json();
        const d = j && (j.detail || j.error || j.message);
        if (typeof d === 'string') return d;
        if (d) return JSON.stringify(d);
    } catch (e) { /* not JSON */ }
    return `Map server error ${resp.status}`;
}

function showToast(text) {
    toast.textContent = text;
    toast.classList.add('show');
    clearTimeout(toastTimer);
    toastTimer = setTimeout(() => toast.classList.remove('show'), 3000);
}

/** Show a filing error in the form; if the form is hidden, in the fading confirmation instead. */
function showError(text) {
    if (form.hidden) showToast(text);
    else errEl.textContent = text;
}

async function file() {
    const title = titleEl.value.trim();
    if (!title || busy) {
        if (!title) { errEl.textContent = 'Title is required'; titleEl.focus(); }
        return;
    }
    busy = true;
    fileBtn.disabled = true;
    errEl.textContent = '';
    let posted = false; // true once the POST (the only request that creates an Item) is on the wire
    try {
        const pr = await api('/projects', { method: 'GET' });
        if (!pr.ok) { showError(await errorText(pr)); return; }
        const projects = await pr.json();
        const pid = projects && projects[0] && projects[0].id;
        if (!pid) { showError('Map server has no project'); return; }
        posted = true;
        const resp = await api(`/projects/${encodeURIComponent(pid)}/tasks`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json', 'X-Dev-User': FILED_BY },
            body: JSON.stringify({
                kind: kindEl() ? kindEl().value : 'bug',
                title,
                description: buildBody(noteEl.value.trim()),
                status: 'inbox',
            }),
        });
        if (!resp.ok) { showError(await errorText(resp)); return; }
        clearDraft();
        titleEl.value = '';
        noteEl.value = '';
        closeForm();
        showToast(`Filed: ${title}`);
    } catch (e) {
        showError(posted && e && e.name === 'AbortError'
            ? 'No answer from the map server - check the inbox before filing again'
            : 'Map server unreachable - text kept');
    } finally {
        busy = false;
        fileBtn.disabled = false;
    }
}

btn.addEventListener('click', () => { if (form.hidden) openForm(); else dismiss(); });
cancelBtn.addEventListener('click', dismiss);
form.addEventListener('submit', (e) => { e.preventDefault(); file(); });
form.addEventListener('input', saveDraft);
form.addEventListener('change', saveDraft);
// Nothing typed in the form may reach the Robot tab's motion-control key handlers.
form.addEventListener('keydown', (e) => {
    e.stopPropagation();
    if (e.key === 'Escape') { e.preventDefault(); dismiss(); }
});
form.addEventListener('keyup', (e) => e.stopPropagation());
form.addEventListener('keypress', (e) => e.stopPropagation());
// The "+" button sits in the bar, outside the form: Space/Enter on it must not reach the
// chart pause (telemetry-charts.js) or the replay trackbar handler either.
for (const type of ['keydown', 'keyup', 'keypress']) {
    btn.addEventListener(type, (e) => e.stopPropagation());
}
