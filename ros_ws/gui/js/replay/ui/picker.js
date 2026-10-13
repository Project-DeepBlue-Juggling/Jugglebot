/**
 * picker.js — the in-GUI "Open recording" modal (charting decision 12).
 *
 * createPicker({document, fetch, mode, baseUrl?}) ->
 *   {open, close, isOpen, probe, retry, setFilter, choose, model, on}
 *
 * open(): GET /api/replay/recordings; rows newest first (format.buildRows). The listing row carries everything (indexed, in_progress, duration_s,
 * topics with counts), so there are no per-row fetches. Rows without an index ("no index", killed
 * recordings) and in-progress rows are dimmed and not selectable. choose(key) calls mode.enterReplay;
 * a rejection maps `.reason` to a message and the modal stays open; success closes it. An unreachable
 * list raises the red banner with [retry]; `overview_available === false` is only a NOTE (the overview
 * strip is optional; opening never needs the worker). `on('backend', cb)` reports {down, why, note}.
 *
 * Rename: the ✎ on a row swaps it for an inline name box (Enter / save, Esc / cancel). The backend renames the
 * folder, the .mcap and metadata.yaml to '<timestamp>_<name>' (POST recordings/<id>/rename); the list reloads.
 * startRename(key) / setRenameValue(s) / commitRename() / cancelRename() drive the same path programmatically.
 */
import { h, fill } from './dom.js';
import { buildRows, stateCell, refusalMessage } from './format.js';

const OVERVIEW_NOTE = 'Timeline overview unavailable (the overview worker is not running). Recordings still open; only the overview strip is missing.';

export function createPicker(deps) {
    const doc = deps.document;
    const fetchFn = deps.fetch;
    const mode = deps.mode;
    const base = deps.baseUrl === undefined ? '/api/replay' : deps.baseUrl;
    const listeners = { backend: new Set() };

    let open = false;
    let listing = null;
    let filter = '';
    let loading = false;
    let backendDown = false;
    let backendWhy = '';
    let message = '';
    let opening = null;          // key of the row being opened
    let loadToken = 0;
    let overviewNote = '';       // non-blocking: the timeline overview worker is absent
    let editing = null;          // key of the row whose name box is open
    let editValue = '';
    let renaming = false;        // a rename POST is in flight
    let root = null;
    let keyHandler = null;
    const refs = {};

    function emitBackend() { for (const cb of Array.from(listeners.backend)) { try { cb({ down: backendDown, why: backendWhy, note: overviewNote }); } catch (e) { console.error(e); } } }

    const rows = () => buildRows(listing, filter);

    /** @returns {Promise<boolean>} true when the listing was reachable */
    async function loadList() {
        const my = ++loadToken;
        loading = true; render();
        try {
            const res = await fetchFn(base + '/recordings');
            if (!res.ok) throw new Error('HTTP ' + res.status);
            const j = await res.json();
            if (my !== loadToken) return false;
            listing = j;
            backendDown = false; backendWhy = '';
            overviewNote = j.overview_available === false ? OVERVIEW_NOTE : '';
        } catch (e) {
            if (my !== loadToken) return false;
            listing = null;
            backendDown = true;
            backendWhy = 'Replay backend unreachable (' + (e && e.message ? e.message : e) + '). Recordings cannot be listed; the live view is unaffected.';
        }
        loading = false;
        emitBackend();
        render();
        return !backendDown;
    }

    // ---- DOM ----
    function build() {
        refs.filter = h(doc, 'input', { id: 'replay-pk-filter', attrs: { placeholder: 'filter by date, name, topic…', type: 'text' },
            on: { input: (e) => setFilter(e.target.value) } });
        refs.close = h(doc, 'button', { cls: 'replay-btn', id: 'replay-pk-close', text: 'close', on: { click: () => close() } });
        refs.banner = h(doc, 'div', { cls: 'replay-banner err', id: 'replay-pk-banner', hidden: true });
        refs.msg = h(doc, 'div', { cls: 'replay-pk-msg', id: 'replay-pk-msg', hidden: true });
        refs.list = h(doc, 'div', { id: 'replay-pk-list' });
        refs.foot = h(doc, 'div', { cls: 'replay-pk-foot', id: 'replay-pk-foot' });
        const card = h(doc, 'div', { cls: 'replay-picker', id: 'replay-picker', attrs: { role: 'dialog', 'aria-label': 'Open recording' } }, [
            h(doc, 'header', { cls: 'replay-pk-head' }, [h(doc, 'h3', { text: 'Open recording' }), refs.filter, refs.close]),
            refs.banner, refs.msg, refs.list, refs.foot,
        ]);
        root = h(doc, 'div', { cls: 'replay-picker-bg', id: 'replay-picker-bg', hidden: true, on: {
            click: (e) => { if (e.target === root) close(); },
        } }, [card]);
        doc.body.appendChild(root);
    }

    function render() {
        if (!root) return;
        root.hidden = !open;
        if (!open) return;
        refs.banner.hidden = !backendDown && !overviewNote;
        refs.banner.className = 'replay-banner ' + (backendDown ? 'err' : 'warn');
        if (!backendDown) fill(refs.banner, overviewNote ? [h(doc, 'span', { text: overviewNote })] : []);
        else {
            fill(refs.banner, [h(doc, 'span', { text: backendWhy + ' ' }),
                h(doc, 'button', { cls: 'replay-btn replay-retry', id: 'replay-pk-retry', text: '[retry]', on: { click: () => retry() } })]);
        }
        refs.msg.hidden = !message;
        refs.msg.textContent = message;
        const rs = rows();
        const kids = rs.map((r) => rowEl(r));
        if (loading && !kids.length) kids.push(h(doc, 'div', { cls: 'replay-pk-row', text: 'loading…' }));
        else if (!kids.length) kids.push(h(doc, 'div', { cls: 'replay-pk-row empty', text: backendDown ? 'no recordings available' : 'no matches' }));
        fill(refs.list, kids);
        if (refs.editInput && refs.editInput.focus && !renaming) refs.editInput.focus();
        refs.foot.textContent = rs.length + ' row' + (rs.length === 1 ? '' : 's') + ' · newest first · recordings open in place, no conversion · dimmed rows (no index, in progress) cannot be opened';
    }

    function editRowEl(r) {
        const stop = (e) => { if (e && e.stopPropagation) e.stopPropagation(); };
        refs.editInput = h(doc, 'input', { id: 'replay-pk-rename-input', attrs: { type: 'text', maxlength: '80', placeholder: 'name (blank = timestamp only)' },
            on: {
                input: (e) => { editValue = e.target.value; },
                click: stop,
                keydown: (e) => {
                    if (e.key === 'Enter') { stop(e); commitRename(); }
                    else if (e.key === 'Escape') { stop(e); cancelRename(); }
                },
            } });
        refs.editInput.value = editValue;
        if (renaming) refs.editInput.disabled = true;
        return h(doc, 'div', { cls: 'replay-pk-row editing', attrs: { 'data-key': r.key }, on: { click: stop } }, [
            h(doc, 'div', { cls: 'replay-pk-edit' }, [
                h(doc, 'span', { cls: 'mono replay-pk-ts', text: r.id.slice(0, 19) + '_' }),
                refs.editInput,
                h(doc, 'button', { cls: 'replay-btn', id: 'replay-pk-rename-save', text: renaming ? 'renaming…' : 'save',
                    on: { click: (e) => { stop(e); commitRename(); } } }),
                h(doc, 'button', { cls: 'replay-btn', id: 'replay-pk-rename-cancel', text: 'cancel',
                    on: { click: (e) => { stop(e); cancelRename(); } } }),
            ]),
        ]);
    }

    function rowEl(r) {
        if (editing === r.key) return editRowEl(r);
        const isOpening = opening === r.key;
        const cell = stateCell(r, isOpening);
        const cls = 'replay-pk-row' + (r.first ? ' first' : '') + (r.selectable ? '' : ' refused') + (isOpening ? ' opening' : '');
        const chips = r.chips.map((c) => h(doc, 'span', { cls: 'replay-chip' + (c.missing ? ' missing' : ''), text: c.label,
            title: c.missing ? 'not in this recording' : '' }));
        return h(doc, 'div', { cls, attrs: { 'data-key': r.key }, on: { click: () => { choose(r.key); } } }, [
            h(doc, 'div', { cls: 'replay-pk-when' }, [
                h(doc, 'b', { text: r.when }),
                r.name ? h(doc, 'span', { text: r.name }) : null,
                r.state === 'inprogress' ? null : h(doc, 'button', { cls: 'replay-pk-rename', text: '✎', title: 'Rename recording',
                    attrs: { 'aria-label': 'Rename ' + r.id }, on: { click: (e) => { if (e && e.stopPropagation) e.stopPropagation(); startRename(r.key); } } }),
            ]),
            h(doc, 'span', { cls: 'mono replay-pk-dur', text: r.duration === null ? '?' : r.duration }),
            h(doc, 'span', { cls: 'mono', text: r.size }),
            h(doc, 'span', { cls: 'mono', text: r.topicCount === null ? '—' : r.topicCount + ' topics' }),
            h(doc, 'div', { cls: 'replay-chips' }, chips),
            h(doc, 'div', { cls: 'replay-st ' + cell.cls, text: cell.text }),
        ]);
    }

    // ---- API ----
    async function openPicker() {
        if (open) return !backendDown;      // already open: no second keydown handler / reload
        if (!root) build();
        open = true; message = ''; opening = null; filter = '';
        refs.filter.value = '';
        keyHandler = (e) => { if (e.key === 'Escape') { close(); } };
        doc.addEventListener('keydown', keyHandler);
        render();
        return loadList();
    }
    function close() {
        if (!open) return;
        open = false; loadToken++; opening = null; editing = null; refs.editInput = null;
        if (keyHandler) { doc.removeEventListener('keydown', keyHandler); keyHandler = null; }
        render();
    }
    function setFilter(s) { filter = s || ''; render(); }
    function retry() { return loadList(); }

    /** Probe the backend without opening the modal (the lobby disables "Open recording…" on false). */
    async function probe() {
        const wasOpen = open;
        const my = loadToken;        // does not bump the token: an in-flight loadList must still render
        try {
            const res = await fetchFn(base + '/recordings');
            const j = await res.json();
            if (my !== loadToken) return !backendDown;
            if (!wasOpen) listing = j;
            backendDown = false; backendWhy = '';
            overviewNote = j.overview_available === false ? OVERVIEW_NOTE : '';
        } catch (e) {
            if (my !== loadToken) return !backendDown;
            backendDown = true;
            backendWhy = 'Replay backend unreachable (' + (e && e.message ? e.message : e) + '). Recordings cannot be listed; the live view is unaffected.';
        }
        emitBackend();
        return !backendDown;
    }

    // ---- rename ----
    function startRename(key) {
        if (renaming || opening) return false;
        const r = rows().find((x) => x.key === key);
        if (!r || r.state === 'inprogress') return false;
        editing = key; editValue = r.label; message = '';
        render();
        return true;
    }
    function setRenameValue(s) { editValue = s || ''; if (refs.editInput) refs.editInput.value = editValue; }
    function cancelRename() {
        if (renaming || editing === null) return;
        editing = null; refs.editInput = null;
        render();
    }
    /** @returns {Promise<string|null>} the new id, or null (the message line says why) */
    async function commitRename() {
        if (renaming || editing === null) return null;
        const id = editing;
        renaming = true; message = ''; render();
        let newId = null;
        try {
            const res = await fetchFn(base + '/recordings/' + encodeURIComponent(id) + '/rename', {
                method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ name: editValue }),
            });
            let j = {};
            try { j = await res.json(); } catch (e) { j = {}; }
            if (!res.ok) throw new Error(j.error || ('HTTP ' + res.status));
            newId = j.id;
        } catch (e) {
            renaming = false;
            message = 'Rename failed: ' + (e && e.message ? e.message : e);
            render();
            return null;
        }
        renaming = false; editing = null; refs.editInput = null;
        if (open) await loadList(); else render();
        return newId;
    }

    /** @param {string} key a recording id */
    async function choose(key) {
        if (opening || editing !== null) return false;
        const r = rows().find((x) => x.key === key);
        if (!r || !r.selectable) return false;
        opening = key; message = ''; render();
        try {
            await mode.enterReplay({ kind: 'recording', id: r.id });
        } catch (e) {
            opening = null;
            message = refusalMessage(e && (e.reason || e.message));
            render();
            return false;
        }
        opening = null;
        close();
        return true;
    }

    return {
        open: openPicker, close, isOpen: () => open, probe, retry, setFilter, choose,
        startRename, setRenameValue, commitRename, cancelRename,
        model: () => ({ open, loading, backendDown, backendWhy, overviewNote, message, opening, filter, editing, renaming, rows: rows() }),
        on(evt, cb) { listeners[evt].add(cb); return () => listeners[evt].delete(cb); },
    };
}
