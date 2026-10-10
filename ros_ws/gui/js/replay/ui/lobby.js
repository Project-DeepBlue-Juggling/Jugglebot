/**
 * lobby.js — the disconnected command overlay IS the lobby (charting decisions 7/8).
 *
 * createLobby({document, ros, mode, picker, toaster, getSessionBuffer}) -> {update, dock, state}
 *
 * Visible when the rosbridge state is not 'connected' and replay is idle: the live command buttons
 * (direct children of #command-overlay) are hidden by CSS and the lobby ("Replay last session" /
 * "Open recording…") sits in their place, with the "Not connected…" banner. While replay is OPENING /
 * REPLAY the lobby hides and `#replay-dock` (a stable id; unit 2 fills it) owns the overlay region.
 * On 'connected' the live overlay returns. Also owns the header text (`REPLAY  <date>`) because the
 * live connection listener is suppressed while replay is active, and routes mode 'notice' to the toaster.
 */
import { h, fill, ensure } from './dom.js';
import { fmtDuration, fmtDateTime, dateFromId } from './format.js';

export const BANNER_TEXT = 'Not connected to the robot. Live commands are unavailable — you can still review a recording.';

export function createLobby(deps) {
    const doc = deps.document;
    const { ros, mode, picker, toaster } = deps;
    const getSession = deps.getSessionBuffer;

    const overlay = doc.getElementById('command-overlay');
    const viewerPane = doc.getElementById('viewer-pane') || (overlay && overlay.parentNode) || doc.body;
    const dock = ensure(doc, 'replay-dock', overlay, 'div', 'replay-dock');
    const lobbyEl = ensure(doc, 'replay-lobby', overlay, 'div', 'replay-lobby');
    const banner = ensure(doc, 'replay-banner', viewerPane, 'div', 'replay-banner warn');
    const backendBanner = ensure(doc, 'replay-backend-banner', viewerPane, 'div', 'replay-banner err');

    let backendDown = false;
    let overviewNote = '';
    let failed = false;          // a real connect attempt has failed (a 'disconnected' EDGE; never the initial default)
    let lobbyShown = false;
    let btnSession = null, btnOpen = null;
    let savedHeader = null;

    function sessionSeconds() {
        try {
            const b = getSession();
            const r = b.range();
            return b.chunkCount() > 0 && r.frontier > r.t0 ? r.frontier - r.t0 : 0;
        } catch (e) { return 0; }
    }

    function buildButtons() {
        btnSession = h(doc, 'button', { cls: 'replay-btn replay-primary', id: 'replay-btn-session', on: { click: () => {
            if (btnSession.disabled) return;
            mode.enterReplay({ kind: 'session' }).catch((e) => {
                toaster.show('Could not replay the session: ' + (e && (e.reason || e.message)), 6000);
            });
        } } });
        btnOpen = h(doc, 'button', { cls: 'replay-btn', id: 'replay-btn-open', text: 'Open recording…', on: { click: () => {
            if (!btnOpen.disabled) picker.open();
        } } });
        fill(lobbyEl, [btnSession, btnOpen, h(doc, 'span', { cls: 'replay-note', text: 'Home / Level / Activate are hidden while disconnected' })]);
    }

    function refreshButtons() {
        if (!btnSession) buildButtons();
        const secs = sessionSeconds();
        btnSession.textContent = secs > 0 ? 'Replay last session (in memory, ' + fmtDuration(secs) + ' of data)' : 'Replay last session';
        btnSession.disabled = !(secs > 0);   // the session replay needs no backend
        btnSession.title = secs > 0 ? '' : 'No live session in this page\'s memory';
        btnOpen.disabled = !!backendDown;
        btnOpen.title = backendDown ? 'Replay backend unreachable' : '';
    }

    function headerText() {
        const st = ros.getConnectionState();
        if (st === 'connected') return ['status-dot connected', 'Connected'];
        if (st === 'connecting') return ['status-dot disconnected', 'Connecting...'];
        return ['status-dot disconnected', 'Disconnected'];
    }

    function replayDate() {
        const src = mode.source && mode.source();
        if (src && src.range) {
            const r = src.range();
            if (r && r.t0 > 1e8) return fmtDateTime(r.t0).slice(0, 10);
            if (src.id) { const d = dateFromId(src.id); if (d) return d.slice(0, 10); }
        }
        return '';
    }

    function paintHeader(active) {
        const dot = doc.getElementById('conn-dot');
        const text = doc.getElementById('conn-text');
        if (!dot || !text) return;
        if (active) {
            dot.className = 'status-dot replay';
            const opening = mode.state && mode.state().mode === 'OPENING';
            text.textContent = opening ? 'REPLAY  opening…' : 'REPLAY  ' + replayDate();   // held until slot 0 is resident
        } else {
            const [c, t] = headerText();
            dot.className = c; text.textContent = t;
        }
    }

    let wasActive = false;
    function update() {
        const active = mode.isActive();
        const connected = ros.getConnectionState() === 'connected';
        const showLobby = !active && !connected && failed;
        if (overlay) {
            overlay.classList.toggle('replay-lobby-on', showLobby);
            overlay.classList.toggle('replay-active', active);
        }
        lobbyEl.hidden = !showLobby;
        dock.hidden = !active;
        banner.textContent = BANNER_TEXT;
        banner.hidden = !showLobby;
        if (showLobby) {
            refreshButtons();
            if (!lobbyShown) { lobbyShown = true; picker.probe(); }
        } else lobbyShown = false;
        backendBanner.hidden = !(showLobby && (backendDown || overviewNote));
        backendBanner.className = 'replay-banner ' + (backendDown ? 'err' : 'warn');
        if (showLobby && backendDown) backendBanner.textContent = 'Replay backend unreachable. Recordings cannot be listed; the live view is unaffected.';
        else if (showLobby && overviewNote) backendBanner.textContent = 'Timeline overview unavailable. Recordings still open.';
        if (active || wasActive) paintHeader(active);
        wasActive = active;
    }

    mode.on('state', update);
    mode.on('notice', (n) => { if (n && n.text) toaster.show(n.text, 6000); });
    picker.on('backend', (b) => { backendDown = b.down; overviewNote = b.note || ''; update(); });
    let registering = true;
    ros.onConnectionStateChange((st) => {
        if (!registering) {
            if (st === 'disconnected') failed = true;
            else if (st === 'connected') failed = false;
        }
        update();
    });
    registering = false;
    update();

    return { update, dock, state: () => ({ lobby: !lobbyEl.hidden, active: mode.isActive(), backendDown, overviewNote }) };
}
