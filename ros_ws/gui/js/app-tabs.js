/**
 * app-tabs.js — Robot | Map tab bar.
 *
 * Robot = the existing GUI, never unmounted.  Map = an iframe of the project-map
 * app (same host, port 8765), created on the FIRST switch to Map and then kept
 * alive (the pane is hidden with CSS, not destroyed) so switching is instant and
 * the map keeps updating.  Nothing on the robot side is paused while Map shows.
 *
 * No persistence: the page loads on Robot unless the URL hash is `#map`, so the
 * E-stop / Fault / ROS state is what an operator sees on load.  While Map shows,
 * `#app` is `inert` (no focus or keyboard handlers reach the hidden robot view);
 * inert does not stop websockets or rendering.
 */

const MAP_PORT = 8765;
const PROBE_TIMEOUT_MS = 3000;

const tabs = document.getElementById('app-tabs');
const pane = document.getElementById('map-pane');
const appEl = document.getElementById('app');
const mapUrl = (() => {
    const u = new URL(location.href);
    u.port = String(MAP_PORT);
    u.pathname = '/';
    u.search = '';
    u.hash = '';
    return u.href;
})();
let mapCreated = false;

async function createMap() {
    mapCreated = true;
    pane.replaceChildren();
    const note = document.createElement('div');
    note.className = 'map-note';
    note.textContent = `Loading map from ${mapUrl} ...`;
    pane.appendChild(note);
    let reachable = false;
    const ctl = new AbortController();
    const t = setTimeout(() => ctl.abort(), PROBE_TIMEOUT_MS);
    try {
        await fetch(`${mapUrl}api/v1/health`, { mode: 'no-cors', signal: ctl.signal });
        reachable = true;
    } catch (e) { /* unreachable or timed out */ } finally {
        clearTimeout(t);
    }
    if (!reachable) {
        note.textContent = `Map unreachable at ${mapUrl} - is the map service running? Switch away and back to retry.`;
        mapCreated = false;  // allow a retry on the next switch
        return;
    }
    pane.replaceChildren();
    const frame = document.createElement('iframe');
    frame.src = mapUrl;
    frame.title = 'Project map';
    pane.appendChild(frame);
}

function select(name, { updateHash = true } = {}) {
    if (name !== 'robot' && name !== 'map') name = 'robot';
    for (const b of tabs.querySelectorAll('[role="tab"]')) {
        b.setAttribute('aria-selected', String(b.dataset.tab === name));
    }
    pane.hidden = name !== 'map';
    if (appEl) appEl.inert = name === 'map';
    if (name === 'map' && !mapCreated && !pane.querySelector('iframe')) createMap();
    if (updateHash && location.hash !== '#' + name) {
        history.replaceState(null, '', '#' + name);
    }
}

tabs.addEventListener('click', (e) => {
    const b = e.target.closest('[role="tab"]');
    if (b) select(b.dataset.tab);
});
window.addEventListener('hashchange', () => {
    const h = location.hash.slice(1);
    if (h === 'map' || h === 'robot') select(h, { updateHash: false });
});

const hash = location.hash.slice(1);
const initial = hash === 'map' ? 'map' : 'robot';
select(initial, { updateHash: false });
