/**
 * index.js — Phase 3 replay UI entry: toaster + picker + lobby over one mode singleton.
 * main.js imports this (never the reverse); the trackbar/transport mounts into `#replay-dock` while the mode is OPENING/REPLAY.
 */
import { createToaster } from './toast.js';
import { createPicker } from './picker.js';
import { createLobby } from './lobby.js';
import { createTrackbar } from './trackbar.js';
import { createAbsent } from './absent.js';
import { createFenceDom } from './fence-dom.js';
import { createHide } from './hide.js';
import { createDeepLink } from './deeplink.js';
import { refusalMessage } from './format.js';
import { setJugglePanelReplayHidden } from '../../juggle-panel.js';
import * as ros from '../../ros-bridge.js';

let ui = null;
/** The UI singleton created by initReplayUi (null before main.js init). */
export function getReplayUi() { return ui; }

/** @param {object} mode getReplayMode(...) singleton @returns {{toaster, picker, lobby}} */
export function initReplayUi(mode) {
    const document_ = document;
    const toaster = createToaster({ document: document_ });
    const picker = createPicker({ document: document_, fetch: (u, o) => fetch(u, o), mode });
    const lobby = createLobby({ document: document_, ros, mode, picker, toaster });
    const trackbar = createTrackbar({
        document: document_, mode, dock: lobby.dock, toaster,
        raf: { request: (cb) => requestAnimationFrame(cb), cancel: (id) => cancelAnimationFrame(id) },
    });
    const absent = createAbsent({ document: document_, mode });
    const fenceDom = createFenceDom({ document: document_, mode, MutationObserver: globalThis.MutationObserver });
    const hide = createHide({ document: document_, mode, setJuggleHidden: setJugglePanelReplayHidden });
    const deepLink = createDeepLink({
        search: document_.defaultView && document_.defaultView.location ? document_.defaultView.location.search : '',
        ros, mode, toaster, refusalMessage,
        list: async () => { const r = await fetch('/api/replay/recordings'); if (!r.ok) throw new Error('HTTP ' + r.status); const j = await r.json(); return (j.recordings || []).map((x) => x.id); },
    });
    ui = { deepLink, toaster, picker, lobby, trackbar, absent, fenceDom, hide, mode, dock: lobby.dock };
    return ui;
}
