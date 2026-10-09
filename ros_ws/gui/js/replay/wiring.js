/**
 * wiring.js — production dependencies for replay/mode.js (browser only).
 *
 * main.js is passed IN (`{resetForSeek, blankDisconnectedState}`) rather than imported, so this module
 * never forms an import cycle with the entry module that will eventually own the singleton (Phase 3).
 */
import * as ros from '../ros-bridge.js';
import * as clock from '../clock.js';
import { setReplayFence } from './fence.js';
import {
    snapshotAndBeginReplayEvents, restoreEvents, setEventsMuted, trimEventsAfter,
} from '../event-store.js';
import {
    enterReplayCharts, exitReplayCharts, setReplayPlayhead, telemetrySample,
} from '../telemetry-charts.js';
import { setCanTrafficRosLink, resetTrafficRing as resetCanTrafficRing } from '../can-traffic.js';
import { setUdpTrafficRosLink, resetTrafficRing as resetUdpTrafficRing } from '../udp-traffic.js';
import { setHardwareVersionsRosLink } from '../hardware-versions.js';
import { createReplayChartStore } from './chart-store.js';
import { RecordingSource } from './sources.js';
import { getSessionBuffer } from './session.js';
import { createChunkCache } from './cache.js';
import { createEngine } from './engine.js';
import { indexLatestBefore } from './chunk.js';
import { createReplayMode } from './mode.js';

let singleton = null;

/** Empty both traffic panels' sample rings (replay seek / exit). */
export function resetTrafficRings() {
    resetCanTrafficRing();
    resetUdpTrafficRing();
}

/**
 * @param {{resetForSeek:function, blankDisconnectedState:function}} mainApi
 * @param {{visibleSpanSec?:function, resetTrafficRings?:function}} [extra]
 */
export function getReplayMode(mainApi, extra) {
    if (singleton) return singleton;
    extra = extra || {};
    singleton = createReplayMode({
        ros, clock,
        fence: { setReplayFence },
        events: { snapshotAndBeginReplayEvents, restoreEvents, setEventsMuted, trimEventsAfter },
        charts: {
            createStore: createReplayChartStore,
            enter: enterReplayCharts, exit: exitReplayCharts, setPlayhead: setReplayPlayhead, telemetrySample,
        },
        links: { can: setCanTrafficRosLink, udp: setUdpTrafficRosLink, hw: setHardwareVersionsRosLink },
        resetForSeek: mainApi.resetForSeek,
        blankDisconnectedState: mainApi.blankDisconnectedState,
        getReplayLatches: mainApi.getReplayLatches,
        restoreReplayLatches: mainApi.restoreReplayLatches,
        resetTrafficRings: extra.resetTrafficRings || resetTrafficRings,
        makeRecordingSource: (id) => RecordingSource({ id, fetch: (u, o) => fetch(u, o) }),
        getSessionBuffer,
        createCache: (source) => createChunkCache({ source }),
        createEngine,
        indexLatestBefore,
        visibleSpanSec: extra.visibleSpanSec,
        raf: {
            request: (cb) => requestAnimationFrame(cb),
            cancel: (id) => cancelAnimationFrame(id),
            hidden: () => document.hidden,
        },
    });
    return singleton;
}
