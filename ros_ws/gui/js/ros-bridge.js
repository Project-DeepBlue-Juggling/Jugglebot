/**
 * ros-bridge.js — ROSLIB connection with auto-reconnect and subscription management.
 *
 * Manages a single WebSocket connection to rosbridge_server (port 9090).
 * Automatically reconnects on disconnection and re-subscribes to all topics.
 * Provides throttled subscription helpers.
 *
 * Replay (design § 3): dispatchLocal() feeds recorded messages to the same
 * subscription callbacks; publish()/callService() refuse while
 * clock.isReplay() (transport fence); setBeforeConnectedHook() lets the replay
 * mode exit before any listener sees 'connected'.
 */

import * as clock from './clock.js';
import { getSessionBuffer } from './replay/session.js';

const RECONNECT_INTERVAL_MS = 2000;
const STALE_TIMEOUT_MS = 5000;  // Mark disconnected if no messages for 5 seconds

/** @type {ROSLIB.Ros | null} */
let ros = null;

/** @type {'connected' | 'disconnected' | 'connecting'} */
let connectionState = 'disconnected';

/** Callbacks for connection state changes: (state) => void */
const stateListeners = [];

/**
 * Registered subscriptions. Kept so we can re-subscribe on reconnect.
 * Each entry: { topicName, messageType, throttleRate, callback, rosTopic }
 */
const subscriptions = [];

/** Registered publishers. Re-created on reconnect. */
const publishers = {};

/** Set to true on page unload so the final close event doesn't trigger a
 *  reconnect.  ONLY the unload path may set this.  Setting it around routine
 *  socket-generation swaps used to latch it true whenever the discarded
 *  socket was already closed — close() on a CLOSED WebSocket fires no event
 *  (spec), so nothing consumed the flag — and the latched flag then swallowed
 *  the next GENUINE disconnect: the GUI sat "Connected" for ~5 s after a
 *  launch shutdown until the staleness watchdog noticed.  Events from
 *  discarded instances are made inert by the per-generation guards in
 *  connect() instead. */
let intentionalClose = false;

/** Timestamp of last message received on any subscription */
let lastMessageTime = 0;

/** Timer for staleness check */
let staleCheckTimer = null;

/** Saved URL for reconnect */
let savedUrl = null;

/** Called synchronously on a 'connected' edge BEFORE the state listeners
 *  (replay mode exit — it must not run after main.js's connect handling). */
let beforeConnectedHook = null;

/**
 * Initialise the ROS connection. Call once at startup.
 * @param {string} [url] - WebSocket URL. Defaults to ws://<page-host>:9090
 */
export function init(url) {
    if (!url) {
        const host = window.location.hostname || 'localhost';
        url = `ws://${host}:9090`;
    }
    savedUrl = url;

    // Gracefully close the WebSocket on page unload so rosbridge
    // doesn't keep an orphaned connection that blocks future connects.
    window.addEventListener('beforeunload', () => {
        intentionalClose = true;
        if (ros) {
            try { ros.close(); } catch { /* ignore */ }
        }
    });

    connect(url);
}

function connect(url) {
    if (connectionState === 'connecting') return;

    // Close and discard any previous ROSLIB.Ros instance.
    // Reusing a closed instance can leave stale internal WebSocket state
    // that prevents rosbridge from accepting the new connection.  Teardown
    // must NOT depend on the old socket firing a close event — close() on
    // an already-closed WebSocket fires nothing — so instead of flagging
    // the close as intentional, every handler below is guarded by instance
    // identity: once `ros` points at a newer generation, events from this
    // one are inert.
    stopStaleCheck();
    if (ros) {
        try { ros.close(); } catch { /* ignore */ }
        ros = null;
    }

    setConnectionState('connecting');

    const myRos = new ROSLIB.Ros();
    ros = myRos;

    myRos.on('connection', () => {
        if (myRos !== ros) return;  // superseded generation — ignore
        setConnectionState('connected');
        resubscribeAll();
        recreatePublishers();
    });

    myRos.on('close', () => {
        if (myRos !== ros) return;  // superseded generation — ignore
        if (intentionalClose) {
            // Page unload in progress.  Reset so a canceled unload (e.g.
            // a download link fires beforeunload but the page stays) does
            // not eat a later genuine close.
            intentionalClose = false;
            return;
        }
        setConnectionState('disconnected');
        scheduleReconnect(url);
    });

    myRos.on('error', () => {
        // In some browsers / ROSLIB versions, a refused WebSocket fires
        // 'error' without a subsequent 'close', leaving us stuck in
        // 'connecting' forever. Schedule a reconnect as a safety net;
        // scheduleReconnect() is idempotent so a duplicate is harmless.
        if (myRos !== ros) return;  // superseded generation — ignore
        if (connectionState === 'connecting') {
            setConnectionState('disconnected');
            scheduleReconnect(url);
        }
    });

    try {
        myRos.connect(url);
    } catch {
        setConnectionState('disconnected');
        scheduleReconnect(url);
    }
}

let reconnectTimer = null;

function scheduleReconnect(url) {
    if (reconnectTimer) return;
    reconnectTimer = setTimeout(() => {
        reconnectTimer = null;
        connect(url);
    }, RECONNECT_INTERVAL_MS);
}

function setConnectionState(state) {
    if (state === connectionState) return;
    connectionState = state;

    if (state === 'connected') {
        lastMessageTime = Date.now(); // wall-clock: stale-socket detection
        startStaleCheck();
        if (beforeConnectedHook) {
            try { beforeConnectedHook(); } catch (e) { console.error('Before-connected hook error:', e); }
        }
    }

    for (const cb of stateListeners) {
        try { cb(state); } catch (e) { console.error('State listener error:', e); }
    }
}

/** Bump the last-message timestamp. Called from wrapped subscription callbacks. */
function touchActivity() {
    lastMessageTime = Date.now(); // wall-clock: stale-socket detection
    // If we were marked stale-disconnected, restore connected state
    if (connectionState === 'disconnected' && ros && ros.isConnected) {
        setConnectionState('connected');
    }
}

function startStaleCheck() {
    if (staleCheckTimer) return;
    staleCheckTimer = setInterval(() => {
        if (connectionState !== 'connected') return;
        if (Date.now() - lastMessageTime >= STALE_TIMEOUT_MS) { // wall-clock: stale-socket detection
            // Silence on a socket the browser still believes is open is the
            // half-open-transport signature: the TCP/NAT path died but no
            // 'close' event ever fired, so the reconnect-on-close path can't
            // recover us — we'd sit 'disconnected' until a manual refresh.
            // Actively tear the dead socket down and reconnect. connect() (via
            // scheduleReconnect) discards this ros instance and opens a fresh
            // one, so recovery doesn't depend on the stale socket ever closing.
            // scheduleReconnect() is single-flight (reconnectTimer), so the
            // old socket's later 'close' scheduling a second reconnect is a
            // harmless no-op — no reconnect storm.
            setConnectionState('disconnected');
            if (ros) {
                try { ros.close(); } catch { /* ignore */ }
            }
            scheduleReconnect(savedUrl);
        }
    }, 1000);
}

function stopStaleCheck() {
    if (staleCheckTimer) {
        clearInterval(staleCheckTimer);
        staleCheckTimer = null;
    }
}

/**
 * Register a callback for connection state changes.
 * @param {function} cb - Called with 'connected', 'disconnected', or 'connecting'
 */
export function onConnectionStateChange(cb) {
    stateListeners.push(cb);
    // Fire immediately with current state
    cb(connectionState);
}

/**
 * Register the one pre-notify hook run on every 'connected' edge before the
 * state listeners (null clears it).
 * @param {function|null} fn
 */
export function setBeforeConnectedHook(fn) {
    beforeConnectedHook = typeof fn === 'function' ? fn : null;
}

/** @returns {'connected' | 'disconnected' | 'connecting'} */
export function getConnectionState() {
    return connectionState;
}

/**
 * Subscribe to a ROS topic with optional client-side throttling.
 * Subscriptions are remembered and re-created on reconnect.
 *
 * @param {string} topicName
 * @param {string} messageType - e.g. 'jugglebot_interfaces/msg/RobotState'
 * @param {function} callback - Called with the message object
 * @param {number} [throttleRate=0] - Minimum ms between callback invocations (0 = no throttle)
 */
export function subscribe(topicName, messageType, callback, throttleRate = 0) {
    const entry = { topicName, messageType, throttleRate, callback, rosTopic: null };
    subscriptions.push(entry);

    if (connectionState === 'connected') {
        createSubscription(entry);
    }
}

function createSubscription(entry) {
    const topic = new ROSLIB.Topic({
        ros,
        name: entry.topicName,
        messageType: entry.messageType,
        throttle_rate: entry.throttleRate,
    });

    topic.subscribe((msg) => {
        touchActivity();
        // Session-buffer tap (replay design § 8): every DELIVERED message, stamped with clock.now().
        // Nothing arrives in replay (the socket is down) but guard anyway.
        if (!clock.isReplay()) {
            try { getSessionBuffer().record(entry.topicName, msg, clock.now() / 1000); } catch (e) { /* never break a subscriber */ }
        }
        entry.callback(msg);
    });
    entry.rosTopic = topic;
}

/**
 * Replay dispatch: invoke the callbacks subscribe() registered for
 * `topicName` (leading '/' ignored on both sides) with a recorded message.
 * Does NOT touch the activity/stale bookkeeping. A throwing callback is
 * logged and skipped so one handler cannot stall the rest of a replay tick.
 * @param {string} topicName
 * @param {object} msg
 * @param {object} [opts] reserved (muting is applied by the caller)
 * @returns {number} callbacks invoked
 */
export function dispatchLocal(topicName, msg, opts) {
    const name = topicName.charAt(0) === '/' ? topicName.slice(1) : topicName;
    let n = 0;
    for (const entry of subscriptions) {
        const en = entry.topicName.charAt(0) === '/' ? entry.topicName.slice(1) : entry.topicName;
        if (en !== name) continue;
        n++;
        try { entry.callback(msg); } catch (e) { console.error('Replay dispatch error on ' + name + ':', e); }
    }
    return n;
}

function resubscribeAll() {
    for (const entry of subscriptions) {
        // Unsubscribe old if exists
        if (entry.rosTopic) {
            try { entry.rosTopic.unsubscribe(); } catch { /* ignore */ }
        }
        createSubscription(entry);
    }
}

/**
 * Create (or get cached) a publisher for a topic.
 * @param {string} topicName
 * @param {string} messageType
 * @returns {{ publish: function }}
 */
export function advertise(topicName, messageType) {
    if (publishers[topicName]) return publishers[topicName].api;

    const entry = { topicName, messageType, rosTopic: null };

    if (connectionState === 'connected') {
        entry.rosTopic = new ROSLIB.Topic({
            ros,
            name: topicName,
            messageType: messageType,
        });
    }

    const pub = {
        publish(msg) {
            if (clock.isReplay()) return;  // transport fence: never publish during replay
            if (entry.rosTopic && connectionState === 'connected') {
                entry.rosTopic.publish(new ROSLIB.Message(msg));
            }
        },
    };

    entry.api = pub;
    publishers[topicName] = entry;
    return pub;
}

function recreatePublishers() {
    for (const key of Object.keys(publishers)) {
        const entry = publishers[key];
        entry.rosTopic = new ROSLIB.Topic({
            ros,
            name: entry.topicName,
            messageType: entry.messageType,
        });
    }
}

/**
 * Call a ROS2 service.
 * @param {string} serviceName - e.g. 'bb/calibrate'
 * @param {string} serviceType - e.g. 'std_srvs/srv/Trigger'
 * @param {object} [request={}] - Service request fields
 * @returns {Promise<object>} - Resolves with the response, rejects on error
 */
export function callService(serviceName, serviceType, request = {}) {
    return new Promise((resolve, reject) => {
        if (!ros || connectionState !== 'connected') {
            reject(new Error('Not connected to ROS'));
            return;
        }
        if (clock.isReplay()) {
            reject(new Error('Replay active - commands disabled'));
            return;
        }

        const service = new ROSLIB.Service({
            ros,
            name: serviceName,
            serviceType: serviceType,
        });

        service.callService(
            new ROSLIB.ServiceRequest(request),
            (result) => { touchActivity(); resolve(result); },
            (error) => { reject(new Error(error)); },
        );
    });
}

/**
 * Wrap `promise` so it rejects after `ms` if `promise` has not settled by
 * then, independent of however long the server side takes.
 *
 * `callService()` above sets no client-side timeout of its own — several
 * callers rely entirely on rosbridge's server-side bound instead
 * (`rosbridge_websocket_lean.py::CALL_SERVICE_TIMEOUT_S`, 50 s). That bound
 * protects rosbridge itself; since 2026-09-28 each call also runs on its own
 * server thread (`rosbridge_websocket_lean.py` item 5), so one unanswered call
 * no longer stalls the rest of the tab. A caller that wraps its
 * `callService()` in `withTimeout()` still gives up locally in a few seconds,
 * so its own UI recovers promptly instead of waiting out the 50 s bound —
 * see `bb-aim.js` for the worked example.
 * (Mirrors the identical local helper in `state-minimap.js`.)
 *
 * @param {Promise} promise
 * @param {number} ms
 * @param {string} label - named in the timeout's error message
 * @returns {Promise}
 */
export function withTimeout(promise, ms, label) {
    return new Promise((resolve, reject) => {
        const t = setTimeout(
            () => reject(new Error(`${label} timed out after ${ms / 1000} s`)),
            ms);
        promise.then(
            (v) => { clearTimeout(t); resolve(v); },
            (e) => { clearTimeout(t); reject(e); },
        );
    });
}

/** Local bound on one topic-discovery poll (rosbridge's own is 50 s). */
const T_DISCOVERY_MS = 5000;
let discoveryInFlight = false;

/**
 * Discover all active ROS2 topics via rosbridge.
 *
 * One poll in flight at a time, bounded locally. main.js polls every 3 s and
 * rosbridge gives each call its own thread with a 50 s bound, so a slow or
 * silent rosapi used to collect ~16 concurrent calls per tab, each ending in
 * its own rosbridge TimeoutError (logbook
 * 2026-10-02-rosbridge-service-clients-on-executor-thread.md). Now a tick
 * skips while a poll is pending, and a reply that never comes frees the slot
 * after T_DISCOVERY_MS.
 * @param {function} callback - Called with { topics: string[], types: string[] }
 */
export function discoverTopics(callback) {
    if (!ros || connectionState !== 'connected') return;
    if (discoveryInFlight) return;
    discoveryInFlight = true;
    withTimeout(
        new Promise((resolve, reject) => ros.getTopics(resolve, reject)),
        T_DISCOVERY_MS, 'topic discovery')
        .then(callback, (err) => { console.warn('Failed to discover topics:', err); })
        .finally(() => { discoveryInFlight = false; });
}

/**
 * Create a lightweight subscription for topic monitoring (counting only).
 * Unlike subscribe(), these are NOT re-created on reconnect — the discovery
 * timer handles re-creation.
 * @param {string} topicName
 * @param {string} messageType
 * @param {function} callback
 * @param {number} [throttleRate=200]
 * @returns {ROSLIB.Topic} The topic object (for later unsubscription)
 */
export function subscribeSpy(topicName, messageType, callback, throttleRate = 200) {
    if (!ros || connectionState !== 'connected') return null;
    const topic = new ROSLIB.Topic({
        ros,
        name: topicName,
        messageType: messageType,
        throttle_rate: throttleRate,
        // Spy callbacks only count arrivals (recordTopicMessage in main.js) —
        // they never read a field — so raw CBOR (rclpy raw=True) skips
        // rosbridge's per-message deserialisation, including for the two
        // 100 Hz topics a spy can land on. Measured -15 to -17% CPU; the
        // monitor's displayed rate is unaffected, since that's the THROTTLED
        // delivery rate above, not decode cost.
        //
        // rosbridge shares ONE ROS subscription per topic and fixes it
        // raw-vs-decoded at first creation, so a spy must never target a
        // topic the GUI also subscribes decoded, or whichever subscription
        // was created first wins for both. main.js's GUI_SUBSCRIBED_TOPICS
        // guarantees that by skipping every topic this spy would otherwise
        // hit — see the contract note there.
        compression: 'cbor-raw',
    });
    topic.subscribe(callback);
    return topic;
}

/**
 * Unsubscribe a spy subscription.
 * @param {ROSLIB.Topic} topic
 */
export function unsubscribeSpy(topic) {
    if (topic) {
        try { topic.unsubscribe(); } catch { /* ignore */ }
    }
}
