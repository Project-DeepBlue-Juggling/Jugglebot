/**
 * ros-bridge.js — ROSLIB connection with auto-reconnect and subscription management.
 *
 * Manages a single WebSocket connection to rosbridge_server (port 9090).
 * Automatically reconnects on disconnection and re-subscribes to all topics.
 * Provides throttled subscription helpers.
 */

const RECONNECT_INTERVAL_MS = 2000;

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

/**
 * Initialise the ROS connection. Call once at startup.
 * @param {string} [url] - WebSocket URL. Defaults to ws://<page-host>:9090
 */
export function init(url) {
    if (!url) {
        const host = window.location.hostname || 'localhost';
        url = `ws://${host}:9090`;
    }

    ros = new ROSLIB.Ros();

    ros.on('connection', () => {
        setConnectionState('connected');
        resubscribeAll();
        recreatePublishers();
    });

    ros.on('close', () => {
        setConnectionState('disconnected');
        scheduleReconnect(url);
    });

    ros.on('error', () => {
        // Error fires before close; we handle reconnect on close.
    });

    connect(url);
}

function connect(url) {
    if (connectionState === 'connecting') return;
    setConnectionState('connecting');
    try {
        ros.connect(url);
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
    connectionState = state;
    for (const cb of stateListeners) {
        try { cb(state); } catch (e) { console.error('State listener error:', e); }
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

    topic.subscribe(entry.callback);
    entry.rosTopic = topic;
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
    if (publishers[topicName]) return publishers[topicName];

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
            if (entry.rosTopic && connectionState === 'connected') {
                entry.rosTopic.publish(new ROSLIB.Message(msg));
            }
        },
    };

    publishers[topicName] = { ...entry, api: pub };
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
 * Discover all active ROS2 topics via rosbridge.
 * @param {function} callback - Called with { topics: string[], types: string[] }
 */
export function discoverTopics(callback) {
    if (!ros || connectionState !== 'connected') return;
    ros.getTopics(
        (result) => { callback(result); },
        (err) => { console.warn('Failed to discover topics:', err); }
    );
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
