/**
 * juggle-panel.js — The Juggle command panel: pattern, throws, apex,
 * separation, a read-only dwell, the Ball Butler reload, a hold-to-confirm
 * Start, an immediate Stop, the live attempt status, and an animated 2-D
 * front-view diagram of the selected pattern on its real timing.
 *
 * Mounted by state-minimap.js under the state-machine graph (it replaced
 * that file's one-row juggle strip, 2026-10-04).  The minimap owns WHEN this
 * renders — renderJugglePanel() runs on every applySnapshot frame with the
 * same snapshot the sequencer reads — and this module owns WHAT renders.
 * The panel never decides reachability itself: it only reads the booleans it
 * is handed, so the minimap's staleness rules (a stale topic reads as
 * "unknown", never as "fine") apply here unchanged.
 *
 * Request surface (orchestrator_node._parse_juggle_request relays it to the
 * jugglebot/juggle action — rosbridge on Foxy has no action transport):
 *
 *     jugglebot/juggle_request  (jugglebot_interfaces/srv/SetString)
 *     data = <pattern>[,reload][,apex_m=<f>][,separation_mm=<f>][,num_cycles=<int>]
 *
 * THE GUI NEVER ENCODES A LAUNCH DEFAULT.  A blank field sends nothing; the
 * relay then fills the goal field with 0, and 0 is skill_node's "use my own
 * parameter" sentinel (Juggle.action).  The node's live values are fetched
 * from its parameter service only to SHOW them (placeholders, the diagram,
 * the timing line) — they are never copied into a request.  Were they, a
 * `ros2 param set` between the fetch and the Start would be silently
 * overridden by the GUI's stale copy, and a relaunch with a new default
 * would keep flying the old one from any tab left open.
 *
 * Two compatibility rules for a robot still running the PRE-2026-10-04
 * relay (it only checks `parts[1] == 'reload'` and drops everything else):
 *   1. `reload` is always emitted immediately after the pattern, so the old
 *      relay still gets reload right.
 *   2. Cross-check: the next skills/attempt juggle_start after our dispatch
 *      carries the RESOLVED request; any operator-typed value it does not
 *      match raises an amber warning (the old relay silently flies the node
 *      defaults instead of the typed values — the fix is a colcon build).
 *
 * Dwell is NOT a goal field: it is skill_node's `dwell_s` parameter and the
 * admissible box is swept AT that dwell (a different live dwell is
 * refused), so it is shown read-only.
 */

import * as clock from './clock.js';
import * as ros from './ros-bridge.js';
import { holdToConfirm } from './hold-to-confirm.js';
import { emitEvent, EVENT_TYPES } from './event-store.js';

// ====================================================================
// Constants
// ====================================================================

/** Same hold every other motion-starting control on the minimap uses. */
const HOLD_MS = 800;
/** Start stays disabled this long after a dispatch — stops accidental
 *  double-holds only; a genuine concurrent goal is refused by skill_node. */
const JUGGLE_COOLDOWN_MS = 4000;
/** A dispatch/stop status line clears itself after this (never lingers stale). */
const STATUS_CLEAR_MS = 8000;

// Service timeouts.  ros.callService has none of its own, and an
// un-answered dispatch used to leave Start latched busy forever (the
// cooldown only started in .finally()).
const T_SVC_REQUEST = 10000;
const T_SVC_STOP = 5000;
const T_SVC_PARAMS = 3000;
/** Param-fetch retry while connected and skill_node isn't answering yet
 *  (it starts after rosbridge): a few fast tries, then slow. */
const PARAM_RETRY_FAST_MS = 5000;
const PARAM_RETRY_SLOW_MS = 30000;
const PARAM_RETRY_FAST_N = 3;
/** How long after a dispatch a juggle_start still counts as OURS for the
 *  stale-relay cross-check (accept is prompt; the feed wait comes after). */
const CROSSCHECK_WINDOW_MS = 30000;

/** schedule.py's _G_SI (ballistics_bc.GRAVITY_MMS2 / 1000). */
const G_SI = 9.81;

const SETSTRING = 'jugglebot_interfaces/srv/SetString';
const TRIGGER = 'std_srvs/srv/Trigger';
const PARAM_SERVICE = '/skill_node/get_parameters';
const PARAM_SERVICE_TYPE = 'rcl_interfaces/srv/GetParameters';
/** columns_feed_site is display-only: it decides which site each columns
 *  ball flies at (skill_node._run_columns / _run_columns_1ball). */
const PARAM_NAMES = ['apex_m', 'separation_mm', 'dwell_s', 'n_throws', 'columns_feed_site'];

const LS_SPEED_KEY = 'jugglebot-juggle-diagram-speed';
const SPEEDS = [0.5, 1, 0.25];
const SPEED_LABEL = { 0.5: '½× speed', 1: '1× speed', 0.25: '¼× speed' };

const SVG_NS = 'http://www.w3.org/2000/svg';

/**
 * The goal-facing patterns (SkillNode._PATTERNS).  `label` feeds the Event
 * Log via minimapOnSkillAttempt — changing one changes the log text.
 *   kind     'one' = compile_one_ball (self_toss / hop), 'columns' =
 *            compile_columns, flown by _run_columns / _run_columns_1ball
 *   phantom  schedule ball id flown motion-only (Pattern.phantom_balls)
 *   reloadRequired  the relay/skill_node refuse reload=False
 */
export const JUGGLE_PATTERNS = [
    { value: 'self_toss', label: 'Self-toss', kind: 'one', sites: 1, phantom: null,
      blurb: 'One ball, one site: straight up and caught where it was thrown.' },
    { value: 'hop', label: 'Hop', kind: 'one', sites: 2, phantom: null,
      blurb: 'One ball, two sites: thrown across and caught at the other site, ' +
             'the hand coasting under it; back and forth.' },
    { value: 'columns', label: 'Columns', kind: 'columns', sites: 2, phantom: null,
      blurb: 'Two balls, each rising and falling in its own column; the one hand ' +
             'alternates sites (throw, transit, catch + throw).' },
    // B2: columns motion flown with one real ball (a phantom at the other
    // site) -- same apex_m/separation_mm/reload fields as Columns
    // (SkillNode._run_columns_1ball).
    { value: 'columns_1ball', label: 'Columns (1 ball)', kind: 'columns', sites: 2, phantom: 1,
      blurb: 'Columns motion with Ball 1 real; Ball 2 is a phantom (motion only).' },
    // B2, R5 sitting 4: the OTHER half -- ball A phantom (Jugglebot's own
    // strokes fly empty), ball B real and fed by Ball Butler. reload=True
    // is required (SkillNode._start_pattern refuses reload=False for this
    // pattern) (SkillNode._run_columns(..., phantom_a=True)).
    { value: 'columns_1ball_fed', label: 'Columns (1 ball, fed)', kind: 'columns', sites: 2,
      phantom: 0, reloadRequired: true,
      blurb: 'Columns motion with Ball 1 a phantom; Ball 2 is real and fed by Ball Butler.' },
];

/**
 * Operator-typed fields.  The bounds are TYPO GUARDS, not authority: the
 * admissible box in skill_node is what decides whether a value flies.  They
 * catch the unit slips that matter (apex typed in mm reads as a 900 m
 * throw) and the 0 that would silently mean "node default" on the wire.
 */
const FIELDS = {
    num_cycles: {
        label: 'Throws', unit: '', integer: true, max: 999, param: 'n_throws',
        tip: 'num_cycles — the number of throws (both balls count). Blank = skill_node\'s n_throws.',
        tooBig: 'more than 999 throws — a typo?',
    },
    apex_m: {
        label: 'Apex', unit: 'm', integer: false, max: 3, param: 'apex_m',
        tip: 'apex_m — ball apex above the release, metres. Blank = skill_node\'s apex_m.',
        tooBig: 'apex is in METRES',
    },
    separation_mm: {
        label: 'Sep.', unit: 'mm', integer: false, max: 1000, param: 'separation_mm',
        tip: 'separation_mm — distance between the two sites P1/P2. Blank = skill_node\'s separation_mm.',
        tooBig: 'over 1000 mm — check the unit',
    },
};
const FIELD_KEYS = ['num_cycles', 'apex_m', 'separation_mm'];

// Display-only stand-ins when the node's defaults are unknown: the diagram
// needs SOME numbers to move.  They never reach a request, and every label
// derived from them shows "?" instead of a number.
const NOMINAL = { apex_m: 0.9, dwell_s: 0.27, separation_mm: 250 };

// ====================================================================
// State
// ====================================================================

let dom = null;
let transport = {
    callService: (name, type, req) => ros.callService(name, type, req),
};

// expanded starts false: the minimap builds the panel before its first
// render, and a rAF loop started for a panel that is about to be collapsed
// would run one wasted frame.
const gate = { connected: false, active: false, expanded: false, mainState: null };
const ui = { pattern: JUGGLE_PATTERNS[0].value, userReload: false };
const fields = {};
for (const k of FIELD_KEYS) fields[k] = { raw: '', value: null, error: null };

/** skill_node's live parameters (display only — see the module header). */
let defaults = null;            // { apex_m, separation_mm, dwell_s, n_throws, columns_feed_site } | null
let defaultsState = 'unknown';  // unknown | loading | ok | failed
let paramsInFlight = false;
let paramRetryTimer = null;
let paramFailures = 0;

let busy = false;               // post-dispatch cooldown latch
let cooldownTimer = null;
let transient = null;           // { text, cls } — dispatch/stop ack line
let transientTimer = null;
let running = null;             // juggle_start fields + t0, until juggle_end
let last = null;                // { kind: 'end'|'refused', ... }
let crossCheck = null;          // { pattern, typed, reload, t }
let warn = null;                // stale-relay mismatch text

let speed = 0.5;
try {
    const s = parseFloat(localStorage.getItem(LS_SPEED_KEY));
    if (SPEEDS.includes(s)) speed = s;
} catch (e) { /* storage blocked — default speed */ }

// Diagram animation state.
let diagramKey = '';
let model = null;               // see buildModel()
let rafId = 0;
let replayHidden = false;      // replay mode hides the panel: no renders, no rAF loop
let lastGate = null;            // last renderJugglePanel input, re-applied once on un-hide
let animT0 = 0;
const reducedMotion = typeof matchMedia === 'function'
    && matchMedia('(prefers-reduced-motion: reduce)').matches;

// ====================================================================
// Small helpers
// ====================================================================

function el(tag, cls, text) {
    const e = document.createElement(tag);
    if (cls) e.className = cls;
    if (text != null) e.textContent = text;
    return e;
}

function svgEl(tag, attrs, cls, parent) {
    const e = document.createElementNS(SVG_NS, tag);
    if (attrs) for (const k of Object.keys(attrs)) e.setAttribute(k, attrs[k]);
    if (cls) e.setAttribute('class', cls);
    if (parent) parent.appendChild(e);
    return e;
}

// Write-if-changed setters: renderJugglePanel runs every minimap frame, so
// an unchanged value must not touch the DOM (no style recalc, no mutation
// records for the probes, no flicker of a hovered tooltip).
function setText(e, text) {
    if (e._jpText !== text) { e._jpText = text; e.textContent = text; }
}
function setTitle(e, title) {
    if (e._jpTitle !== title) { e._jpTitle = title; e.title = title; }
}
function setDisabled(e, d) {
    if (e.disabled !== d) e.disabled = d;
}
function setClassSet(e, base, mods) {
    const cls = mods ? base + ' ' + mods : base;
    if (e._jpCls !== cls) { e._jpCls = cls; e.className = cls; }
}
function setHidden(e, h) {
    if (e.hidden !== h) e.hidden = h;
}

function patternOf(value) {
    return JUGGLE_PATTERNS.find((p) => p.value === value) || JUGGLE_PATTERNS[0];
}

/** Wire float: plain decimal, never exponent notation, no trailing zeros
 *  (toFixed never uses an exponent below 1e21; the typo guards keep us far
 *  below it). */
function wireNum(v, integer) {
    if (integer) return String(Math.round(v));
    return v.toFixed(6).replace(/\.?0+$/, '');
}

const fmt2 = (v) => v.toFixed(2);
const flightS = (apex) => 2 * Math.sqrt(2 * apex / G_SI);   // schedule.flight_s

function fmtDur(s) {
    return s < 10 ? s.toFixed(1) + ' s' : Math.round(s) + ' s';
}

// ====================================================================
// Fields, effective values, request
// ====================================================================

function parseField(key, raw) {
    const spec = FIELDS[key];
    const s = String(raw).trim();
    if (s === '') return { value: null, error: null };
    const v = Number(s);
    if (!Number.isFinite(v)) return { value: null, error: spec.label + ': not a number' };
    if (spec.integer && !Number.isInteger(v)) {
        return { value: null, error: spec.label + ': a whole number' };
    }
    // 0 is the wire's "node default" sentinel — typing it would look like an
    // override and silently fly the default; a blank field says so honestly.
    if (v <= 0) return { value: null, error: spec.label + ': must be > 0 (blank = node default)' };
    if (v > spec.max) return { value: null, error: spec.label + ': ' + spec.tooBig };
    return { value: v, error: null };
}

/** Does `key` apply to the selected pattern?  Separation is meaningless for
 *  a one-site pattern (skill_node ignores it), so it is never sent there. */
function fieldApplies(key, p) {
    return !(key === 'separation_mm' && p.sites === 1);
}

function effectiveReload(p) {
    return !!(p.reloadRequired || ui.userReload);
}

/** The operator-typed overrides that WILL be sent for the selected pattern. */
function typedOverrides(p) {
    const out = {};
    for (const k of FIELD_KEYS) {
        if (fieldApplies(k, p) && fields[k].value != null) out[k] = fields[k].value;
    }
    return out;
}

function firstFieldError(p) {
    for (const k of FIELD_KEYS) {
        if (fieldApplies(k, p) && fields[k].error) return fields[k].error;
    }
    return null;
}

function buildRequestData(p) {
    // `reload` immediately after the pattern (old-relay compatibility rule 1).
    const parts = [p.value];
    if (effectiveReload(p)) parts.push('reload');
    const typed = typedOverrides(p);
    if (typed.apex_m != null) parts.push('apex_m=' + wireNum(typed.apex_m, false));
    if (typed.separation_mm != null) parts.push('separation_mm=' + wireNum(typed.separation_mm, false));
    if (typed.num_cycles != null) parts.push('num_cycles=' + wireNum(typed.num_cycles, true));
    return parts.join(',');
}

/** Value a field would fly with: typed, else the node's live default, else
 *  null (unknown). */
function effective(key) {
    if (fields[key].value != null) return fields[key].value;
    const spec = FIELDS[key];
    if (defaults && defaults[spec.param] != null) return defaults[spec.param];
    return null;
}

// ====================================================================
// skill_node defaults (display only)
// ====================================================================

function paramValue(pv) {
    if (!pv) return null;
    switch (pv.type) {
        case 1: return !!pv.bool_value;
        case 2: return Number(pv.integer_value);
        case 3: return Number(pv.double_value);
        case 4: return String(pv.string_value);
        default: return null;   // 0 = PARAMETER_NOT_SET
    }
}

function scheduleParamRetry() {
    clearTimeout(paramRetryTimer);
    const ms = paramFailures <= PARAM_RETRY_FAST_N ? PARAM_RETRY_FAST_MS : PARAM_RETRY_SLOW_MS;
    paramRetryTimer = setTimeout(() => fetchDefaults(), ms);
}

function fetchDefaults() {
    if (!gate.connected || paramsInFlight) return;
    clearTimeout(paramRetryTimer);
    paramsInFlight = true;
    if (defaultsState !== 'ok') defaultsState = 'loading';
    ros.withTimeout(
        transport.callService(PARAM_SERVICE, PARAM_SERVICE_TYPE, { names: PARAM_NAMES }),
        T_SVC_PARAMS, 'skill_node get_parameters')
        .then((res) => {
            const vals = (res && res.values) || [];
            const d = {};
            PARAM_NAMES.forEach((n, i) => { d[n] = paramValue(vals[i]); });
            // A reply that answers none of the four numeric names is not a
            // skill_node we understand — show "node default", not zeros.
            if (d.apex_m == null && d.separation_mm == null && d.dwell_s == null
                && d.n_throws == null) {
                throw new Error('no known parameters in the reply');
            }
            defaults = d;
            defaultsState = 'ok';
            paramFailures = 0;
        })
        .catch(() => {
            // Keep the last-known values on a failed REFRESH (still the best
            // guess, and labelled as such); only a first fetch reads "failed".
            if (defaultsState !== 'ok') defaultsState = 'failed';
            paramFailures += 1;
            if (gate.connected) scheduleParamRetry();
        })
        .finally(() => {
            paramsInFlight = false;
            sync();
        });
}

// ====================================================================
// Start / Stop
// ====================================================================

function setTransient(text, cls) {
    transient = text ? { text, cls: cls || '' } : null;
    sync();
}

function armTransientClear() {
    clearTimeout(transientTimer);
    transientTimer = setTimeout(() => { transient = null; sync(); }, STATUS_CLEAR_MS);
}

/** Why Start is unavailable right now, or null.  Order = what the operator
 *  must fix first. */
function startGateReason(p) {
    if (!gate.connected) return 'rosbridge disconnected';
    if (!gate.active) {
        return 'Start is available only in ACTIVE'
            + (gate.mainState && gate.mainState !== 'ACTIVE' ? ' (now ' + gate.mainState + ')' : '');
    }
    if (busy) return 'Juggle dispatched — waiting for the attempt…';
    const err = firstFieldError(p);
    if (err) return 'Fix ' + err;
    return null;
}

function onStartConfirm() {
    const p = patternOf(ui.pattern);
    // Re-check at CONFIRM time (holdToConfirm re-checks .disabled, but the
    // gate inputs may have moved since the last render).
    if (startGateReason(p)) return;
    const data = buildRequestData(p);
    busy = true;
    warn = null;
    crossCheck = {
        pattern: p.value, typed: typedOverrides(p), reload: effectiveReload(p), t: Date.now(), // wall-clock: command path
    };
    setTransient('Dispatching ' + data + '…', '');
    // No Event Log entry here: skill_node announces the attempt on
    // skills/attempt (minimapOnSkillAttempt), for this button and a terminal
    // `ros2 action send_goal` alike.  The ack below is the relay's DISPATCH
    // ack, not the attempt's outcome.
    ros.withTimeout(transport.callService('jugglebot/juggle_request', SETSTRING, { data }),
        T_SVC_REQUEST, 'juggle request')
        .then((res) => {
            if (res && res.success) {
                setTransient(res.message || 'Juggle dispatched.', 'ok');
            } else {
                crossCheck = null;   // nothing will start — nothing to cross-check
                setTransient((res && res.message) || 'Juggle rejected.', 'err');
            }
        })
        .catch((err) => {
            // A timeout may still land (the goal can be accepted after we
            // gave up waiting) — keep the cross-check armed for that case.
            setTransient('Juggle request: ' + (err && err.message ? err.message : err)
                + ' — check the Event Log before re-sending', 'err');
        })
        .finally(() => {
            clearTimeout(cooldownTimer);
            cooldownTimer = setTimeout(() => { busy = false; sync(); }, JUGGLE_COOLDOWN_MS);
            armTransientClear();
            sync();
        });
}

function onStopClick() {
    // Gated on the connection ONLY — never on Start's cooldown, the ACTIVE
    // gate or a field error: a stop must never be slow or blocked.
    if (!gate.connected) return;
    setTransient('Stopping…', '');
    emitEvent({
        type: EVENT_TYPES.COMMAND,
        label: 'Juggle Stop',
        detail: 'jugglebot/juggle_stop',
    });
    ros.withTimeout(transport.callService('jugglebot/juggle_stop', TRIGGER, {}),
        T_SVC_STOP, 'juggle stop')
        .then((res) => {
            if (res && res.success) setTransient(res.message || 'Stopped.', 'ok');
            else setTransient((res && res.message) || 'Nothing to stop.', '');
        })
        .catch((err) => {
            setTransient('Stop failed: ' + (err && err.message ? err.message : err), 'err');
        })
        .finally(armTransientClear);
}

// ====================================================================
// skills/attempt
// ====================================================================

/**
 * Feed one skills/attempt message (diagnostic_msgs/DiagnosticStatus) — the
 * minimap forwards every one after writing its Event Log line.  Shapes
 * (skill_node.py ATTEMPT_*):
 *   juggle_start   pattern n_throws reload apex_m(%.3f) separation_mm(%.1f)
 *   juggle_refused pattern; message = the refusal reason
 *   juggle_end     pattern outcome throws caught
 */
export function jugglePanelOnSkillAttempt(msg) {
    if (!msg) return;
    const f = {};
    for (const v of (msg.values || [])) f[v.key] = v.value;
    const now = clock.now();
    if (msg.name === 'juggle_start') {
        running = {
            pattern: f.pattern, n: f.n_throws, reload: f.reload === '1',
            apex: f.apex_m, sep: f.separation_mm, t0: now,
        };
        checkResolved(f, now);
    } else if (msg.name === 'juggle_refused') {
        // A refusal never ends a RUNNING attempt: a second goal while one
        // runs is refused at accept and the first one carries on.
        last = {
            kind: 'refused', pattern: f.pattern, t: now,
            reason: String(msg.message || '').replace(/^refused:\s*/i, ''),
        };
        if (crossCheck && crossCheck.pattern === f.pattern) crossCheck = null;
    } else if (msg.name === 'juggle_end') {
        running = null;
        last = {
            kind: 'end', pattern: f.pattern, t: now,
            outcome: f.outcome || '?', throws: f.throws, caught: f.caught,
        };
        // A param may have been changed while it flew; cheap to re-read.
        fetchDefaults();
    }
    sync();
}

/** Old-relay rule 2: compare the RESOLVED request with what we typed. */
function checkResolved(f, now) {
    const cc = crossCheck;
    if (!cc || now - cc.t > CROSSCHECK_WINDOW_MS || f.pattern !== cc.pattern) return;
    crossCheck = null;
    const misses = [];
    const t = cc.typed;
    if (t.apex_m != null && !(Math.abs(parseFloat(f.apex_m) - t.apex_m) <= 6e-4)) {
        misses.push('apex ' + f.apex_m + ' m (asked ' + wireNum(t.apex_m) + ')');
    }
    if (t.separation_mm != null && !(Math.abs(parseFloat(f.separation_mm) - t.separation_mm) <= 0.06)) {
        misses.push(f.separation_mm + ' mm separation (asked ' + wireNum(t.separation_mm) + ')');
    }
    if (t.num_cycles != null && parseInt(f.n_throws, 10) !== t.num_cycles) {
        misses.push(f.n_throws + ' throws (asked ' + t.num_cycles + ')');
    }
    if (cc.reload && f.reload !== '1') misses.push('without reload (asked reload)');
    if (misses.length) {
        warn = '⚠ Node flew ' + misses.join(', ') + ' — relay out of date? colcon build + relaunch';
    }
}

// ====================================================================
// Diagram
// ====================================================================

// viewBox geometry (front view, robot X right, Z up; NOT to scale — the
// numbers on the labels are the truth, the drawing only moves with them).
const VB_W = 180, VB_H = 104;
const BASE_Y = 85;          // platform reference line
const RIM_Y = 75.5;         // cup rim at rest = release / catch height
const CUP_D = 6;
const BALL_R = 4;
const DIP_PX = 4;           // dwell scoop depth
const CX = 104;             // P1/P2 midpoint (BB glyph lives left of it)
const HALF_SEP_PX = 36;     // at 250 mm
const APEX_PX_MAX = 56;
const BB_ARM = { x: 21, y: 62 };

const smooth = (u) => 0.5 - 0.5 * Math.cos(Math.PI * Math.min(1, Math.max(0, u)));
const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));

/**
 * Everything the diagram draws for the current selection.  Ball ids follow
 * schedule.ball_label: id 0 = "Ball 1" (in Jugglebot's hand at rest), id 1
 * = "Ball 2" (fed, or waiting at the second site).
 *
 * Site layout is the NODE's (not a guess): columns put ball A at the site
 * that is NOT `columns_feed_site` and ball B at it — default 'P1', so Ball 1
 * at P2 and Ball 2 at P1, the deliberate crossover that keeps B's descent
 * off A's column (skill_node._run_columns, R5 sitting 2).  Hop starts at
 * P1 (columns_sites()[0]); self-toss uses the single site (site_x_mm,
 * default x = -50 mm).
 */
function buildModel(p) {
    const apexKnown = effective('apex_m') != null;
    const dwellKnown = defaults && defaults.dwell_s != null;
    const sepKnown = effective('separation_mm') != null;
    const apex = apexKnown ? effective('apex_m') : NOMINAL.apex_m;
    const dwell = dwellKnown ? defaults.dwell_s : NOMINAL.dwell_s;
    const sep = sepKnown ? effective('separation_mm') : NOMINAL.separation_mm;
    const reload = effectiveReload(p);

    const hs = HALF_SEP_PX * clamp(sep / 250, 0.55, 1.2);
    const P1 = CX - hs, P2 = CX + hs;
    const apexPx = APEX_PX_MAX * clamp(apex, 0.35, 1.2) / 1.2;
    const m = {
        p, apex, dwell, sep, apexKnown, dwellKnown, sepKnown, reload,
        tf: flightS(apex), apexPx, apexY: RIM_Y - BALL_R - apexPx,
        sites: [], balls: [], feed: null, lob: null,
    };
    if (p.sites === 1) {
        const x = CX - 8;
        m.sites = [{ x, name: 'site' }];
        m.balls = [{ id: 0, x, phantom: false }];
        if (reload) m.feed = { x, ball: 0 };
    } else {
        m.sites = [{ x: P1, name: 'P1' }, { x: P2, name: 'P2' }];
        if (p.kind === 'one') {
            m.balls = [{ id: 0, x: P1, phantom: false }];   // hop starts at P1
            if (reload) m.feed = { x: P1, ball: 0 };
        } else {
            const feedName = defaults && defaults.columns_feed_site === 'P2' ? 'P2' : 'P1';
            const feedX = feedName === 'P1' ? P1 : P2;
            const aX = feedName === 'P1' ? P2 : P1;
            m.balls = [
                { id: 0, x: aX, phantom: p.phantom === 0 },
                { id: 1, x: feedX, phantom: p.phantom === 1 },
            ];
            if (p.value === 'columns_1ball') {
                // Its reload is the self-toss choreography for ball A ALONE
                // (_run_columns_1ball); nothing is fed at the phantom's site.
                if (reload) m.feed = { x: aX, ball: 0 };
            } else if (reload) {
                m.feed = { x: feedX, ball: 1 };
            } else {
                // Plain columns without Ball Butler: ball B is a human lob
                // onto the feed site (the tracker resolves the un-announced
                // landing, _maybe_resolve_columns_feed).
                m.lob = { x: feedX, ball: 1 };
            }
        }
    }
    return m;
}

function diagramKeyOf(p) {
    return [p.value, effectiveReload(p) ? 1 : 0, effective('apex_m'), effective('separation_mm'),
        defaults ? defaults.dwell_s : '', defaults ? defaults.columns_feed_site : ''].join('|');
}

function ballCls(b) {
    return 'jp-ball b' + (b.id + 1) + (b.phantom ? ' phantom' : '');
}

function rebuildDiagram(p) {
    const svg = dom.svg;
    model = buildModel(p);
    const m = model;
    const g = dom.diagStatic;
    while (g.firstChild) g.removeChild(g.firstChild);

    // Legend (top-left): which ball is which, phantoms ghosted.
    let lx = 5;
    for (const b of m.balls) {
        svgEl('circle', { cx: lx + 3, cy: 6, r: 2.8 }, ballCls(b) + ' legend', g);
        const txt = 'Ball ' + (b.id + 1) + (b.phantom ? ' phantom'
            : (m.feed && m.feed.ball === b.id) || (m.lob && m.lob.ball === b.id) ? ' fed' : '');
        const t = svgEl('text', { x: lx + 8, y: 8.6 }, 'jp-legend', g);
        t.textContent = txt;
        lx += 13 + txt.length * 4.0;
    }

    // Platform reference + site ticks + labels.
    svgEl('rect', { x: 30, y: BASE_Y, width: VB_W - 33, height: 2.4, rx: 1.2 }, 'jp-plat', g);
    for (const s of m.sites) {
        svgEl('line', { x1: s.x, y1: BASE_Y + 2.4, x2: s.x, y2: BASE_Y + 5.5 }, 'jp-tick', g);
        const t = svgEl('text', { x: s.x, y: 101, 'text-anchor': 'middle' }, 'jp-site-label', g);
        t.textContent = s.name;
    }
    if (m.sites.length === 2) {
        // Dimension line P1 <-> P2 with the separation centred in a gap.
        const [a, b] = m.sites;
        const mid = (a.x + b.x) / 2, y = 97.5, gap = 13;
        if (mid - gap - (a.x + 8) > 2) {
            svgEl('line', { x1: a.x + 8, y1: y, x2: mid - gap, y2: y }, 'jp-dim', g);
            svgEl('line', { x1: mid + gap, y1: y, x2: b.x - 8, y2: y }, 'jp-dim', g);
        }
        const t = svgEl('text', { x: mid, y: 100, 'text-anchor': 'middle' }, 'jp-dim-label', g);
        t.textContent = m.sepKnown ? Math.round(m.sep) + ' mm' : '? mm';
    }

    // Apex line, the word at its left end and the value at its right — split
    // so neither can sit on a ball at its apex (the columns sit at P1/P2 and
    // the hop peaks midway; the HALF_SEP clamp keeps P2 clear of the value).
    const ay = m.apexY;
    svgEl('line', { x1: 30, y1: ay, x2: VB_W - 3, y2: ay }, 'jp-apex-line', g);
    svgEl('text', { x: 31, y: ay - 2.6 }, 'jp-apex-word', g).textContent = 'apex';
    const at = svgEl('text', { x: VB_W - 3, y: ay - 2.6, 'text-anchor': 'end' },
        'jp-apex-label' + (fields.apex_m.value != null ? ' override' : ''), g);
    at.textContent = m.apexKnown ? fmt2(m.apex) + ' m' : '? m';

    // Ball paths: a column per site (columns / self-toss), the arc for hop.
    const top = RIM_Y - BALL_R;
    if (m.p.kind === 'one' && m.sites.length === 2) {
        const [a, b] = m.sites;
        svgEl('path', {
            d: 'M ' + a.x + ' ' + top + ' Q ' + (a.x + b.x) / 2 + ' ' + (2 * ay - top)
                + ' ' + b.x + ' ' + top,
        }, 'jp-path b1', g);
    } else {
        for (const ball of m.balls) {
            svgEl('line', { x1: ball.x, y1: top, x2: ball.x, y2: ay },
                'jp-path b' + (ball.id + 1) + (ball.phantom ? ' phantom' : ''), g);
        }
    }

    // Ball Butler glyph + feed arc, or the human lob.
    if (m.feed) {
        const bb = svgEl('g', {}, 'jp-bb', g);
        svgEl('rect', { x: 5, y: BASE_Y - 6, width: 15, height: 6, rx: 1.5 }, 'jp-bb-base', bb);
        svgEl('rect', { x: 9, y: BASE_Y - 15, width: 7, height: 9, rx: 1.5 }, 'jp-bb-body', bb);
        svgEl('line', { x1: 12.5, y1: BASE_Y - 13, x2: BB_ARM.x, y2: BB_ARM.y }, 'jp-bb-arm', bb);
        const bt = svgEl('text', { x: 12.5, y: 101, 'text-anchor': 'middle' }, 'jp-site-label', g);
        bt.textContent = 'BB';
        const ex = m.feed.x, ey = top - 1;
        const cy = Math.min(ay, BB_ARM.y) - 16;
        svgEl('path', {
            d: 'M ' + BB_ARM.x + ' ' + BB_ARM.y + ' Q ' + (BB_ARM.x + ex) / 2 + ' ' + cy
                + ' ' + ex + ' ' + ey,
        }, 'jp-feed', g).setAttribute('marker-end', 'url(#jp-arrow)');
        const ft = svgEl('text', { x: BB_ARM.x - 3, y: BB_ARM.y - 3, 'text-anchor': 'end' },
            'jp-feed-label', g);
        ft.textContent = 'feed';
    } else if (m.lob) {
        const ex = m.lob.x, ey = top - 1;
        svgEl('path', {
            d: 'M 4 ' + (ay + 8) + ' Q ' + (4 + ex) / 2 + ' ' + (ay - 10) + ' ' + ex + ' ' + ey,
        }, 'jp-feed lob', g).setAttribute('marker-end', 'url(#jp-arrow-lob)');
        const ft = svgEl('text', { x: 4, y: ay + 16 }, 'jp-feed-label lob', g);
        ft.textContent = 'hand lob';
    }

    // Dynamic elements: only as many balls as the pattern has.
    dom.balls.forEach((c, i) => {
        const b = m.balls[i];
        if (b) {
            c.setAttribute('class', ballCls(b));
            c.style.display = '';
        } else {
            c.style.display = 'none';
        }
    });

    dom.diagTitle.textContent = m.p.label + ' — ' + m.p.blurb
        + (m.apexKnown && m.dwellKnown ? '' : '  (node defaults unknown: the motion is illustrative)');
    svg.classList.toggle('jp-guess', !(m.apexKnown && m.dwellKnown));
    drawFrame(staticPhase());
}

/** A representative still: balls visibly airborne. */
function staticPhase() {
    return model ? model.tf * 0.3 : 0;
}

function placeHand(x, dip) {
    const rim = RIM_Y + dip;
    dom.hand.setAttribute('transform', 'translate(' + x.toFixed(2) + ' ' + rim.toFixed(2) + ')');
    dom.handStem.setAttribute('y2', (BASE_Y - rim).toFixed(2));
}

function placeBall(i, x, z, inHand, dip) {
    const c = dom.balls[i];
    const cy = inHand ? RIM_Y + dip - BALL_R + 1.2 : RIM_Y - BALL_R - z;
    c.setAttribute('cx', x.toFixed(2));
    c.setAttribute('cy', cy.toFixed(2));
}

/** Height (viewBox px) of a ball `u` seconds into a flight of `tf`. */
function flightZ(u, tf, apexPx) {
    const r = u / tf;
    return 4 * apexPx * r * (1 - r);
}

/**
 * Pose everything at pattern time `s` (seconds, real timing).  The timing is
 * schedule.py's: flight t_f = 2·sqrt(2·apex/g); one ball repeats every
 * t_f + dwell; columns throws alternate balls every beat = (t_f + dwell)/2
 * and the hand transits sites in transit = (t_f − dwell)/2.
 */
function drawFrame(s) {
    const m = model;
    if (!m) return;
    const tf = m.tf, d = m.dwell;
    const dipAt = (u) => DIP_PX * Math.sin(Math.PI * clamp(u, 0, 1));
    if (m.p.kind === 'one' && m.sites.length === 1) {
        const x = m.sites[0].x;
        const u = s % (tf + d);
        if (u < tf) { placeHand(x, 0); placeBall(0, x, flightZ(u, tf, m.apexPx), false, 0); }
        else { const dip = dipAt((u - tf) / d); placeHand(x, dip); placeBall(0, x, 0, true, dip); }
    } else if (m.p.kind === 'one') {
        // Hop: the cup coasts under the ball at the ball's lateral speed.
        const leg = tf + d;
        const u = s % (2 * leg);
        const k = u < leg ? 0 : 1;
        const v = u - k * leg;
        const from = m.sites[k].x, to = m.sites[1 - k].x;
        if (v < tf) {
            const x = from + (to - from) * (v / tf);
            placeHand(x, 0);
            placeBall(0, x, flightZ(v, tf, m.apexPx), false, 0);
        } else {
            const dip = dipAt((v - tf) / d);
            placeHand(to, dip);
            placeBall(0, to, 0, true, dip);
        }
    } else {
        // Columns: ball 0 thrown at 0, ball 1 at beat; the hand transits
        // [0, τ] to ball 1's site, dwells [τ, β], transits back [β, β+τ],
        // dwells [t_f, t_f+d] with ball 0.
        const T = tf + d, beta = T / 2, tau = (tf - d) / 2;
        if (!(tau > 0)) { placeHand(m.balls[0].x, 0); return; }
        const u = s % T;
        const x0 = m.balls[0].x, x1 = m.balls[1].x;
        let hx, dip = 0;
        if (u < tau) hx = x0 + (x1 - x0) * smooth(u / tau);
        else if (u < beta) { hx = x1; dip = dipAt((u - tau) / d); }
        else if (u < beta + tau) hx = x1 + (x0 - x1) * smooth((u - beta) / tau);
        else { hx = x0; dip = dipAt((u - beta - tau) / d); }
        placeHand(hx, dip);
        for (let i = 0; i < 2; i++) {
            const w = ((s - i * beta) % T + T) % T;
            const bx = m.balls[i].x;
            if (w < tf) placeBall(i, bx, flightZ(w, tf, m.apexPx), false, 0);
            else placeBall(i, bx, 0, true, dip);
        }
    }
}

function animShouldRun() {
    return !!(dom && model && gate.expanded && !reducedMotion && !replayHidden
        && !document.hidden && dom.root.isConnected);
}

function animLoop(now) {
    if (!animShouldRun()) { rafId = 0; return; }
    drawFrame(((now - animT0) / 1000) * speed);
    rafId = requestAnimationFrame(animLoop);
}

function ensureAnim() {
    if (rafId || !animShouldRun()) return;
    animT0 = performance.now() - (staticPhase() / speed) * 1000; // wall-clock: UI animation
    rafId = requestAnimationFrame(animLoop);
}

function onSpeedClick(ev) {
    ev.stopPropagation();
    const i = SPEEDS.indexOf(speed);
    const now = performance.now(); // wall-clock: UI animation
    const s = ((now - animT0) / 1000) * speed;
    speed = SPEEDS[(i + 1) % SPEEDS.length];
    animT0 = now - (s / speed) * 1000;   // keep the pose continuous
    try { localStorage.setItem(LS_SPEED_KEY, String(speed)); } catch (e) { /* blocked */ }
    sync();
}

// ====================================================================
// Build
// ====================================================================

function buildDiagram() {
    const svg = svgEl('svg', {
        viewBox: '0 0 ' + VB_W + ' ' + VB_H, preserveAspectRatio: 'xMidYMid meet',
        role: 'img',
    }, 'jp-diagram');
    const title = svgEl('title', {}, null, svg);
    const defs = svgEl('defs', {}, null, svg);
    for (const [id, cls] of [['jp-arrow', 'jp-arrowhead'], ['jp-arrow-lob', 'jp-arrowhead lob']]) {
        const mk = svgEl('marker', {
            id, viewBox: '0 0 6 6', refX: 5, refY: 3, markerWidth: 5, markerHeight: 5,
            orient: 'auto-start-reverse',
        }, null, defs);
        svgEl('path', { d: 'M 0 0 L 6 3 L 0 6 z' }, cls, mk);
    }
    const stat = svgEl('g', {}, 'jp-static', svg);
    const hand = svgEl('g', {}, 'jp-hand', svg);
    const handStem = svgEl('line', { x1: 0, y1: CUP_D, x2: 0, y2: BASE_Y - RIM_Y }, 'jp-hand-stem', hand);
    svgEl('path', { d: 'M -8.5 0 L -6.8 ' + (CUP_D - 1) + ' Q 0 ' + (CUP_D + 1.6) + ' 6.8 '
        + (CUP_D - 1) + ' L 8.5 0 Z' }, 'jp-hand-cup', hand);
    const balls = [0, 1].map(() => svgEl('circle', { cx: -20, cy: -20, r: BALL_R }, 'jp-ball', svg));
    const speedT = svgEl('text', { x: VB_W - 3, y: 8.6, 'text-anchor': 'end' }, 'jp-speed', svg);
    const speedTitle = svgEl('title', {}, null, speedT);
    speedTitle.textContent = 'Animation speed (the motion runs on the real schedule '
        + 'timing × this factor) — click to change';
    speedT.addEventListener('click', onSpeedClick);
    return { svg, title, stat, hand, handStem, balls, speedT };
}

function buildField(key) {
    const spec = FIELDS[key];
    const label = el('label', 'jp-flabel', spec.label);
    const input = el('input', 'jp-input');
    input.id = 'juggle-' + key.replace('_', '-');
    label.htmlFor = input.id;
    input.type = 'text';
    input.inputMode = spec.integer ? 'numeric' : 'decimal';
    input.autocomplete = 'off';
    input.spellcheck = false;
    input.addEventListener('input', () => {
        fields[key].raw = input.value;
        Object.assign(fields[key], parseField(key, input.value));
        sync();
    });
    input.addEventListener('keydown', (ev) => {
        if (ev.key === 'Escape') {
            input.value = '';
            input.dispatchEvent(new Event('input'));
        } else if (ev.key === 'Enter') {
            input.blur();   // never a submit — Start is hold-only
        }
    });
    const unit = el('span', 'jp-unit', spec.unit);
    return { label, input, unit };
}

/**
 * Build the panel into `container` (emptied first).  Call once.
 */
export function buildJugglePanel(container) {
    if (!container) return false;
    container.innerHTML = '';
    container.classList.add('jp-root');

    const head = el('div', 'jp-head');
    const title = el('span', 'jp-title', 'Juggle');
    const clear = el('button', 'jp-clear', '↺ defaults');
    clear.type = 'button';
    clear.title = 'Clear every typed value — blank fields fly skill_node\'s own parameters';
    clear.addEventListener('click', () => {
        for (const k of FIELD_KEYS) {
            fields[k] = { raw: '', value: null, error: null };
            dom.fields[k].input.value = '';
        }
        sync();
    });
    const badge = el('span', 'badge jp-badge', '—');
    badge.id = 'juggle-badge';
    head.append(title, clear, badge);

    const row = el('div', 'jp-row');
    const select = el('select', 'jp-select');
    select.id = 'juggle-pattern';
    for (const p of JUGGLE_PATTERNS) {
        const opt = el('option', null, p.label);
        opt.value = p.value;
        opt.title = p.blurb;
        select.appendChild(opt);
    }
    select.addEventListener('change', () => { ui.pattern = select.value; sync(); });
    const reloadLabel = el('label', 'jp-reload');
    const reload = el('input');
    reload.type = 'checkbox';
    reload.id = 'juggle-reload';
    reload.addEventListener('change', () => {
        // Only the operator's OWN choice is remembered; a pattern that forces
        // reload doesn't overwrite it, so switching back restores it.
        if (!patternOf(ui.pattern).reloadRequired) ui.userReload = reload.checked;
        sync();
    });
    reloadLabel.append(reload, document.createTextNode('Reload first'));
    row.append(select, reloadLabel);

    const main = el('div', 'jp-main');
    const diag = buildDiagram();
    const diagWrap = el('div', 'jp-diagram-wrap');
    diagWrap.appendChild(diag.svg);
    const fieldsBox = el('div', 'jp-fields');
    const fieldDom = {};
    for (const k of FIELD_KEYS) {
        const f = buildField(k);
        fieldDom[k] = f;
        fieldsBox.append(f.label, f.input, f.unit);
    }
    const dwellLabel = el('span', 'jp-flabel', 'Dwell');
    const dwellVal = el('span', 'jp-readonly', '—');
    dwellVal.id = 'juggle-dwell';
    const dwellUnit = el('span', 'jp-unit', 's');
    const dwellTip = 'Read-only: dwell is skill_node\'s dwell_s parameter, not a Juggle goal '
        + 'field — the admissible box is swept AT this dwell, so a different live dwell '
        + 'is refused.';
    dwellLabel.title = dwellTip;
    dwellVal.title = dwellTip;
    fieldsBox.append(dwellLabel, dwellVal, dwellUnit);
    main.append(diagWrap, fieldsBox);

    const timing = el('div', 'jp-timing');
    timing.title = 'Derived from apex + dwell exactly as schedule.py does: flight '
        + '2·√(2·apex/g); one ball repeats every flight + dwell; columns alternate balls '
        + 'every beat = (flight + dwell)/2 and the hand crosses sites in transit = '
        + '(flight − dwell)/2. "≈" = first release to last landing (excludes the opening '
        + 'REST and any feed wait).';

    const actions = el('div', 'jp-actions');
    const start = el('button', 'cmd-btn hold-fillable btn-juggle jp-start', 'Start');
    start.id = 'juggle-start';
    start.type = 'button';
    start.disabled = true;
    holdToConfirm(start, onStartConfirm, HOLD_MS);
    const stop = el('button', 'cmd-btn jp-stop', 'Stop');
    stop.id = 'juggle-stop';
    stop.type = 'button';
    stop.disabled = true;
    stop.addEventListener('click', onStopClick);
    const status = el('div', 'jp-status');
    status.id = 'juggle-status';
    actions.append(start, stop, status);

    const summary = el('div', 'jp-summary');
    const attempt = el('div', 'jp-attempt');
    attempt.id = 'juggle-attempt';
    const warnEl = el('div', 'jp-warn');
    warnEl.id = 'juggle-warn';

    container.append(head, row, main, timing, summary, actions, attempt, warnEl);

    dom = {
        root: container, title, clear, badge, select, reload, reloadLabel,
        svg: diag.svg, diagTitle: diag.title, diagStatic: diag.stat, hand: diag.hand,
        handStem: diag.handStem, balls: diag.balls, speedT: diag.speedT,
        fields: fieldDom, dwellVal, timing, start, stop, status, summary, attempt, warn: warnEl,
    };
    document.addEventListener('visibilitychange', ensureAnim);
    sync();
    return true;
}

// ====================================================================
// Render
// ====================================================================

/**
 * Called by the minimap on EVERY applySnapshot frame — cheap, and touches
 * the DOM only when something changed.
 * @param {{connected:boolean, active:boolean, expanded?:boolean, mainState?:string|null}} g
 */
export function renderJugglePanel(g) {
    if (g) lastGate = g;
    if (replayHidden) return;   // replay: panel is display:none, nothing to paint
    if (!dom || !g) return;
    const wasConnected = gate.connected, wasActive = gate.active, wasExpanded = gate.expanded;
    gate.connected = !!g.connected;
    gate.active = !!g.active;
    gate.expanded = g.expanded !== false;
    gate.mainState = g.mainState || null;

    if (!gate.connected && wasConnected) {
        // Nothing the node told us survives a disconnect as trusted: a
        // relaunch may come back with other params, and a RUNNING shown
        // across the gap would be a guess.  The last outcome is history, so
        // it stays.
        defaults = null;
        defaultsState = 'unknown';
        paramFailures = 0;
        clearTimeout(paramRetryTimer);
        running = null;
        crossCheck = null;
    }
    // (Re)read the node's defaults on connect, on entering ACTIVE (the
    // launch may have restarted skill_node behind a live rosbridge) and on
    // expand (the operator is about to look at them).
    if (gate.connected && (!wasConnected || (gate.active && !wasActive)
        || (gate.expanded && !wasExpanded))) {
        fetchDefaults();
    }
    sync();
}

function badgeView(p) {
    if (!gate.connected) return { text: 'Offline', cls: 'off', tip: 'rosbridge disconnected' };
    if (running) return { text: 'Running', cls: 'running', tip: 'skills/attempt juggle_start seen; no juggle_end yet' };
    if (busy) return { text: 'Sent', cls: 'busy', tip: 'dispatched — Start re-enables after a short cooldown' };
    if (!gate.active) return { text: 'Not active', cls: 'gated', tip: startGateReason(p) };
    if (firstFieldError(p)) return { text: 'Check field', cls: 'gated', tip: firstFieldError(p) };
    return { text: 'Ready', cls: 'ready', tip: 'hold Start to juggle' };
}

function timingText(m) {
    if (!m.apexKnown || !m.dwellKnown) {
        return { text: defaultsState === 'loading' ? 'reading skill_node defaults…'
            : 'timing needs skill_node\'s apex/dwell (node defaults unknown)', cls: 'muted' };
    }
    const tf = m.tf, d = m.dwell;
    const n = effective('num_cycles');
    const parts = ['flight ' + tf.toFixed(2) + ' s'];
    let total = null;
    if (m.p.kind === 'columns') {
        const beta = (tf + d) / 2, tau = (tf - d) / 2;
        if (!(tau > 0)) {
            return { text: 'dwell ' + d.toFixed(2) + ' s ≥ flight ' + tf.toFixed(2)
                + ' s leaves no transit — skill_node will refuse', cls: 'warn' };
        }
        parts.push('beat ' + beta.toFixed(2) + ' s', 'transit ' + tau.toFixed(2) + ' s');
        if (n != null) total = (n - 1) * beta + tf;
    } else {
        parts.push('cycle ' + (tf + d).toFixed(2) + ' s');
        if (m.sites.length === 2 && m.sepKnown) {
            parts.push('coast ' + Math.round(m.sep / tf) + ' mm/s');
        }
        if (n != null) total = (n - 1) * (tf + d) + tf;
    }
    if (total != null) parts.push(n + (n === 1 ? ' throw' : ' throws') + ' ≈ ' + fmtDur(total));
    return { text: parts.join(' · '), cls: '' };
}

function attemptView() {
    if (running) {
        const label = patternOf(running.pattern).value === running.pattern
            ? patternOf(running.pattern).label : (running.pattern || '?');
        const secs = Math.max(0, Math.floor((clock.now() - running.t0) / 1000));
        const text = 'Running · ' + label + ' · ' + running.n + ' throws · apex ' + running.apex
            + ' m' + (patternOf(running.pattern).sites === 2 ? ' · ' + running.sep + ' mm' : '')
            + (running.reload ? ' · reload' : '') + ' · ' + secs + ' s';
        return { text, cls: 'run', tip: 'resolved request from skills/attempt juggle_start' };
    }
    if (!last) return { text: 'No attempt yet', cls: 'muted none', tip: '' };
    const known = JUGGLE_PATTERNS.find((p) => p.value === last.pattern);
    const label = known ? known.label : (last.pattern || '?');
    const at = new Date(last.t).toLocaleTimeString([], { hour12: false });
    if (last.kind === 'refused') {
        return { text: 'Refused · ' + label + ' · ' + last.reason, cls: 'err',
            tip: at + ' — ' + last.reason };
    }
    const cls = last.outcome === 'COMPLETED' ? 'ok' : last.outcome === 'STOPPED' ? 'muted' : 'err';
    return {
        text: 'Last · ' + label + ' · ' + last.outcome + ' · ' + last.caught + '/' + last.throws
            + ' caught',
        cls, tip: at + ' — ' + last.throws + ' throws finalised, ' + last.caught + ' caught',
    };
}

/** Apply the current state to the DOM (write-if-changed throughout). */
function sync() {
    if (!dom) return;
    const p = patternOf(ui.pattern);
    const reload = effectiveReload(p);

    // Compact (minimap collapsed): pattern, reload, Start/Stop, status and
    // an overrides summary — typed values must never be HIDDEN while they
    // would still be sent.
    dom.root.classList.toggle('jp-compact', !gate.expanded);

    // Pattern + reload.
    if (dom.select.value !== p.value) dom.select.value = p.value;
    setTitle(dom.select, p.blurb);
    if (dom.reload.checked !== reload) dom.reload.checked = reload;
    setDisabled(dom.reload, !!p.reloadRequired);
    setTitle(dom.reloadLabel, p.reloadRequired
        ? 'Required for ' + p.label + ': Ball 2 is the only real ball and Ball Butler is its '
          + 'only feed (the relay refuses it without reload)'
        : 'Open the attempt with a Ball Butler feed throw');
    dom.reloadLabel.classList.toggle('forced', !!p.reloadRequired);

    // Fields.
    for (const k of FIELD_KEYS) {
        const f = dom.fields[k];
        const spec = FIELDS[k];
        const applies = fieldApplies(k, p);
        setDisabled(f.input, !applies);
        const dv = defaults ? defaults[spec.param] : null;
        let ph;
        if (!applies) ph = 'n/a';
        else if (dv != null) ph = spec.integer ? String(dv) : (k === 'apex_m' ? fmt2(dv) : String(Math.round(dv)));
        else if (defaultsState === 'loading') ph = '…';
        else ph = 'node';   // "node default", in the width a 5em field has
        if (f.input.placeholder !== ph) f.input.placeholder = ph;
        const st = fields[k];
        setClassSet(f.input, 'jp-input',
            (applies && st.error ? 'invalid' : '') + (applies && st.value != null ? ' override' : ''));
        setTitle(f.input, !applies
            ? 'Not used by ' + p.label + ' (one site)'
            : st.error ? st.error
                : spec.tip + (dv != null ? ' Node: ' + dv + (spec.unit ? ' ' + spec.unit : '') + '.' : ''));
        f.label.classList.toggle('na', !applies);
    }
    const dwell = defaults && defaults.dwell_s != null ? defaults.dwell_s.toFixed(2) : '—';
    setText(dom.dwellVal, dwell);
    const anyTyped = FIELD_KEYS.some((k) => fields[k].raw.trim() !== '');
    setHidden(dom.clear, !anyTyped);

    // Diagram (rebuilt only when what it draws changed).
    const key = diagramKeyOf(p);
    if (key !== diagramKey) {
        diagramKey = key;
        rebuildDiagram(p);
    }
    setText(dom.speedT, SPEED_LABEL[speed]);
    const t = timingText(model);
    setText(dom.timing, t.text);
    setClassSet(dom.timing, 'jp-timing', t.cls);

    // Overrides summary (compact mode only, CSS-gated).
    const typed = typedOverrides(p);
    const sumParts = [];
    if (typed.num_cycles != null) sumParts.push(typed.num_cycles + ' throws');
    if (typed.apex_m != null) sumParts.push('apex ' + wireNum(typed.apex_m) + ' m');
    if (typed.separation_mm != null) sumParts.push(wireNum(typed.separation_mm) + ' mm');
    setText(dom.summary, sumParts.length ? 'Overrides: ' + sumParts.join(' · ') : '');
    setHidden(dom.summary, sumParts.length === 0);

    // Badge.
    const b = badgeView(p);
    setText(dom.badge, b.text);
    setClassSet(dom.badge, 'badge jp-badge', b.cls);
    setTitle(dom.badge, b.tip || '');

    // Start / Stop.
    const reason = startGateReason(p);
    const data = buildRequestData(p);
    setDisabled(dom.start, !!reason);
    setTitle(dom.start, reason || ('hold to confirm — starts platform/hand motion\nsends: ' + data));
    // Stop is gated on connection alone — it must stay reachable even while
    // Start's own cooldown is latched (that cooldown is about NOT double-
    // dispatching a start, never about blocking a stop).
    setDisabled(dom.stop, !gate.connected);
    setTitle(dom.stop, gate.connected ? 'ends the running attempt immediately' : 'rosbridge disconnected');

    // Status line: the transient ack first, else why Start is unavailable,
    // else exactly what Start will send.
    let st;
    if (transient) st = { text: transient.text, cls: transient.cls };
    else if (reason) st = { text: reason, cls: 'muted' };
    else st = { text: '→ ' + data, cls: 'preview' };
    setText(dom.status, st.text);
    setClassSet(dom.status, 'jp-status', st.cls);
    setTitle(dom.status, st.text);

    const a = attemptView();
    setText(dom.attempt, a.text);
    setClassSet(dom.attempt, 'jp-attempt', a.cls);
    setTitle(dom.attempt, a.tip);

    setText(dom.warn, warn || '');
    setHidden(dom.warn, !warn);
    setTitle(dom.warn, warn ? warn + '\nThe relay before 2026-10-04 drops every field '
        + 'except reload, so skill_node flies its own parameters.' : '');

    if (gate.expanded) ensureAnim();
}

// ====================================================================
// Harness seam
// ====================================================================

/**
 * Swap the service transport (test_juggle_panel.html only — the live GUI
 * never calls this).  `t.callService(name, type, request) -> Promise`.
 */
export function setJugglePanelTransport(t) {
    if (t && typeof t.callService === 'function') transport = t;
}

/**
 * Replay mode hides the Juggle panel (replay/ui/hide.js). While hidden the rAF animation is cancelled and
 * renderJugglePanel is a no-op; un-hiding re-renders once from the last snapshot and resumes the loop.
 */
export function setJugglePanelReplayHidden(on) {
    on = !!on;
    if (on === replayHidden) return;
    replayHidden = on;
    if (on) {
        if (rafId) { cancelAnimationFrame(rafId); rafId = 0; }
    } else {
        if (lastGate) renderJugglePanel(lastGate);
        ensureAnim();
    }
}
