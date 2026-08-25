/**
 * odrive-errors.js — ODrive error bitmask → human-readable names (GUI mirror).
 *
 * HAND-MIRRORED from `ros_ws/src/jugglebot/jugglebot/can/odrive.py`
 * (`ERROR_CODES` + `error_names`).  There is no JS codegen for this table, so
 * it is a copy — and copies drift silently.  It is therefore PINNED by
 * `tests/ros/test_gui_geometry.py::TestODriveErrorTablePins`, which regex-parses
 * this file and asserts equality with the Python table.  Add a code there and
 * the pin fails here; that failure IS the deploy checklist.
 *
 * WHY THE GUI NEEDS ITS OWN COPY: /robot_state carries the raw 32-bit
 * `active_errors` / `disarm_reason` per axis and nothing else.  The bridge
 * decodes names onto the node log and (when fatal) into `error[]`, but the
 * event log wants a per-axis row the moment a mask CHANGES, fatal or not —
 * which means decoding client-side.
 *
 * BOTH MASKS MATTER.  `active_errors` self-heals; `disarm_reason` is sticky
 * until CLEAR_ERRORS.  The 2026-08-10 leg-0 spinout had `active_errors === 0`
 * and the whole story in `disarm_reason` — never render one without the other.
 */

/** Bit → name.  Mirror of jugglebot.can.odrive.ERROR_CODES (keep in bit order). */
export const ODRIVE_ERROR_CODES = {
    0x1: 'INITIALIZING',
    0x2: 'SYSTEM_LEVEL',
    0x4: 'TIMING_ERROR',
    0x8: 'MISSING_ESTIMATE',
    0x10: 'BAD_CONFIG',
    0x20: 'DRV_FAULT',
    0x40: 'MISSING_INPUT',
    0x100: 'DC_BUS_OVER_VOLTAGE',
    0x200: 'DC_BUS_UNDER_VOLTAGE',
    0x400: 'DC_BUS_OVER_CURRENT',
    0x800: 'DC_BUS_OVER_REGEN_CURRENT',
    0x1000: 'CURRENT_LIMIT_VIOLATION',
    0x2000: 'MOTOR_OVER_TEMP',
    0x4000: 'INVERTER_OVER_TEMP',
    0x8000: 'VELOCITY_LIMIT_VIOLATION',
    0x10000: 'POSITION_LIMIT_VIOLATION',
    0x1000000: 'WATCHDOG_TIMER_EXPIRED',
    0x2000000: 'ESTOP_REQUESTED',
    0x4000000: 'SPINOUT_DETECTED',
    0x8000000: 'BRAKE_RESISTOR_DISARMED',
    0x10000000: 'THERMISTOR_DISCONNECTED',
    0x40000000: 'CALIBRATION_ERROR',
};

/**
 * Decode a 32-bit ODrive error mask to names.
 *
 * Same contract as Python's `error_names`: known bits become names in
 * ascending bit order; any leftover bits are surfaced as ONE aggregate
 * `UNKNOWN(0x…)` entry rather than being silently dropped, so a firmware error
 * code this build has never heard of still reaches the operator.
 *
 * @param {number} mask 32-bit bitfield (`active_errors` or `disarm_reason`).
 * @returns {string[]} names, empty when the mask is 0.
 */
export function errorNames(mask) {
    // >>> 0 coerces to unsigned: bit 31 arrives from rosbridge as a positive
    // number, but any intermediate signed arithmetic would flip it negative.
    const m = (mask >>> 0);
    if (!m) return [];
    const names = [];
    let knownMask = 0;
    // Object keys that look like array indices iterate in ascending numeric
    // order, matching the Python dict's insertion (== bit) order.
    for (const key of Object.keys(ODRIVE_ERROR_CODES)) {
        const bit = Number(key);
        if (m & bit) names.push(ODRIVE_ERROR_CODES[key]);
        knownMask |= bit;
    }
    const unknown = (m & ~knownMask) >>> 0;
    if (unknown) names.push(`UNKNOWN(0x${unknown.toString(16).toUpperCase()})`);
    return names;
}

/**
 * Render one axis's error pair as one line of decoded names plus their hex mask:
 * `active=SPINOUT_DETECTED (0x4000000); disarm=—`.
 *
 * @param {number} active `active_errors`.
 * @param {number} disarm `disarm_reason`.
 * @returns {string} single-line summary; em-dash for an empty mask.
 */
export function formatAxisErrors(active, disarm) {
    const a = (active >>> 0);
    const d = (disarm >>> 0);
    const part = (mask, names) =>
        (mask ? `${names.join(',')} (0x${mask.toString(16).toUpperCase()})` : '—');
    return `active=${part(a, errorNames(a))}; disarm=${part(d, errorNames(d))}`;
}
