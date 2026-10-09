/**
 * BB calibration status — the pure view logic behind the panel's
 * "Calibrated" indicator and its last-attempt note (no DOM, no imports, so
 * tests/ros/test_gui_bb_calibration_status.py drives it under node).
 *
 * Two latched topics from mocap_node (keep-last-good, 2026-10-10):
 *   bb/calibration_result  — the calibration IN FORCE. A failed sweep never
 *                            replaces a success here; it carries success=false
 *                            only while that mocap_node has never succeeded.
 *   bb/calibration_attempt — the most recent sweep's outcome, success or
 *                            failure, as one short line.
 * So the indicator follows the first and the note follows the second: a failed
 * recalibration leaves "Calibrated · <time>" up and shows its reason beside it.
 */

const ACCEPTED_RE = /accepted (\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2})Z/;

/** The acceptance time mocap_node stamps on a success message
 *  ("Calibration successful (accepted 2026-10-10T12:34:56Z) · ..."), or null. */
export function parseAcceptedAt(message) {
    const m = ACCEPTED_RE.exec(String(message || ''));
    if (!m) return null;
    const t = new Date(m[1] + 'Z');
    return Number.isNaN(t.getTime()) ? null : t;
}

function hhmm(t) {
    const p = (n) => String(n).padStart(2, '0');
    return `${p(t.getHours())}:${p(t.getMinutes())}`;
}

/** Indicator state for a bb/calibration_result message (or null = unknown). */
export function calibrationIndicator(result) {
    if (!result || result.success !== true) {
        return { calibrated: false, text: 'Not Calibrated' };
    }
    const t = parseAcceptedAt(result.message);
    return { calibrated: true, text: t ? `Calibrated · ${hhmm(t)}` : 'Calibrated' };
}

/** The note under the indicator for a bb/calibration_attempt message: the
 *  failed attempt's short reason, or '' (no attempt yet, or it succeeded). */
export function attemptNote(attempt) {
    if (!attempt || attempt.success !== false) return '';
    const reason = String(attempt.message || '').trim();
    return reason ? `Last attempt failed: ${reason}` : 'Last attempt failed';
}
