/**
 * session.js — the singleton live session buffer (replay design § 8, owner decision 11).
 *
 * ros-bridge.js's subscription wrapper records every delivered message into it;
 * the replay mode module snapshots it for "replay last session". Memory only:
 * a page reload loses it. Dependency-light (sources.js only) for node harnesses.
 */
import { createSessionBuffer, SESSION_BUFFER_SEC } from './sources.js';

let buffer = null;

/** @returns {object} the lazily created live SessionBufferSource. */
export function getSessionBuffer() {
    if (!buffer) buffer = createSessionBuffer({ horizonSec: SESSION_BUFFER_SEC });
    return buffer;
}

/** Tests: drop the singleton. */
export function _resetSessionBuffer() { buffer = null; }
