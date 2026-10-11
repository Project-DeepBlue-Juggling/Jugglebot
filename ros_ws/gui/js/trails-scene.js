/**
 * trails-scene.js — browser-only wiring of the trail layer, the trail feed and
 * the ball-sphere pool onto the viewer scene (Phase 5).
 *
 * One singleton: initTrails() builds the layer + feed, registers the 'Trails'
 * and 'Balls' scene groups and ONE onFrame hook (feed.render, then the ball
 * spheres from feed.balls()). Live handlers in main.js and the replay columns
 * feeder both write into getTrailFeed(); this module only renders.
 *
 * The far-tier ball history (js/ball-history-layer.js, fed by replay/ball-history.js) is a third scene group,
 * 'Ball history', drawn only while a replay is active and the effective tail exceeds the near tier.
 *
 * No allocation per frame: the sphere pool, the materials (one per colour,
 * cached) and the resolution vector are built once.
 */

import * as THREE from 'three';
import { scene, renderer, sceneGroups, onFrame } from './viewer.js';
import { createTrailLayer } from './trails.js';
import { createTrailFeed } from './trail-feed.js';
import { createBallHistoryLayer } from './ball-history-layer.js';
import { getNearTailMs, getEffectiveTailMs } from './trail-settings.js';
import { ballColour } from './marker-palette.js';

const MAX_BALLS = 8;
/** Juggling ball radius (m). hardware_config.yaml juggling_ball_radius_mm = 37.0, but that
 *  constant is not emitted into the generated geometry-config.js, so it is mirrored here. */
const BALL_RADIUS_M = 0.037;
const MM_TO_M = 0.001;

/**
 * Far-tier vertex budget (2 per segment, 28 B per vertex -> 28 MB of attributes). Sizing, measured on the busy bag
 * 2026-10-02_12-43-28 (18 min): 31,835 decimated /balls rows -> 31,740 segments -> 63,480 vertices, with balls in 50 of 108
 * slots. The worst case this has to hold is a 20 min recording with THREE balls in flight the whole time at the 100 Hz/id
 * decimation floor: 1200 s x 100 x 3 = 360k segments = 720k vertices; 1,000,000 leaves ~39 % headroom.
 */
export const HISTORY_MAX_VERTICES = 1000000;

let layer = null;
let history = null;
let feed = null;
let ballGroup = null;
const spheres = [];
const materialByColour = new Map();
const resolution = new THREE.Vector2();

function materialFor(hex) {
    let m = materialByColour.get(hex);
    if (!m) {
        m = new THREE.MeshStandardMaterial({ color: hex, emissive: hex, emissiveIntensity: 0.3 });
        materialByColour.set(hex, m);
    }
    return m;
}

function renderFrame() {
    feed.render(getNearTailMs());
    // Far tier: replay only (the feed's render time is the replay playhead; null while live -> nothing drawn).
    const rt = feed.renderTimeSec();
    history.render(rt === null ? 0 : rt, rt === null ? 0 : getEffectiveTailMs());
    const b = feed.balls();
    const n = b.n;
    for (let i = 0; i < MAX_BALLS; i++) {
        const s = spheres[i];
        if (i >= n) { s.visible = false; continue; }
        const mat = materialFor(ballColour(b.id[i]));
        if (s.material !== mat) s.material = mat;
        // robot (x, y, z) mm -> three (x, z, -y) m, as robotToThreeScaled
        s.position.set(b.x[i] * MM_TO_M, b.z[i] * MM_TO_M, -b.y[i] * MM_TO_M);
        s.visible = true;
    }
}

/** Build the trail layer, feed and ball pool on the viewer scene. Idempotent. */
export function initTrails() {
    if (feed) return;
    layer = createTrailLayer(scene, {
        capacity: 640,
        maxTracks: 64,
        getResolution: (v) => {
            renderer.getDrawingBufferSize(resolution);
            v.copy(resolution);
        },
    });
    feed = createTrailFeed(layer);
    sceneGroups['Trails'] = layer.group;
    history = createBallHistoryLayer(scene, { maxVertices: HISTORY_MAX_VERTICES });
    sceneGroups['Ball history'] = history.group;

    ballGroup = new THREE.Group();
    ballGroup.name = 'balls';
    const geometry = new THREE.SphereGeometry(BALL_RADIUS_M, 16, 12);
    for (let i = 0; i < MAX_BALLS; i++) {
        const mesh = new THREE.Mesh(geometry, materialFor(ballColour(i)));
        mesh.visible = false;
        ballGroup.add(mesh);
        spheres.push(mesh);
    }
    scene.add(ballGroup);
    sceneGroups['Balls'] = ballGroup;

    onFrame(renderFrame);
}

/** @returns the trail feed, or null before initTrails(). */
export function getTrailFeed() { return feed; }
/** @returns the far-tier ball-history layer, or null before initTrails(). */
export function getBallHistoryLayer() { return history; }
/** @returns the trail layer (diagnostic: stats()), or null before initTrails(). */
export function getTrailLayer() { return layer; }
