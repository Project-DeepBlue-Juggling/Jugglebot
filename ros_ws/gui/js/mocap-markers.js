/**
 * mocap-markers.js — Render mocap markers and rigid body axes in the 3D viewer.
 *
 * Marker colours by label prefix:
 *   Platform*    -> blue    (#3b82f6)
 *   Base*        -> red     (#ef4444)
 *   Ball Butler* -> yellow  (#eab308)
 *   Ball*        -> green   (#22c55e)   (future — ball prediction process)
 *   unlabelled   -> light grey (#d1d5db)
 *
 * Rigid body axes: small coordinate frames rendered from rigid_body_poses.
 *
 * Uses object pools to avoid GC churn.
 */

import * as THREE from 'three';
import { scene, sceneGroups, robotToThreeScaled } from './viewer.js';
import { INITIAL_HEIGHT_MM } from './geometry-config.js';

/** Three.js group containing all mocap objects (markers + axes) */
let markerGroup;

// ---- Marker spheres ----

const spherePool = [];
const MARKER_RADIUS = 0.008; // 8mm in Three.js units (metres)
const sharedGeometry = new THREE.SphereGeometry(MARKER_RADIUS, 12, 8);

/** Colour lookup by label prefix (ordered: longest prefix first for correct matching) */
const COLOUR_MAP = [
    { prefix: 'Ball Butler', color: 0xeab308, group: 'Ball Butler' }, // yellow
    { prefix: 'Platform',    color: 0x3b82f6, group: 'Platform' },    // blue
    { prefix: 'Base',        color: 0xef4444, group: 'Base' },        // red
    { prefix: 'Ball',        color: 0x22c55e, group: 'Ball' },        // green
];
const DEFAULT_COLOUR = 0xd1d5db; // light grey for unlabelled

/** CSS colour strings matching COLOUR_MAP for the info box dots */
const CSS_COLOURS = {
    'Platform':    '#3b82f6',
    'Base':        '#ef4444',
    'Ball Butler': '#eab308',
    'Ball':        '#22c55e',
    'Unlabelled':  '#d1d5db',
};

/** Material cache keyed by hex colour */
const materialCache = new Map();

function getMaterial(colorHex) {
    if (!materialCache.has(colorHex)) {
        materialCache.set(colorHex, new THREE.MeshStandardMaterial({
            color: colorHex,
            emissive: colorHex,
            emissiveIntensity: 0.3,
        }));
    }
    return materialCache.get(colorHex);
}

function labelToGroup(label) {
    if (!label) return 'Unlabelled';
    for (const entry of COLOUR_MAP) {
        if (label.startsWith(entry.prefix)) return entry.group;
    }
    return 'Unlabelled';
}

function labelToColour(label) {
    if (!label) return DEFAULT_COLOUR;
    for (const entry of COLOUR_MAP) {
        if (label.startsWith(entry.prefix)) return entry.color;
    }
    return DEFAULT_COLOUR;
}

// ---- Rigid body axes ----

const AXES_SIZE = 0.06; // 60mm axes
const axesPool = []; // pool of { axes: THREE.AxesHelper, group: THREE.Group }

// ---- Info box ----

let infoEl = null;

function updateInfoBox(counts) {
    if (!infoEl) {
        infoEl = document.getElementById('mocap-info');
    }
    if (!infoEl) return;

    const total = Object.values(counts).reduce((s, n) => s + n, 0);
    if (total === 0) {
        infoEl.style.display = 'none';
        return;
    }

    // Only show groups that have markers
    let html = '';
    for (const [group, cssColor] of Object.entries(CSS_COLOURS)) {
        const n = counts[group] || 0;
        if (n === 0) continue;
        html += `<div class="mocap-info-row">` +
            `<span><span class="mocap-info-dot" style="background:${cssColor}"></span>${group}</span>` +
            `<span class="mocap-info-count">${n}</span>` +
            `</div>`;
    }

    infoEl.innerHTML = html;
    infoEl.style.display = markerGroup && markerGroup.visible ? '' : 'none';
}

// ---- Public API ----

export function initMocapMarkers() {
    markerGroup = new THREE.Group();
    markerGroup.name = 'mocap-markers';
    scene.add(markerGroup);
    sceneGroups['Mocap Markers'] = markerGroup;
}

/**
 * Update marker spheres from a mocap_data message.
 * @param {Array<{position: {x,y,z}, label: string}>} markers
 */
export function updateMocapMarkers(markers) {
    if (!markerGroup) return;

    const count = markers.length;

    // Grow pool if needed
    while (spherePool.length < count) {
        const mesh = new THREE.Mesh(sharedGeometry, getMaterial(DEFAULT_COLOUR));
        mesh.visible = false;
        markerGroup.add(mesh);
        spherePool.push(mesh);
    }

    // Count markers per group for info box
    const counts = {};

    // Update active markers
    for (let i = 0; i < count; i++) {
        const m = markers[i];
        const sphere = spherePool[i];
        const p = robotToThreeScaled(m.position.x, m.position.y, m.position.z);
        sphere.position.set(p.x, p.y, p.z);

        const label = m.label || '';
        sphere.material = getMaterial(labelToColour(label));
        sphere.visible = true;

        const group = labelToGroup(label);
        counts[group] = (counts[group] || 0) + 1;
    }

    // Hide unused spheres
    for (let i = count; i < spherePool.length; i++) {
        spherePool[i].visible = false;
    }

    updateInfoBox(counts);
}

/**
 * Update rigid body coordinate axes from a rigid_body_poses message.
 * @param {Array<{name: string, pose: {header: {frame_id: string}, pose: {position: {x,y,z}, orientation: {x,y,z,w}}}}>} bodies
 */
export function updateRigidBodyAxes(bodies) {
    if (!markerGroup) return;

    const count = bodies.length;

    // Grow axes pool if needed
    while (axesPool.length < count) {
        const axes = new THREE.AxesHelper(AXES_SIZE);
        axes.visible = false;
        markerGroup.add(axes);
        axesPool.push(axes);
    }

    for (let i = 0; i < count; i++) {
        const body = bodies[i];
        const axes = axesPool[i];

        const poseStamped = body.pose;
        const pose = poseStamped.pose || poseStamped;
        const p = pose.position;
        const q = pose.orientation;
        if (!p || !q) { axes.visible = false; continue; }

        // Apply frame offset (platform_start bodies need Z offset)
        const frameId = (poseStamped.header && poseStamped.header.frame_id) || '';
        const zOffset = frameId === 'platform_start' ? INITIAL_HEIGHT_MM : 0;

        const pos = robotToThreeScaled(p.x, p.y, p.z + zOffset);
        axes.position.set(pos.x, pos.y, pos.z);

        // Convert robot-frame quaternion to Three.js frame quaternion
        // Robot: (qx, qy, qz, qw) rotating in Z-up frame
        // Three.js: Y-up, mapping robot(x,y,z) -> three(x,z,-y)
        // The rotation quaternion transforms similarly
        axes.quaternion.set(q.x, q.z, -q.y, q.w);

        axes.visible = true;
    }

    // Hide unused axes
    for (let i = count; i < axesPool.length; i++) {
        axesPool[i].visible = false;
    }
}
