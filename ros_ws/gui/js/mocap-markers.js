/**
 * mocap-markers.js — Render mocap markers as coloured spheres in the 3D viewer.
 *
 * Marker colours by label prefix:
 *   Platform*  -> blue    (#3b82f6)
 *   Base*      -> red     (#ef4444)
 *   Ball Butler* -> yellow (#eab308)
 *   Ball*      -> green   (#22c55e)   (future — ball prediction process)
 *   unlabelled -> light grey (#d1d5db)
 *
 * Uses a sphere pool to avoid GC churn: extra spheres are hidden, new ones
 * created on demand.
 */

import * as THREE from 'three';
import { scene, sceneGroups, robotToThreeScaled } from './viewer.js';

/** Three.js group containing all marker spheres */
let markerGroup;

/** Pool of reusable sphere meshes */
const spherePool = [];

/** Shared geometry for all markers (small sphere) */
const MARKER_RADIUS = 0.008; // 8mm in Three.js units (metres)
const sharedGeometry = new THREE.SphereGeometry(MARKER_RADIUS, 12, 8);

/** Colour lookup by label prefix */
const COLOUR_MAP = [
    { prefix: 'Platform', color: 0x3b82f6 },   // blue
    { prefix: 'Base',     color: 0xef4444 },    // red
    { prefix: 'Ball Butler', color: 0xeab308 }, // yellow
    { prefix: 'Ball',     color: 0x22c55e },    // green
];
const DEFAULT_COLOUR = 0xd1d5db; // light grey for unlabelled

/** Material cache keyed by hex colour to avoid creating duplicate materials */
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

/**
 * Determine the colour for a marker based on its label.
 * @param {string} label - QTM marker label (empty for unlabelled)
 * @returns {number} Three.js hex colour
 */
function labelToColour(label) {
    if (!label) return DEFAULT_COLOUR;
    for (const entry of COLOUR_MAP) {
        if (label.startsWith(entry.prefix)) return entry.color;
    }
    return DEFAULT_COLOUR;
}

/**
 * Initialise the mocap markers group and register it as a scene toggle.
 */
export function initMocapMarkers() {
    markerGroup = new THREE.Group();
    markerGroup.name = 'mocap-markers';
    scene.add(markerGroup);
    sceneGroups['Mocap Markers'] = markerGroup;
}

/**
 * Update displayed markers from a mocap_data message.
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

    // Update active markers
    for (let i = 0; i < count; i++) {
        const m = markers[i];
        const sphere = spherePool[i];
        const p = robotToThreeScaled(m.position.x, m.position.y, m.position.z);
        sphere.position.set(p.x, p.y, p.z);

        const color = labelToColour(m.label || '');
        sphere.material = getMaterial(color);
        sphere.visible = true;
    }

    // Hide unused spheres
    for (let i = count; i < spherePool.length; i++) {
        spherePool[i].visible = false;
    }
}
