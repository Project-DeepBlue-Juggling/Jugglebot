/**
 * cad-loader.js — Loads CAD model GLB files via GLTFLoader.
 *
 * Loads separate parts (base, platform, leg_lower, leg_upper, hand) from
 * ros_ws/gui/models/. Each part is optional — missing files are silently
 * skipped, allowing graceful fallback to procedural geometry.
 *
 * Leg meshes are cloned 6x (shared geometry, independent transforms).
 */

import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';

const MODEL_BASE_PATH = 'models/';

const PART_FILES = {
    base: 'base.glb',
    platform: 'platform.glb',
    legLower: 'leg_lower.glb',
    legUpper: 'leg_upper.glb',
    hand: 'hand.glb',
};

/**
 * Load a single GLB file. Returns the root scene object on success, null on failure.
 * @param {GLTFLoader} loader
 * @param {string} path
 * @returns {Promise<import('three').Group|null>}
 */
function loadPart(loader, path) {
    return new Promise((resolve) => {
        loader.load(
            path,
            (gltf) => resolve(gltf.scene),
            undefined,
            (err) => {
                console.warn(`[cad-loader] Could not load ${path}:`, err.message || err);
                resolve(null);
            }
        );
    });
}

/**
 * @typedef {Object} CADModels
 * @property {import('three').Group|null} base - Static base frame
 * @property {import('three').Group|null} platform - Moving platform
 * @property {import('three').Group[]} legLowers - 6 lower leg halves (cloned)
 * @property {import('three').Group[]} legUppers - 6 upper leg halves (cloned)
 * @property {import('three').Group|null} hand - Hand/cup piece
 * @property {boolean} hasAnyModel - True if at least one model loaded
 */

/**
 * Load all CAD models. Missing files are silently skipped.
 * Leg parts are cloned 6 times each.
 *
 * @returns {Promise<CADModels>}
 */
export async function loadAllModels() {
    const loader = new GLTFLoader();

    const [base, platform, legLower, legUpper, hand] = await Promise.all([
        loadPart(loader, MODEL_BASE_PATH + PART_FILES.base),
        loadPart(loader, MODEL_BASE_PATH + PART_FILES.platform),
        loadPart(loader, MODEL_BASE_PATH + PART_FILES.legLower),
        loadPart(loader, MODEL_BASE_PATH + PART_FILES.legUpper),
        loadPart(loader, MODEL_BASE_PATH + PART_FILES.hand),
    ]);

    // Clone leg meshes 6x (first instance is the original, 5 clones)
    const legLowers = [];
    const legUppers = [];
    for (let i = 0; i < 6; i++) {
        legLowers.push(legLower ? (i === 0 ? legLower : legLower.clone()) : null);
        legUppers.push(legUpper ? (i === 0 ? legUpper : legUpper.clone()) : null);
    }

    const hasAnyModel = !!(base || platform || legLower || legUpper || hand);

    if (hasAnyModel) {
        const loaded = [
            base && 'base',
            platform && 'platform',
            legLower && 'leg_lower',
            legUpper && 'leg_upper',
            hand && 'hand',
        ].filter(Boolean);
        console.log(`[cad-loader] Loaded CAD models: ${loaded.join(', ')}`);
    } else {
        console.log('[cad-loader] No CAD models found in models/ — using procedural geometry');
    }

    return { base, platform, legLowers, legUppers, hand, hasAnyModel };
}
