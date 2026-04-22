/**
 * viewer.js — Three.js scene, camera, OrbitControls, render loop.
 *
 * Sets up the 3D viewport with dark background, coordinate grid, and orbit controls.
 * All geometry in mm. Three.js Y-up ↔ Robot Z-up (swap Y/Z).
 */

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { INITIAL_HEIGHT_MM } from './geometry-config.js';

/** @type {THREE.Scene} */
export let scene;

/** @type {THREE.PerspectiveCamera} */
export let camera;

/** @type {THREE.WebGLRenderer} */
export let renderer;

/** @type {OrbitControls} */
export let controls;

/** Named groups for visibility toggles */
export const sceneGroups = {};

/** Meshes that should be hit-tested by the click-to-pick raycaster.  Each
 *  mesh carries `userData.chartIdx` identifying the chart-panel cell it
 *  maps to.  Models register themselves via `registerPickables`. */
const pickableMeshes = [];

/** Listeners notified with a chartIdx when the user clicks a pickable. */
const pickListeners = new Set();

/** Register one or more pickable meshes.  Safe to call multiple times. */
export function registerPickables(meshes) {
    for (const m of meshes) {
        if (m && !pickableMeshes.includes(m)) pickableMeshes.push(m);
    }
}

/**
 * Subscribe to pick events.  The callback fires with the chartIdx of the
 * mesh the user clicked.  Returns an unsubscribe fn.
 */
export function onMeshPick(cb) {
    pickListeners.add(cb);
    return () => pickListeners.delete(cb);
}

/**
 * Convert robot coordinates (Z-up, right-handed) to Three.js (Y-up, right-handed).
 *
 * Robot frame:  X = right,  Y = forward (away from viewer),  Z = up
 * Three.js:     X = right,  Y = up,                          Z = towards viewer
 *
 * Mapping: robot(x, y, z) → Three(x, z, -y)
 * The negation of Y preserves right-handedness.
 */
export function robotToThree(x, y, z) {
    return new THREE.Vector3(x, z, -y);
}

/** Scale factor: mm → Three.js units (metres for nice camera distances). */
const SCALE = 0.001;

export function robotToThreeScaled(x, y, z) {
    return new THREE.Vector3(x * SCALE, z * SCALE, -y * SCALE);
}

/**
 * Snapshot the camera state — used by the Save Preset feature.
 * Returns plain-object coords so it round-trips cleanly through JSON.
 */
export function getCameraState() {
    if (!camera || !controls) return null;
    return {
        position: { x: camera.position.x, y: camera.position.y, z: camera.position.z },
        target:   { x: controls.target.x,  y: controls.target.y,  z: controls.target.z },
    };
}

/**
 * Instantly restore a previously-captured camera state.  No tweening —
 * would add complexity and isn't strictly needed for the preset UX.
 */
export function setCameraState(state) {
    if (!camera || !controls || !state || !state.position || !state.target) return;
    camera.position.set(state.position.x, state.position.y, state.position.z);
    controls.target.set(state.target.x, state.target.y, state.target.z);
    controls.update();
}

/**
 * Initialise the 3D viewer.
 * @param {HTMLElement} container - DOM element to attach the renderer to
 */
export function initViewer(container) {
    // Scene
    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x0f172a);

    // Camera
    const aspect = container.clientWidth / container.clientHeight;
    camera = new THREE.PerspectiveCamera(50, aspect, 0.01, 50);
    // Position camera ~1.5m away, looking at platform mid-height
    const midHeight = INITIAL_HEIGHT_MM * 0.5 * SCALE;
    camera.position.set(1.2, 1.0, 1.2);
    camera.lookAt(0, midHeight, 0);

    // Renderer
    renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setPixelRatio(window.devicePixelRatio);
    renderer.setSize(container.clientWidth, container.clientHeight);
    container.appendChild(renderer.domElement);

    // OrbitControls
    controls = new OrbitControls(camera, renderer.domElement);
    controls.target.set(0, midHeight, 0);
    controls.enableDamping = true;
    controls.dampingFactor = 0.1;
    controls.update();

    // Lighting
    const ambient = new THREE.AmbientLight(0xffffff, 0.6);
    scene.add(ambient);

    const dirLight = new THREE.DirectionalLight(0xffffff, 0.8);
    dirLight.position.set(2, 3, 1);
    scene.add(dirLight);

    // Grid (on XZ plane in Three.js = XY ground plane in robot frame)
    const grid = new THREE.GridHelper(2, 20, 0x334155, 0x1e293b);
    grid.name = 'grid';
    scene.add(grid);
    sceneGroups['Grid'] = grid;

    // Coordinate axes helper — rotated so RGB = robot X/Y/Z (Z-up frame)
    const axes = new THREE.AxesHelper(0.3);
    axes.name = 'axes';
    axes.rotation.x = -Math.PI / 2; // Rotate so Three.js Y-up → robot Z-up
    scene.add(axes);
    sceneGroups['Axes'] = axes;

    // Handle resize
    const resizeObserver = new ResizeObserver(() => {
        const w = container.clientWidth;
        const h = container.clientHeight;
        camera.aspect = w / h;
        camera.updateProjectionMatrix();
        renderer.setSize(w, h);
    });
    resizeObserver.observe(container);

    // Raycast pick — click a pickable mesh (leg / hand / BB element) to
    // fire `onMeshPick` listeners with the chartIdx stored in userData.
    // Uses mousedown-then-up-without-drag so OrbitControls orbit-drags
    // don't fire picks (a 4 px movement threshold is enough for a typical
    // orbit gesture).
    const raycaster = new THREE.Raycaster();
    const ndc = new THREE.Vector2();
    let pickDownPx = null;
    renderer.domElement.addEventListener('pointerdown', (ev) => {
        if (ev.button !== 0) return;
        pickDownPx = { x: ev.clientX, y: ev.clientY };
    });
    renderer.domElement.addEventListener('pointerup', (ev) => {
        if (ev.button !== 0 || !pickDownPx) return;
        const dx = ev.clientX - pickDownPx.x;
        const dy = ev.clientY - pickDownPx.y;
        pickDownPx = null;
        if (dx * dx + dy * dy > 16) return;  // dragged — not a click

        const rect = renderer.domElement.getBoundingClientRect();
        ndc.x = ((ev.clientX - rect.left) / rect.width) * 2 - 1;
        ndc.y = -((ev.clientY - rect.top) / rect.height) * 2 + 1;
        raycaster.setFromCamera(ndc, camera);
        const hits = raycaster.intersectObjects(pickableMeshes, false);
        if (hits.length === 0) return;
        const idx = hits[0].object.userData?.chartIdx;
        if (typeof idx !== 'number') return;
        for (const cb of pickListeners) {
            try { cb(idx); } catch { /* ignore listener errors */ }
        }
    });

    // Start render loop
    animate();
}

function animate() {
    requestAnimationFrame(animate);
    controls.update();
    renderer.render(scene, camera);
}
