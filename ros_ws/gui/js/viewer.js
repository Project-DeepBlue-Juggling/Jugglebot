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

    // Coordinate axes helper
    const axes = new THREE.AxesHelper(0.3);
    axes.name = 'axes';
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

    // Start render loop
    animate();
}

function animate() {
    requestAnimationFrame(animate);
    controls.update();
    renderer.render(scene, camera);
}
