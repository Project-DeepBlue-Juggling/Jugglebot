/** QTM-origin CAD, driven by the existing rigid-body stream (millimetres). */
import * as THREE from 'three';
import { scene, sceneGroups, onFrame, robotToThreeScaled } from './viewer.js';
import { cadMesh } from './robot-meshes.js';
import { INITIAL_HEIGHT_MM } from './geometry-config.js';

let model;
let lastSeen = -Infinity;
export function initCatchingConeModel() {
    const group = new THREE.Group();
    group.name = 'catching-cone';
    model = cadMesh('catching_cone', 0xffffff);
    model.visible = false;
    group.add(model);
    scene.add(group);
    sceneGroups['Catching Cone'] = group;
    onFrame(now => {
        if (now - lastSeen > 1500) model.visible = false;
    });
}

export function updateCatchingCone(bodies, now = performance.now()) {
    if (!model) return;
    const body = bodies.find(b => b.name?.replace(/[ -]/g, '_') === 'Catching_Cone');
    const stamped = body?.pose;
    const pose = stamped?.pose || stamped;
    const p = pose?.position, q = pose?.orientation;
    if (!p || !q || ![p.x, p.y, p.z, q.x, q.y, q.z, q.w].every(Number.isFinite)
        || Math.hypot(q.x, q.y, q.z, q.w) < 1e-8) {
        model.visible = false;
        lastSeen = -Infinity;
        return;
    }
    const zOffset = stamped.header?.frame_id === 'platform_start' ? INITIAL_HEIGHT_MM : 0;
    const pos = robotToThreeScaled(p.x, p.y, p.z + zOffset);
    model.position.set(pos.x, pos.y, pos.z);
    model.quaternion.set(q.x, q.z, -q.y, q.w).normalize();
    model.visible = true;
    lastSeen = now;
}
