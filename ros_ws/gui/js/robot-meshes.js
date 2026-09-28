/** Static, offline-reduced CAD assets. One fetch, shared GPU geometry, no ROS work. */
import * as THREE from 'three';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';

let partsPromise;
const axes = Array.from({ length: 9 }, () => ({
    materials: [], state: null, seen: -Infinity, fault: false, faultAt: 0, highlighted: false,
}));
const normalColor = new THREE.Color();

function loadParts() {
    if (!partsPromise) {
        partsPromise = new GLTFLoader().loadAsync(new URL('../assets/robots/robot-parts.glb', import.meta.url).href)
            .then(({ scene }) => {
                const parts = new Map();
                scene.updateMatrixWorld(true);
                scene.traverse(obj => {
                    if (!obj.isMesh) return;
                    // Bake loader node transforms once, never on a telemetry update.
                    obj.geometry.applyMatrix4(obj.matrixWorld);
                    obj.geometry.computeBoundingSphere();
                    parts.set(obj.name, obj.geometry);
                    obj.material.dispose();
                });
                return parts;
            });
    }
    return partsPromise;
}

/** Stable mesh identity lets picking work before and after the asynchronous load. */
export function cadMesh(name, color, axis = null) {
    const material = new THREE.MeshStandardMaterial({ color, metalness: .25, roughness: .55 });
    material.userData.baseColor = color;
    const mesh = new THREE.Mesh(new THREE.BufferGeometry(), material);
    mesh.name = name;
    if (axis != null) {
        mesh.userData.chartIdx = axis;
        axes[axis].materials.push(material);
    }
    mesh.userData.ready = loadParts().then(parts => {
        if (!parts.has(name)) throw new Error(`Missing CAD part: ${name}`);
        mesh.geometry.dispose();
        mesh.geometry = parts.get(name);
        material.vertexColors = !!mesh.geometry.getAttribute('color');
        if (material.vertexColors) {
            material.userData.baseColor = 0xffffff;
            material.color.setHex(0xffffff);
        }
        material.needsUpdate = true;
    }).catch(error => {
        console.error('Robot mesh unavailable', error);
        if (!document.getElementById('robot-mesh-error')) {
            const notice = document.createElement('div');
            notice.id = 'robot-mesh-error';
            notice.textContent = 'Robot models unavailable — reload to retry. Telemetry and markers remain available.';
            notice.style.cssText = 'position:fixed;bottom:12px;left:12px;z-index:1000;padding:10px;background:#7f1d1d;color:white;border-radius:6px';
            document.body.appendChild(notice);
        }
    });
    return mesh;
}

export function setRobotAxisStates(motors, now = performance.now()) {
    for (let i = 0; i < axes.length; i++) {
        axes[i].state = motors[i]?.current_state ?? null;
        axes[i].seen = motors[i] ? now : -Infinity;
    }
}

export function setAxisFault(index, faulted) {
    const axis = axes[index];
    if (!axis) return;
    if (faulted && !axis.fault) axis.faultAt = performance.now();
    axis.fault = !!faulted;
}

export function setAxisHighlight(index, highlighted) {
    if (axes[index]) axes[index].highlighted = highlighted;
}

/** Called from the existing viewer loop: no additional rAF, geometry or allocations.
 * Only confirmed IDLE (ODrive state 1) breathes. Unknown/stale state is matte;
 * CLOSED_LOOP (8) is steady. A fresh fault overrides both and pulses for 2s.
 */
export function updateRobotMaterials(now) {
    const idleBrightness = .62 + .18 * Math.sin(now * Math.PI * 2 / 3200);
    for (const axis of axes) {
        const fresh = now - axis.seen < 1500;
        const fault = fresh && axis.fault;
        for (const mat of axis.materials) {
            normalColor.setHex(mat.userData.baseColor);
            const brightness = fresh && axis.state === 8 ? 1
                : (fresh && axis.state === 1 ? idleBrightness : .5);
            mat.color.copy(normalColor).multiplyScalar(brightness);
            mat.emissive.setHex(fault ? 0xef4444 : 0xffffff);
            mat.emissiveIntensity = fault
                ? (now - axis.faultAt < 2000 ? .4 + .35 * (1 + Math.sin((now - axis.faultAt) / 125)) : .7)
                : (axis.highlighted ? .45 : 0);
            if (fault) mat.color.setHex(0xef4444);
        }
    }
}
