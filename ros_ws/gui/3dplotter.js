import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

const scale = 0.0025;  // Scale factor to make the scene look nice

var scene
var gridHelper

// Function to initialize ROS and subscribe to /robot_plotter_topic
function initROS() {
    // Initialize ROS
    var ros = new ROSLIB.Ros({
        url: 'ws://localhost:9090'
    });

    // Subscribe to /robot_plotter_topic 
    var listener = new ROSLIB.Topic({
        ros: ros,
        name: '/jugglebot/robot_plotter_topic',
        messageType: '/jugglebot_interfaces/RobotPlotterMessage'
    });

    const throttledUpdatePlot = throttle(updatePlot, 100); // Throttle the updatePlot function

    // When a message is received, parse it and update the plot
    listener.subscribe(function (message) {
        throttledUpdatePlot(
            scene,
            message.base_nodes, 
            message.new_plat_nodes, 
            message.new_arm_nodes, 
            message.new_hand_nodes
            );
    });
}

// Function to throttle the rate of the 3dPlot updating
function throttle(callback, delay) {
    let lastCall = 0;
    let timeoutId = null;

    return function() {
        const now = new Date().getTime();
        const timeSinceLastCall = now - lastCall;
        const args = arguments;
        clearTimeout(timeoutId);

        if (timeSinceLastCall >= delay) {
            lastCall = now;
            callback.apply(null, args);
        } else {
            timeoutId = setTimeout(() => {
                lastCall = new Date().getTime();
                callback.apply(null, args);
            }, delay - timeSinceLastCall);
        }
    };
}

// Function to scale an array of geometry_msgs/Point objects by the nominated factor
function scalePoints(pointsArray, scaleFactor) {
    return pointsArray.map(point => {
        return {
            x: point.x * scaleFactor,
            y: point.y * scaleFactor,
            z: point.z * scaleFactor
        };
    });
}

// Function to add points to the scene
function createPoints(scene, pointsArray, color) {
    // Start by scaling the pointsArray
    pointsArray = scalePoints(pointsArray, scale);

    // Create points, swapping the y and z axes (I use z-up, three.js uses y-up)
    const geometry = new THREE.BufferGeometry().setFromPoints(
        pointsArray.map(point => new THREE.Vector3(
            point.x, point.z, point.y)) // Adjusted to use object properties
    );

    const material = new THREE.PointsMaterial({ color: color, size: 0.05 });
    const points = new THREE.Points(geometry, material);
    scene.add(points);
}

// Function to add lines to the scene
function createLine(scene, start, end, color, thickness=0.01) {
    start = new THREE.Vector3(start.x, start.z, start.y).multiplyScalar(scale);
    end = new THREE.Vector3(end.x, end.z, end.y).multiplyScalar(scale);

    const direction = new THREE.Vector3().subVectors(end, start);
    const length = direction.length();

    // Create the cylinder geometry
    const edgeGeometry = new THREE.CylinderGeometry(thickness, thickness, length, 8, 1);
    
    // Calculate midpoint
    const midpoint = new THREE.Vector3().addVectors(start, end).multiplyScalar(0.5);

    // Calculate orientation
    const orientation = new THREE.Matrix4();
    const upAxis = new THREE.Vector3(0, 1, 0);
    const axis = new THREE.Vector3().crossVectors(upAxis, direction).normalize();
    const radians = Math.acos(upAxis.dot(direction.normalize()));
    orientation.makeRotationAxis(axis, radians);

    // Apply the orientation
    edgeGeometry.applyMatrix4(orientation);

    // Translate the geometry to the midpoint
    edgeGeometry.translate(midpoint.x, midpoint.y, midpoint.z);

    // Create the material and mesh
    const edgeMaterial = new THREE.MeshBasicMaterial({ color: color });
    const edge = new THREE.Mesh(edgeGeometry, edgeMaterial);

    scene.add(edge);
}

// Function to create volume for the base
function createBaseVolume(scene, baseNodes, color, thickness) {
    // Scale the base nodes
    baseNodes = scalePoints(baseNodes, scale);

    // Create a shape based on the base nodes
    const shape = new THREE.Shape();
    shape.moveTo(baseNodes[0].x, baseNodes[0].y);
    for (let i = 1; i < baseNodes.length - 1; i++) { // -1 because the last node is the origin
        shape.lineTo(baseNodes[i].x, baseNodes[i].y);
    }

    // Extrude settings
    const extrudeSettings = {
        steps: 1,
        depth: thickness,
        bevelEnabled: false
    };

    // Create geometry from the shape
    const geometry = new THREE.ExtrudeGeometry(shape, extrudeSettings);

    // Rotate the geometry so that it is flat on the ground
    geometry.rotateX(-Math.PI / 2);

    // Offset the geometry so that its middle is at the origin
    geometry.translate(0, -thickness / 2, 0);

    // Create material
    const material = new THREE.MeshBasicMaterial({
        color: color,
        transparent: true,
        opacity: 0.3
    });

    // Create mesh and add to the scene
    const mesh = new THREE.Mesh(geometry, material);
    scene.add(mesh);
}

// Function to clear all objects from the scene properly
function clearScene(scene) {
    const toRemove = [];
    scene.traverse((child) => {
        if (child.isMesh && child !== gridHelper && !child.userData.keep) {
            toRemove.push(child);
        }
    });

    toRemove.forEach((child) => {
        if (child.geometry) {
            child.geometry.dispose();
        }
        if (child.material) {
            if (Array.isArray(child.material)) {
                child.material.forEach(material => material.dispose());
            } else {
                child.material.dispose();
            }
        }
        scene.remove(child);
    });
}

// Function to update the robot plot
function updatePlot(scene, base_nodes, new_plat_nodes, new_arm_nodes, new_hand_nodes) {
    // Clear existing objects from the scene
    scene.clear();  // Seems to be required to remove points. clearScene doesn't work...
    clearScene(scene);

    // Add a grid and coordinate axes
    scene.add(gridHelper);
    createCoordAxes(scene, 1, 0.01);

    // Add points for the platform and base
    createPoints(scene, new_plat_nodes.slice(0, 6), 0xff0000);  // red points for the platform. Exclude the 7th point
    createPoints(scene, base_nodes, 0x0000ff);  // blue points for the base

    // Plot the legs (between the platform and base)
    for (let i = 0; i < 6; i++) {
        createLine(scene, new_plat_nodes[i], base_nodes[i], 0x00ff00, 0.01); // Green color for legs
    }

    // Plot the struts (connect platform nodes to arm nodes)
    for (let i = 0; i < 6; i++) {
        let temp = Math.ceil(i / 2) % 3;
        createLine(scene, new_plat_nodes[i], new_arm_nodes[temp], 0x000000, 0.005); // Black color for struts
    }

    // Plot the rods (connect lower arm to upper arm)
    for (let i = 0; i < 3; i++) {
        createLine(scene, new_arm_nodes[i], new_arm_nodes[i + 3], 0x000000, 0.005); // Black color for rods
    }

    // Create the volume for the base
    createBaseVolume(scene, base_nodes, 0x0000ff, 0.05); // Blue color for base volume
}

// Function to create coordinate axes
function createCoordAxes(scene, axisLength, axisThickness) {
    // Materials for each axis
    const axesMaterial = {
        x: new THREE.MeshBasicMaterial({ color: 0xff0000 }), // Red for X axis
        y: new THREE.MeshBasicMaterial({ color: 0x0000ff }), // Green for Y axis
        z: new THREE.MeshBasicMaterial({ color: 0x00ff00 })  // Blue for Z axis
    };

    // Function to create an axis (cylinder)
    function createAxis(axisDirection, material) {
        // Create a cylinder to represent the axis
        const axisGeometry = new THREE.CylinderGeometry(axisThickness, axisThickness, axisLength, 32);
        const axis = new THREE.Mesh(axisGeometry, material);

        // Position the axis
        axis.position.add(axisDirection.clone().multiplyScalar(axisLength / 2));

        // Rotate the axis to align it correctly
        if (axisDirection.x !== 0) { // X axis
            axis.rotation.z = Math.PI / 2;
        } else if (axisDirection.z !== 0) { // Z axis
            axis.rotation.x = Math.PI / 2;
        }
        // No rotation needed for Y axis as it aligns with the cylinder's default orientation

        return axis;
    }

    // Create and add axes to the scene
    scene.add(createAxis(new THREE.Vector3(1, 0, 0), axesMaterial.x)); // X Axis
    scene.add(createAxis(new THREE.Vector3(0, 1, 0), axesMaterial.y)); // Y Axis
    scene.add(createAxis(new THREE.Vector3(0, 0, 1), axesMaterial.z)); // Z Axis
}


export function initPlotter() {
    // Initialize ROS
    initROS();

    // Create scene, camera and renderer
    scene = new THREE.Scene();
    var camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 10000);

    // Scale the y axis so that its size matches the x/z axes
    scene.scale.y = 0.5;

    // Transform the scene to match the robot's coordinate frame
    scene.scale.z = -1;

    // Set the camera position to be above the scene looking down
    camera.position.set(1, 1.8, 1);
    camera.lookAt(0, 0.5, 0);

    // Make background white
    scene.background = new THREE.Color(0xffffff);

    // Add a grid
    gridHelper = new THREE.GridHelper(5, 5);
    scene.add(gridHelper);
    gridHelper.userData.keep = true; // Don't remove the grid when clearing the scene

    // Function to create a tick mark
    function createTickMark(position, color) {
        const geometry = new THREE.BoxGeometry(0.1, 0.1, 0.1);
        const material = new THREE.MeshBasicMaterial({ color: color });
        const tickMark = new THREE.Mesh(geometry, material);
        tickMark.position.set(position.x, position.y, position.z);
        return tickMark;
    }

    // Create tick marks along the x, y, and z axes
    for (let i = -5; i <= 5; i++) {
        scene.add(createTickMark(new THREE.Vector3(i, 0, 0), 'red')); // x-axis tick marks
        scene.add(createTickMark(new THREE.Vector3(0, i, 0), 'green')); // y-axis tick marks
        scene.add(createTickMark(new THREE.Vector3(0, 0, i), 'blue')); // z-axis tick marks
    }

    // Renderer
    var renderer = new THREE.WebGLRenderer();
    var container = document.getElementById("plot-3d");
    renderer.setSize(container.clientWidth, container.clientHeight);
    container.appendChild(renderer.domElement);

    // Initialize orbit controls
    var controls = new OrbitControls(camera, renderer.domElement);
    controls.target.set(0, 0, 0); // Set the orbit controls target to the center of the scene

    // Render loop
    function animate() {
        requestAnimationFrame(animate);

        controls.update();

        // Render the scene
        renderer.render(scene, camera);
    };

    animate();
}

window.initPlotter = initPlotter;