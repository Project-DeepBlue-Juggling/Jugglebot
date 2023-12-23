import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

const scale = 0.001;  // Scale factor to convert from mm to m

// Dummy data for testing
const dummyData = {
    base_nodes: [
        [401.04, 85.244, 0.0],
        [-126.70, 389.93, 0.0],
        [-274.34, 304.69, 0.0],
        [-274.34, -304.69, 0.0],
        [-126.70, -389.93, 0.0],
        [401.04, -85.24, 0.0],
        [0.0, 0.0, 0.0]],

    new_plat_nodes: [
        [127.50, 190.83, 626.25],
        [101.51, 205.83, 626.25],
        [-229.01, 15.00, 626.25],
        [-229.01, -15.00, 626.25],
        [101.51, -205.83, 626.25],
        [127.50, -190.83, 626.25],
        [0.0, 0.0, 520]],

    new_arm_nodes: [
        [70.0, 0.0, 836.5],
        [-35.0, 60.6, 836.5],
        [-35, -60.6, 836.5],
        [70.0, 0.0, 520],
        [-35.0, 60.62, 520],
        [-35.0, -60.62, 520]],

    new_hand_nodes: [
        [17.5, 30.31, 520],
        [-35.0, 0.0, 520],
        [17.5, -30.31, 520]]
}

// Function to initialize ROS and subscribe to the /robot_plotter_topic topic
function initROS() {
    // Initialize ROS
    var ros = new ROSLIB.Ros({
        url: 'ws://localhost:9090'
    });

    // Subscribe to the /robot_plotter_topic topic
    var listener = new ROSLIB.Topic({
        ros: ros,
        name: '/robot_plotter_topic',
        messageType: 'std_msgs/String'
    });

    // When a message is received, parse it and plot it
    listener.subscribe(function (message) {
        updateRobot(
            message.base_nodes, 
            message.new_plat_nodes, 
            message.new_arm_nodes, 
            message.new_hand_nodes
            );
    });
}

// Function to scale an array of points by the nominated factor
function scalePoints(pointsArray, scaleFactor) {
    return pointsArray.map(point => point.map(coordinate => coordinate * scaleFactor));
}

// Function to add points to the scene
function createPoints(scene, pointsArray, color) {
    // Start by scaling the pointsArray
    pointsArray = scalePoints(pointsArray, scale);

    // Create points, swapping the y and z axes (I use z-up, three.js uses y-up)
    const geometry = new THREE.BufferGeometry().setFromPoints(
        pointsArray.map(point => new THREE.Vector3(
            point[0], point[2], point[1]))
        );

    const material = new THREE.PointsMaterial({ color: color, size: 0.05 });
    const points = new THREE.Points(geometry, material);
    scene.add(points);
}

// Function to add lines to the scene
function createLine(scene, start, end, color, thickness=0.01) {
    start = new THREE.Vector3(start[0], start[2], start[1]).multiplyScalar(scale);
    end = new THREE.Vector3(end[0], end[2], end[1]).multiplyScalar(scale);

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
    shape.moveTo(baseNodes[0][0], baseNodes[0][1]);
    for (let i = 1; i < baseNodes.length - 1; i++) { // -1 because the last node is the origin
        shape.lineTo(baseNodes[i][0], baseNodes[i][1]);
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

// Function to update the robot plot
function updateRobot(scene, base_nodes, new_plat_nodes, new_arm_nodes, new_hand_nodes) {
    // Clear existing objects from the scene
    // scene.clear();

    // Add points for the platform and base
    createPoints(scene, new_plat_nodes.slice(0, 6), 0xff0000);  // red points for the platform. Exclude the 7th point
    createPoints(scene, base_nodes, 0x0000ff);  // blue points for the base

    // Plot the legs (between the platform and base)
    for (let i = 0; i < 6; i++) {
        createLine(scene, new_plat_nodes[i], base_nodes[i], 0x00ff00, 0.01);
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

export function initPlotter() {
    // Create scene, camera and renderer
    var scene = new THREE.Scene();
    var camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 10000);
    scene.scale.y = 1 / (window.innerWidth / window.innerHeight);

    // Set the camera position to be above the scene looking down
    camera.position.set(1, 1.2, 1);
    camera.lookAt(0, 0, 0);

    // Make background white
    scene.background = new THREE.Color(0xffffff);

    // Add a grid
    var gridHelper = new THREE.GridHelper(5, 5);
    scene.add(gridHelper);

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

    // Add a cube
    var geometry = new THREE.BoxGeometry(0.5, 0.5, 0.5);
    var material = new THREE.MeshBasicMaterial({ color: 0x00ff00 });
    var cube = new THREE.Mesh(geometry, material);
    // Move the cube to the side
    cube.position.x = 1;
    scene.add(cube);

    // Update the robot plot with dummy data
    updateRobot(
        scene,
        dummyData.base_nodes,
        dummyData.new_plat_nodes,
        dummyData.new_arm_nodes,
        dummyData.new_hand_nodes
    );

    // Render loop
    function animate() {
        requestAnimationFrame(animate);

        controls.update();

        // Rotate the cube
        cube.rotation.x += 0.01;
        cube.rotation.y += 0.01;

        // Render the cube
        renderer.render(scene, camera);
    };

    animate();
}

window.initPlotter = initPlotter;