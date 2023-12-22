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

// Function to add points to the scene
function createPoints(scene, pointsArray, color) {
    // Create points, including the scale value and swapping the y and z axes
    const geometry = new THREE.BufferGeometry().setFromPoints(
        pointsArray.map(point => new THREE.Vector3(
            point[0] * scale, 
            point[2] * scale, 
            point[1] * scale
        )));

    const material = new THREE.PointsMaterial({ color: color, size: 0.05 });
    const points = new THREE.Points(geometry, material);
    scene.add(points);
}

// Function to add lines to the scene
function createLine(scene, start, end, color) {
    const material = new THREE.LineBasicMaterial({ color: color, linewidth: 5 });
    // Create the line geometry, including the scale value and swapping the y and z axes
    const points = [
        new THREE.Vector3(
            start[0] * scale,
            start[2] * scale,
            start[1] * scale
        ), 
        new THREE.Vector3(
            end[0] * scale,
            end[2] * scale,
            end[1] * scale
        )];
    const geometry = new THREE.BufferGeometry().setFromPoints(points);
    const line = new THREE.Line(geometry, material);
    scene.add(line);
}

// Function to create volume for the base
function createBaseVolume(scene, baseNodes, color, thickness) {
    // Create a shape based on the base nodes
    const shape = new THREE.Shape();
    shape.moveTo(baseNodes[0][0], baseNodes[0][2]);
    for (let i = 1; i < baseNodes.length; i++) {
        shape.lineTo(baseNodes[i][0], baseNodes[i][2]);
    }
    shape.lineTo(baseNodes[0][0], baseNodes[0][2]); // Close the shape

    // Extrude settings
    const extrudeSettings = {
        steps: 1,
        depth: thickness,
        bevelEnabled: false
    };

    // Create geometry from the shape
    const geometry = new THREE.ExtrudeGeometry(shape, extrudeSettings);

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
        createLine(scene, new_plat_nodes[i], base_nodes[i], 0x00ff00);
    }

    // Plot the struts (connect platform nodes to arm nodes)
    for (let i = 0; i < 6; i++) {
        let temp = Math.ceil(i / 2) % 3;
        createLine(scene, new_plat_nodes[i], new_arm_nodes[temp], 0x000000); // Black color for struts
    }

    // Plot the rods (connect lower arm to upper arm)
    for (let i = 0; i < 3; i++) {
        createLine(scene, new_arm_nodes[i], new_arm_nodes[i + 3], 0x000000); // Black color for rods
    }

    // Create the volume for the base
    createBaseVolume(scene, base_nodes, 0x0000ff, 0.1); // Blue color for base volume
}

export function initPlotter() {
    // Create scene, camera and renderer
    var scene = new THREE.Scene();
    var camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 10000);
    
    // Set the camera position to be above the scene looking down
    camera.position.set(1, 1.2, 1);
    camera.lookAt(0, 0, 0);

    // Make background white
    scene.background = new THREE.Color(0xffffff);

    // Add a grid
    var gridHelper = new THREE.GridHelper(5, 5);
    scene.add(gridHelper);

    // Renderer
    var renderer = new THREE.WebGLRenderer();
    renderer.setSize(600, 400);
    var container = document.getElementById('3dplotter');
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