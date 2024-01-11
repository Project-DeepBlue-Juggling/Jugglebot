import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

// Initialize all constants and variables that have multi-function scope
// Create scene and camera
let scene, camera, renderer;

// Initialize the geometry
let platformPointsGeometry, basePointsGeometry;
let legLinesGeometry, strutLinesGeometry, rodLinesGeometry; // Struts span arm to plat, rods span lower arm to upper arm
let baseVolume;

const scale = 0.0025;  // Scale factor to make the scene look nice

var gridHelper
const MAX_POINTS = 10;  // Maximum number of points to plot in any given set (eg. platform nodes, base nodes, etc.)

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

    const throttledUpdatePlot = throttle(updateScene, 10); // Throttle the updatePlot function (delay is in ms)

    // When a message is received, parse it and update the plot
    listener.subscribe(function (message) {
        throttledUpdatePlot(
            message.base_nodes, 
            message.new_plat_nodes, 
            message.new_arm_nodes, 
            message.new_hand_nodes
            );
    });
}

// Function to throttle the rate of the scene updating (to help with performance)
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

// ################################################################## //
//                      Creating Scene Geometry                       //
// ################################################################## //

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

// Function to initialize points
function initPoints(scene, colour) {
    // Create an empty BufferGeometry
    const geometry = new THREE.BufferGeometry();

    // Define attributes for the geometry
    const positions = new Float32Array(MAX_POINTS * 3); // 3 vertices per point
    geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));

    // Define material
    const material = new THREE.PointsMaterial({ color: colour, size: 0.05 });
    const points = new THREE.Points(geometry, material);

    // Add points to the scene
    scene.add(points);

    return points; // Return the points object for future updates
}

// Function to initialize a line group
function initLineGroup(scene, colour, thickness=0.01, numberOfLines) {
    const lineGroup = new THREE.Group();

    for (let i = 0; i < numberOfLines; i++) {
        // Initialize cylinder geometry with dummy length; will be updated later
        const edgeGeometry = new THREE.CylinderGeometry(thickness, thickness, 1, 8, 1);

        // Create the material and mesh
        const edgeMaterial = new THREE.MeshBasicMaterial({ color: colour });
        const edge = new THREE.Mesh(edgeGeometry, edgeMaterial);

        // Add edge to the scene
        lineGroup.add(edge);
    }

    // Add lineGroup to the scene
    scene.add(lineGroup);

    return lineGroup; // Return the group for future updates
}

// Function to initialize a volume
function initVolume(scene, colour, thickness) {
    // Create a shape with dummy points; will be updated later
    const shape = new THREE.Shape();
    shape.moveTo(0, 0);
    shape.lineTo(1, 0);
    shape.lineTo(1, 1);
    shape.lineTo(0, 1);
    shape.lineTo(0, 0);
    
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
        color: colour,
        transparent: true,
        opacity: 0.3
    });

    // Create mesh and add to the scene
    const mesh = new THREE.Mesh(geometry, material);
    scene.add(mesh);

    return mesh; // Return the mesh for future updates
}

// ################################################################## //
//                      Updating Scene Geometry                       //
// ################################################################## //

function updatePoints(points, pointsArray) {
    // Scale the pointsArray
    pointsArray = scalePoints(pointsArray, scale);

    const positions = points.geometry.attributes.position.array;
    
    pointsArray.forEach((point, index) => {
        positions[index * 3] = point.x;
        positions[index * 3 + 1] = point.z; // Swap y and z
        positions[index * 3 + 2] = point.y;
    });

    points.geometry.attributes.position.needsUpdate = true;
}

function updateLine(edge, start, end) {
    start = new THREE.Vector3(start.x, start.z, start.y).multiplyScalar(scale);
    end = new THREE.Vector3(end.x, end.z, end.y).multiplyScalar(scale);

    const direction = new THREE.Vector3().subVectors(end, start);
    const length = direction.length();
    
    // Reset the scale and reapply to adjust the length
    edge.scale.set(1, 1, 1);
    edge.scale.y = length;

    // Calculate midpoint
    const midpoint = new THREE.Vector3().addVectors(start, end).multiplyScalar(0.5);

    // Calculate orientation
    const upAxis = new THREE.Vector3(0, 1, 0);
    const axis = new THREE.Vector3().crossVectors(upAxis, direction).normalize();
    const radians = Math.acos(upAxis.dot(direction.normalize()));

    // Update position and rotation
    edge.position.copy(midpoint);
    edge.quaternion.setFromAxisAngle(axis, radians);
}

function updateLineGroup(lineGroup, startPoints, endPoints, isStruts=false) {
    if (isStruts) { // If the line group is for struts, the points are chosen a little differently
        for (let i = 0; i < lineGroup.children.length; i++) {
            let temp = Math.ceil(i / 2) % 3;
            updateLine(lineGroup.children[i], startPoints[i], endPoints[temp]);
        }
    } else {
        for (let i = 0; i < lineGroup.children.length; i++) {
            updateLine(lineGroup.children[i], startPoints[i], endPoints[i]);
        }
    }
}

function updateVolume(volume, pointsArray, thickness) {
    // Scale the pointsArray
    pointsArray = scalePoints(pointsArray, scale);

    // Create a shape based on the points
    const shape = new THREE.Shape();
    shape.moveTo(pointsArray[0].x, pointsArray[0].y);
    for (let i = 1; i < pointsArray.length - 1; i++) { // -1 because the last node is the origin
        shape.lineTo(pointsArray[i].x, pointsArray[i].y);
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
    geometry.translate(0, -thickness / 2, 0);

    // Update the geometry
    volume.geometry.dispose();
    volume.geometry = geometry;

    // Update the position
    // volume.position.y = thickness / 2;
}

// Function to update the scene
function updateScene(base_nodes, new_plat_nodes, new_arm_nodes, new_hand_nodes) {
    // Update points for platform and base
    updatePoints(platformPointsGeometry, new_plat_nodes.slice(0, 6));
    updatePoints(basePointsGeometry, base_nodes);

    // Update the leg lines
    updateLineGroup(legLinesGeometry, base_nodes, new_plat_nodes);

    // Update the strut lines (spanning plat to arm)
    updateLineGroup(strutLinesGeometry, new_plat_nodes, new_arm_nodes, true);

    // Update the rod lines (spanning upper arm to lower arm)
    updateLineGroup(rodLinesGeometry, new_arm_nodes, new_arm_nodes.slice(3));

    // Update the base volume
    updateVolume(baseVolume, base_nodes, 0.01);
}

// ################################################################## //
//                           Main Function                            //
// ################################################################## //

export function initPlotter() {
    // Initialize ROS
    initROS();

    // Initialize scene, camera and renderer
    scene = new THREE.Scene();
    camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 10000);
    renderer = new THREE.WebGLRenderer();
    var container = document.getElementById("plot-3d");
    renderer.setSize(container.clientWidth, container.clientHeight);
    container.appendChild(renderer.domElement);

    // Scale the y axis so that its size matches the x/z axes
    scene.scale.y = 0.5;  // Found experimentally

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

    // Create coordinate axes
    createCoordAxes(scene, 1, 0.01);

    // Function to create a tick mark
    function createTickMark(position, color) {
        const geometry = new THREE.BoxGeometry(0.1, 0.1, 0.1);
        const material = new THREE.MeshBasicMaterial({ color: color });
        const tickMark = new THREE.Mesh(geometry, material);
        tickMark.position.set(position.x, position.y, position.z);
        return tickMark;
    }

    // Create tick marks along the x, y, and z axes
    // for (let i = -5; i <= 5; i++) {
    //     scene.add(createTickMark(new THREE.Vector3(i, 0, 0), 'red')); // x-axis tick marks
    //     scene.add(createTickMark(new THREE.Vector3(0, i, 0), 'green')); // y-axis tick marks
    //     scene.add(createTickMark(new THREE.Vector3(0, 0, i), 'blue')); // z-axis tick marks
    // }

    // Initialize the points
    platformPointsGeometry = initPoints(scene, 0xff0000); // Red points for platform nodes
    basePointsGeometry = initPoints(scene, 0x0000ff); // Blue points for base nodes

    // Initialize the lines
    legLinesGeometry = initLineGroup(scene, 0x00ff00, 0.015, 6); // Green color for legs
    strutLinesGeometry = initLineGroup(scene, 0x000000, 0.005, 6); // Black color for struts
    rodLinesGeometry = initLineGroup(scene, 0x000000, 0.005, 3); // Black color for rods
    
    // Initialize the base volume
    baseVolume = initVolume(scene, 0x0000ff, 0.01); // Blue color for base volume

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