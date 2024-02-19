import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

// Initialize all constants and variables that have multi-function scope
// Create scene and camera
let camera, renderer;
export const scene = new THREE.Scene();

// Initialize the geometry
let platformPointsGeometry, basePointsGeometry, centroid, handPointsGeometry;
let legLinesGeometry, strutLinesGeometry, rodLinesGeometry; // Struts span arm to plat, rods span lower arm to upper arm
let currentPathObjects, nextPathObjects, transitionPathObjects; // Objects for the current, new, and transition paths
let baseVolume;

let initHeight; // Initial height of the robot (used to put the paths into the correct coordinate frame)

var arrowPools = {  // Pools of arrows for the paths
    currentPath: [],
    nextPath: [],
    transitionPath: [],
    centroid: []
}

// Create groups for all the geometries that share a name
var groups = {
    'robot': new THREE.Group(),
    'centroid': new THREE.Group(),
    'currentPath': new THREE.Group(),
    'nextPath': new THREE.Group(),
    'transitionPath': new THREE.Group(),
}

const scale = 0.0025;  // Scale factor to make the scene look nice

var gridHelper

// ################################################################## //
//                       Interacting with ROS2                        //
// ################################################################## //

// Function to initialize ROS and subscribe to /robot_plotter_topic
function initROS() {
    // Initialize ROS
    var ros = new ROSLIB.Ros({
        url : `ws://${window.location.hostname}:9090`
    });

    // Set up service client for /get_robot_geometry (to convert between coordinate frames)
    var getRobotGeometryClient = new ROSLIB.Service({
        ros: ros,
        name: 'get_robot_geometry',
        serviceType: 'jugglebot_interfaces/srv/GetRobotGeometry'
    });

    // Function to send a service request to /get_robot_geometry
    function getRobotGeometry() {
        var request = new ROSLIB.ServiceRequest();

        getRobotGeometryClient.callService(request, function (response) {
            if (response) {
                initHeight = response.start_pos[2];
            }
            else {
                console.log('Error calling get_robot_geometry service');
            }
        });
    }

    getRobotGeometry();

    // Subscribe to /robot_plotter_topic 
    var plotterListener = new ROSLIB.Topic({
        ros: ros,
        name: 'robot_plotter_topic',
        messageType: 'jugglebot_interfaces/msg/RobotPlotterMessage'
    });

    const throttledUpdatePlot = throttle(updateScene, 100); // Throttle the updatePlot function (delay is in ms)

    // When a message is received, parse it and update the plot
    plotterListener.subscribe(function (message) {
        throttledUpdatePlot(
            message.base_nodes, 
            message.new_plat_nodes, 
            message.new_arm_nodes, 
            message.new_hand_nodes,
            message.orientation
            );
    });

    // Subscribe to /leg_state_topic (to know which, if any, legs are outside of allowable bounds)
    var legStateListener = new ROSLIB.Topic({
        ros: ros,
        name: 'leg_state_topic',
        messageType: 'std_msgs/msg/Int8MultiArray'
    });

    // When a message is received, parse it and update the plot, colouring overextended legs red and overcontracted legs blue
    // The message is an array of 6 integers, each of which is either -1 (overcontracted), 0 (in bounds), or 1 (overextended)
    legStateListener.subscribe(function (message) {
        // Create an array of colours to use for the legs
        const colours = [
            0x0000ff, // Blue for overcontracted
            0x00ff00, // Green for in bounds
            0xff0000  // Red for overextended
        ];

        // Loop through each leg and update its colour
        for (let i = 0; i < message.data.length; i++) {
            let colour = colours[message.data[i] + 1]; // +1 because the message data is -1, 0, or 1
            legLinesGeometry.children[i].material.color.setHex(colour);
        }
    }); 

    // Subscribe to /paths_topic to get the robot paths
    var pathsListener = new ROSLIB.Topic({
        ros: ros,
        name: 'paths_topic',
        messageType: 'jugglebot_interfaces/msg/PathMessage'
    });

    // When a message is received, parse it and update the plot with the new paths
    pathsListener.subscribe(function (message) {
        if (Array.isArray(message.current_path) && message.current_path.length > 0) {
            updateSplineCurve(currentPathObjects, message.current_path);
            updateArrows('currentPath', message.current_path);
        }
        if (Array.isArray(message.next_path) && message.next_path.length > 0) {
            updateSplineCurve(nextPathObjects, message.next_path);
            updateArrows('nextPath', message.next_path);
        }
        if (Array.isArray(message.transition_path) && message.transition_path.length > 0) {
            updateSplineCurve(transitionPathObjects, message.transition_path);
            updateArrows('transitionPath', message.transition_path);
        }
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

    // Create and add axes to the 'axes' group
    const axesGroup = new THREE.Group();
    axesGroup.name = 'axes';
    axesGroup.add(createAxis(new THREE.Vector3(1, 0, 0), axesMaterial.x))
    axesGroup.add(createAxis(new THREE.Vector3(0, 1, 0), axesMaterial.y))
    axesGroup.add(createAxis(new THREE.Vector3(0, 0, 1), axesMaterial.z))

    // Add the axes group to the scene
    scene.add(axesGroup);
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
function initPoints(scene, colour, numPoints=6, objectName='') {
    // Create an empty BufferGeometry
    const geometry = new THREE.BufferGeometry();

    // Define attributes for the geometry
    const positions = new Float32Array(numPoints * 3); // 3 vertices per point
    geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));

    // Define material
    const material = new THREE.PointsMaterial({ color: colour, size: 0.05 });
    const points = new THREE.Points(geometry, material);
    
    // Add the points object to the appropriate group
    if (groups[objectName]) {
        groups[objectName].add(points);
    }

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

// Function to initialise the paths
function initSplineCurve(colour, objectName='') {
    const splineCurve = new THREE.CatmullRomCurve3();
    const splineMaterial = new THREE.LineBasicMaterial({ color: colour });

    // Create a line mesh with empty geometry for now
    const splineMesh = new THREE.Line(new THREE.BufferGeometry(), splineMaterial);

    // Add the spline mesh to the appropriate group
    if (groups[objectName]) {
        groups[objectName].add(splineMesh);
    }

    return { splineCurve, splineMesh };
}

// Initialize the arrows for orientation along the paths
function initArrows(arrowGroupName='', colour, arrowCount=500, arrowSize=0.4){
    const arrowColour = new THREE.Color(colour);

    // Create a pool of arrows
    for (let i = 0; i < arrowCount; i++) {
        // Create the arrow
        const arrow = new THREE.ArrowHelper(
            new THREE.Vector3(1, 0, 0), // Direction
            new THREE.Vector3(0, 0, 0), // Origin
            arrowSize, // Length
            arrowColour, // Color
        );

        arrow.visible = false; // Hide the arrow for now

        // Add the arrow to the pool
        arrowPools[arrowGroupName].push(arrow);
    }

    // Add the arrow pool to the relevant group
    if (groups[arrowGroupName]) {
        groups[arrowGroupName].add(...arrowPools[arrowGroupName]);
    }
}

// Function to add the robot workspace (from a JSON file) to the scene
function addWorkspaceVolume(scene) {
    fetch('convex_hull_points_big.json')
        .then(response => response.json())
        .then(hullData => {
            const geometry = new THREE.BufferGeometry();

            // Create an array of vertices from the hull data
            const verticesArray = [];
            hullData.vertices.forEach(point => {
                verticesArray.push(point[0] * scale, point[2] * scale, point[1] * scale);
            });

            const vertices = new Float32Array(verticesArray);

            // Set the geometry attributes
            geometry.setAttribute('position', new THREE.BufferAttribute(vertices, 3));

            // Create an array of faces (indices) from the hull data
            const facesArray = [];
            hullData.faces.forEach(face => {
                facesArray.push(face[0], face[1], face[2]);
            });

            const faces = new Uint16Array(facesArray);
            geometry.setIndex(new THREE.BufferAttribute(faces, 1));

            // Create the material
            const material = new THREE.MeshBasicMaterial({
                color: 0xADD8E6, // Light blue color
                transparent: true,
                opacity: 0.3,
                side: THREE.DoubleSide
            });

            // Create the mesh and add it to the scene
            const workspaceMesh = new THREE.Mesh(geometry, material);
            workspaceMesh.name = 'workspace';
            scene.add(workspaceMesh);

            // Now that the scene is fully rendered, send a dispatch event to let the parent know
            var event = new Event('scene-loaded');
            document.dispatchEvent(event);
        });
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

function updateSplineCurve(splineObjects, pathPoints) {
    // console.log(pathPoints);
    const newPoints = pathPoints.map(p => new THREE.Vector3(
        p.pose.position.x * scale, 
        (p.pose.position.z + initHeight) * scale, 
        p.pose.position.y * scale
        ));
    splineObjects.splineCurve.points = newPoints;

    // console.log(splineObjects.splineCurve.points);

    const splinePoints = splineObjects.splineCurve.getPoints(500); // Adjust based on desired smoothness

    splineObjects.splineMesh.geometry.dispose();
    splineObjects.splineMesh.geometry = new THREE.BufferGeometry().setFromPoints(splinePoints);
}

function updateArrows(pathName, pathPoints) {
    const pool = arrowPools[pathName];
    if (!pool) {
        console.log('Invalid path name');
        return;
    }

    var interval = 5;

    for (let i = 0, arrowIndex = 0; i < pathPoints.length && arrowIndex < pool.length; i += interval, arrowIndex++) {
        const point = pathPoints[i].pose;
        const arrowHelper = pool[arrowIndex];
        arrowHelper.position.set(
            point.position.x * scale, 
            (point.position.z + initHeight )* scale,
            point.position.y * scale);

        const originalQuaternion = new THREE.Quaternion(-point.orientation.x, point.orientation.z, -point.orientation.y, point.orientation.w);

        arrowHelper.quaternion.copy(originalQuaternion);
        arrowHelper.visible = true;
    }

    // Hide remaining arrows in the pool
    for (let j = Math.ceil(pathPoints.length / interval); j < pool.length; j++) {
        pool[j].visible = false;
    }
}

function updateCentroidArrow(position, orientation){
    const arrowHelper = arrowPools['centroid'][0];
    arrowHelper.position.set(
        position.x * scale, 
        position.z * scale, // No need to add the initial height here, since the centroid is found from the platform points
        position.y * scale);

    const orientationQuaternion = new THREE.Quaternion(-orientation.x, orientation.z, -orientation.y, orientation.w);

    arrowHelper.quaternion.copy(orientationQuaternion);
    arrowHelper.visible = true;
}

// Function to update the scene
function updateScene(base_nodes, new_plat_nodes, new_arm_nodes, new_hand_nodes, orientation) {
    // Update points for platform and base
    updatePoints(platformPointsGeometry, new_plat_nodes);
    updatePoints(handPointsGeometry, new_hand_nodes);
    updatePoints(basePointsGeometry, base_nodes);

    // Update the platform centroid, and centroid arrow
    const centroidPos = averagePoints(new_plat_nodes.slice(0, 6));
    updatePoints(centroid, [centroidPos]);
    updateCentroidArrow([centroidPos][0], orientation);

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
//                    Background Calcs/Functions                      //
// ################################################################## //

// Function to calculate the average of a set of points
function averagePoints(pointsArray) {
    let average = { x: 0, y: 0, z: 0 };

    pointsArray.forEach(point => {
        average.x += point.x;
        average.y += point.y;
        average.z += point.z;
    });

    average.x /= pointsArray.length;
    average.y /= pointsArray.length;
    average.z /= pointsArray.length;

    return average;
}

// ################################################################## //
//                           Main Function                            //
// ################################################################## //

export function initPlotter() {
    // Initialize ROS
    initROS();

    // Initialize scene, camera and renderer
    camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 10000);
    renderer = new THREE.WebGLRenderer({antialias: true});
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

    // Set the colours of the various elements
    const colours = {
        platform: 0xff0000,
        hand: 0xff8000,
        base: 0x0000ff,
        centroid: 0x800080,
        legs: 0x00ff00,
        struts: 0x000000,
        rods: 0x000000,
        currentPath: 0x000000,
        nextPath: 0xff0000,
        transitionPath: 0x00ff00
    };

    // Initialize the points
    platformPointsGeometry = initPoints(scene, colours.platform, 7); // Red points for platform nodes
    basePointsGeometry     = initPoints(scene, colours.base, 7); // Blue points for base nodes
    centroid               = initPoints(scene, colours.centroid, 1, 'centroid'); // Purple point for platform center
    handPointsGeometry     = initPoints(scene, colours.hand, 3); // Orange points for hand nodes

    // Initialize the lines
    legLinesGeometry = initLineGroup(scene, colours.legs, 0.015, 6); // Green color for legs
    strutLinesGeometry = initLineGroup(scene, colours.struts, 0.005, 6); // Black color for struts
    rodLinesGeometry = initLineGroup(scene, colours.rods, 0.005, 3); // Black color for rods
    
    // Initialize the base volume
    baseVolume = initVolume(scene, colours.base, 0.01); // Blue color for base volume

    // Initialize the robot group, for toggling visibility
    groups.robot.add(platformPointsGeometry);
    groups.robot.add(handPointsGeometry);
    groups.robot.add(basePointsGeometry);
    groups.robot.add(legLinesGeometry);
    groups.robot.add(strutLinesGeometry);
    groups.robot.add(rodLinesGeometry);
    groups.robot.add(baseVolume);

    // Initialize the paths
    currentPathObjects = initSplineCurve(colours.currentPath, 'currentPath');
    nextPathObjects = initSplineCurve(colours.nextPath, 'nextPath');
    transitionPathObjects = initSplineCurve(colours.transitionPath, 'transitionPath');

    // Initialize the arrow pools
    initArrows('currentPath', colours.currentPath);
    initArrows('nextPath', colours.nextPath);
    initArrows('transitionPath', colours.transitionPath);
    initArrows('centroid', colours.centroid, 1, 0.5);

    // Set the name of each group correctly, and add all the groups to the scene
    for (const groupName in groups) {
        groups[groupName].name = groupName;
        scene.add(groups[groupName]);
    }

    // Initialize orbit controls
    var controls = new OrbitControls(camera, renderer.domElement);
    controls.target.set(0, 0, 0); // Set the orbit controls target to the center of the scene

    /* Add the robot workspace to the scene. MAKE SURE THIS IS THE LAST THING ADDED TO THE SCENE
       This is because the scene-loaded event is dispatched after this is added, and we want to 
       make sure that the scene is fully rendered before dispatching the event */
    addWorkspaceVolume(scene);

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