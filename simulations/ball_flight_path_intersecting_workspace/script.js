// script.js

// Global variables
var scene, camera, renderer;
var controls, transformControls;
var workspaceMesh;
var ballMesh;
var trajectoryLineInside, trajectoryLineOutside;
var gui;
var initialPosition = new THREE.Vector3();
var initialVelocity = new THREE.Vector3(1.3, 0, 4); // Default velocity

// Initialize the scene
function init() {
    // Create the scene
    scene = new THREE.Scene();

    // Create the camera
    camera = new THREE.PerspectiveCamera(
        45,
        window.innerWidth / window.innerHeight,
        0.1,
        1000
    );
    camera.up.set(0, 0, 1); // Set Z as up
    camera.position.set(-0.04, -2.14, 1.72);

    // Create the renderer
    renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(window.innerWidth, window.innerHeight);
    document.body.appendChild(renderer.domElement);

    // Add orbit controls
    controls = new THREE.OrbitControls(camera, renderer.domElement);
    controls.screenSpacePanning = true; // Enable screen space panning

    // Set the controls' target
    controls.target.set(-0.31, 0, 0.91);

    // Create ambient light
    scene.add(new THREE.AmbientLight(0xFFFFFF, 0.5));

    // Create directional light
    var dirLight = new THREE.DirectionalLight(0xFFFFFF, 1);
    dirLight.position.set(5, 5, 5);
    scene.add(dirLight);

    // Add grid helper
    var gridHelper = new THREE.GridHelper(10, 10);
    gridHelper.rotation.x = Math.PI / 2; // Rotate grid to lie on the XY plane
    scene.add(gridHelper);

    // Add axes helper
    var axesHelper = new THREE.AxesHelper(1);
    scene.add(axesHelper);

    // Load the workspace volume
    loadWorkspaceVolume().then(function() {
        // After loading, center the camera
        centerCamera();

        // Create the ball
        createBall();

        // Create GUI controls
        createGUI();

        // Simulate the initial trajectory
        simulateTrajectory();

        // Start the animation loop
        animate();
    });

    // Handle window resize
    window.addEventListener('resize', onWindowResize, false);
}


// Load the workspace volume
function loadWorkspaceVolume() {
    return fetch('convex_hull_points_big.json')
        .then(response => response.json())
        .then(hullData => {
            const geometry = new THREE.BufferGeometry();

            // Convert vertices from mm to meters
            const scale = 0.001;
            const verticesArray = [];
            hullData.vertices.forEach(point => {
                verticesArray.push(
                    point[0] * scale,
                    point[1] * scale,
                    point[2] * scale
                );
            });
            const vertices = new Float32Array(verticesArray);
            geometry.setAttribute('position', new THREE.BufferAttribute(vertices, 3));

            // Faces (indices)
            const facesArray = [];
            hullData.faces.forEach(face => {
                facesArray.push(face[0], face[1], face[2]);
            });
            geometry.setIndex(facesArray);

            // Compute normals for shading
            geometry.computeVertexNormals();

            // Material
            const material = new THREE.MeshStandardMaterial({
                color: 0xADD8E6,
                transparent: true,
                opacity: 0.5,
                side: THREE.DoubleSide
            });

            // Mesh
            workspaceMesh = new THREE.Mesh(geometry, material);
            scene.add(workspaceMesh);

            // Build the BVH for efficient point-in-mesh testing
            workspaceMesh.geometry.computeBoundingBox();
        });
}

// Center the camera on the workspace
function centerCamera() {
    const box = new THREE.Box3().setFromObject(workspaceMesh);
    const center = box.getCenter(new THREE.Vector3());
    const size = box.getSize(new THREE.Vector3()).length();
    const halfSize = size * 0.5;

    // Adjust camera and controls
    camera.near = size / 100;
    camera.far = size * 100;
    camera.updateProjectionMatrix();


    // Set the initial position of the ball
    initialPosition.copy(center.clone().add(new THREE.Vector3(-1, 0, 0)));
}

// Create the ball and add transform controls
function createBall() {
    // Ball geometry and material
    const ballGeometry = new THREE.SphereGeometry(0.05, 32, 32); // Radius of 5 cm
    const ballMaterial = new THREE.MeshBasicMaterial({ color: 0xff0000 });
    ballMesh = new THREE.Mesh(ballGeometry, ballMaterial);
    ballMesh.position.copy(initialPosition);
    scene.add(ballMesh);

    // Add transform controls to the ball
    transformControls = new THREE.TransformControls(camera, renderer.domElement);
    transformControls.attach(ballMesh);
    transformControls.setMode('translate');
    scene.add(transformControls);

    // Prevent orbit controls from conflicting with transform controls
    transformControls.addEventListener('dragging-changed', function(event) {
        controls.enabled = !event.value;
        if (!event.value) {
            // Update the initial position when dragging ends
            initialPosition.copy(ballMesh.position);
            simulateTrajectory();
        }
    });
}

// Create GUI controls for velocity adjustment
function createGUI() {
    gui = new dat.GUI({ autoPlace: false, width: 300 });
    document.getElementById('gui-container').appendChild(gui.domElement);

    // Add initial position display to the GUI
    gui.add(ballMesh.position, 'x').name('Position X (m)').listen();
    gui.add(ballMesh.position, 'y').name('Position Y (m)').listen();
    gui.add(ballMesh.position, 'z').name('Position Z (m)').listen();

    // Add velocity sliders directly to the GUI
    gui.add(initialVelocity, 'x', -2, 2).step(0.1).name('Velocity X (m/s)').onChange(simulateTrajectory);
    gui.add(initialVelocity, 'y', -2, 2).step(0.1).name('Velocity Y (m/s)').onChange(simulateTrajectory);
    gui.add(initialVelocity, 'z', 0, 6).step(0.1).name('Velocity Z (m/s)').onChange(simulateTrajectory);
}

// Handle window resize
function onWindowResize() {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();

    renderer.setSize(window.innerWidth, window.innerHeight);

    // Update LineMaterial resolutions
    if (trajectoryLineInside && trajectoryLineInside.material instanceof THREE.LineMaterial) {
        trajectoryLineInside.material.resolution.set(window.innerWidth, window.innerHeight);
    }
}


function simulateTrajectory() {
    // Remove existing trajectory lines
    if (trajectoryLineInside) {
        scene.remove(trajectoryLineInside);
        trajectoryLineInside.geometry.dispose();
        trajectoryLineInside.material.dispose();
        trajectoryLineInside = undefined;
    }
    if (trajectoryLineOutside) {
        scene.remove(trajectoryLineOutside);
        trajectoryLineOutside.geometry.dispose();
        trajectoryLineOutside.material.dispose();
        trajectoryLineOutside = undefined;
    }

    // Simulate the trajectory
    const g = new THREE.Vector3(0, 0, -9.81); // Gravity acceleration (m/s^2)
    const timeStep = 0.005;
    const positions = [];
    const times = []; // Array to store time associated with each position
    let t = 0.0;
    let position = initialPosition.clone();
    let velocity = initialVelocity.clone();
    const heightLimit = 0.5; // Stop simulation when below this height

    while (position.z > heightLimit) {
        position = initialPosition.clone()
            .add(velocity.clone().multiplyScalar(t))
            .add(g.clone().multiplyScalar(0.5 * t * t));
        positions.push(position.clone());
        times.push(t); // Store the time
        t += timeStep;
    }

    // Highlight the trajectory segments inside and outside the workspace
    highlightTrajectory(positions, times);
}

// Highlight the trajectory segments inside and outside the workspace
function highlightTrajectory(positions, times) {
    const insidePoints = [];
    const outsidePoints = [];
    let totalDurationInside = 0; // Initialize total duration inside

    for (let i = 0; i < positions.length - 1; i++) {
        const point = positions[i];
        const nextPoint = positions[i + 1];
        const isInsideCurrent = isPointInsideMesh(point, workspaceMesh);
        const isInsideNext = isPointInsideMesh(nextPoint, workspaceMesh);

        if (isInsideCurrent) {
            insidePoints.push(point);
            // If the next point is also inside, add the time interval
            if (isInsideNext) {
                totalDurationInside += times[i + 1] - times[i];
            } else {
                // Estimate the exit time (optional for higher accuracy)
                totalDurationInside += estimateCrossingTime(point, nextPoint, times[i], times[i + 1]);
            }
        } else {
            outsidePoints.push(point);
            // If the next point is inside, estimate the entry time (optional)
            if (isInsideNext) {
                totalDurationInside += estimateCrossingTime(nextPoint, point, times[i + 1], times[i]);
            }
        }
    }

    // Add the last point
    const lastPoint = positions[positions.length - 1];
    const isInsideLast = isPointInsideMesh(lastPoint, workspaceMesh);
    if (isInsideLast) {
        insidePoints.push(lastPoint);
    } else {
        outsidePoints.push(lastPoint);
    }

    // Display the total duration inside the workspace
    updateDurationDisplay(totalDurationInside*1000); // Convert to milliseconds

    // Create the trajectory line for inside points using Line2 (thicker line)
    if (insidePoints.length > 0) {
        const positionsArray = [];
        insidePoints.forEach(point => {
            positionsArray.push(point.x, point.y, point.z);
        });

        const geometryInside = new THREE.LineGeometry();
        geometryInside.setPositions(positionsArray);

        const materialInside = new THREE.LineMaterial({
            color: 0x00ff00, // Green
            linewidth: 15  // Adjust the line width as needed (in world units)
        });

        materialInside.resolution.set(window.innerWidth, window.innerHeight); // Important for LineMaterial

        trajectoryLineInside = new THREE.Line2(geometryInside, materialInside);
        trajectoryLineInside.computeLineDistances(); // Required for dashed lines (optional)
        scene.add(trajectoryLineInside);
    }

    // Create the trajectory line for outside points using Line (thinner line)
    if (outsidePoints.length > 0) {
        const geometryOutside = new THREE.BufferGeometry().setFromPoints(outsidePoints);
        const materialOutside = new THREE.LineBasicMaterial({ color: 0xff0000 }); // Red
        trajectoryLineOutside = new THREE.Line(geometryOutside, materialOutside);
        scene.add(trajectoryLineOutside);
    }
}

function estimateCrossingTime(point1, point2, time1, time2) {
    // Simple linear interpolation based on the inside/outside transition
    // Assume the boundary is crossed halfway for approximation
    return Math.abs(time2 - time1) / 2;
}

function updateDurationDisplay(duration) {
    var durationDisplay = document.getElementById('duration-display');
    durationDisplay.innerHTML = `
        <strong>Duration Inside Workspace:</strong><br>
        ${duration.toFixed(0)} milliseconds
    `;
}

// Check if a point is inside a mesh
function isPointInsideMesh(point, mesh) {
    var direction = new THREE.Vector3(1, 0, 0); // Arbitrary direction
    var raycaster = new THREE.Raycaster(point, direction, 0, Infinity);
    var intersects = raycaster.intersectObject(mesh, false);
    var intersections = 0;
    intersects.forEach(function(intersect) {
        if (intersect.distance > 0) {
            intersections++;
        }
    });
    return (intersections % 2) === 1;
}

function updateCameraInfo() {
    var cameraInfo = document.getElementById('camera-info');
    var position = camera.position;
    var target = controls.target;
    cameraInfo.innerHTML = `
        <strong>Camera Position:</strong><br>
        x: ${position.x.toFixed(2)}, y: ${position.y.toFixed(2)}, z: ${position.z.toFixed(2)}<br>
        <strong>Camera Target:</strong><br>
        x: ${target.x.toFixed(2)}, y: ${target.y.toFixed(2)}, z: ${target.z.toFixed(2)}
    `;
}


// Animation loop
function animate() {
    requestAnimationFrame(animate);

    // Update controls
    controls.update();

    // Render the scene
    renderer.render(scene, camera);

    // Update camera info
    updateCameraInfo();
}

// Start the application
init();
