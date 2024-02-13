import { scene } from './3dplotter.js';

window.onload = function () {
    // ################################################################## //
    //                          Connect to ROS                            //
    // ################################################################## //

    var ros = new ROSLIB.Ros({
        url : `ws://${window.location.hostname}:9090`
    });

    ros.on('connection', function() {
        console.log('Connected to websocket server.');
    });

    ros.on('error', function(error) {
        console.log('Error connecting to websocket server: ', error);
    });

    ros.on('close', function() {
        console.log('Connection to websocket server closed.');
    });

    // ################################################################## //
    //                      Initialize the 3D Plotter                     //
    // ################################################################## //

    initPlotter();

    // ################################################################## //
    //                  Subscribe to /rosout (for logging)                //
    // ################################################################## //

    // Get the console div element
    var consoleDiv = document.getElementById('ros-console');

    // Set a limit for how many messages are stored in the console
    var maxConsoleMessages = 100;

    // Subscribe to the /rosout topic
    var listener = new ROSLIB.Topic({
        ros: ros,
        name: '/rosout',
        messageType: 'rcl_interfaces/msg/Log'
    });

    // Define a function to map the log level to a color
    function getLogLevelColor(level) {
        switch(level) {
            case 10: return '#808080'; // DEBUG: Grey
            case 20: return '#000000'; // INFO: Black
            case 30: return '#FFA500'; // WARN: Orange
            case 40: return '#FF4500'; // ERROR: Red-Orange
            case 50: return '#FF0000'; // FATAL: Red
            default: return '#000000'; // Default color for unknown levels
        }
    }

    // Add a listener function to /rosout
    listener.subscribe(function(message) {
        // Check if the message level is enabled
        if (!document.getElementById(`log-level-${message.level}`).checked) {
            return; // Do nothing if the message level is not enabled
        }

        // Set the color of the console div based on the log level
        var logColor = getLogLevelColor(message.level);

        // Get the current time
        var currentDate = new Date();
        var timestamp = currentDate.getHours() + ':' + currentDate.getMinutes() + ':' + currentDate.getSeconds(); 

        // Create a new log entry
        var logEntry = document.createElement('div');
        logEntry.textContent = `[${timestamp}] [${message.name}]: ${message.msg}`;
        logEntry.style.color = logColor;

        // Add bold font and background color for FATAL messages
        if (message.level === 50) {
            logEntry.style.fontWeight = 'bold';
            logEntry.style.backgroundColor = '#ffcccc'; // Light red background
        }

        // If the message contains "Heartbeat received" then colour the background green
        if (message.msg.includes("Heartbeat received")) {
            logEntry.style.fontWeight = 'bold';
            logEntry.style.backgroundColor = '#ccffcc'; // Light green background
            logEntry.style.color = 'darkgreen'; // Dark green text
        }

        // Add a tolerance in case the browser rounds the scroll height
        var scrollTolerance = 5;

        // Check if the console is scrolled to the bottom before adding new message
        var isScrolledToBottom = consoleDiv.scrollHeight - consoleDiv.scrollTop - consoleDiv.clientHeight < scrollTolerance;

        // Add the log entry to the console
        consoleDiv.appendChild(logEntry);

        // Remove the oldest log entry if the console is full
        if (consoleDiv.childElementCount > maxConsoleMessages) {
            consoleDiv.removeChild(consoleDiv.firstChild);
        }

        // Use requestAnimationFrame to defer the scrolling
        requestAnimationFrame(function() {
            if (isScrolledToBottom) {
                consoleDiv.scrollTop = consoleDiv.scrollHeight;
            }
        });
    });

    // ################################################################## //
    //                        CAN Traffic Handling                        //
    // ################################################################## //

    // Subscribe to the /can_traffic topic
    var canTrafficListener = new ROSLIB.Topic({
        ros: ros,
        name: '/can_traffic',
        messageType: 'jugglebot_interfaces/msg/CanTrafficReportMessage'
    });

    var latestDataPoints = []; // Store the latest data points with custom time
    var timeWindow = 10 * 1000; // Time window in ms (minutes * seconds * ms)

    // Listen for messages on the /can_traffic topic and update the data array
    canTrafficListener.subscribe(function(message) {
        if (message.report_interval > 0) {
            var rate = (message.received_count / message.report_interval) * 1000; // Calculate msgs/sec
            // Shift the time of all existing data points by the report_interval
            latestDataPoints.forEach(point => point.x -= message.report_interval);

            // Add new data point with x value of 0 (representing "now")
            latestDataPoints.push({x: 0, y: rate});

            // Remove points outside the desired time window
            latestDataPoints = latestDataPoints.filter(point => point.x >= -timeWindow);
        }
    });

    var ctx = document.getElementById('can-traffic-chart').getContext('2d');
    var canTrafficChart = new Chart(ctx, {
        type: 'line',
        data: {
            datasets: [{
                label: 'CAN Traffic',
                data: latestDataPoints,
                fill: true,
                backgroundColor: 'rgba(255, 128, 0, 0.2)',
                borderColor: '#ff8000',
                tension: 0.1
            }]
        },
        options: {
            scales: {
                x: {
                    type: 'linear',
                    position: 'bottom',
                    title: {
                        display: true,
                        text: 'Seconds ago'
                    },
                    ticks: {
                        // Customize tick labels to show time relative to 'currentTime'
                        callback: function(value) {
                            return value/1000;
                        }
                    },
                    max: 0, // Start with the rightmost point at 0
                },
                y: {
                    beginAtZero: true,
                    title: {
                        display: true,
                        text: 'Msgs / sec'
                    }
                }
            },
            animation: {
                duration: 0 // Turn off animation for performance
            },
            plugins: {
                legend: {
                    display: false
                },
                title: {
                    display: true,
                    text: 'CAN Bus Traffic',
                    font: {
                        size: 16
                    }
                }
            }
        }
    });

    // Function to update the chart
    function updateChart() {
        canTrafficChart.data.datasets.forEach((dataset) => {
            dataset.data = latestDataPoints; // Update dataset with the latest data points
        });
        canTrafficChart.update();
    }

    // Update the chart at a frequency that matches your data update frequency
    setInterval(updateChart, 500);
    
    // ################################################################## //
    //                    Querying for Active ROS Topics                  //
    // ################################################################## //

    var excludedTopics = [
        "/client_count",
        "/connected_clients",
        "/parameter_events",
        "/rosout",
    ]
    
    var topicsStats = {} // Dictionary to store stats for each Topic

    var topicTypeClient = new ROSLIB.Service({
        ros : ros,
        name : '/rosapi/topics',
        serviceType : 'rosapi/Topics'
    });

    function updateTopicListAndTypes() {
        var request = new ROSLIB.ServiceRequest(); // Empty request for Topics call
        topicTypeClient.callService(request, function(response) {
            subscribeToTopics(response.topics, response.types);
        });
    }

    function subscribeToTopics(topics, types) {
        // First, mark all topics as inactive
        Object.keys(topicsStats).forEach(function(topic) {
            if (topicsStats[topic]) {
                topicsStats[topic].isActive = false;
            }
        });

        // Iterate through each topic
        for (var i = 0; i < topics.length; i++) {
            var topicName = topics[i];
            var topicType = types[i];

            // Skip topics that are excluded
            if (excludedTopics.includes(topicName)) {
                continue;
            }

            // If not already subscribed, initialize stats and subscribe
            if (!topicsStats[topicName]) {
                topicsStats[topicName] = {
                    topicType: topicType,
                    lastReceived: null,
                    messageCount: 0,
                    messageRate: 0,
                    startTime: Date.now(),
                    isActive: true,
                    subscriber: null // To store the subscriber object
                };
            } else {
                // Mark the topic as active
                topicsStats[topicName].isActive = true;
            }

            // Subscribe to the topic if not already subscribed
            if (!topicsStats[topicName].subscriber) {
                var topicSubscriber = new ROSLIB.Topic({
                    ros: ros,
                    name: topicName,
                    messageType: topicType
                });

                topicSubscriber.subscribe((function(topicName) {
                    return function(message) {
                        var now = new Date().getTime();
                        var stats = topicsStats[topicName];
                        stats.lastReceived = now;
                        stats.messageCount++;
                        var timeElapsed = (now - stats.startTime) / 1000; // Time in seconds
                        stats.messageRate = stats.messageCount / timeElapsed;
                    };
                })(topicName));

                // Store the subscriber in the stats
                topicsStats[topicName].subscriber = topicSubscriber;
            }
        }
    }

    function cleanupInactiveTopics() {
        Object.keys(topicsStats).forEach(function(topic) {
            var stats = topicsStats[topic];
            if (stats && !stats.isActive) {
                // Unsubscribe from the topic
                if (stats.subscriber) {
                    stats.subscriber.unsubscribe();
                }
                // Remove the topic's stats
                delete topicsStats[topic];
            }
        });
    }

    function updateTopicStatsTable() {
        var tableBody = document.getElementById('topic-stats-table-body');
        tableBody.innerHTML = ''; // Clear the table rows

        // Iterate through each topic and create a new row
        Object.keys(topicsStats).forEach(function(topic) {
            var stats = topicsStats[topic];
            if (stats.isActive) {
                var row = tableBody.insertRow();
                var displayTopicName = topic;

                // Topic Name
                var cellTopic = row.insertCell(0);
                cellTopic.textContent = displayTopicName;

                // Last Message Time
                var cellLastMessageTime = row.insertCell(1);
                var lastMessageAgo = stats.lastReceived ? (Date.now() - stats.lastReceived) : 'N/A';
                cellLastMessageTime.textContent = lastMessageAgo;

                // Average Message Rate
                var cellMessageRate = row.insertCell(2);
                var rate = stats.messageRate ? stats.messageRate : 0;
                cellMessageRate.textContent = rate.toFixed(2);

                // Set the color of the message rate
                if (rate >= 20) {
                    cellMessageRate.classList.add('rate-green');
                } else if (rate >= 10 && rate < 20) {
                    cellMessageRate.classList.add('rate-yellow');
                } else {
                    cellMessageRate.classList.add('rate-red');
                }
            }
        });
    }

    // Function to call all functions needed to update the stats table
    function updateStats() {
        updateTopicListAndTypes();
        cleanupInactiveTopics();
        updateTopicStatsTable();
    }

    setInterval(updateStats, 1000); // Periodically update the topic list (rate is in ms)


    // ################################################################## //
    //                      Subscribe to Robot State                      //
    // ################################################################## //

    var robotStateTopic = new ROSLIB.Topic({
        ros : ros,
        name : 'robot_state_topic',
        messageType : 'jugglebot_interfaces/msg/RobotStateMessage'
    });

    robotStateTopic.subscribe(function(message) {
        // If the robot is homed, enable the control mode buttons
        var homeButton = document.getElementById('home-btn');
        var homeButtonIcon = homeButton.querySelector('.button-icon i');

        if (message.is_homed) {
            document.getElementById('spacemouse-control-button').disabled = false;
            document.getElementById('pattern-control-button').disabled = false;
            document.getElementById('jogging-control-button').disabled = false;

            // Change the "home" button, since we no longer need to home the robot
            homeButton.querySelector('.button-text').textContent = 'Center Robot';
            homeButtonIcon.className = 'fa-solid fa-crosshairs';
            homeButton.setAttribute('is-homed', 'true');
        }
        else if (!message.is_homed) {
            // If the robot is not homed, disable the control mode buttons
            document.getElementById('spacemouse-control-button').disabled = true;
            document.getElementById('pattern-control-button').disabled = true;
            document.getElementById('jogging-control-button').disabled = true;

            // Change the "home" button, since we need to home the robot
            homeButton.querySelector('.button-text').textContent = 'Home';
            homeButtonIcon.className = 'fa-solid fa-house-chimney';
            homeButton.setAttribute('is-homed', 'false');
        }
        else {
            console.log(message);
        }
    });

    // ################################################################## //
    //                    Interfacing with Pose Topic                     //
    // ################################################################## //

    var poseTopic = new ROSLIB.Topic({
        ros : ros,
        name : 'platform_pose_topic',
        messageType : 'geometry_msgs/msg/Pose'
    });

    // Initialize current pose to null
    var currentPose = null;

    poseTopic.subscribe(function(message) {
        // Store the current pose
        currentPose = message;
    });

    // Method to adjust the current pose
    function adjustPose(deltaX, deltaY, deltaZ) {
        if (currentPose) {
            currentPose.position.x += deltaX;
            currentPose.position.y += deltaY;
            currentPose.position.z += deltaZ;
        }

        // console.log(currentPose)

        // Publish the new pose
        poseTopic.publish(currentPose);
    }

    // Method to set the current pose
    function setPose(x, y, z) {
        if (currentPose) {
            currentPose.position.x = x;
            currentPose.position.y = y;
            currentPose.position.z = z;
            currentPose.orientation.x = 0; // Just have flat orientation for now
            currentPose.orientation.y = 0;
            currentPose.orientation.z = 0;
            currentPose.orientation.w = 1;
        }
        else {
            // If currentPose is null, construct a new pose message
            currentPose = new ROSLIB.Message({
                position: {
                    x: x,
                    y: y,
                    z: z
                },
                orientation: {
                    x: 0,
                    y: 0,
                    z: 0,
                    w: 1
                }
            });
        }

        // Publish the new pose
        poseTopic.publish(currentPose);
    }

    // ################################################################## //
    //                     Calling a Trigger Service                      //
    // ################################################################## //

    // A few services are Trigger types, so reuse this function
    function callTriggerService(serviceName) {
        var service = new ROSLIB.Service({
            ros : ros,
            name : serviceName,
            serviceType : 'std_srvs/srv/Trigger'
        });

        var request = new ROSLIB.ServiceRequest(); // Empty request for Trigger call

        service.callService(request, function(response) {
            console.log('Received response for service ' + serviceName + ": ", response);
        }, function(error) {
            console.log('Service call failed: ', error);
        });
    }

    // ################################################################## //
    //                        Sending Control Mode                        //
    // ################################################################## //

    function sendControlMode(mode) {
        var cmdTopic = new ROSLIB.Topic({
            ros : ros,
            name : 'control_state_topic',
            messageType : 'std_msgs/msg/String'
        });

        var message = new ROSLIB.Message({
            data : mode
        });

        cmdTopic.publish(message);
    }

    // ################################################################## //
    //                        Define Service Clients                      //
    // ################################################################## //

    var odriveService = new ROSLIB.Service({
        ros : ros,
        name : 'odrive_command',
        serviceType : 'jugglebot_interfaces/srv/ODriveCommandService'
    });

    // ################################################################## //
    //                       Define Service Requests                      //
    // ################################################################## //

    var rebootODrivesRequest = new ROSLIB.ServiceRequest({
        command : 'reboot_odrives'
    });

    var clearODriveErrorsRequest = new ROSLIB.ServiceRequest({
        command : 'clear_errors'
    });

    // ################################################################## //
    //                           Button Listeners                         //
    // ################################################################## //

    document.getElementById('home-btn').addEventListener('click', function() {
        var isHomed = this.getAttribute('is-homed');

        if (isHomed === 'true') {
            // If the robot is homed, send a command to center the robot
            setPose(0, 0, 140);
        }
        else if (isHomed === 'false') {
            // If the robot is not homed, send a command to home the robot
            callTriggerService('/home_robot');

            // Also send a pose command so the topic isn't empty
            setPose(0, 0, 0);
        }
        else {
            console.log('Invalid isHomed value: ', isHomed);
        }
    });

    document.getElementById('end-session-btn').addEventListener('click', function() {
        callTriggerService('/end_session');
    });

    document.getElementById('odrive-reboot-btn').addEventListener('click', function() {
        odriveService.callService(rebootODrivesRequest, function(response) {
            console.log('Received response: ', response);
        }, function(error) {
            console.log('ODrive reboot service call failed: ', error);
        });
    });

    document.getElementById('odrive-clear-errors-btn').addEventListener('click', function() {
        odriveService.callService(clearODriveErrorsRequest, function(response) {
            console.log('Received response: ', response);
        }, function(error) {
            console.log('ODrive clear errors service call failed: ', error);
        });
    });

    // Define an array of control modes
    const controlModes = ['spacemouse', 'jogging', 'pattern'];

    // Initialize the control state to null
    var control_state = null;

    // Define a function to handle control button clicks
    function handleControlButtonClick(mode) {
        return function() {
            if (this.disabled || control_state === mode) {
                return; // Do nothing if button is disabled or already in this control mode
            }

            // Switch to the selected control mode
            control_state = mode;
            this.style.backgroundColor = '#4CAF50'; // Green for active control mode

            // Send command to ROS
            sendControlMode(mode);

            // Grey-out the other buttons
            controlModes.forEach(otherMode => {
                if (otherMode !== mode) {
                    document.getElementById(`${otherMode}-control-button`).style.backgroundColor = '#aaaaaa'; // AHHHH!
                }
            });

            // If selected mode is not jogging, disable the jogging buttons
            if (mode !== 'jogging') {
                setJoggingButtonsEnabled(false);
            }
            else {
                setJoggingButtonsEnabled(true);
            }
        };
    }

    // Attach the event listeners to the control mode buttons (Jogging, Spacemouse, Pattern etc) 
    controlModes.forEach(mode => {
        document.getElementById(`${mode}-control-button`).addEventListener('click', handleControlButtonClick(mode));
    });
    
    // To clear the console
    document.getElementById('clear-console-btn').addEventListener('click', function() {
        var consoleDiv = document.getElementById('ros-console');
        consoleDiv.innerHTML = '';
    });
    
    // For jogging buttons
    var amountToJogBy = 10.0 // mm

    function setJoggingButtonsEnabled(enabled) {
        var joggingButtons = document.querySelectorAll('#jogging-buttons button');
        joggingButtons.forEach(button => {
            button.disabled = !enabled;
        });
    }

    document.getElementById('jog-x-pos').addEventListener('click', function() {adjustPose(amountToJogBy, 0, 0);});
    document.getElementById('jog-x-neg').addEventListener('click', function() {adjustPose(-amountToJogBy, 0, 0);});
    document.getElementById('jog-y-pos').addEventListener('click', function() {adjustPose(0, amountToJogBy, 0);});
    document.getElementById('jog-y-neg').addEventListener('click', function() {adjustPose(0, -amountToJogBy, 0);});
    document.getElementById('jog-z-pos').addEventListener('click', function() {adjustPose(0, 0, amountToJogBy);});
    document.getElementById('jog-z-neg').addEventListener('click', function() {adjustPose(0, 0, -amountToJogBy);});

    // For the scene menu buttons
    document.querySelectorAll('.scene-menu-item').forEach(item => {
        item.addEventListener('click', function() {
            toggleObject(this.dataset.object);
        });
    });

    // ################################################################## //
    //                           Slider Listeners                         //
    // ################################################################## //

    var slidersTopic = new ROSLIB.Topic({
        ros : ros,
        name : 'pattern_control_topic',
        messageType : 'jugglebot_interfaces/msg/PatternControlMessage'
    });

    function publishSliderValues() {
        var message = new ROSLIB.Message({
            radius: Number(document.getElementById('slider-radius').value),
            freq: Number(document.getElementById('slider-freq').value),
            z_offset: Number(document.getElementById('slider-z-offset').value),
            cone_height: Number(document.getElementById('slider-cone-height').value)
        });

        slidersTopic.publish(message);
    }

    // Add event listeners to the sliders
    ['slider-radius', 'slider-freq', 'slider-z-offset', 'slider-cone-height'].forEach(sliderId => {
        document.getElementById(sliderId).addEventListener('input', function() {
            var sliderValue = document.getElementById(`${sliderId}-value`);
            sliderValue.textContent = this.value;
            publishSliderValues();
        });
    });

    // ################################################################## //
    //                            3D Scene Menu                           //
    // ################################################################## //

    // Add an event listener to ensure the menu buttons are only initialized after the full scene has been loaded
    document.addEventListener('scene-loaded', initializeSceneMenuButtons);

    // Initialize the menu buttons based on their 'active/inactive' state
    function initializeSceneMenuButtons() {
        document.querySelectorAll('.scene-menu-item').forEach(button => {
            // var objectName = button.dataset.object;
            const objectName = button.getAttribute('data-object');
            const object = scene.getObjectByName(objectName);
            if (object) {
                // If the corresponding button has been set to 'inactive', hide the object
                if (button.classList.contains('inactive')) {
                    object.visible = false;
                }

                const icon = button.querySelector('.visibility-icon i');
                if (icon) {
                    if (object.visible) {
                        icon.className = 'fa-regular fa-circle-check';
                        button.classList.add('active');
                        button.classList.remove('inactive');
                    } else {
                        icon.className = 'fa-regular fa-circle-xmark';
                        button.classList.add('inactive');
                        button.classList.remove('active');
                    }
                }
            }
        });
    }
    
    // Toggle the visibility of the menu when the menu button is clicked
    document.getElementById('scene-menu-toggle-btn').addEventListener('click', function() {
        var menuContent = document.getElementById('scene-menu-content');
        if (menuContent.style.display === 'none' || menuContent.style.display === '') {
            menuContent.style.display = 'block';
        } else {
            menuContent.style.display = 'none';
        }
    });

    // Function to toggle visibility of an object in the scene
    function toggleObject(objectName) {
        var object = scene.getObjectByName(objectName);
        
        if (object) {
            object.visible = !object.visible;

            // Update button class and icon based on visibility
            var button = document.querySelector(`.scene-menu-item[data-object="${objectName}"]`);
            if (button) {
                const icon = button.querySelector('.visibility-icon i');
                if (icon) {
                    if (object.visible) {
                        icon.className = 'fa-regular fa-circle-check';
                        // Change the state of the button to active
                        button.classList.add('active');
                        button.classList.remove('inactive');
                    } else {
                        icon.className = 'fa-regular fa-circle-xmark';
                        // Change the state of the button to inactive
                        button.classList.add('inactive');
                        button.classList.remove('active');
                    }
                }
            }
        }
    }
};