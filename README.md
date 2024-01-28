# Jugglebot
For all CAD and code related to Project: DeepBlue Juggling

I'll try to release an updated folder here for every episode in which major changes happen.

For further information about the project as a whole, as well as more detail on specific elements of the project, please head to:
pdj.zulipchat.com

## Developer Setup

### Scripts

Add the scripts directory to your path so you can run them from anywhere by running the following command in the root of the Jugglebot cloned directory.

```bash
echo "export PATH=$(pwd)/scripts:\$PATH" >> ~/.bashrc
```

You can also source the install directory if you'd like to have access to the ROS2 packages automatically.

```bash
echo "source $(pwd)/ros_ws/install/setup.bash" >> ~/.bashrc
```

Use `jugglebot_build` to build the jugglebot ROS2 workspace natively. Use `jugglebot_build_docker` to build the same inside a docker container. Use `jugglebot_clean` to clean up the install, build and logs directories.

### GUI

The GUI runs on a web server. This can be maually executed by running `http-server` in the `ros_ws/gui` directory, or a background server can be activated using `jugglebot_enable_server_service`. This only has to be run once and will start the server on startup and keep it alive.

### Running Jugglebot

The main Jugglebot launch file can be executed using:

```bash
ros2 launch jugglebot jugglebot_launch.py
```


### Simulator

There is a Webots simulator to simulate Jugglebot's juggling dynamics. Start the Jugglebot stack with the simulator enabled using:

```bash
ros2 launch jugglebot jugglebot_launch.py use_simulator:=true
```

### Dependencies

#### Manual Dependencies

Install the latest ROS2 distribution compatible with your hardware using the official docs [here](https://docs.ros.org/en/iron/Releases.html).

Install npm using the relevant tutorial for your system [here](https://nodejs.org/en/download/package-manager).

#### Automated Dependencies

Package dependencies are managed using apt, pip, rosdep and npm.

Dependencies can be installed using the `install_jugglebot_dependencies` script.

# TODO

[ ] - Convert `leg_lengths_topic` to use metres for intercompatibility between robots
[ ] - Change all units to SI units
