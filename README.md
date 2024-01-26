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

Use `jugglebot_build` to build the jugglebot ROS2 workspace natively. Use `jugglebot_build_docker` to build the same inside a docker container.

### Dependencies

Dependencies are managed using rosdep. Ensure you have a working rosdep setup using the following commands.

```bash
sudo rosdep init
rosdep update
```
