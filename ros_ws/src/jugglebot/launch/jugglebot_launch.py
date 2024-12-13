from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    use_simulator = LaunchConfiguration('use_simulator')
    use_simulator_arg = DeclareLaunchArgument(
        'use_simulator',
        default_value='false'
    )

    simulator_launch_file_path = os.path.join(
        get_package_share_directory('jugglebot_simulator'),
        'launch',
        'launch.py'
    )
    simulator_include_description = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(simulator_launch_file_path),
        condition=IfCondition(use_simulator)
    )

    can_bus_node = Node(
        package="jugglebot",
        executable="can_interface_node",
        condition=UnlessCondition(use_simulator)
    )


    jugglebot_node_names = [
        'yasmin_state_machine',
        'spacemouse_handler',
        'sp_ik',
        'robot_geometry',
        'mocap_interface_node',
        'ball_prediction_node',
    ]
    jugglebot_nodes = [
        Node(
            package='jugglebot',
            # namespace='jugglebot',
            executable=node_name,
            # name=node_name,
            # arguments=['--ros-args', '--log-level', 'debug']
            # parameters=[{'use_sim_time': True}],
        ) for node_name in jugglebot_node_names
    ]


    rosbridge_launch_file_path = os.path.join(
        get_package_share_directory('rosbridge_server'),
        'launch',
        'rosbridge_websocket_launch.xml'
    )
    rosbridge_include_description = IncludeLaunchDescription(AnyLaunchDescriptionSource(rosbridge_launch_file_path))


    return LaunchDescription([
        use_simulator_arg,
        rosbridge_include_description,
        simulator_include_description,
        can_bus_node,
        *jugglebot_nodes,
    ])
