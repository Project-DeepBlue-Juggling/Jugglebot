from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='jugglebot',
            namespace='jugglebot',
            executable='spacemouse_handler',
            name='spacemouse_handler',
            arguments=['--ros-args', '--log-level', 'debug']
        ),
        Node(
            package='jugglebot',
            namespace='jugglebot',
            executable='robot_geometry',
            name='robot_geometry',
            arguments=['--ros-args', '--log-level', 'debug']
        ),
        Node(
            package='jugglebot',
            namespace='jugglebot',
            executable='platform_plotter',
            name='platform_plotter',
            arguments=['--ros-args', '--log-level', 'debug']
        ),
        Node(
            package='jugglebot',
            namespace='jugglebot',
            executable='sp_ik',
            name='sp_ik',
            arguments=['--ros-args', '--log-level', 'debug']
        ),
        Node(
            package='jugglebot',
            namespace='jugglebot',
            executable='can_bus_handler_node',
            name='can_bus_handler_node',
            arguments=['--ros-args', '--log-level', 'debug']
        ),
        Node(
            package='jugglebot',
            namespace='jugglebot',
            executable='state_manager_node',
            name='state_manager_node',
            arguments=['--ros-args', '--log-level', 'debug']
        ),
    ])