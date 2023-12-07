from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='jugglebot',
            namespace='jugglebot',
            executable='spacemouse_handler',
            name='spacemouse_handler'
        ),
        Node(
            package='jugglebot',
            namespace='jugglebot',
            executable='robot_geometry',
            name='robot_geometry'
        ),
        Node(
            package='jugglebot',
            namespace='jugglebot',
            executable='platform_plotter',
            name='platform_plotter'
        ),
    ])