from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
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
        Node(
            package='jugglebot',
            namespace='jugglebot',
            executable='sp_ik',
            name='sp_ik'
        ),
        Node(
            package='jugglebot',
            namespace='jugglebot',
            executable='state_manager_node',
            name='state_manager_node'
        ),
    ])