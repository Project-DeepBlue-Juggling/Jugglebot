from launch import LaunchDescription
from launch_ros.actions import Node
# from launch.actions import ExecuteProcess, IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
# from ament_index_python.packages import get_package_share_directory
# import os

def generate_launch_description():
    nodes = [
        'robot_geometry',
        'spacemouse_handler',
        'platform_plotter',
        'sp_ik',
        'can_bus_handler_node',
        'state_manager_node',
        'pattern_creator',
        # 'statistics_collector_node',  # Not using this for now. Unsure of its utility.
    ]

    # # Path to the rosbridge_websocket_launch.xml file
    # rosbridge_launch_file_path = os.path.join(
    #     get_package_share_directory('rosbridge_server'),
    #     'launch',
    #     'rosbridge_websocket_launch.xml'
    # )

    return LaunchDescription([
        # # Begin by starting the server that serves the http files to the browser
        # ExecuteProcess(
        #     cmd=['http-server', '.'],
        #     shell=True,
        #     output='screen',
        #     emulate_tty=True
        # ),

        # # Now start the rosbridge websocket to allow the ROS network to communicate with the GUI
        # IncludeLaunchDescription(
        #     AnyLaunchDescriptionSource(rosbridge_launch_file_path)
        # ),

        # Finally, start the ROS network itself
        *[
            Node(
                package='jugglebot',
                # namespace='jugglebot',
                executable=node_name,
                # name=node_name,
                # arguments=['--ros-args', '--log-level', 'debug']
            ) for node_name in nodes
        ]
    ])