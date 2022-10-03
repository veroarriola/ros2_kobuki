import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    path_to_urdf = os.path.join(
            get_package_share_directory('kobuki_description'),
            'urdf',
            'kobuki_standalone.urdf'
        )
    with open(path_to_urdf, 'r') as f:
        robot_desc = f.read()

    return LaunchDescription([
        Node(
            package='kobuki_softnode',
            executable='kobuki_softnode'
        ),
        Node(
            package='diagnostic_aggregator',
            executable='diagnostic_aggregator',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_desc,
                'publish_frequency': 30.0,
            }]
        ),
    ]) 
