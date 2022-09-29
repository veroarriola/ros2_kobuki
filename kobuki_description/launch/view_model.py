import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue

def generate_launch_description():
    rviz_config = os.path.join(
            get_package_share_directory('kobuki_description'),
            'rviz',
            'model.rviz'
        )
    path_to_urdf = os.path.join(
            get_package_share_directory('kobuki_description'),
            'urdf',
            'kobuki_standalone.urdf'
        )
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            arguments=[path_to_urdf,]
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config]
        )
    ])
