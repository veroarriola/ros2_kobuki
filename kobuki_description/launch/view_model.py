# from lauch_ros.substitutions import FindPackageShare, PathJoinSubstitution
import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue

#                PathJoinSubstitution([
#                    FindPackageShare('kobuki_description'),
#                    'example_substitutions.launch.py'
#                ])

def generate_launch_description():
    rviz_config = os.path.join(
            get_package_share_directory('kobuki_description'),
            'rviz',
            'model.rviz'
        )
    path_to_xacro = os.path.join(
            get_package_share_directory('kobuki_description'),
            'urdf',
            'kobuki_standalone.urdf.xacro'
        )
    print("+++++++++++++", rviz_config)
    print("*************", path_to_xacro)
    #get_package_share_path('kobuki_description') / 'urdf' / 'kobuki_standalone.urdf.xacro'
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': ParameterValue(
                    Command(['xacro', str(path_to_xacro)]),
                    value_type=str
                )
            }]
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
            #arguments=['-d', rviz_config]
        )
    ])
