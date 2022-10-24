import os
#from symbol import parameters
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

import yaml

def generate_launch_description():
    path_to_urdf = os.path.join(
        get_package_share_directory('kobuki_description'),
        'urdf',
        'kobuki_standalone.urdf'
    )
    with open(path_to_urdf, 'r') as f:
        robot_desc = f.read()
    path_to_base = os.path.join(
        get_package_share_directory('kobuki_softnode'),
        'param',
        'base.yaml'
    )
    with open(path_to_base, 'r') as f:
        base_params = yaml.safe_load(f)
    # path_to_diagnostics = os.path.join(
    #     get_package_share_directory('kobuki_softnode'),
    #     'param',
    #     'diagnostics.yaml'
    # )
    # with open(path_to_diagnostics, 'r') as f:
    #     diagnostics_params = yaml.safe_load(f)
    #     print(diagnostics_params)
    rviz_config = os.path.join(
        get_package_share_directory('kobuki_description'),
        'rviz',
        'kobuki.rviz'
    )

    return LaunchDescription([
        Node(
            package='kobuki_softnode',
            executable='kobuki_softnode',
            name='mobile_base',
            parameters=[base_params],
            remappings=[
                ('mobile_base/odom', 'odom'),
                ('mobile_base/joint_states', 'joint_states'),
            ]
        ),
        # Node(
        #     package='diagnostic_aggregator',
        #     executable='aggregator_node',
        #     name='diagnostic_aggregator',
        #     parameters=[path_to_diagnostics]
        # ),
        Node(
            package='robot_state_publisher',
            name='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_desc,
                'publish_frequency': 30.0,
            }]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config]
        )
    ]) 
