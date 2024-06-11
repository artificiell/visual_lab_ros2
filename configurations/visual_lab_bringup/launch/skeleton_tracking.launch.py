#!/usr/bin/env python3
from launch_ros.actions import Node
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():

    # Launch configuration
    zed_camera_model = LaunchConfiguration('zed_camera_model')

    # Launch arguments
    zed_camera_model_arg = DeclareLaunchArgument(
        'zed_camera_model',
        default_value = 'zed2i'
    )
    
    # Include camera launch
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('visual_lab_bringup'),
                'launch',
                'camera.launch.py'
            ])
        ]),
        launch_arguments = {
            'camera_model': LaunchConfiguration('zed_camera_model')
        }.items()
    )

    # Tracking node
    tracking_node = Node(
        package = 'visual_features',
        executable = 'tracking',
        name = 'tracking_node'
    )
    
    # Lanch the description
    return LaunchDescription([
        zed_camera_model_arg,
        camera_launch,
        tracking_node
    ])
