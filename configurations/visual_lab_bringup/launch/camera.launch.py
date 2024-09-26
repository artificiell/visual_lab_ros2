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
    zed_publish_tf = LaunchConfiguration('zed_publish_tf')
    zed_publish_urdf = LaunchConfiguration('zed_publish_urdf')

    # Launch arguments
    zed_camera_model_arg = DeclareLaunchArgument(
        'zed_camera_model',
        default_value = 'zed2i'
    )
    zed_publish_tf_arg = DeclareLaunchArgument(
        'zed_publish_tf',
        default_value = 'false'
    )
    zed_publish_urdf_arg = DeclareLaunchArgument(
        'zed_publish_urdf',
        default_value = 'true'
    )
    
    # ZED camera launch
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('zed_wrapper'),
                'launch',
                'zed_camera.launch.py'
            ])
        ]),
        launch_arguments = {
            'camera_model': LaunchConfiguration('zed_camera_model'),
            'publish_tf': LaunchConfiguration('zed_publish_tf'),
            'publish_urdf': LaunchConfiguration('zed_publish_urdf')
        }.items()
    )

    # Lanch the description
    return LaunchDescription([
        zed_camera_model_arg,
        zed_publish_tf_arg,
        zed_publish_urdf_arg,
        camera_launch
    ])
