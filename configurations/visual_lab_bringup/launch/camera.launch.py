#!/usr/bin/env python3
from launch_ros.actions import Node
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():

    # Launch configuration
    zed_camera_name = LaunchConfiguration('zed_camera_name')
    zed_camera_model = LaunchConfiguration('zed_camera_model')
    zed_serial_number = LaunchConfiguration('zed_serial_number')
    zed_publish_tf = LaunchConfiguration('zed_publish_tf')
    zed_publish_urdf = LaunchConfiguration('zed_publish_urdf')

    # Launch arguments
    zed_camera_name_arg = DeclareLaunchArgument(
        'zed_camera_name',
        default_value = 'zed',
        description = 'ZED camera name (used for the namespace of camera topics)'
        
    )
    zed_camera_model_arg = DeclareLaunchArgument(
        'zed_camera_model',
        default_value = 'zed2i',
        description = 'ZED camera model (zed2i, zed2, zed, or zedm)'
        
    )
    zed_serial_number_arg = DeclareLaunchArgument(
        'zed_serial_number',
        default_value = '37817095',
        description = 'ZED camera serial number (38160741, 37817095, 36374190, or 36240144)'
        
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
            'camera_name': LaunchConfiguration('zed_camera_name'),
            'camera_model': LaunchConfiguration('zed_camera_model'),
            'serial_number': LaunchConfiguration('zed_serial_number'),
            'publish_tf': LaunchConfiguration('zed_publish_tf'),
            'publish_urdf': LaunchConfiguration('zed_publish_urdf')
        }.items()
    )

    # Lanch the description
    return LaunchDescription([
        zed_camera_name_arg,
        zed_camera_model_arg,
        zed_serial_number_arg,
        zed_publish_tf_arg,
        zed_publish_urdf_arg,
        camera_launch
    ])
