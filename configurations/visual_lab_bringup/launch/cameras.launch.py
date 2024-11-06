#!/usr/bin/env python3
from launch_ros.actions import Node
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution

def generate_launch_description():

    # Define a dict for camera node names
    node_names = {
        '38160741': 'back',
        '37817095': 'stair',
        '36374190': 'centre',
        '36240144': 'entrance'
    }
    
    # Launch configuration
    zed_camera_name = LaunchConfiguration('zed_camera_name')
    zed_camera_model = LaunchConfiguration('zed_camera_model')

    # Launch arguments
    zed_camera_name_arg = DeclareLaunchArgument(
        'zed_camera_name',
        default_value = 'zed',
        description = 'ZED camera name (used as prefix for the namespace of camera topics)'
        
    )
    zed_camera_model_arg = DeclareLaunchArgument(
        'zed_camera_model',
        default_value = 'zed2i',
        description = 'ZED camera model (zed2i, zed2, zed, or zedm)'
        
    )
    
    # ZED camera launch
    description = [zed_camera_name_arg, zed_camera_model_arg]
    for serial_number in ['38160741', '37817095', '36374190', '36240144']:
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
                'node_name': TextSubstitution(text = str(node_names[serial_number])),
                'serial_number': TextSubstitution(text = str(serial_number))
            }.items()
        )
        description.append(camera_launch)
        
    # Lanch the description
    return LaunchDescription(description)
