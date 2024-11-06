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
        
    # Include camera launches
    description = []
    for serial_number in ['38160741', '37817095', '36374190', '36240144']:
        camera_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('visual_lab_bringup'),
                    'launch',
                    'camera.launch.py'
                ])
            ]),
            launch_arguments = {
                'zed_serial_number': TextSubstitution(text = str(serial_number)),
                'zed_node_name': TextSubstitution(text = str(node_names[serial_number]))
            }.items()
        )
        description.append(camera_launch)
        
    # Lanch the description
    return LaunchDescription(description)
