#!/usr/bin/env python3
from launch_ros.actions import Node
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():

    # Launch configuration
    zed_node_name = LaunchConfiguration('zed_node_name')
    zed_serial_number = LaunchConfiguration('zed_serial_number')

    # Launch arguments
    zed_node_name_arg = DeclareLaunchArgument(
        'zed_node_name',
        default_value = 'camera',
        description = 'ZED node name (used for the namespace of camera topics)'
        
    )
    zed_serial_number_arg = DeclareLaunchArgument(
        'zed_serial_number',
        default_value = '37817095',
        description = 'ZED camera serial number (38160741, 37817095, 36374190, or 36240144)'
        
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
            'zed_node_name': LaunchConfiguration('zed_camera_model'),
            'zed_serial_number': LaunchConfiguration('zed_serial_number')
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
        zed_node_name_arg,
        zed_serial_number_arg,        
        camera_launch,
        tracking_node
    ])
