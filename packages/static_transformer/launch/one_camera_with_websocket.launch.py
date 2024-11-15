#!/usr/bin/env python3
from launch_ros.actions import Node
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

def generate_launch_description():

    # Launch configuration
    zed_serial_number = LaunchConfiguration('zed_serial_number')
    zed_node_name = LaunchConfiguration('zed_node_name')
        
    # Launch arguments
    zed_serial_number_arg = DeclareLaunchArgument(
        'zed_serial_number',
        default_value = '37817095',
        description = 'ZED camera serial number (38160741, 37817095, 36374190, or 36240144)'
        
    )
    zed_node_name_arg = DeclareLaunchArgument(
        'zed_node_name',
        default_value = 'stair',
        description = 'ZED node name (used for the namespace of camera topics)'

    )

    # Transformer launch (including camera)
    transformer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('static_transformer'),
                'launch',
                'transformer.launch.py'
            ])
        ]),
        launch_arguments = {
            'zed_serial_number': LaunchConfiguration('zed_serial_number'),
            'zed_node_name': LaunchConfiguration('zed_node_name')
        }.items()
    )

    # Websocket launch
    websocket_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('rosbridge_server'),
                'launch',
                'rosbridge_websocket_launch.xml'
            ])
        ])
    )
    
    # Lanch the description
    return LaunchDescription([
        zed_serial_number_arg,
        zed_node_name_arg,
        transformer_launch,
        websocket_launch
    ])
