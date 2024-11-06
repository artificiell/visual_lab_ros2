#!/usr/bin/env python3
from launch_ros.actions import Node
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():

    # Launch configuration
    display_width = LaunchConfiguration('display_width')
    display_height = LaunchConfiguration('display_height')
    display_fullscreen = LaunchConfiguration('display_fullscreen')
    zed_node_name = LaunchConfiguration('zed_node_name')
    zed_serial_number = LaunchConfiguration('zed_serial_number')

    # Launch arguments
    display_width_arg = DeclareLaunchArgument(
        'display_width',
        default_value = '5700'
    )
    display_height_arg = DeclareLaunchArgument(
        'display_height',
        default_value = '1200'
    )
    display_fullscreen_arg = DeclareLaunchArgument(
        'display_fullscreen',
        default_value = 'True'
    )
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
    
    # Include screen launch
    screen_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('visual_lab_bringup'),
                'launch',
                'screen.launch.py'
            ])
        ]),
        launch_arguments = {
            'width': LaunchConfiguration('display_width'),
            'height': LaunchConfiguration('display_height'),
            'fullscreen': LaunchConfiguration('display_fullscreen')
        }.items()
    )

    # Lanch the description
    return LaunchDescription([
        display_width_arg,
        display_height_arg,
        display_fullscreen_arg,
        zed_node_name_arg,
        zed_serial_number_arg,        
        camera_launch,
        screen_launch
    ])
