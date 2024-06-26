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
    zed_camera_model = LaunchConfiguration('zed_camera_model')

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
        zed_camera_model_arg,
        camera_launch,
        screen_launch
    ])
