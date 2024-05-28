#!/usr/bin/env python3
from launch_ros.actions import Node
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():

    # Launch configuration
    microphone_channels = LaunchConfiguration('microphone_channels')
    speaker_channels = LaunchConfiguration('speaker_channels')
    display_width = LaunchConfiguration('display_width')
    display_height = LaunchConfiguration('display_height')
    zed_camera_model = LaunchConfiguration('microphone_channels')

    # Launch arguments
    microphone_channels_arg = DeclareLaunchArgument(
        'microphone_channels',
        default_value = '2'
    )
    speaker_channels_arg = DeclareLaunchArgument(
        'speaker_channels',
        default_value = '2'
    )
    display_width_arg = DeclareLaunchArgument(
        'display_width',
        default_value = '1920'
    )
    display_height_arg = DeclareLaunchArgument(
        'display_height',
        default_value = '1200'
    )
    zed_camera_model_arg = DeclareLaunchArgument(
        'zed_camera_model',
        default_value = 'zed2i'
    )
    
    # Launch audio
    audio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('visual_lab_bringup'),
                'launch',
                'audio.launch.py'
            ])
        ]),
        launch_arguments = {
            'microphone_channels': LaunchConfiguration('microphone_channels'),
            'speaker_channels': LaunchConfiguration('speaker_channels')
        }.items()
    )
    
    # Launch visual
    visual_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('visual_lab_bringup'),
                'launch',
                'visual.launch.py'
            ])
        ]),
        launch_arguments = {
            'display_width': LaunchConfiguration('display_width'),
            'display_height': LaunchConfiguration('display_height'),
            'zed_camera_model': LaunchConfiguration('zed_camera_model')
        }.items()
    )

    # Lanch the description
    return LaunchDescription([
        microphone_channels_arg,
        speaker_channels_arg,
        display_width_arg,
        display_height_arg,
        zed_camera_model_arg,
        audio_launch,
        visual_launch
    ])
