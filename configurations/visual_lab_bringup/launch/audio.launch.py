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

    # Launch arguments
    microphone_channels_arg = DeclareLaunchArgument(
        'microphone_channels',
        default_value = '2'
    )
    speaker_channels_arg = DeclareLaunchArgument(
        'speaker_channels',
        default_value = '2'
    )

    # Include microphone launch
    microphone_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('visual_lab_bringup'),
                'launch',
                'microphone.launch.py'
            ])
        ]),
        launch_arguments = {
            'microphone_channels': LaunchConfiguration('microphone_channels')
        }.items()
    )

    # Include speaker launch
    speaker_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('visual_lab_bringup'),
                'launch',
                'speaker.launch.py'
            ])
        ]),
        launch_arguments = {
            'speaker_channels': LaunchConfiguration('speaker_channels')
        }.items()
    )

    # Lanch the description
    return LaunchDescription([
        microphone_channels_arg,
        speaker_channels_arg,
        microphone_launch,
        speaker_launch
    ])
