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
    transcribe_model = LaunchConfiguration('transcribe_model')
    language = LaunchConfiguration('transcribe_model')
    
    # Launch arguments
    microphone_channels_arg = DeclareLaunchArgument(
        'microphone_channels',
        default_value = '2'
    )
    transcribe_model_arg = DeclareLaunchArgument(
        'transcribe_model',
        default_value = 'whisper'
    )
    language_arg = DeclareLaunchArgument(
        'language',
        default_value = 'en'
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

    # Speech-to-text node
    transcribe_node = Node(
        package = 'speech_to_text',
        executable = 'transcribe',
        name = 'speech_transcribe_node',
        parameters=[{
            'model': LaunchConfiguration('transcribe_model'),
            'channels': LaunchConfiguration('microphone_channels'),
            'lang': LaunchConfiguration('language')
        }]
    )

    # Lanch the description
    return LaunchDescription([
        microphone_channels_arg,
        transcribe_model_arg,
        language_arg,
        microphone_launch,
        transcribe_node
    ])
