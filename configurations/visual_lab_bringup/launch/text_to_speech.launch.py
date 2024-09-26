#!/usr/bin/env python3
from launch_ros.actions import Node
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():

    # Launch configuration
    speaker_channels = LaunchConfiguration('speaker_channels')
    speech_language = LaunchConfiguration('speech_language')
    speech_pitch = LaunchConfiguration('speech_pitch')
    speech_rate = LaunchConfiguration('speech_rate')

    # Launch arguments
    speaker_channels_arg = DeclareLaunchArgument(
        'speaker_channels',
        default_value = '2'
    )
    speech_language_arg = DeclareLaunchArgument(
        'speech_language',
        default_value = 'english'
    )
    speech_pitch_arg = DeclareLaunchArgument(
        'speech_pitch',
        default_value = '75'
    )
    speech_rate_arg = DeclareLaunchArgument(
        'speech_rate',
        default_value = '150'
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

    # Text-to-speech node
    speech_node = Node(
        package = 'text_to_speech',
        executable = 'speech',
        name = 'text_sspeech_node',
        parameters=[{
            'channels': LaunchConfiguration('speaker_channels'),
            'voice': LaunchConfiguration('speech_language'),
            'pitch': LaunchConfiguration('speech_pitch'),
            'rate': LaunchConfiguration('speech_rate')
        }]
    )
    
    # Keyword extraction node
    keyword_node = Node(
        package = 'text_to_speech',
        executable = 'keyword',
        name = 'keyword_node', 

    )

    # Lanch the description
    return LaunchDescription([
        speaker_channels_arg,
        speech_language_arg,
        speech_pitch_arg,
        speech_rate_arg,
        speaker_launch,
        speech_node,
        keyword_node
    ])
