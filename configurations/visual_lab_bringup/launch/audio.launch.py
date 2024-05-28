#!/usr/bin/env python3
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

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
    
    # Audio recorder node
    recorder_node = Node(
        package = 'audio_recorder',
        executable = 'record',
        name = 'audio_recorder_node',
        parameters=[{
            'channels': LaunchConfiguration('microphone_channels')
        }]
    )

    # Audio player node
    player_node = Node(
        package = 'audio_player',
        executable = 'playback',
        name = 'audio_playback_node',
        parameters=[{
            'channels': LaunchConfiguration('speaker_channels')
        }]
    )

    # Lanch the description
    return LaunchDescription([
        microphone_channels_arg,
        speaker_channels_arg,
        recorder_node,
        player_node
    ])
