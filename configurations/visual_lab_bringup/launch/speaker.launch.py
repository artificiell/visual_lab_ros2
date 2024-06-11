#!/usr/bin/env python3
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # Launch configuration
    speaker_channels = LaunchConfiguration('speaker_channels')

    # Launch arguments
    speaker_channels_arg = DeclareLaunchArgument(
        'speaker_channels',
        default_value = '2'
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
        speaker_channels_arg,
        player_node
    ])
