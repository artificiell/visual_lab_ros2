#!/usr/bin/env python3
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # Launch configuration
    microphone_channels = LaunchConfiguration('microphone_channels')

    # Launch arguments
    microphone_channels_arg = DeclareLaunchArgument(
        'microphone_channels',
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

    # Lanch the description
    return LaunchDescription([
        microphone_channels_arg,
        recorder_node,
    ])
