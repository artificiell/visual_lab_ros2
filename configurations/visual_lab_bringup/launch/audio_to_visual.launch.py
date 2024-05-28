#!/usr/bin/env python3
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # Launch configuration
    microphone_channels = LaunchConfiguration('microphone_channels')
    display_width = LaunchConfiguration('display_width')

    # Launch arguments
    microphone_channels_arg = DeclareLaunchArgument(
        'microphone_channels',
        default_value = '2'
    )
    display_width_arg = DeclareLaunchArgument(
        'display_width',
        default_value = '5760'
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

    # Visual display node
    display_node = Node(
        package = 'visual_display',
        executable = 'visualize',
        name = 'visual_display_node',
        parameters=[{
            'width': LaunchConfiguration('display_width'),
            'fullscreen': True
        }]
    )

    # Audio visualizer node
    visualizer_node = Node(
        package = 'audio_to_visual',
        executable = 'visualizer',
        name = 'audio_visualizer_node',
        parameters=[{
            'width': LaunchConfiguration('display_width'),
            'channels': LaunchConfiguration('microphone_channels')
        }]
    )

    # Lanch the description
    return LaunchDescription([
        microphone_channels_arg,
        display_width_arg,
        recorder_node,
        display_node,
        visualizer_node
    ])
