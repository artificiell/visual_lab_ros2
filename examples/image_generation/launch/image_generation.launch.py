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
    display_width = LaunchConfiguration('display_width')
    scale_factor = LaunchConfiguration('scale_factor')

    # Launch arguments
    microphone_channels_arg = DeclareLaunchArgument(
        'microphone_channels',
        default_value = '2'
    )
    display_width_arg = DeclareLaunchArgument(
        'display_width',
        default_value = '5760'
    )
    scale_factor_arg = DeclareLaunchArgument(
        'scale_factor',
        default_value = '1.0'
    )
    
    # Include speech-to-text launch
    speech_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('visual_lab_bringup'),
                'launch',
                'speech_to_text.launch.py'
            ])
        ]),
        launch_arguments = {
            'microphone_channels': LaunchConfiguration('microphone_channels')
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
            'fullscreen': 'True'
        }.items()
    )

    # Image generation node
    generator_node = Node(
        package = 'image_generation',
        executable = 'generator',
        name = 'image_generator_node',
        parameters=[{
            'screen_width': LaunchConfiguration('display_width'),
            'scale_factor': LaunchConfiguration('scale_factor')
        }]
    )

    # Lanch the description
    return LaunchDescription([
        microphone_channels_arg,
        display_width_arg,
        scale_factor_arg,
        speech_launch,
        screen_launch,
        generator_node
    ])
