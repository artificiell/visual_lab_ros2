#!/usr/bin/env python3
from launch_ros.actions import Node
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():

    # Launch configuration
    model = LaunchConfiguration('generation_model_size')
    tokens = LaunchConfiguration('max_generation_tokens')

    # Launch arguments
    model_arg = DeclareLaunchArgument(
        'generation_model_size',
        default_value = '8B'
    )
    tokens_arg = DeclareLaunchArgument(
        'max_generation_tokens',
        default_value = '8192'
    )

    # Text generation node
    generation_node = Node(
        package = 'text_generator',
        executable = 'generate',
        name = 'text_generator_node',
        parameters=[{
            'model': LaunchConfiguration('generation_model_size'),
            'tokens': LaunchConfiguration('max_generation_tokens')
        }]
    )

    # Lanch the description
    return LaunchDescription([
        model_arg,
        tokens_arg,
        generation_node
    ])
