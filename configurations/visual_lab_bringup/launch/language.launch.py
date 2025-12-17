#!/usr/bin/env python3
from launch_ros.actions import Node
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():

    # Launch configuration
    language = LaunchConfiguration('embeddings_model_language')
    model = LaunchConfiguration('generation_model_size')
    tokens = LaunchConfiguration('max_generation_tokens')

    # Launch arguments
    language_arg = DeclareLaunchArgument(
        'embeddings_model_language',
        default_value = 'sv'
    )
    model_arg = DeclareLaunchArgument(
        'generation_model_size',
        default_value = '8B'
    )
    tokens_arg = DeclareLaunchArgument(
        'max_generation_tokens',
        default_value = '8192'
    )

    # Include text embeddings launch
    embeddings_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('visual_lab_bringup'),
                'launch',
                'sentence_embeddings.launch.py'
            ])
        ]),
        launch_arguments = {
            'embeddings_model_language': LaunchConfiguration('embeddings_model_language')
        }.items()
    )

    # Include text generation launch
    generation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('visual_lab_bringup'),
                'launch',
                'text_generation.launch.py'
            ])
        ]),
        launch_arguments = {
            'generation_model_size': LaunchConfiguration('generation_model_size'),
            'max_generation_tokens': LaunchConfiguration('max_generation_tokens'),
            
        }.items()
    )

    # Lanch the description
    return LaunchDescription([
        language_arg,
        model_arg,
        tokens_arg,
        embeddings_launch,
        generation_launch
    ])
