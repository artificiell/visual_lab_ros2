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

    # Launch arguments
    language_arg = DeclareLaunchArgument(
        'embeddings_model_language',
        default_value = 'sv'
    )

    # Text embeddings node
    embeddings_node = Node(
        package = 'text_embeddings',
        executable = 'embed',
        name = 'text_embeddings_node',
        parameters=[{
            'language': LaunchConfiguration('embeddings_model_language')
        }]
    )

    # Lanch the description
    return LaunchDescription([
        language_arg,
        embeddings_node
    ])
