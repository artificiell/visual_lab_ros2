#!/usr/bin/env python3
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # Launch configuration
    display_width = LaunchConfiguration('display_width')
    display_height = LaunchConfiguration('display_height')
    display_fullscreen = LaunchConfiguration('display_fullscreen')

    # Launch arguments
    display_width_arg = DeclareLaunchArgument(
        'display_width',
        default_value = '5760'
    )
    display_height_arg = DeclareLaunchArgument(
        'display_height',
        default_value = '1200'
    )
    display_fullscreen_arg = DeclareLaunchArgument(
        'display_fullscreen',
        default_value = 'True'
    )
    
    # Display visualizer node
    visualizer_node = Node(
        package = 'visual_display',
        executable = 'visualize',
        name = 'display_visualizer_node',
        parameters=[{
            'width': LaunchConfiguration('display_width'),
            'height': LaunchConfiguration('display_height'),
            'fullscreen': LaunchConfiguration('display_fullscreen')
        }]
    )

    # Lanch the description
    return LaunchDescription([
        display_width_arg,
        display_height_arg,
        display_fullscreen_arg,
        visualizer_node
    ])
