#!/usr/bin/env python3
from launch_ros.actions import Node
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():

    # Launch configuration
    display_width = LaunchConfiguration('display_width')
    display_height = LaunchConfiguration('display_height')
    display_fullscreen = LaunchConfiguration('display_fullscreen')
    zed_camera_model = LaunchConfiguration('zed_camera_model')

    # Launch arguments
    display_width_arg = DeclareLaunchArgument(
        'display_width',
        default_value = '5670'
    )
    display_height_arg = DeclareLaunchArgument(
        'display_height',
        default_value = '1200'
    )
    display_fullscreen_arg = DeclareLaunchArgument(
        'display_fullscreen',
        default_value = 'True'
    )
    zed_camera_model_arg = DeclareLaunchArgument(
        'zed_camera_model',
        default_value = 'zed2i'
    )
    
    # ZED camera launch
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('zed_wrapper'),
                'launch',
                'zed_camera.launch.py'
            ])
        ]),
        launch_arguments = {
            'camera_model': LaunchConfiguration('zed_camera_model'),
        }.items()
    )

    # Tracking node
    tracking_node = Node(
        package = 'visual_features',
        executable = 'tracking',
        name = 'tracking_node'
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
        zed_camera_model_arg,
        camera_launch,
        tracking_node,
        visualizer_node
    ])
