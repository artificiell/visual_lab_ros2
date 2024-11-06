#!/usr/bin/env python3
from launch_ros.actions import Node
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():

    # Launch configuration
    zed_serial_number = LaunchConfiguration('zed_serial_number')
    number_of_samples = LaunchConfiguration('number_of_samples')
    
    # Launch arguments
    zed_serial_number_arg = DeclareLaunchArgument(
        'zed_serial_number',
        default_value = '37817095',
        description = 'ZED camera serial number (38160741, 37817095, 36374190, or 36240144)'
        
    )
    number_of_samples_arg = DeclareLaunchArgument(
        'number_of_samples',
        default_value = '100',
        description = 'Number of ArUco position and orientation samples'
        
    )

    # Include camera launch
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('visual_lab_bringup'),
                'launch',
                'camera.launch.py'
            ])
        ]),
        launch_arguments = {
            'zed_serial_number': LaunchConfiguration('zed_serial_number'),
        }.items()
    )

    # ArUco marker detection node
    aruco_detection_node = Node(
        package = 'ros2_aruco',
        executable = 'aruco_detection',
        name = 'aruco_detection_node',
        parameters=[{
            'marker_size': 0.19,
            'aruco_dictionary_id': 'DICT_ARUCO_ORIGINAL',
            'image_topic': '/zed/camera/rgb/image_rect_color',
            'camera_info_topic': '/zed/camera/rgb/camera_info'
        }]
    )

    # Transformation recorder node
    transform_recorder_node = Node(
        package = 'static_transformer',
        executable = 'record',
        name = 'transform_recorder_node',
        parameters=[{
            'serial_number': LaunchConfiguration('zed_serial_number'),
            'num_samples': LaunchConfiguration('number_of_samples')
        }]
        
    )

    # Lanch the description
    return LaunchDescription([
        zed_serial_number_arg,
        number_of_samples_arg,
        camera_launch,
        aruco_detection_node,
        transform_recorder_node
    ])
