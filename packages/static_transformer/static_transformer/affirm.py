#!/usr/bin/env python3

# MIT License

# Copyright (c) 2024 Andreas Persson

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import os
import yaml
import numpy as np
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from ros2_aruco_interfaces.msg import ArucoMarkers
from ament_index_python.packages import get_package_share_directory

# Transform affirm node for verifying that the trannsformation is correct
class TransformAffirm(Node):
    def __init__(self) -> None:
        super().__init__("transform_affirm_node")

        # Declare parameters
        self.declare_parameter('serial_number', 37817095)
 
        # Path to data log file
        config_file = os.path.join(
            get_package_share_directory('static_transformer'),
            'config',
            'transformations.yaml'
        )

        # Try to read data from exisiting config file
        data = {}
        if os.path.isfile(config_file):
            with open(config_file, 'r') as f:
                data = yaml.safe_load(f)
        else:
            self.get_logger().warn(f'Could not read config file: {config_file}')

            
        # Try to read position and orientation from config file
        self.position, self.orientation = None, None
        sn = str(self.get_parameter('serial_number').get_parameter_value().integer_value)
        self.get_logger().info(f'Camera serial number: {sn}')
        if sn in data:
            self.position = np.array([
                data[sn]['position']['x'],
                data[sn]['position']['y'],
                data[sn]['position']['z']
            ])
            self.orientation = R.from_quat([
                data[sn]['orientation']['x'],
                data[sn]['orientation']['y'],
                data[sn]['orientation']['z'],
                data[sn]['orientation']['w']
            ])
            self.get_logger().info(f'Position: {self.position}')
            self.get_logger().info(f'Orientation: {self.orientation.as_quat()}')            
        else:
            self.get_logger().warn(f'Could not read position and orientation for camera with serial number: {sn}')

        # Setup ROS subscriber
        self.subscription = self.create_subscription(
            ArucoMarkers,
            'aruco/markers',
            self.marker_callback,
            10
        )
        self.publisher = self.create_publisher(
            ArucoMarkers,
            'aruco/transformed',
            10
        )

    # Callback function for transfroming the position of ArUco markers
    def marker_callback(self, msg):

        # Create a new ArucoMarkers to store the transformed poses
        markers = ArucoMarkers()
        markers.header = msg.header
        markers.marker_ids = []
        markers.poses = []

        # Transform the poses of the other markers relative to the reference marker
        for i in range(len(msg.marker_ids)):
            original_pose = np.array([msg.poses[i].position.x, msg.poses[i].position.y, msg.poses[i].position.z])
            self.get_logger().info(f'Original ({msg.marker_ids[i]}): x={original_pose[0]}, y={original_pose[1]}, z={original_pose[2]}')
            transformed_pose = self.orientation.inv().apply(original_pose - self.position)
            pose = Pose()
            pose.position.x = transformed_pose[0]
            pose.position.y = transformed_pose[1]
            pose.position.z = transformed_pose[2]
            
            # Add the marker id and transformed pose to the new message
            markers.marker_ids.append(msg.marker_ids[i])
            markers.poses.append(pose)

            self.get_logger().info(f'Transformed ({msg.marker_ids[i]}): x={transformed_pose[0]}, y={transformed_pose[1]}, z={transformed_pose[2]}')
            
        # Publish the transformed poses
        self.publisher.publish(markers)
        self.get_logger().info(f'---------------------')
        
# Main function
def main(args = None):
    rclpy.init(args = args)
    node = TransformAffirm()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
