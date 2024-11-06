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
from ament_index_python.packages import get_package_share_directory
from zed_interfaces.msg import ObjectsStamped

# Body skeleton transform node
class SkeletonTransform(Node):
    def __init__(self) -> None:
        super().__init__("skeleton_transform_node")

        # Declare parameters
        self.declare_parameter('camera_serial_number', 37817095)
        self.declare_parameter('camera_node_name', 'camera')
 
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
        sn = str(self.get_parameter('camera_serial_number').get_parameter_value().integer_value)
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
            self.get_logger().info(f'Orientation: {self.orientation}')            
        else:
            self.get_logger().warn(f'Could not read position and orientation for camera with serial number: {sn}')

        # Setup ROS publisher abd subscriber
        name = self.get_parameter('camera_node_name').get_parameter_value().string_value
        self.subscription = self.create_subscription(
            ObjectsStamped,
            f'/zed/{name}/body_trk/skeletons',
            self.skeletons_callback,
            qos_profile_sensor_data
        )
        self.publisher = self.create_publisher(
            ObjectsStamped,
            f'/zed/{name}/body_trk/transformed',
            qos_profile_sensor_data
        )

    # Callback function for reciving and applying a transfrom to human bodey skeleton keypoints
    def skeletons_callback(self, skeletons_msg):
        if self.position is None or self.orientation is None:
            self.get_logger().warn("Transformation parameters not loaded; cannot transform skeletons.")
            return

        transformed_msg = ObjectsStamped()
        transformed_msg.header = skeletons_msg.header

        # Apply transformation to each object (skeleton)
        for obj in skeletons_msg.objects:
            if not obj.skeleton_available:
                continue

            # Transform the 3D keypoints of the skeleton
            transformed_skeleton = []
            for keypoint in obj.skeleton_3d.keypoints:
                original_kp = np.array([keypoint.kp[0], keypoint.kp[1], keypoint.kp[2]])
                
                # Apply rotation and translation
                rotated_kp = self.orientation.apply(original_kp)
                transformed_kp = rotated_kp + self.position

                # Create a new Keypoint3D with the transformed coordinates
                transformed_keypoint = type(keypoint)()
                transformed_keypoint.kp = [float(transformed_kp[0]), float(transformed_kp[1]), float(transformed_kp[2])]
                transformed_skeleton.append(transformed_keypoint)

            # Update the skeleton keypoints in the transformed message
            transformed_obj = obj  # copy existing data
            transformed_obj.skeleton_3d.keypoints = transformed_skeleton
            transformed_msg.objects.append(transformed_obj)

        # Publish the transformed skeletons
        self.publisher.publish(transformed_msg)

        
# Main function
def main(args = None):
    rclpy.init(args = args)
    node = SkeletonTransform()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
