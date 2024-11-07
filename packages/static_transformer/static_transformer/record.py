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
from ros2_aruco_interfaces.msg import ArucoMarkers
from ament_index_python.packages import get_package_share_directory

# Transform recorder node
class TransformRecorder(Node):
    def __init__(self) -> None:
        super().__init__("transform_recorder_node")

        # Declare parameters
        self.declare_parameter('ref_marker_id', 0)
        self.declare_parameter('serial_number', 37817095)
        self.declare_parameter('num_samples', 100)
        
        # Setup ROS subscriber
        self.subscription = self.create_subscription(
            ArucoMarkers,
            'aruco/markers',
            self.marker_callback,
            10
        )

        # Initialize lists to store positions and orientations
        self.positions = []
        self.orientations = []

        # Set the number of samples for averaging
        self.num_samples = self.get_parameter('num_samples').get_parameter_value().integer_value
        self.sample_count = 0

        # Path to data log file
        self.config_file = os.path.join(
            get_package_share_directory('static_transformer'),
            'config',
            'transformations.yaml'
        )
        self.get_logger().info("ArUco transform recorder node started.")

        
    # Callback function for handle ArUco markers
    def marker_callback(self, msg):
        if self.sample_count >= self.num_samples:
            return
        
        # Check if the marker_ids and poses lists have the same length
        if len(msg.marker_ids) != len(msg.poses):
            self.get_logger().warn("Number of marker_ids and poses don't match!")
            return
        try:
        
            # Find the index of the reference marker
            ref_marker_index = msg.marker_ids.index(
                self.get_parameter('ref_marker_id').get_parameter_value().integer_value
            )        
        except:
            self.get_logger().warn("Reference marker not found!")
            return

        # Get the pose of the reference marker
        pose = msg.poses[ref_marker_index]

        # Collect position
        position = [pose.position.x, pose.position.y, pose.position.z]
        self.positions.append(position)

        # Collect orientation (convert quaternion to rotation matrix)
        orientation = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        self.orientations.append(orientation)

        # Increment sample count
        self.sample_count += 1
        self.get_logger().info(f"Sample {self.sample_count} collected.")


        # Log position
        if self.sample_count % 10 == 0:
            self.get_logger().info(f'Position: {pose.position}')
            self.get_logger().info(f'Orientation: {pose.orientation}')            
        
        # When we have enough samples, compute the average
        if self.sample_count >= self.num_samples:
            avg_position, avg_orientation = self.compute_average()
            self.save_to_yaml(avg_position, avg_orientation)
            self.get_logger().info(f"Averaged position and orientation saved to file: {self.config_file}")
            
            
    # Compute average position and orientation
    def compute_average(self):
        
        # Compute average position
        avg_position = np.mean(self.positions, axis=0)

        # Compute average orientation using rotation averaging
        rotations = [R.from_quat(orientation) for orientation in self.orientations]
        avg_rotation = R.from_quat(np.mean([r.as_quat() for r in rotations], axis=0)).as_quat()
        #avg_rotation = np.mean(self.orientations, axis=0)

        return avg_position, avg_rotation

    
    # Save to config file
    def save_to_yaml(self, avg_position, avg_orientation):
        data = {}
        serial_number = str(self.get_parameter('serial_number').get_parameter_value().integer_value)
        
        # Try to read exisiting config file
        if os.path.isfile(self.config_file):
            with open(self.config_file, 'r') as f:
                data = yaml.safe_load(f)        
        
        # Prepare the data for the YAML file
        data[serial_number] = {
            
            "position": {
                "x": float(avg_position[0]),
                "y": float(avg_position[1]),
                "z": float(avg_position[2]),
            },
            "orientation": {
                "x": float(avg_orientation[0]),
                "y": float(avg_orientation[1]),
                "z": float(avg_orientation[2]),
                "w": float(avg_orientation[3]),
            },
        }
        
        # Write to a YAML file
        with open(self.config_file, 'w') as file:
            yaml.dump(data, file)
        
           
# Main function
def main(args = None):
    rclpy.init(args = args)
    node = TransformRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

