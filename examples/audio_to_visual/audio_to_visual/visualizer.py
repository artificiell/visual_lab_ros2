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
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Int16MultiArray, MultiArrayDimension
from visual_lab_interfaces.srv import SetScreenBackground, SetScreenImage
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge

# Audio playback node
class AudioVisualizer(Node):
    def __init__(self) -> None:
        super().__init__("audio_visualizer_node")

        # Declare parameters
        self.declare_parameters("", [
            ("width", 1920),
            ("height", 1200),
            ("channels", 2),
        ])
        self.width = self.get_parameter(
            "width").get_parameter_value().integer_value
        self.height = self.get_parameter(
            "height").get_parameter_value().integer_value    
        self.channels = self.get_parameter(
            "channels").get_parameter_value().integer_value

        # Read image from directory
        if self.width > 1920:
            fname = os.path.join(
                get_package_share_directory('audio_to_visual'),
                'images',
                'oru_logo_5760.jpg'
            )
        else:
            fname = os.path.join(
                get_package_share_directory('audio_to_visual'),
                'images',
                'oru_logo_1920.jpg'
            )
        background = cv2.imread(fname)

        # OpenCV <-> ROS bridge
        self.bridge = CvBridge()
        self.image = None
        self.min, self.max = None, None
        
        # Setup ROS service clients
        self.bg_cli = self.create_client(SetScreenBackground, 'screen/background')
        self.img_cli = self.create_client(SetScreenImage, 'screen/set/image')
        while not (self.bg_cli.wait_for_service(timeout_sec = 1.0) or self.img_cli.wait_for_service(timeout_sec = 1.0)):
            self.get_logger().info('Services not available, waiting...')

        # Send backround image request
        request = SetScreenBackground.Request()
        request.image = self.bridge.cv2_to_imgmsg(background)
        self.bg_cli.call_async(request)
        
        # Setup ROS subscriber
        self.audio_subscriber = self.create_subscription(
            Int16MultiArray,
            "audio/microphone",
            self.audio_callback,
            qos_profile_sensor_data
        )

    # Audio playback callback method
    def audio_callback(self, msg) -> None:
        data = np.frombuffer(msg.data, np.int16)
        data = data.astype(np.float32)
        if self.min is None or data.min() < self.min:
            self.min = data.min()
        if self.max is None or data.max() > self.max:
            self.max = data.max()
        data = (data - self.min) / (self.max - self.min)

        # Create an image based on the audio signal
        height = int(self.height * 0.25)
        image = np.zeros((height, data.size, 3), dtype=np.uint8)
        image.fill(255)
        if self.image is None:
            self.image = image
        else:
            self.image = cv2.addWeighted(self.image, 0.8, image, 0.2, 0.0)
        for x in range(data.size):
            y = int(height - height * data[x])
            self.image = cv2.circle(self.image, (x, y), 2, (209, 150, 73), -1) 
        image = cv2.resize(self.image, (self.width, height))                
        
        # Send backround image request
        request = SetScreenImage.Request()
        request.id = 'waves'
        request.x = 0
        request.y = int(self.height * 0.75)
        request.image = self.bridge.cv2_to_imgmsg(image)
        self.img_cli.call_async(request)
        
        
# Main function
def main(args = None):
    rclpy.init(args = args)
    node = AudioVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
