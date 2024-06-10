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

import torch
from diffusers import AutoPipelineForText2Image
from diffusers.pipelines.wuerstchen import DEFAULT_STAGE_C_TIMESTEPS

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from visual_lab_interfaces.srv import SetScreenBackground, SetScreenImage
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge

# Image generator node
class ImageGenerator(Node):
    def __init__(self) -> None:
        super().__init__("image_generator_node")
        
        # Declare parameters
        self.declare_parameters("", [
            ("screen_width", 1920),
            ("screen_height", 1200),
            ("image_width", 1536),
            ("image_height", 1024),
            ("scale_factor", 1.0),
        ])
        width = self.get_parameter(
            "screen_width").get_parameter_value().integer_value
        height = self.get_parameter(
            "screen_height").get_parameter_value().integer_value    
        self.width = self.get_parameter(
            "image_width").get_parameter_value().integer_value
        self.height = self.get_parameter(
            "image_height").get_parameter_value().integer_value    
        self.scale = self.get_parameter(
            "scale_factor").get_parameter_value().double_value    

        # Determine the number of rows and cols of images to generate
        self.cols = width // int(self.width * self.scale)
        self.rows = height // int(self.height * self.scale)
        self.offset_x = (width - int(self.width * self.scale) * self.cols) // (self.cols + 1)
        self.offset_y = (height - int(self.height * self.scale) * self.rows) // (self.rows + 1)

        # Read image from directory
        if width > 1920:
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
        
        # Setup ROS service clients
        self.bg_cli = self.create_client(SetScreenBackground, 'screen/background')
        self.img_cli = self.create_client(SetScreenImage, 'screen/set/image')
        while not (self.bg_cli.wait_for_service(timeout_sec = 1.0) or self.img_cli.wait_for_service(timeout_sec = 1.0)):
            self.get_logger().info('Services not available, waiting...')

        # Send backround image request
        request = SetScreenBackground.Request()
        request.image = self.bridge.cv2_to_imgmsg(background)
        self.bg_cli.call_async(request)

        # Create the text 2 image pipeline
        self.pipeline = AutoPipelineForText2Image.from_pretrained("warp-ai/wuerstchen", torch_dtype=torch.float16).to("cuda")
        
        # Setup ROS subscriber
        self.audio_subscriber = self.create_subscription(
            String,
            "text/sentence",
            self.text_callback,
            qos_profile_sensor_data
        )

    # Text callback method
    def text_callback(self, msg) -> None:
        txt = msg.data.lower()
        if 'generate' in txt:
            txt = txt.split('generate')[1]
            self.get_logger().info(f'Got sentence: {txt}')

            # Generate images
            x, y = self.offset_x, self.offset_y
            for i in range(self.rows):
                for j in range(self.cols):
                    images = self.pipeline(
                        txt,
                        height = self.height, # 1024
                        width = self.width,   # 1536
                        prior_timesteps = DEFAULT_STAGE_C_TIMESTEPS,
                        prior_guidance_scale = 4.0,
                        num_images_per_prompt = 1, # n = 1
                    ).images
                    self.get_logger().info(f'Generated {len(images)} images.')
                    for image in images:                        

                        # Convert to BGR
                        image = np.array(image)
                        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                        image = cv2.resize(image, (0, 0), fx = self.scale, fy = self.scale, interpolation = cv2.INTER_AREA)
                        
                        # Send image request
                        request = SetScreenImage.Request()
                        request.id = f'image_{i}_{j}'
                        request.x = x
                        request.y = y
                        request.image = self.bridge.cv2_to_imgmsg(image)
                        self.img_cli.call_async(request)
                    x = x + int(self.width * self.scale) + self.offset_x
                y = y + int(self.height * self.scale) + self.offset_y
                x = self.offset_x
        
# Main function
def main(args = None):
    rclpy.init(args = args)
    node = ImageGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
