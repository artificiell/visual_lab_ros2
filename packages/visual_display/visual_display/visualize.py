#!/usr/bin/env python3

# MIT License

# Copyright (c) 2024  Andreas Persson

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


import atexit
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from visual_lab_interfaces.srv import SetScreenBackground, SetScreenImage
from cv_bridge import CvBridge

# Helper class for handle images
class Image:
    def __init__(self, image, x, y) -> None:
        self._image = image
        self._x, self._y = x, y
        self._height, self._width = self._image.shape[:2]

    @property
    def image(self):
        return self._image
    @image.setter
    def image(self, value):
        self._image = value
        self._height, self._width = self._image.shape[:2]

    @property
    def x(self):
        return self._x
    @x.setter
    def x(self, value):
        self._x = value

    @property
    def y(self):
        return self._y
    @y.setter
    def x(self, value):
        self._y = value

    # Draw method
    def draw_on(self, canvas):
        height, width = canvas.shape[:2]
        if self._x + self._width >= width:
            self._width = width - self._x
        if self._y + self._height >= height:
            self._height = height - self._y
        canvas[self._y:self._y + self._height, self._x:self._x + self._width] = self._image[0:self._height, 0:self._width]            
        
# Display visualizer node 
class DisplayVisualizer(Node):
    def __init__(self) -> None:
        super().__init__("display_visualizer_node")

        # Declare parameters
        self.declare_parameters("", [
            ("width", 1920),
            ("height", 1200),
            ("fullscreen", False),
            ("rate", 30),
        ])
        self.width = self.get_parameter(
            "width").get_parameter_value().integer_value
        self.height = self.get_parameter(
            "height").get_parameter_value().integer_value
        fullscreen = self.get_parameter(
            "fullscreen").get_parameter_value().bool_value
        frame_rate = self.get_parameter(
            "rate").get_parameter_value().integer_value

        # Create named OpenCV window
        self.name = "Visual Lab Display"
        cv2.namedWindow(self.name, cv2.WINDOW_NORMAL)
        if fullscreen:
            cv2.setWindowProperty(self.name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        self.background = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        self.background.fill(255)
        
        # Register cleanup callback method
        atexit.register(self.cleanup)
        
        # Setup ROS timer 
        self.display_timer = self.create_timer(
            1. / frame_rate,
            self.timed_callback,
        )

        # Setup ROS services
        self.background_srv = self.create_service(SetScreenBackground, 'screen/background', self.background_callback)
        self.image_srv = self.create_service(SetScreenImage, 'screen/image', self.image_callback)
        self.bridge = CvBridge()
        self.images = {}
        
    # Screen background callback method
    def background_callback(self, request, response):

        # Handle request
        try:
            self.background = self.bridge.imgmsg_to_cv2(request.image, desired_encoding = 'passthrough')
            self.background = cv2.resize(self.background, (self.width, self.height))
            response.success = True
        except Exception as e:
            self.get_logger().error(f'Background service error: {e}')

        # Send response
        return response

    # Screen image callback method
    def image_callback(self, request, response):

        # Handle request
        try:
            id = request.id
            if id in self.images:
                self.images[id].image = self.bridge.imgmsg_to_cv2(request.image, desired_encoding = 'passthrough')
                self.images[id].x = request.x 
                self.images[id].y = request.y 
            else:
                self.images[id] = Image(
                    self.bridge.imgmsg_to_cv2(request.image, desired_encoding = 'passthrough'),
                    request.x,
                    request.y
                )
            response.success = True
        except Exception as e:
            self.get_logger().error(f'Background service error: {e}')

        # Send response
        return response

    
    # Timed display callback method
    def timed_callback(self) -> None:
        canvas = self.background.copy()
        for image in self.images:
            image.draw_on(canvas)
        cv2.imshow(self.name, canvas)   
        cv2.waitKey(1)
        
    # Cleanup method
    def cleanup(self) -> None:
        cv2.destroyWindow(self.name)

# Main function
def main(args = None):
    rclpy.init(args = args)
    node = DisplayVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
