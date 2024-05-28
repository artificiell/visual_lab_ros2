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

import pyaudio

import atexit
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Int16MultiArray, MultiArrayDimension

# Audio playback node
class AudioPlayback(Node):
    def __init__(self) -> None:
        super().__init__("audio_playback_node")

        # Declare parameters
        self.declare_parameters("", [
            ("format", pyaudio.paInt16),
            ("channels", 2),
            ("rate", 16000),
            ("device", -1),
        ])
        self.format = self.get_parameter(
            "format").get_parameter_value().integer_value
        self.channels = self.get_parameter(
            "channels").get_parameter_value().integer_value
        self.rate = self.get_parameter(
            "rate").get_parameter_value().integer_value
        device = self.get_parameter(
            "device").get_parameter_value().integer_value
        if device < 0:
            device = None  

        # Open audio stream 
        self.audio = pyaudio.PyAudio()
        self.stream = self.audio.open(
            input_device_index = device,
            channels = self.channels,
            format = self.format,
            output = True,
            rate = self.rate
        )

        # Register cleanup callback method
        atexit.register(self.cleanup)
        
        # Setup ROS subscriber
        self.audio_subscriber = self.create_subscription(
            Int16MultiArray,
            "audio/speaker",
            self.audio_callback,
            qos_profile_sensor_data
        )

    # Audio playback callback method
    def audio_callback(self, msg) -> None:
        data = np.frombuffer(msg.data, np.int16)
        #data = np.repeat(data, 2)
        data = data.tobytes()
        self.stream.write(data)
                                                            
    # Cleanup method
    def cleanup(self) -> None:
        self.stream.close()
        self.audio.terminate()
        
# Main function
def main(args = None):
    rclpy.init(args = args)
    node = AudioPlayback()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
