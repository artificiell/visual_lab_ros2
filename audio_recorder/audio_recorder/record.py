#!/usr/bin/env python3

# MIT License

# Copyright (c) 2023  Miguel Ángel González Santamarta

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
from std_msgs.msg import Int16MultiArray, MultiArrayDimension

# Audio recorder node 
class AudioRecorder(Node):
    def __init__(self) -> None:
        super().__init__("audio_recorder_node")

        # Declare parameters
        self.declare_parameters("", [
            ("format", pyaudio.paInt16),
            ("channels", 1),
            ("rate", 16000),
            ("chunk", 4096),
            ("device", -1),
        ])
        self.format = self.get_parameter(
            "format").get_parameter_value().integer_value
        self.channels = self.get_parameter(
            "channels").get_parameter_value().integer_value
        self.rate = self.get_parameter(
            "rate").get_parameter_value().integer_value
        self.chunk = self.get_parameter(
            "chunk").get_parameter_value().integer_value
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
            input = True,
            frames_per_buffer = self.chunk,
            rate = self.rate
        )

        # Register cleanup callback method
        atexit.register(self.cleanup)
        
        # Setup ROS publisher
        self.audio_publisher = self.create_publisher(
            Int16MultiArray, "audio", 5
        )
        self.record_timer = self.create_timer(
            float(self.chunk) / float(self.rate),
            self.timed_callback,
        )

    # Timed audio record callback method
    def timed_callback(self) -> None:
        audio = self.stream.read(self.chunk)
        audio = np.frombuffer(audio, dtype = np.int16)
        audio_msg = Int16MultiArray()
        audio_msg.data = audio.tolist()
        audio_msg.layout.data_offset = 0
        audio_msg.layout.dim.append(
            MultiArrayDimension(label = "audio", size = self.chunk, stride = 1)
        )
        self.audio_publisher.publish(audio_msg)
                                                            
    # Cleanup method
    def cleanup(self) -> None:
        self.stream.close()
        self.audio.terminate()
            
# Main function
def main(args = None):
    rclpy.init(args = args)
    node = AudioRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
