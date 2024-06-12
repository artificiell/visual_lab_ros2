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

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Int16MultiArray, MultiArrayDimension
from std_msgs.msg import String

import io
import pyttsx4
import numpy as np
import array

# Text speech node
class TextSpeech(Node):
    def __init__(self) -> None:
        super().__init__("text_speech_node")

        # Declare parameters
        self.declare_parameters("", [
            ("channels", 2),
            ("model", "sapi5"),
            ("voice", "english"),
            ("pitch", 75),
            ("rate", 150),
        ])
        self.channels = self.get_parameter(
            "channels").get_parameter_value().integer_value
        model = self.get_parameter(
            "model").get_parameter_value().string_value
        voice = self.get_parameter(
            "voice").get_parameter_value().string_value
        pitch = self.get_parameter(
            "pitch").get_parameter_value().integer_value
        rate = self.get_parameter(
            "rate").get_parameter_value().integer_value

        # Load speech-to-text engine
        self.get_logger().info(f"Loding text-to-speech engine")
        self.engine = pyttsx4.init() # 'coqui_ai_tts'
        self.engine.setProperty('voice', voice)
        self.engine.setProperty('pitch', pitch)
        self.engine.setProperty('rate', rate)
        
        # Setup ROS publiser and subscriber
        self.audio_publisher = self.create_publisher(
            Int16MultiArray,
            "audio/speaker",
            qos_profile_sensor_data
        )
        self.audio_subscriber = self.create_subscription(
            String,
            "text/speech",
            self.text_callback,
            qos_profile_sensor_data
        )

    # Text callback method
    def text_callback(self, msg) -> None:
        self.get_logger().info(f"Got message: '{msg.data}'")
        buffer = io.BytesIO()
        self.engine.save_to_file(msg.data, buffer)
        self.engine.runAndWait()
        audio = np.frombuffer(buffer.getbuffer(), dtype = np.int16)
        audio = audio.repeat(self.channels)
        audio_msg = Int16MultiArray()
        audio_msg.data = audio.tolist()
        audio_msg.layout.data_offset = 0
        audio_msg.layout.dim.append(
            MultiArrayDimension(label = "audio",
                                size = audio.size,
                                stride = 1)
        )
        self.audio_publisher.publish(audio_msg)
        
# Main function
def main(args = None):
    rclpy.init(args = args)
    node = TextSpeech()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
