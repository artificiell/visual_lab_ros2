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

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Int16MultiArray, MultiArrayDimension
from std_msgs.msg import String

import pyaudio
from vosk import Model, KaldiRecognizer
from whisper import load_model

import numpy as np

# Speech transcribe node
class SpeechTranscribe(Node):
    def __init__(self) -> None:
        super().__init__("speech_transcribe__node")

        # Declare parameters
        self.declare_parameters("", [
            ("model", "vosk"),
            ("lang", "en-us"),
            ("size", "base"),
            ("rate", 16000),
        ])
        self.model = self.get_parameter(
            "model").get_parameter_value().string_value
        lang = self.get_parameter(
            "lang").get_parameter_value().string_value
        model_size = self.get_parameter(
            "size").get_parameter_value().string_value
        rate = self.get_parameter(
            "rate").get_parameter_value().integer_value

        # Load text-to-speech model
        self.get_logger().info(f"Loding model '{self.model}.{model_size}'")
        if self.model == 'vosk':
            model = Model(lang = lang)
            self.rec = KaldiRecognizer(model, rate)
        elif self.model == 'whisper':
            self.rec = load_model(model_size, device = "cuda")
        else: 
            self.get_logger().warning(f"Model '{self.model}' not supported.")
            
        # Setup ROS publiser and subscriber
        self.text_publiser = self.create_publisher(
            String,
            "text",
            qos_profile_sensor_data
        )
        self.audio_subscriber = self.create_subscription(
            Int16MultiArray,
            "audio",
            self.audio_callback,
            qos_profile_sensor_data
        )

    # Timed audio record callback method
    def audio_callback(self, msg) -> None:
        data = np.frombuffer(msg.data, np.int16)
        if self.model == 'vosk':
            data = data.tobytes()
            if self.rec.AcceptWaveform(data):
                text_msg = String()
                text_msg.data = self.rec.Result()
                self.text_publiser.publish(text_msg)
        elif self.model == 'whisper':
            data = data.astype(np.float32)
            text_msg = String()
            text_msg.data = self.rec.transcribe(data)['text']
            self.text_publiser.publish(text_msg)

# Main function
def main(args = None):
    rclpy.init(args = args)
    node = SpeechTranscribe()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
