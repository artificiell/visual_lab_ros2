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

import pyaudio
from vosk import Model, KaldiRecognizer
from whisper import load_model
import torch

import numpy as np
import json

# Speech transcribe node
class SpeechTranscribe(Node):
    def __init__(self) -> None:
        super().__init__("speech_transcribe_node")

        # Declare parameters
        self.declare_parameters("", [
            ("model", "vosk"),
            ("lang", "en-us"),
            ("size", "base"),
            ("rate", 16000),
            ("channels", 2),
        ])
        self.model = self.get_parameter(
            "model").get_parameter_value().string_value
        lang = self.get_parameter(
            "lang").get_parameter_value().string_value
        model_size = self.get_parameter(
            "size").get_parameter_value().string_value
        rate = self.get_parameter(
            "rate").get_parameter_value().integer_value
        self.channels = self.get_parameter(
            "channels").get_parameter_value().integer_value

        # Load text-to-speech model
        self.get_logger().info(f"Loding model '{self.model}.{model_size}'")
        if self.model == 'whisper':
            vosk_model_name = 'vosk-model-small-en-us-0.15'
        else:
            if model_size == 'base':
                vosk_model_name = 'vosk-model-en-us-0.22'
            else:
                vosk_model_name = 'vosk-model-small-en-us-0.15'
        model = Model(model_name = vosk_model_name, lang = lang)
        self.recognizer = KaldiRecognizer(model, rate)
        if self.model == 'whisper':
            self.transcriber = load_model(model_size)
            self.frames = []
            
        # Setup ROS publiser and subscriber
        self.text_sentence_publiser = self.create_publisher(
            String,
            "text/sentence",
            qos_profile_sensor_data
        )
        self.text_partial_publiser = self.create_publisher(
            String,
            "text/partial",
            qos_profile_sensor_data
        )
        self.audio_subscriber = self.create_subscription(
            Int16MultiArray,
            "audio/microphone",
            self.audio_callback,
            qos_profile_sensor_data
        )

    # Audio transcribing callback method
    def audio_callback(self, msg) -> None:
        data = np.frombuffer(msg.data, np.int16)
        data = np.mean(data.reshape(-1, self.channels), axis=1).astype(np.int16)
        if self.model == 'whisper':
            self.frames.append(data.copy())
        data = data.tobytes()
        if self.recognizer.AcceptWaveform(data):
            result = json.loads(self.recognizer.Result())
            if self.model == 'whisper':
                data = b''.join(self.frames)

                # Convert float 32  and clamp audio stream t PCM compatible wavelength (max 32768Hz) 
                data = np.frombuffer(data, dtype=np.int16).astype(np.float32) / 32768.0

                # Transcribe the frame buffer
                result = self.transcriber.transcribe(data, task = "transcribe", fp16 = True)
                self.frames.clear()
                
            text_msg = String()
            text_msg.data = result['text']
            self.text_sentence_publiser.publish(text_msg)
        else:
            text_msg = String()
            result = json.loads(self.recognizer.PartialResult())
            text_msg.data = result['partial']
            self.text_partial_publiser.publish(text_msg)
                
                
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
