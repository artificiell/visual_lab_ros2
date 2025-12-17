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

import rclpy
from rclpy.node import Node
from visual_lab_interfaces.srv import InstructionPrompt

import torch
from transformers import AutoTokenizer, AutoModelForCausalLM, pipeline


# Text generator node 
class TextGenerator(Node):
    def __init__(self) -> None:
        super().__init__("text_generator_node")

        # Declare parameters
        self.declare_parameter("model", "3B")
        self.declare_parameter("tokens", 8192)
        generator = self.get_parameter("model").get_parameter_value().string_value
        self.tokens = self.get_parameter("tokens").get_parameter_value().integer_value

        # Select model                                                     
        if generator == '8B':
            generator = 'meta-llama/Meta-Llama-3-8B-Instruct'
        elif generator == '3B':
            generator = 'meta-llama/Llama-3.2-3B-Instruct'
        else:
            generator = 'meta-llama/Llama-3.2-1B-Instruct'
        
        # Load model
        self.get_logger().info(f"Loading text generation model: {generator}")
        tokenizer = AutoTokenizer.from_pretrained(generator)
        model = AutoModelForCausalLM.from_pretrained(
            generator,
            torch_dtype = torch.bfloat16,
            device_map = "auto",
        )
        if tokenizer.pad_token_id is None: # Explicitly set pad_token to eos_token (common practice for Llama)                  
            tokenizer.pad_token = tokenizer.eos_token
            model.config.pad_token_id = model.config.eos_token_id
        self.generation_model = pipeline(
            "text-generation",
            model = model,
            tokenizer = tokenizer,
        )
        
        # Setup ROS services
        self.generation_srv = self.create_service(InstructionPrompt, 'text/generation', self.generation_callback)

        
    # Text generation callback method
    def generation_callback(self, request: InstructionPrompt.Request, response: InstructionPrompt.Response):

        # Handle request
        try:

            # Determine if sampling should be used
            do_sample = (request.temperature > 0.0) or (request.top_p > 0.0)
            temperature = request.temperature if request.temperature > 0.0 else None
            top_p = request.top_p if request.top_p > 0.0 else None            

            # Generate text and process the response
            outputs = self.generation_model(
                request.prompt,
                max_new_tokens = self.tokens,
                do_sample = do_sample,
                temperature = temperature,
                top_p = top_p
            )
            response.text = outputs[0]["generated_text"]
            response.success = True

        except Exception as e:
            self.get_logger().error(f'Text generation error: {e}')
            response.error = e
            response.success = False

        # Send response
        return response
    
        
# Main function
def main(args = None):
    rclpy.init(args = args)
    node = TextGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
