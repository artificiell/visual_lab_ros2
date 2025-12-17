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
from visual_lab_interfaces.msg import SentenceEmbedding
from visual_lab_interfaces.srv import SentenceEmbeddings

import torch
import numpy as np
from sentence_transformers import SentenceTransformer


# Text embeddings node 
class TextEmbeddings(Node):
    def __init__(self) -> None:
        super().__init__("text_embeddings_node")

        # Declare parameters
        self.declare_parameter("language", "sv")
        language = self.get_parameter("language").get_parameter_value().string_value

        # Select model based on language                                                     
        if language == 'sv':
            self.model = SentenceTransformer("KBLab/sentence-bert-swedish-cased")
            self.get_logger().info("Loading embedding model: KBLab/sentence-bert-swedish-cased")
        else:
            self.model = SentenceTransformer("sentence-transformers/paraphrase-multilingual-MiniLM-L12-v2")
            self.get_logger().info("Loading embedding model: sentence-transformers/paraphrase-multilingual-MiniLM-L12-v2")
        
        # Setup ROS services
        self.embeddings_srv = self.create_service(SentenceEmbeddings, 'text/embeddings', self.embeddings_callback)

        
    # Text embeddings callback method
    def embeddings_callback(self, request: SentenceEmbeddings.Request, response: SentenceEmbeddings.Response):

        # Handle request
        try:

            # Generate sentence embeddings and process the response
            embeddings = self.model.encode(
                request.sentences,
                convert_to_numpy = True,
                normalize_embeddings = True
            )
            embeddings = np.asarray(embeddings, dtype = np.float32)
            response.embeddings = []
            for sentence, embedding in zip(request.sentences, embeddings):
                msg = SentenceEmbedding()
                msg.sentence = sentence
                msg.embedding = embedding
                response.embeddings.append(msg)
            response.success = True

        except Exception as e:
            self.get_logger().error(f'Text embeddings error: {e}')
            response.error = e
            response.success = False

        # Send response
        return response
    
        
# Main function
def main(args = None):
    rclpy.init(args = args)
    node = TextEmbeddings()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
