#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data

from sentence_transformers import SentenceTransformer
import warnings

class TextListener(Node):

    def __init__(self):
        super().__init__('keyword')
        self.keywords = ['Fish', 'Tree', 'rock']
        self.threshold = 0.1

        self.model = SentenceTransformer("paraphrase-MiniLM-L3-v2")
        self.kw_embeddings = self.model.encode(self.keywords)

        self.subscription = self.create_subscription(
            String,
            '/text/sentence',
            self.listener_callback,
            qos_profile_sensor_data)
        self.publisher = self.create_publisher(String, '/keyword', 10)
        self.subscription

    def listener_callback(self, msg):
        sentence = msg.data
        if 'Anna' in sentence or 'Ana' in sentence or 'Allah' in sentence or 'Ala' in sentence:
            self.get_logger().info(f'Anna awake!')

            sentence_emb = self.model.encode(sentence)
            similarities = self.model.similarity(sentence_emb, self.kw_embeddings)[0]
            most_similar_index = similarities.argmax()

            if similarities[most_similar_index] > self.threshold:

                keyword = self.keywords[most_similar_index]
                self.get_logger().info(f'Found keyword "{keyword}" in: "{sentence}"')
                self.publish_keyword(keyword)
            else:
                self.publish_keyword('noise')

    def publish_keyword(self, keyword):
        msg = String()
        msg.data = keyword
        self.publisher.publish(msg)
        self.get_logger().info(f'Published keyword: "{keyword}"')

def main(args=None):
    rclpy.init(args=args)

    text_listener = TextListener()

    rclpy.spin(text_listener)

    text_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
