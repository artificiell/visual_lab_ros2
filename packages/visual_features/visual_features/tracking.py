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
from rclpy.qos import qos_profile_sensor_data
from visual_lab_interfaces.msg import Point, Body, BodyTracking
from zed_interfaces.msg import ObjectsStamped
        
# Tracking node 
class Tracking(Node):
    def __init__(self) -> None:
        super().__init__("tracking_node")

        # Setup ROS publisher and subscriber
        self.zed_subscriber = self.create_subscription(
            ObjectsStamped,
            "zed/zed_node/body_trk/skeletons",
            self.zed_callback,
            qos_profile_sensor_data
        )
        self.track_publisher = self.create_publisher(
            BodyTracking,
            "features/body_trk",
            qos_profile_sensor_data
        )

    # ZED object callback method
    def zed_callback(self, msg):
        body_trk_msg = BodyTracking()
        body_trk_msg.header = msg.header
        for object in msg.objects:
            trk = Body()
            trk.label = object.label
            trk.position.x = float(object.position[0]) 
            trk.position.y = float(object.position[1]) 
            trk.position.z = float(object.position[2])
            trk.velocity.x = float(object.velocity[0]) 
            trk.velocity.y = float(object.velocity[1]) 
            trk.velocity.z = float(object.velocity[2])
            body_trk_msg.tracks.append(trk)
        self.track_publisher.publish(body_trk_msg)

# Main function
def main(args = None):
    rclpy.init(args = args)
    node = Tracking()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
