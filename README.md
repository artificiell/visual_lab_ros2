# visual_lab_ros2
Repository for ORU - Visual Lab. ROS 2 packages.  

## Installation

As prerequisite, an installation of [ROS Humble](https://docs.ros.org/en/humble/Installation/Linux-Install-Debians.html) (or [ROS Iron](https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debians.html)) is required.
To install necessary **visual_lab_ros2** interfaces, open a bash terminal, clone the package from Github, and build the interfaces (i.e., ROS *messages* and *services*):

```bash
mkdir -p ~/ros2_ws/src/                            # create your workspace if it does not exist
cd ~/ros2_ws/src/                                  
git clone --filter=blob:none --sparse https://github.com/artificiell/visual_lab_ros2.git
cd visual_lab_ros2/
git sparse-checkout add configurations/visual_lab_interfaces
cd ~/ros2_ws/
colcon build
echo source $(pwd)/install/setup.bash >> ~/.bashrc # automatically source the installation in every new bash (optional)
echo export ROS_DOMAIN_ID=1 >> ~/.bashrc           # automatically set the domain id in every new bash (optional)
source ~/.bashrc
```

## Usage

As minimal usage example, the following steps are required for writing a subscriber node that subscribes to a topic and logs data from incoming messages using a custom message type from the `visual_lab_interfaces` package.

1. Start by importing the necessary ROS 2 libraries and the custom message type `BodyTracking` from the `visual_lab_interfaces` package:

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from visual_lab_interfaces.msg import BodyTracking
```
2. Create a class that inherits from `Node` and initialize the subscriber:

```python
class BodyTrackingSubscriberNode(Node):
    def __init__(self) -> None:
        super().__init__("body_tracking_subscriber_node")

        self.body_trk_subscriber = self.create_subscription(
            BodyTracking,
            "features/body_trk",
            self.callback,
            qos_profile_sensor_data
        )
```
In the `__init__` method, the node is initialized with the name `body_tracking_node`. A subscription to the `features/body_trk` topic is set up using the `BodyTracking` message type. The `callback` method is specified to handle incoming messages, and `qos_profile_sensor_data` ensures reliable data delivery.

3. Define a callback method to process incoming messages:

```python
    def callback(self, msg):
        for body in msg.tracks:
            self.get_logger().info(f'Label: {body.label}')
            self.get_logger().info(f'Position: {body.position.x}, {body.position.y}, {body.position.z}')
            self.get_logger().info(f'Velocity: {body.velocity.x}, {body.velocity.y}, {body.velocity.z}')
```

For more details regarding how to write publisher and subscriber (Python), see official [ROS Tutorial](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html). 
For more examples specific for **visual_lab_ros2** (e.g., how to use ROS services to *public images to screen*), se the [example](https://github.com/artificiell/visual_lab_ros2/tree/main/examples) folder.
