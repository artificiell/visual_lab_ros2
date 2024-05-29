# visual_lab_ros2
Repository for ORU - Visual Lab. ROS 2 packages.  

## Installation

As a prerequisite, an installation of [ROS Humble](https://docs.ros.org/en/humble/Installation/Linux-Install-Debians.html) (or [ROS Iron](https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debians.html)) is required.
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

As a simple usage example, the following steps are required to write a subscriber node that subscribes to a topic and logs data from incoming messages using a custom *message* type from the `visual_lab_interfaces` package.

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
In the `__init__` method, the node is initialized with the name `body_tracking_node`. Using the' BodyTracking' message type, a subscription to the `features/body_trk` topic is set up. The `callback` method is specified to handle incoming messages, and `qos_profile_sensor_data` ensures reliable data delivery.

3. Define a callback method to process incoming messages:

```python
    def callback(self, msg):
        for body in msg.tracks:
            self.get_logger().info(f'Label: {body.label}')
            self.get_logger().info(f'Position: {body.position.x}, {body.position.y}, {body.position.z}')
            self.get_logger().info(f'Velocity: {body.velocity.x}, {body.velocity.y}, {body.velocity.z}')
```
For more details regarding how to write publisher and subscriber (Python), see the official [ROS Tutorial](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html). 

For a slightly more advanced usage example, the following steps are required to write a service client node that uses a custom *services* from the `visual_lab_interfaces` package.

1. Start by importing the necessary ROS 2 libraries and the custom services `SetScreenBackground` and `SetScreenImage` from the `visual_lab_interfaces` package:

```python
from visual_lab_interfaces.srv import SetScreenBackground, SetScreenImage
from cv_bridge import CvBridge
```
The import further includes `cv_bridge`, a ROS package that provides an interface between ROS and OpenCV, allowing for easy conversion between ROS image messages and OpenCV images. If `cv_bridge` is not installed, you can install it using standard packages (e.g., `sudo apt install ros-humble-cv-bridge`). 

2. Create a class that inherits from `Node` and initialize the service clients:

```python
class ScreenDisplayerNode(Node):
    def __init__(self) -> None:
        super().__init__("screen_displayer_node")

        self.bridge = CvBridge()
        self.bg_cli = self.create_client(SetScreenBackground, 'screen/background')
        self.img_cli = self.create_client(SetScreenImage, 'screen/image')
        while not (self.bg_cli.wait_for_service(timeout_sec = 1.0) or self.img_cli.wait_for_service(timeout_sec = 1.0)):
            self.get_logger().info('Services not available, waiting...')
```

3. Use background service clients to set background image:

```python
        bg = np.zeros((1200, 5760, 3), dtype=np.uint8)
        bg.fill(255)
        request = SetScreenBackground.Request()
        request.image = self.bridge.cv2_to_imgmsg(bg)
        self.bg_cli.call_async(request)
```

4. Use image service clients to draw an image on the background:

```python
        img = np.zeros((100, 100, 3), dtype=np.uint8)
        request = SetScreenImage.Request()
        request.id = 'my_little_square'
        request.x = 0
        request.y = 0
        request.image = self.bridge.cv2_to_imgmsg(img)
        self.img_cli.call_async(request)
```
The example above will *1)* set a white background image of size `5670 x 1200`, and *2)* draw a small black `100 x 100` square in the upper right corner of the screen. 

For more advanced examples specific to **visual_lab_ros2**, see the [example](https://github.com/artificiell/visual_lab_ros2/tree/main/examples) folder (in particular the `audio_to_visual` package).
