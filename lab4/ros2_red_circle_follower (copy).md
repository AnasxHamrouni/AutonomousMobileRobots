# ROS2 Red Circle Follower Tutorial

## Assumptions

- Your robot has a camera already publishing images.
- The robot can be driven by publishing `geometry_msgs/Twist` messages .
- You have installed OpenCV (`opencv-python`) `pip3 install opencv-python` and `cv_bridge` for Python (`sudo apt install ros-humble-cv-bridge*` might be needed) in your workspace environment.

## Tutorial: Following a Red Circle with a Camera in ROS2

### 1. Create a ROS2 Package 

- Create a new package:

```bash
cd ~/AMR_ws/src
ros2 pkg create --build-type ament_python red_circle_follower --dependencies rclpy sensor_msgs geometry_msgs
```


### 2. Node Overview

We’ll write a node, `red_circle_follower.py`, that:

- Subscribes to the camera topic.
- Converts the ROS image to an OpenCV numpy array using `cv_bridge`.
- Processes the image to detect a red circle:
  - Convert from BGR to HSV.
  - Threshold to find red color.
  - Find the largest red circle (or red blob).
  - Compute its area (as a percentage of total image area).
  - Determine its center (x, y) relative to the image center.
- Publishes velocity commands (`geometry_msgs/Twist`) to drive the robot:
  - If no red circle is found: rotate in place looking for it.
  - If the circle is found but is less than 30% of the image area:
    - Turn left/right to center the circle.
    - Move forward if the circle is roughly centered.
  - If the circle area is >= 30%: stop.

### 3. Write the Python Node

Create the `red_circle_follower.py` file inside `red_circle_follower/`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import cv2
import numpy as np

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class RedCircleFollower(Node):
    def __init__(self):
        super().__init__('red_circle_follower')
        self.get_logger().info("Initializing Red Circle Follower Node...")

        self.declare_parameter('camera_topic', 'choose a topic to subscribe')
        self.declare_parameter('area_threshold_percentage',) # add value for the size of the red circle
        self.declare_parameter('linear_speed', ) # add a speed value
        self.declare_parameter('angular_speed',) # add a speed value

        self.camera_topic = self.get_parameter('camera_topic').value
        self.area_threshold_pct = self.get_parameter('area_threshold_percentage').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value

        self.subscription = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            10
        )
        self.cmd_vel_pub = self.create_publisher(Twist, 'choose a topic to publish your command', 10)
        self.bridge = CvBridge()
        self.img_width = None
        self.img_height = None

    def image_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        if self.img_width is None or self.img_height is None:
            self.img_height, self.img_width = frame.shape[:2]

        circle_found, circle_area, circle_center = self.detect_red_circle(frame)
        twist_cmd = Twist()

        if not circle_found:
            #rotate your robot in it's place
        else:
            image_area = float(self.img_width * self.img_height)
            circle_area_pct = (circle_area / image_area) * 100.0

            if circle_area_pct >= self.area_threshold_pct:
                #stop your robot
                # linea and angular movement 
            else:
                center_x = self.img_width // 2
                x_offset = circle_center[0] - center_x
                twist_cmd.angular.z = -float(x_offset)/float(center_x) * self.angular_speed
                twist_cmd.linear.x = self.linear_speed

        self.cmd_vel_pub.publish(twist_cmd)

    def detect_red_circle(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 | mask2

        kernel = np.ones((5,5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return (False, 0, (0,0))

        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)
        (x, y), radius = cv2.minEnclosingCircle(largest_contour)

        if radius < 5:
            return (False, 0, (0,0))

        return (True, area, (int(x), int(y)))

def main(args=None):
    rclpy.init(args=args)
    node = RedCircleFollower()
    # run your node continuously
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 4. Add Entry Point in `setup.py`

Modify `setup.py`:

```python
from setuptools import setup

package_name = 'red_circle_follower'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='Node for following a red circle with a camera',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'add entry point'
        ],
    },
)
```

### 5. Build and Run

```bash
cd ~/AMR_ws
colcon build --packages-select red_circle_follower
source install/setup.bash
ros2 run red_circle_follower red_circle_follower
```

### 6. Debugging

Check the camera feed:
- Visualize the camera feed with a tool like `rqt` > `plugins` > `image view` or `rviz2` (Image display) to confirm the robot is indeed seeing a red circle.

- Check the `cmd_vel` topic:

```bash
ros2 topic echo /cmd_vel
```

*Tune parameters like HSV thresholds, area threshold, and speeds as needed.*
