# ROS2 Keyboard and Python Node Control for Gazebo-Simulated Robot

## 1. Introduction
This tutorial extends keyboard control by creating a **custom Python ROS2 node** to send velocity commands. We will:
- Write a Python script to control the robot
- Publish velocity commands programmatically
- Integrate with the Gazebo simulation

---

## 2. Install Required Packages
Ensure you have the ROS2 Python package installed:
```bash
sudo apt install python3-colcon-common-extensions
```

---

## 3. Launch the Robot in Gazebo
Start your robot simulation:
```bash
ros2 launch my_robot_bringup my_robot_gazebo.launch.xml
```
Wait for Gazebo to fully load.

---

## 4. Writing a Custom Python Control Node
Create a new Python script named **keyboard_control.py** inside your ROS2 package.

### 4.1. Navigate to Your ROS2 Package
```bash
cd ~/AMR_ws/src/my_robot_bringup
cd my_robot_bringup
touch keyboard_control.py
```

### 4.2. Write the Python Control Node
Edit **keyboard_control.py** and add:
```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty

class RobotTeleop(Node):
    def __init__(self):
        super().__init__('robot_teleop')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("Robot teleop node started. Use WASD keys to move. Press 'q' to stop.")

    def get_key(self):
        """Reads a single key press."""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def control_loop(self):
        """Main loop to read keyboard inputs and send velocity commands."""
        twist = Twist()
        speed = 0.5
        turn = 1.0

        while rclpy.ok():
            key = self.get_key()
            if key == 'w':  # Forward
                twist.linear.x = speed
                twist.angular.z = 0.0
            elif key == 's':  # Backward
                twist.linear.x = -speed
                twist.angular.z = 0.0
            elif key == 'a':  # Left
                twist.linear.x = 0.0
                twist.angular.z = turn
            elif key == 'd':  # Right
                twist.linear.x = 0.0
                twist.angular.z = -turn
            elif key == 'q':  # Stop
                twist.linear.x = 0.0
                twist.angular.z = 0.0
            elif key == '':  # CTRL+C
                break
            self.publisher_.publish(twist)
            self.get_logger().info(f"Velocity: linear={twist.linear.x}, angular={twist.angular.z}")

def main(args=None):
    rclpy.init(args=args)
    node = RobotTeleop()
    try:
        node.control_loop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## 5. Making the Node Executable
```bash
chmod +x keyboard_control.py
```

---

## 6. Updating `package.xml` and `setup.py`
Edit **package.xml** in **my_robot_bringup** and add:
```xml
<exec_depend>rclpy</exec_depend>
<exec_depend>geometry_msgs</exec_depend>
```

Edit **setup.py** and ensure:
```python
entry_points={
    'console_scripts': [
        'keyboard_control = my_robot_bringup.keyboard_control:main',
    ],
},
```

---

## 7. Building and Running the Node
### 7.1. Build the Package
```bash
cd ~/AMR_ws
colcon build --packages-select my_robot_bringup
source install/setup.bash
```

### 7.2. Run the Keyboard Control Node
```bash
ros2 run my_robot_bringup keyboard_control
```
Use:
- **W** → Forward
- **S** → Backward
- **A** → Turn left
- **D** → Turn right
- **Q** → Stop

---
## 8. make a node to read the robot position from `/odom` call it `odom_listener.py` then modify the entry point in `setup.py` then run it:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdomListener(Node):
    def __init__(self):
        super().__init__('odom_listener')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        self.subscription  # Prevent unused variable warning
        self.get_logger().info("Odom Listener Node Started, waiting for messages...")

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        self.get_logger().info(f"Position -> x: {position.x:.2f}, y: {position.y:.2f}, z: {position.z:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = OdomListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

```

## 9. Conclusion
With this tutorial, you have:
- Created a **custom ROS2 Python node** for controlling the robot
- Used **keyboard input to send velocity commands**
- Published **/cmd_vel** messages programmatically

