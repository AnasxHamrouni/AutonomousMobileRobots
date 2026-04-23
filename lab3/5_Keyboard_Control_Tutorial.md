# ROS2 Keyboard Control for Gazebo-Simulated Robot

## 1. Introduction
This tutorial explains how to **control a ROS2 robot using a keyboard** in the Gazebo simulation. We will:
- Use `teleop_twist_keyboard` to send velocity commands
- Set up the ROS2 `cmd_vel` topic
- Launch the robot and control it interactively

---

## 2. Install the Teleop Package
Ensure you have the **teleop_twist_keyboard** package installed:
```bash
sudo apt install ros-humble-teleop-twist-keyboard
```

---

## 3. Launch the Robot in Gazebo
Start your robot simulation:
```bash
ros2 launch my_robot_bringup my_robot_gazebo.launch.xml
```
Wait for Gazebo to load the robot in the environment.

---

## 4. Run the Keyboard Control Node
In a new terminal, run:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

This will display:
```
Reading from the keyboard and publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

q/z : increase/decrease max speeds
w/x : increase/decrease only linear speed
e/c : increase/decrease only angular speed
CTRL-C to quit
```

---

## 5. Understanding the Commands
- **i** → Move forward
- **,** → Move backward
- **j** → Turn left
- **l** → Turn right
- **u/o/m/.** → Move diagonally
- **k** → Stop the robot
- **q/z** → Increase/decrease max speed

---

## 6. How It Works
This package publishes **velocity commands** (`Twist` messages) to:
```
/cmd_vel
```
The robot’s **differential drive plugin** listens to `/cmd_vel` and moves accordingly.

---

## 7. Manually Publish a Velocity Command (Optional)
You can manually send a velocity command:
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}, angular: {z: 0.0}}'
```
This moves the robot **forward** at **0.5 m/s**.

---

## 8. Conclusion
You can now **control the robot in Gazebo using your keyboard** ! 

