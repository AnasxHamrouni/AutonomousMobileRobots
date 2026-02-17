# ROS2 Gazebo Simulation Tutorial (Extended)

## 1. Introduction
This tutorial expands on integrating Gazebo with ROS2 for realistic robot simulation. Now, we will also:
- Apply Gazebo-specific colors
- Add a differential drive plugin for motion control
- Create and launch a custom world in Gazebo
---


## 2. Fixing the Colors with Gazebo Material
Gazebo does not use standard URDF colors. To apply color in Gazebo, inside robot tag add:
```xml
<gazebo reference="base_link">
    <material>Gazebo/Blue</material>
</gazebo>
<gazebo reference="right_wheel_link">
    <material>Gazebo/Gray</material>
</gazebo>
<gazebo reference="left_wheel_link">
    <material>Gazebo/Gray</material>
</gazebo>
<gazebo reference="caster_wheel_link">
    <material>Gazebo/Gray</material>
</gazebo>
```
Save these configurations in **mobile_base_gazebo.xacro**.

---

## 3. Adding a Gazebo Plugin for Motion Control
To move the robot, add a **differential drive** plugin in **mobile_base_gazebo.xacro**:
```xml
    <gazebo>
        <plugin name="diff_drive_controller" filename="libgazebo_ros_diff_drive.so">
            <!-- Update rate in Hz -->
            <update_rate>50</update_rate>
            <!-- wheels -->
            <left_joint>base_left_wheel_joint</left_joint>
            <right_joint>base_right_wheel_joint</right_joint>
            <!-- kinematics -->
            <wheel_separation>0.45</wheel_separation>
            <wheel_diameter>0.2</wheel_diameter>
            <!-- output -->
            <publish_odom>true</publish_odom>
            <publish_odom_tf>true</publish_odom_tf>
            <publish_wheel_tf>true</publish_wheel_tf>
            <odometry_topic>odom</odometry_topic>
            <odometry_frame>odom</odometry_frame>
            <robot_base_frame>base_footprint</robot_base_frame>
        </plugin>
    </gazebo>

```
- This listens to `/cmd_vel` and moves the robot based on velocity commands.

---

## 4. Controlling the Robot
Run the simulation:
```bash
ros2 launch my_robot_bringup my_robot_gazebo.launch.xml
```
show the topics available :
```bash
ros2 topic list -t
```
show info about a topic `/cmd_vel`
```bash
ros2 topic info /cmd_vel
```
show the data type for this topic
```bash
ros2 interface show geometry_mmmsgs/msg/Twist
```


Send movement commands:
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}, angular: {z: 0.0}}'
```
- This moves the robot **forward** at **0.5 m/s**.
- Modify `angular.z` for **rotation**.

---

## 5. Creating a Custom World
To enhance the simulation, create a custom **world** in Gazebo:
1. Start Gazebo and add objects:
   - `Insert > Bookshelf`
   - `Insert > Walls`
   - Use `Building Editor` to create rooms.
2. Save the world:
```bash
File > Save World As
```
3. Move the saved `.world` file to the **my_robot_bringup/worlds/** folder.

---

## 6. Launch the Robot in the World
Modify the launch file to load the custom world:
```xml
<include file="$(find-pkg-share gazebo_ros)/launch/gazebo.launch.py">
    <arg name="world" value="$(find-pkg-share my_robot_bringup)/worlds/my_world.world"/>
</include>
```
Now, launch the robot in the custom world:
```bash
ros2 launch my_robot_bringup my_robot_gazebo.launch.xml
```
The robot will spawn inside the custom world with physics and collision.

---

## Conclusion
With these improvements, the simulation is now **more stable, visually accurate, and interactive**. You have:
- Applied **realistic colors**.
- Enabled **motion control**.
- Created a **custom environment**.

