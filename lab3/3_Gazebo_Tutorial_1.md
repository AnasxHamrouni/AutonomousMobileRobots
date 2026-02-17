# ROS2 Gazebo Simulation Tutorial

## 1. Introduction
In this tutorial, you will learn how to integrate Gazebo with ROS2 for realistic robot simulation.

We will cover:
- Running and understanding Gazebo
- Adding inertia and collision properties to URDF
- Spawning a robot in Gazebo
- Creating a launch file for Gazebo

---

## 2. Running Gazebo
### Install Gazebo (if not installed)
```bash
sudo apt install ros-humble-gazebo-ros-pkgs
```
### Start Gazebo
```bash
gazebo
```
or using ROS2: `may cause problems`
```bash
ros2 launch gazebo_ros gazebo.launch.py
```

---

## 3. How Gazebo Works with ROS
Gazebo is a physics simulator independent of ROS. To integrate it with ROS, we use:
- **robot_state_publisher** (publishes TF)
- **Gazebo ROS bridge** (connects ROS topics to Gazebo)
- **Gazebo plugins** (simulate hardware)

---

## 4. Adding Inertia and Collision Tags in URDF
### Add Inertia
```xml
<inertial>
    <mass value="5.0"/>
    <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
</inertial>
```
### Add Collision ` you need to fix the origin of the collison box`
```xml
<collision>
    <geometry>
        <box size="0.6 0.4 0.2"/>
    </geometry>
    <origin xyz="0 0 0" rpy="0 0 0" />
</collision>
```
- **Inertia** defines mass distribution.
- **Collision** defines physical interactions.

---

## 5. Spawning the Robot in Gazebo
### Start Robot State Publisher
```bash
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro my_robot.urdf.xacro)"
```
### Start Gazebo
```bash
ros2 launch gazebo_ros gazebo.launch.py
```
### Spawn the Robot
```bash
ros2 run gazebo_ros spawn_entity.py -topic robot_description -entity my_robot
```

---

## 6. Creating a Launch File for Gazebo
### Create a Bring-up Package
```bash
ros2 pkg create my_robot_bringup
```
### Create a Launch File `my_robot_gazebo.launch.xml`
```xml
<launch>


    <let name="urdf_path"
        value="$(find-pkg-share my_robot_description)/urdf/my_robot.urdf.xacro"/>

    <node pkg="robot_state_publisher" exec="robot_state_publisher">
        <param name="robot_description" 
                value="$(command 'xacro $(var urdf_path)')"/>
    </node>

    <include file="$(find-pkg-share gazebo_ros)/launch/gazebo.launch.py"/>
    <node pkg="gazebo_ros" exec="spawn_entity.py" args="-topic robot_description -entity my_robot"/>
</launch>
```

---

### modify `package.xml` File:
```xml
<exec_depend>my_robot_description</exec_depend>
<exec_depend>robot_state_publisher</exec_depend>
<exec_depend>gazebo_ros</exec_depend>
```

modify  `setup.py` file then build and run:
```bash
ros2 launch my_robot_bringup my_robot_gazebo.launch.xml
```
---

## Conclusion
You have now:
- Integrated URDF with Gazebo
- Added physical properties for realistic simulation
- Automated robot spawning with a launch file
