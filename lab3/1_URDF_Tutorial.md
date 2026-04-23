# ROS2 URDF and Robot State Publisher Tutorial

## 1. Introduction
In this tutorial, you will learn how to publish TF using the Robot State Publisher in ROS2.

We will cover:
- Running the Robot State Publisher
- Writing launch files in XML and Python
- Configuring RViz for visualization

---

## 2. How the Robot State Publisher and URDF Work Together
Before running the URDF, let's understand how the Robot State Publisher works:
- The `robot_state_publisher` node reads the URDF and publishes the TF transformations.
- The `joint_state_publisher` node provides joint state information.
- The `TF` topic is used to visualize transformations.
- `RQT Graph` helps visualize node connections.

---

## 3. Running the Robot State Publisher with URDF in the Terminal
To run the publisher manually:
```bash
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro my_robot.urdf)"
```
- `xacro` processes the URDF before passing it.
- Start `joint_state_publisher`:
```bash
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```
- Launch RViz for visualization:
```bash
ros2 run rviz2 rviz2
```

- previously we used to launch the URDF in RViz the following launch file:

```bash
ros2 launch urdf_tutorial display.launch.py model:=~/my_robot.urdf
```

---
## 4. Creating a Robot Description Package

Navigate to the ROS2 workspace and create the package:
```bash
mkdir -p ~/AMR_ws/src
cd ~/AMR_ws/src
ros2 pkg create --build-type ament_python my_robot_description
```

- Create a folder for the URDF files and move your URDF into it:
```bash
mkdir -p my_robot_description/urdf
mv my_robot.urdf my_robot_description/urdf/
```

- Replace the contents of `setup.py` with:
```python
from setuptools import setup
import os
from glob import glob

package_name = 'my_robot_description'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Python package for robot description with URDF',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
```

This ensures that the **URDF files** are installed in `share/my_robot_description/urdf/`.


- Build the package using `colcon`:
```bash
cd ~/AMR_ws
colcon build --packages-select my_robot_description
```
<!-- ## 4. Creating a Robot Description Package
Create a new ROS2 package for storing URDF files:
```bash
mkdir -p ~/AMR_ws/src
cd ~/AMR_ws/src
ros2 pkg create my_robot_description
```
- Move the URDF file into the package:
```bash
mv my_robot.urdf ~/AMR_ws/src/my_robot_description/urdf/
```
- Modify `CMakeLists.txt`:
```cmake
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})
```
- Build the package:
```bash
cd ~/AMR_ws
colcon build
``` -->

---

## 5. Writing an XML Launch File
Create a launch folder in the package and add `display.launch.xml`:
```xml
<launch>
    <let name="urdf_path" value="$(find-pkg-share my_robot_description)/urdf/my_robot.urdf"/>
    <node pkg="robot_state_publisher" exec="robot_state_publisher">
        <param name="robot_description" value="$(command 'xacro $(var urdf_path)')"/>
    </node>
    <node pkg="joint_state_publisher_gui" exec="joint_state_publisher_gui"/>
    <node pkg="rviz2" exec="rviz2"/>
</launch>
```
To run the launch file:
```bash
ros2 launch my_robot_description display.launch.xml
```
* if you face an error you need to include the launch file in the setup file
    
    <!-- ```python
    (os.path.join('share', package_name, 'launch'), glob('launch/*.xml')),
    ``` -->
---

## 6. Writing a Python Launch File
Create `display.launch.py`:
```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    urdf_path = "<path-to-urdf>"
    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)
    
    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}]
        ),
        Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui"),
        Node(package="rviz2", executable="rviz2")
    ])
```
```python
    from launch_ros.substitutions import FindPackageShare
    from launch.substitutions import PathJoinSubstitution
    urdf_path = PathJoinSubstitution([
        FindPackageShare("my_robot_description"),
        "urdf",
        "my_robot.urdf"
    ])
```


Run:
```bash
ros2 launch my_robot_description display.launch.py
```

---

## 7. Adding RViz Configuration
To save an RViz configuration:
- Open RViz and configure it.
- Save the configuration as `urdf_config.rviz` in the package.
- Modify `display.launch.xml`:
```xml
<let name="rviz_config_path" value="$(find-pkg-share my_robot_description)/rviz/urdf_config.rviz"/>
<node pkg="rviz2" exec="rviz2" args="-d $(var rviz_config_path)"/>
```
- Modify `display.launch.py`:
```python
rviz_config_path = "<path-to-rviz-config>"
Node(package="rviz2", executable="rviz2", arguments=["-d", rviz_config_path])
```

Now, RViz will load the saved configuration on startup.

---

## Conclusion
You have successfully:
- Created a URDF-based robot model
- Launched it using Robot State Publisher
- Visualized it in RViz
- Used XML and Python launch files
- Configured RViz to load automatically

This tutorial serves as a base for developing more advanced ROS2 applications.
