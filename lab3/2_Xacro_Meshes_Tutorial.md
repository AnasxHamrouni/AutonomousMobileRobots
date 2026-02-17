# ROS2 URDF, Xacro, and Meshes Tutorial

## 1. Introduction
In this tutorial, you will learn how to enhance your URDF with Xacro for modularity, create reusable macros, and incorporate real 3D meshes.

We will cover:
- Making URDF compatible with Xacro
- Creating variables and macros in Xacro
- Splitting URDF into multiple files
- Generating URDF from Xacro
- Using real 3D meshes in URDF

---

## 2. Making the URDF Compatible with Xacro
### Install Xacro
```bash
sudo apt install ros-humble-xacro
```
### Convert URDF to Xacro
- Rename `my_robot.urdf` to `my_robot.urdf.xacro`
- Modify the robot tag:
```xml
<robot name="my_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
```
- Update launch files to use `.xacro` extension.

---

## 3. Creating Variables with Xacro Properties
Define properties for reusability:
```xml
<xacro:property name="base_length" value="0.6"/>
```
Use it in the URDF:
```xml
<size>${base_length} 0.4 0.2</size>
```
also PI is already defined you can use it as follows:

```xml
${pi}
```

---

## 4. Creating Functions with Xacro Macros
Define a macro for wheels:
```xml
<xacro:macro name="wheel_link" params="prefix">
    <link name="${prefix}_wheel_link">
        <visual>
            <geometry>
                <cylinder radius="0.1" length="0.05"/>
            </geometry>
            <origin xyz="0 0 0" rpy="${pi / 2.0 } 0 0"/>
        </visual>
    </link>
</xacro:macro>
```
Call the macro:
```xml
<xacro:wheel_link prefix="right"/>
<xacro:wheel_link prefix="left"/>
```

---

## 5. Splitting URDF into Multiple Files
### Create modular Xacro files:
- `common_properties.xacro` (stores colors, common values)
- `mobile_base.xacro` (contains the base structure)
- `my_robot.urdf.xacro` (main file, includes others)

Include Xacro files:
```xml
<xacro:include filename="common_properties.xacro"/>
<xacro:include filename="mobile_base.xacro"/>
```

Check the generated URDF file:
```bash
ros2 param get /robot_state_publisher robot_description
```
---

## 6. Generating URDF from Xacro
Check the generated URDF:
```bash
ros2 run xacro xacro my_robot.urdf.xacro > my_robot.urdf
```
This converts the Xacro file into a static URDF for robot state publisher.

---

## 7. Using Real 3D Meshes in URDF
Instead of basic shapes, use CAD-exported meshes.

### Add a Mesh to a Link:
```xml
<visual>
    <geometry>
        <mesh filename="package://my_robot_description/meshes/base.stl" scale="0.01 0.01 0.01"/>
    </geometry>
</visual>
```
### Adjust Orientation: you need to correct the values for origin
```xml
<origin xyz=". . ." rpy="0 0 ${pi / 2}"/>
```

---

## Conclusion
With these techniques, your robot description becomes more modular, scalable, and visually accurate. You are now prepared to integrate your URDF into Gazebo
