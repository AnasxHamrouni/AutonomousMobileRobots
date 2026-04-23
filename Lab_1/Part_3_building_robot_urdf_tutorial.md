# Autonomous Mobile Robotics - S24
**Instructor : Karam Almaghout** , 
**TA : Ghadeer Issa**

# Step-by-Step Tutorial: Building a Complete Robot URDF File

## Overview

In this lesson, you will learn how to:

1. Create a URDF file.
2. Add links to the robot.
3. Connect the links with joints.
4. Visualize the URDF with RViz.
5. Add wheels to the robot and configure them for proper placement and rotation.

---

## Step 1: Create the URDF File

1. **Open a terminal**.
2. Navigate to the desired directory (e.g., your home directory).
   - For now, no ROS package is required; we'll focus on creating the URDF.
3. Create a new file named `my_robot.urdf`.
   - Example:
     ```bash
     touch ~/my_robot.urdf
     ```
4. Open the file using your preferred text editor (e.g., Visual Studio Code).
   - Example:
     ```bash
     code ~/my_robot.urdf
     ```

---

## Step 2: Define the XML Structure

1. Start the file with the XML declaration:

   ```xml
   <?xml version="1.0" ?>
   ```

   - This indicates the file is an XML file.

2. Add the `<robot>` tag:

   ```xml
   <robot name="my_robot">
   </robot>
   ```

   - Replace `my_robot` with your desired robot name.
   - All other tags will go inside the `<robot>` tag.

---

## Step 3: Add the Base Link

1. Add the base link:

   ```xml
   <link name="base_link">
   </link>
   ```

   - A **link** represents a rigid part of the robot. The first link is typically named `base_link`.

2. Add a visual representation to the link:

   ```xml
   <link name="base_link">
       <visual>
           <geometry>
               <box size="0.6 0.4 0.2" />
           </geometry>
           <origin xyz="0 0 0" rpy="0 0 0" />
           <material name="blue">
               <color rgba="0 0 1 1" />
           </material>
       </visual>
   </link>
   ```

   - **Visual tag**: Specifies the appearance of the link.
   - **Geometry tag**: Describes the shape. Here, a box is used.
     - `size`: Length, width, and height in meters.
     - Example: A box 60 cm long, 40 cm wide, and 20 cm high.
   - **Origin tag**: Specifies the offset and rotation of the visual element relative to the link.
     - `xyz`: Translation (meters).
     - `rpy`: Roll, pitch, yaw (radians).
   - **Material tag**: Adds color to the link.

---

## Step 4: Add a Second Link

1. Add the second link:

   ```xml
   <link name="second_link">
       <visual>
           <geometry>
               <cylinder radius="0.1" length="0.2" />
           </geometry>
           <origin xyz="0 0 0" rpy="0 0 0" />
           <material name="gray">
               <color rgba="0.5 0.5 0.5 1" />
           </material>
       </visual>
   </link>
   ```

   - **Cylinder geometry**: Defines a cylinder with a radius of 0.1 meters and a length of 0.2 meters.
   - **Material tag**: Provides a gray color to the second link.

2. Save the file.

---

## Step 5: Connect the Links with a Joint

1. Add a joint to connect the two links:

   ```xml
   <joint name="base_second_joint" type="fixed">
       <parent link="base_link" />
       <child link="second_link" />
       <origin xyz="0 0 0.2" rpy="0 0 0" />
   </joint>
   ```

   - **Name**: Use a descriptive name, e.g., `base_second_joint`.
   - **Type**: Set to `fixed` to indicate no relative motion between the links.
   - **Parent and child**: Define the relationship between `base_link` and `second_link`.
   - **Origin**: Place the joint at the top of the base link (Z-offset = height of the base link).

2. Save the file.

---

## Step 6: Add a Wheel to the Robot

1. Add a right wheel link:

   ```xml
   <link name="right_wheel_link">
       <visual>
           <geometry>
               <cylinder radius="0.1" length="0.05" />
           </geometry>
           <origin xyz="0 0 0" rpy="0 0 0" />
           <material name="gray">
               <color rgba="0.5 0.5 0.5 1" />
           </material>
       </visual>
   </link>
   ```

   - **Geometry**: Cylinder with a radius of 0.1 meters and length of 0.05 meters.

2. Add a joint to attach the wheel to the base link:

   ```xml
   <joint name="base_right_wheel_joint" type="continuous">
       <parent link="base_link" />
       <child link="right_wheel_link" />
       <origin xyz="-0.15 -0.2 0.05" rpy="0 0 0" />
       <axis xyz="0 1 0" />
   </joint>
   ```

   - **Type**: `continuous` to allow infinite rotation.
   - **Origin**: Offset the wheel to the side and slightly back.
   - **Axis**: Specify rotation around the Y-axis.

3. Save the file.

---

## Step 7: Visualize the Updated URDF

1. Ensure you have the `urdf_tutorial` package installed:

   ```bash
   sudo apt install ros-humble-urdf-tutorial
   ```

2. Launch the URDF in RViz:

   ```bash
   ros2 launch urdf_tutorial display.launch.py model:=~/my_robot.urdf
   ```

3. Observe the results:

   - Verify the wheel is correctly attached and can rotate.
   - Check that the TF tree includes the new joint and link.

4. Adjust and re-run as needed:

   - Modify joint origins or visual offsets to improve placement.

---

## Notes

- Use the metric system (meters) for all dimensions.
- Follow XML syntax carefully:
  - Open and close all tags properly.
  - Use consistent indentation for readability.
- First, ensure the TF tree is correct by adjusting joint origins. Then fix visual offsets if needed.
- Refer to URDF documentation for additional tags and features.

---