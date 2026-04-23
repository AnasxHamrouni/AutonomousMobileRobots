# Adding an IMU Sensor to Your Robot in Gazebo.

This tutorial shows how to add an **IMU sensor** ( a simulated generic IMU) to your existing robot in **Gazebo Classic**.

---

## **1.1. Create IMU Xacro**
Create a new **Xacro** file (`imu.xacro`) in your robot’s **urdf/**  folder for the **IMU link**, **joint**, and **plugin configuration**.

### **Add an IMU link and fixed joint** 
Attach the IMU to `base_link`:

```xml
<!-- File: imu.xacro -->
<xacro:property name="imu_offset_x" value="0.0"/>
<xacro:property name="imu_offset_y" value="0.0"/>
<xacro:property name="imu_offset_z" value="0.1"/> <!-- e.g., 10 cm above base -->

<joint name="imu_joint" type="fixed">
  <parent link="base_link"/>
  <child link="imu_link"/>
  <origin xyz="${imu_offset_x} ${imu_offset_y} ${imu_offset_z}" rpy="0 0 0"/>
</joint>

<link name="imu_link">
  <visual>
    <geometry>
      <box size="0.03 0.03 0.01"/>
    </geometry>
    <material name="blue"/>
  </visual>
  <collision>
    <geometry>
      <box size="0.03 0.03 0.01"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.01"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <inertia ixx="1e-6" iyy="1e-6" izz="1e-6" ixy="0" ixz="0" iyz="0"/>
  </inertial>
</link>
```

### **Add the Gazebo IMU plugin**
Inside the `<gazebo>` tag, reference `imu_link` and use the **IMU plugin** `libgazebo_ros_imu_sensor.so`:

```xml
<gazebo reference="imu_link">
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>50</update_rate> <!-- e.g., 50 Hz -->
    <visualize>true</visualize>
    <pose>0 0 0 0 0 0</pose>
    <plugin name="gazebo_ros_imu" filename="libgazebo_ros_imu_sensor.so">
      <ros>
        <!-- The plugin's default output topic is ~/out; we remap to /imu/data -->
        <remapping>~/out:=imu/data</remapping>
      </ros>
      <frame_name>imu_link</frame_name>
      <enable_orientation>true</enable_orientation>
      <enable_angular_velocity>true</enable_angular_velocity>
      <enable_linear_acceleration>true</enable_linear_acceleration>
    </plugin>
  </sensor>
</gazebo>
```

Explanation:
- The **sensor type** is `imu`.
- `update_rate` sets how often the data is published (adjustable, e.g., 50 Hz).
- The **remapping** ensures that the output topic `/imu/data` is published as `sensor_msgs/Imu` messages.
  
### **Include imu.xacro in your main robot Xacro**
In your **main robot Xacro**, include the **IMU Xacro** 

---

## **1.2. Launch Your Robot in Gazebo**
Build and launch your robot as usual.

### **Verify the IMU data on the topic**
Check if the IMU topic is available:

```bash
ros2 topic list | grep imu
ros2 topic echo /imu/data
```

View the IMU sensor output:

```bash
ros2 topic echo /imu/data
```

Expected output:
```
header:
  frame_id: "imu_link"
orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
angular_velocity: {x: 0.01, y: -0.02, z: 0.05}
linear_acceleration: {x: 0.0, y: 0.0, z: -9.81}
```
---



## **Conclusion**
At this point, you have successfully:
- Added an **IMU sensor** to your robot in **Gazebo**.
- Configured a **ROS2 Gazebo plugin** to publish IMU data.
- Visualized the **IMU sensor data** in **RViz2**.
---

This setup allows you to integrate IMU data into:
- **Odometry fusion (e.g., with `robot_localization`)**.
- **Sensor-based control systems**.
- **SLAM and navigation algorithms**.