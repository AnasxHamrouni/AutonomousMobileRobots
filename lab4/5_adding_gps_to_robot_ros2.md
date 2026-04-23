# Adding a GPS Sensor to Your Robot in Gazebo 

## **Introduction**
In this tutorial, we will integrate a **GPS sensor** into  **Gazebo**.
GPS sensors provide **global positioning** (latitude, longitude, altitude). In simulation, we emulate this using **Gazebo's GPS sensor**.

---

## **Step 1: Create GPS Xacro**

### **1.1 Create `gps.xacro`**
Create a new **Xacro file** (`gps.xacro`) inside your **urdf/**  folder to define the GPS sensor’s link, joint, and plugin configuration.

#### **GPS Link and Fixed Joint**
Attach the **GPS link** to the `base_link` using a **fixed joint**.

```xml
<!-- File: gps.xacro -->
<xacro:property name="gps_offset_x" value="0.0"/>
<xacro:property name="gps_offset_y" value="0.0"/>
<xacro:property name="gps_offset_z" value="0.3"/> <!-- e.g., 30 cm above base -->

<joint name="gps_joint" type="fixed">
  <parent link="base_link"/>
  <child link="gps_link"/>
  <origin xyz="${gps_offset_x} ${gps_offset_y} ${gps_offset_z}" rpy="0 0 0"/>
</joint>

<link name="gps_link">
  <visual>
    <geometry>
      <cylinder radius="0.02" length="0.1"/>
    </geometry>
    <material name="yellow"/>
  </visual>
  <collision>
    <geometry>
      <cylinder radius="0.02" length="0.1"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.05"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <inertia ixx="1e-5" iyy="1e-5" izz="1e-5" ixy="0" ixz="0" iyz="0"/>
  </inertial>
</link>
```

---

### **1.2 Add the Gazebo GPS Plugin**
Inside `gps.xacro`, add the following **Gazebo sensor plugin**:

```xml
<gazebo reference="gps_link">
  <sensor name="gps_sensor" type="gps">
    <always_on>true</always_on>
    <update_rate>1</update_rate> <!-- 1 Hz update rate -->
    <visualize>false</visualize>
    <plugin name="gazebo_ros_gps" filename="libgazebo_ros_gps_sensor.so">
      <ros>
        <!-- The plugin’s default output is ~/out; we remap it to /gps/fix -->
        <remapping>~/out:=gps/fix</remapping>
      </ros>
      <frame_name>gps_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

### **1.3 Explanation**
- The **sensor type is "gps"**, which simulates a **global positioning sensor**.
- `update_rate`: Determines the **GPS data publishing frequency** (**1 Hz** is typical for real GPS sensors, but can be increased).
- `~/out:=gps/fix`: **Remaps the output topic** to `/gps/fix`, so **ROS2 publishes it as `sensor_msgs/NavSatFix`**.

### **1.4 Include `gps.xacro` in Your Main Robot Xacro**
Modify your **main robot Xacro file** (e.g., `robot.urdf.xacro`) to include the GPS definition:

---

## **Step 2: Test in Gazebo**

### **2.1 Build and Launch**

### **2.2 Verify the GPS Data**
Check if the GPS topic is active:

```bash
ros2 topic list | grep gps
```

View the GPS sensor output:

```bash
ros2 topic echo /gps/fix
```

Expected output:
```
header:
  frame_id: "gps_link"
latitude: 48.8584
longitude: 2.2945
altitude: 35.0
```
(Note: The latitude, longitude, and altitude will depend on Gazebo’s world coordinates.)

---

## **Conclusion**
✅ **Successfully added a GPS sensor to the robot in Gazebo**.  
✅ **GPS data is published on `/gps/fix`** as `sensor_msgs/NavSatFix`.  

This setup allows you to use GPS for:
- **Localization**
- **Outdoor navigation**
- **Fusion with IMU & Odometry** (e.g., with `robot_localization` package).

