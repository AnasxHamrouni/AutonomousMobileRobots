# Simulating a LIDAR in Gazebo Classic with ROS2 Humble

## **Introduction**
This tutorial demonstrates how to add a simulated 2D LiDAR inside **Gazebo** and **RViz2**.

---

## **Step 1: Add the LiDAR Sensor to the Robot URDF**

We will add a **LiDAR sensor link** and attach it to an existing **base_link** using a **fixed joint**.

### **1.1 Create a New Xacro File for the LiDAR**

Inside your **robot’s description package** (e.g., `urdf/`), create a new file: **`lidar.xacro`**.

```xml
<!-- File: lidar.xacro -->
<xacro:property name="lidar_height" value="0.15"/> <!-- 15 cm above base -->

<joint name="laser_joint" type="fixed">
    <parent link="base_link"/> 
    <child link="laser_frame"/> 
    <origin xyz="0 0 ${lidar_height}" rpy="0 0 0"/> 
</joint>

<link name="laser_frame">
    <visual>
        <geometry>
            <cylinder radius="0.05" length="0.04"/>
        </geometry>
        <material name="black"/>
    </visual>
    <collision>
        <geometry>
            <cylinder radius="0.05" length="0.04"/>
        </geometry>
    </collision>
    <inertial>
        <mass value="0.1"/>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <inertia ixx="0.001" iyy="0.001" izz="0.001" ixy="0" ixz="0" iyz="0"/>
    </inertial>
</link>
```

### **1.2 Include the LiDAR Xacro in the Main Robot Xacro**
Modify your **main robot URDF/Xacro file** (e.g., `robot.urdf.xacro`) to include the **lidar.xacro**:
```xml
<xacro:include filename="lidar.xacro"/>
```

---

## **Step 2: Configure the Gazebo Plugin for LiDAR Simulation**

### **2.1 Add the Gazebo Sensor Plugin**
Modify **`lidar.xacro`** to include a **Gazebo sensor definition**:

```xml
<gazebo reference="laser_frame">
    <sensor name="lidar_sensor" type="ray">
        <pose>0 0 0 0 0 0</pose>
        <visualize>true</visualize>
        <update_rate>10.0</update_rate>
        <ray>
            <scan>
                <horizontal>
                    <samples>360</samples>
                    <resolution>1</resolution>
                    <min_angle>-3.14159</min_angle>
                    <max_angle>3.14159</max_angle>
                </horizontal>
            </scan>
            <range>
                <min>0.1</min>
                <max>12.0</max>
                <resolution>0.01</resolution>
            </range>
        </ray>
        <plugin name="lidar_plugin" filename="libgazebo_ros_ray_sensor.so">
            <ros>
                <remapping>~/out:=scan</remapping>
            </ros>
            <output_type>sensor_msgs/LaserScan</output_type>
            <frame_name>laser_frame</frame_name>
        </plugin>
    </sensor>
</gazebo>
```

---

## **Step 3: Launch the Simulation in Gazebo**

### **3.1 Start Gazebo with Your Robot**
```bash
ros2 launch my_robot_bringup my_robot_gazebo.launch.xml
```

### **3.2 Verify the LiDAR Topic**
Run:
```bash
ros2 topic list | grep scan
```
You should see **`/scan`**.

Check LiDAR output:
```bash
ros2 topic echo /scan --no-arr
```

Expected output:
```
header:
  frame_id: "laser_frame"
angle_min: -3.14159
angle_max: 3.14159
ranges: [ ... 360 values ... ]
```

---

## **Step 4: Visualize the LiDAR in RViz2**

### **4.1 Launch RViz2**
```bash
rviz2
```

### **4.2 Configure RViz2**
- **Set Fixed Frame**: `odom` or `base_link`.
- **Add a LaserScan Display**:
  - Click `Add → By Topic`.
  - Select `/scan` (Type: `sensor_msgs/LaserScan`).

You should see a **360-degree scan** around the robot.

---

## **Step 5: Verify LiDAR Data with a Python Node**

### **5.1 Create a ROS2 Package for LiDAR Testing**
```bash
ros2 pkg create --build-type ament_python lidar_tester --dependencies rclpy sensor_msgs
```

### **5.2 Write a LiDAR Subscriber Node**

Inside `lidar_tester/lidar_tester/`, create `scan_listener.py`:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LidarListener(Node):
    def __init__(self):
        super().__init__('lidar_listener')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg: LaserScan):
        ranges = list(msg.ranges)
        valid_ranges = [r for r in ranges if r < float('inf')]
        if not valid_ranges:
            self.get_logger().info("No obstacles detected.")
            return
        min_dist = min(valid_ranges)
        self.get_logger().info(f"Min Distance: {min_dist:.2f} m")

def main(args=None):
    rclpy.init(args=args)
    node = LidarListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### **5.3 Build and Run the Node**
```bash
colcon build --packages-select lidar_tester
source install/setup.bash
ros2 run lidar_tester scan_listener
```

### **5.4 Expected Output**
```
[INFO] [lidar_listener]: Min Distance: 2.45 m
```

---

## **Conclusion**

At this point, you have successfully:

✅ **Added an RPLIDAR C1 sensor to a ROS2 robot in Gazebo Classic.**  
✅ **Published simulated LiDAR data to the `/scan` topic.**  
✅ **Visualized the scan data in RViz2.**  
✅ **Verified LiDAR readings with a Python ROS2 node.**  

This setup can now be used for **SLAM, obstacle avoidance, or navigation!** 

