# ROS2 Gazebo Simulation Tutorial (Extended with Camera Sensor)

## 1. Introduction
This tutorial builds upon our existing Gazebo simulation and adds **sensor integration**, specifically a **camera**. 
We will:
- Add a camera to the URDF
- Integrate a Gazebo camera plugin
- Publish camera data to ROS topics
- Fix the camera coordinate convention for OpenCV compatibility

---

## 2. Adding a Camera to the URDF
To integrate a camera, we need to:
1. Create a **new link** for the camera.
2. Attach it to the **base link** using a **fixed joint**.

### Define the Camera Link
In **camera.xacro**:
```xml
    <xacro:property name="camera_length" value="0.01" />
    <xacro:property name="camera_width" value="0.1" />
    <xacro:property name="camera_height" value="0.05" />

    <link name="camera_link">
        <visual>
            <geometry>
                <box size="${camera_length} ${camera_width} ${camera_height}" />
            </geometry>
            <material name="grey" />
        </visual>
        <collision>
            <geometry>
                <box size="${camera_length} ${camera_width} ${camera_height}" />
            </geometry>
        </collision>
        <xacro:box_inertia m="0.1" l="${camera_length}" w="${camera_width}" h="${camera_height}"
                           xyz="0 0 0" rpy="0 0 0" />
    </link>

    <joint name="base_camera_joint" type="fixed">
        <parent link="base_link" />
        <child link="camera_link" />
        <origin xyz="${(base_length + camera_length) / 2.0} 0 ${base_height / 2.0}" rpy="0 0 0" />
    </joint>

    <link name="camera_link_optical">
    </link>

    <joint name="camera_optical_joint" type="fixed">
        <!-- these values have to be these values otherwise the gazebo camera
            image won't be aligned properly with the frame it is supposedly
            originating from -->
        <origin xyz="0 0 0" rpy="${-pi/2} 0 ${-pi/2}"/>
        <parent link="camera_link"/>
        <child link="camera_link_optical"/>
    </joint>
```
---

## 3. Adding a Gazebo Plugin for the Camera
We now add a **Gazebo sensor plugin** to simulate the camera.

```xml
    <gazebo reference="camera_link">
        <material>Gazebo/Red</material>
        <sensor name="camera_sensor" type="camera">
            <pose>0 0 0 0 0 0</pose>
            <visualize>true</visualize>
            <update_rate>10.0</update_rate>
            <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
                <frame_name>camera_link_optical</frame_name>
            </plugin>
        </sensor>
    </gazebo>
```
- **update_rate**: Defines the FPS (frames per second).
- **image width/height**: Sets the resolution (640x480).
- **plugin**: Uses `gazebo_ros_camera` to publish camera data to ROS.

---

## 4. Launching and Visualizing Camera Data
### Start the Simulation:
```bash
ros2 launch my_robot_bringup my_robot_gazebo.launch.xml
```

### Check Camera Topics:
```bash
ros2 topic list | grep camera
```
You should see:
```
/camera_sensor/image_raw
/camera_sensor/camera_info
```

### Visualize the Camera in RViz:
1. Add **Image** in RViz
2. Select **/camera_sensor/image_raw** as the topic

---

## 5. Fixing Camera Orientation for OpenCV Compatibility
ROS and OpenCV use different coordinate conventions:
- **ROS**: X (forward), Y (left), Z (up)
- **OpenCV**: Z (forward), X (right), Y (down)

### Solution: Add a Rotated Optical Frame
```xml
<link name="camera_optical_link"/>
<joint name="camera_optical_joint" type="fixed">
    <parent link="camera_link"/>
    <child link="camera_optical_link"/>
    <origin xyz="0 0 0" rpy="-1.57 0 -1.57"/>
</joint>
```

### Update the Plugin Frame
Modify the camera plugin:
```xml
<plugin name="camera_controller" filename="libgazebo_ros_camera.so">
    <frame_name>camera_optical_link</frame_name>
</plugin>
```
This ensures that OpenCV-based processing tools receive correctly aligned images.

---

## Conclusion
With this tutorial, you have:
- Added a **camera** to the URDF
- Simulated it using **Gazebo plugins**
- Published **image data to ROS topics**
- Fixed the **coordinate convention** for OpenCV

