# RTAB-Map with Depth Camera Tutorial

## Prerequisites

* RTAB-Map ROS 2 package (`sudo apt update && sudo apt install ros-humble-rtabmap-ros`)

## Package Setup

1. **Create a ROS 2 package:**

    ```bash
    ros2 pkg create --build-type ament_python rtabmap_diff_drive_tutorial
    ```

2. **Create the following directory structure:**

    ```bash
    rtabmap_diff_drive_tutorial/
    ├── config/
    ├── urdf/
    ├── launch/
    └── worlds/
    ```

## Robot URDF

Create `urdf/robot.urdf.xacro`:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="diff_drive_robot">

    <!-- Constants -->
    <xacro:property name="M_PI" value="3.1415926535897931" />
    <xacro:property name="base_width" value="0.3"/>
    <xacro:property name="base_length" value="0.4"/>
    <xacro:property name="base_height" value="0.1"/>
    <xacro:property name="wheel_radius" value="0.05"/>
    <xacro:property name="wheel_width" value="0.04"/>
    <xacro:property name="caster_radius" value="0.02"/>
    <xacro:property name="base_lift" value="0.01"/>
    <xacro:property name="camera_link_size" value="0.05"/>
    <xacro:property name="imu_link_size" value="0.02"/>

    <!-- Base Link -->
    <link name="base_link">
        <visual>
            <geometry>
                <box size="${base_length} ${base_width} ${base_height}"/>
            </geometry>
            <material name="blue">
                <color rgba="0.0 0.0 0.8 1.0"/>
            </material>
        </visual>
        <collision>
            <geometry>
                <box size="${base_length} ${base_width} ${base_height}"/>
            </geometry>
        </collision>
        <inertial>
            <origin xyz="0 0 0" rpy="0 0 0"/>
            <mass value="5.0"/>
            <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
        </inertial>
    </link>

    <!-- IMU Link -->
    <link name="imu_link">
        <visual>
            <geometry>
                <box size="${imu_link_size} ${imu_link_size} ${imu_link_size}"/>
            </geometry>
            <material name="red">
                 <color rgba="0.8 0.0 0.0 1.0"/>
            </material>
        </visual>
         <collision>
            <geometry>
                <box size="${imu_link_size} ${imu_link_size} ${imu_link_size}"/>
            </geometry>
        </collision>
         <inertial>
            <mass value="0.01"/>
            <inertia ixx="1e-6" ixy="0.0" ixz="0.0" iyy="1e-6" iyz="0.0" izz="1e-6"/>
        </inertial>
    </link>

    <joint name="imu_joint" type="fixed">
        <parent link="base_link"/>
        <child link="imu_link"/>
        <origin xyz="0 0 ${base_height/2 + imu_link_size/2}" rpy="0 0 0"/>
    </joint>

    <!-- Camera Link -->
    <link name="camera_link">
        <visual>
            <geometry>
                <box size="${camera_link_size} ${camera_link_size} ${camera_link_size}"/>
            </geometry>
            <material name="green">
                <color rgba="0.0 0.8 0.0 1.0"/>
            </material>
        </visual>
        <collision>
            <geometry>
                <box size="${camera_link_size} ${camera_link_size} ${camera_link_size}"/>
            </geometry>
        </collision>
        <inertial>
             <mass value="0.1"/>
             <inertia ixx="1e-6" ixy="0.0" ixz="0.0" iyy="1e-6" iyz="0.0" izz="1e-6"/>
        </inertial>
    </link>

    <joint name="camera_joint" type="fixed">
        <parent link="base_link"/>
        <child link="camera_link"/>
        <origin xyz="${base_length/2 - camera_link_size/2} 0 ${base_height/2 + camera_link_size/2}" rpy="0 0 0"/>
    </joint>

    <!-- Camera Optical Frame -->
    <link name="camera_depth_optical_frame"/>
    <joint name="camera_optical_joint" type="fixed">
        <parent link="camera_link"/>
        <child link="camera_depth_optical_frame"/>
        <origin xyz="0 0 0" rpy="${-M_PI/2} 0 ${-M_PI/2}"/>
    </joint>
        <!-- *** ADD LASER SCANNER LINK AND JOINT *** -->
    <link name="laser_link">
        <visual>
            <geometry>
                <cylinder radius="0.03" length="0.05"/>
            </geometry>
            <material name="grey">
                <color rgba="0.5 0.5 0.5 1.0"/>
            </material>
        </visual>
        <collision>
            <geometry>
                <cylinder radius="0.03" length="0.05"/>
            </geometry>
        </collision>
        <inertial>
            <mass value="0.1"/>
            <inertia ixx="1e-6" ixy="0.0" ixz="0.0" iyy="1e-6" iyz="0.0" izz="1e-6"/>
        </inertial>
    </link>

    <joint name="laser_joint" type="fixed">
        <parent link="base_link"/>
        <child link="laser_link"/>
        <!-- Position the laser slightly ahead of the base center and above it -->
        <origin xyz="${base_length/4} 0 ${base_height + 0.025}" rpy="0 0 0"/>
    </joint>
    <!-- *** END LASER SCANNER LINK AND JOINT *** -->
    <!-- Wheels -->
    <xacro:macro name="wheel" params="prefix reflect">
        <link name="${prefix}_wheel_link">
            <visual>
                <origin xyz="0 0 0" rpy="${M_PI/2} 0 0"/>
                <geometry>
                    <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
                </geometry>
                <material name="black">
                    <color rgba="0.1 0.1 0.1 1.0"/>
                </material>
            </visual>
            <collision>
                 <origin xyz="0 0 0" rpy="${M_PI/2} 0 0"/>
                <geometry>
                    <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
                </geometry>
            </collision>
             <inertial>
                <mass value="1.0"/>
                <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
            </inertial>
        </link>

        <joint name="${prefix}_wheel_joint" type="continuous">
            <parent link="base_link"/>
            <child link="${prefix}_wheel_link"/>
            <origin xyz="0 ${reflect*(base_width/2 + wheel_width/2)} ${-(base_height/2) + wheel_radius - base_lift}" rpy="0 0 0"/>
            <axis xyz="0 1 0"/>
        </joint>
    </xacro:macro>

    <xacro:wheel prefix="left" reflect="1"/>
    <xacro:wheel prefix="right" reflect="-1"/>

    <!-- Casters -->
    <xacro:macro name="caster" params="prefix x_offset">
        <link name="${prefix}_caster_link">
            <visual>
                <geometry>
                    <sphere radius="${caster_radius}"/>
                </geometry>
                <material name="grey">
                    <color rgba="0.5 0.5 0.5 1.0"/>
                </material>
            </visual>
            <collision>
                <geometry>
                    <sphere radius="${caster_radius}"/>
                </geometry>
                <surface>
                    <friction>
                        <ode>
                            <mu>0.0</mu>
                            <mu2>0.0</mu2>
                            <slip1>1.0</slip1>
                            <slip2>1.0</slip2>
                        </ode>
                    </friction>
                </surface>
            </collision>
            <inertial>
                <mass value="0.5"/>
                <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
            </inertial>
        </link>

        <joint name="${prefix}_caster_joint" type="fixed">
            <parent link="base_link"/>
            <child link="${prefix}_caster_link"/>
            <origin xyz="${x_offset} 0 ${-(base_height/2) + caster_radius - base_lift}" rpy="0 0 0"/>
        </joint>

        <gazebo reference="${prefix}_caster_link">
            <material>Gazebo/Grey</material>
            <mu1>0.0</mu1>
            <mu2>0.0</mu2>
            <slip1>1.0</slip1>
            <slip2>1.0</slip2>
        </gazebo>
    </xacro:macro>

    <xacro:caster prefix="front" x_offset="${base_length/2 - caster_radius}"/>
    <xacro:caster prefix="rear" x_offset="${-base_length/2 + caster_radius}"/>

    <!-- Gazebo Plugins -->
    <gazebo reference="base_link">
        <material>Gazebo/Blue</material>
    </gazebo>
    <gazebo reference="imu_link">
         <material>Gazebo/Red</material>
    </gazebo>
     <gazebo reference="camera_link">
         <material>Gazebo/Green</material>
    </gazebo>
    <gazebo reference="left_wheel_link">
        <material>Gazebo/Black</material>
    </gazebo>
    <gazebo reference="right_wheel_link">
        <material>Gazebo/Black</material>
    </gazebo>

    <!-- Differential Drive Plugin -->
    <gazebo>
        <plugin name="differential_drive_controller" filename="libgazebo_ros_diff_drive.so">
            <ros>
                <!-- Namespace is empty -->
            </ros>
            <update_rate>100</update_rate>
            <!-- wheels -->
            <left_joint>left_wheel_joint</left_joint>
            <right_joint>right_wheel_joint</right_joint>
            <!-- kinematics -->
            <wheel_separation>${base_width + wheel_width}</wheel_separation>
            <wheel_diameter>${2*wheel_radius}</wheel_diameter>
            <!-- limits -->
            <max_wheel_torque>20</max_wheel_torque>
            <max_wheel_acceleration>1.0</max_wheel_acceleration>
            <!-- output -->
            <odometry_frame>odom</odometry_frame>
            <robot_base_frame>base_link</robot_base_frame>
            <publish_odom>true</publish_odom>
            <publish_odom_tf>true</publish_odom_tf>
            <publish_wheel_tf>true</publish_wheel_tf>
            <publish_joint_states>true</publish_joint_states>
            <publish_tf>true</publish_tf>
        </plugin>
    </gazebo>

    <!-- IMU Plugin -->
    <gazebo reference="imu_link">
        <sensor name="imu_sensor" type="imu">
            <always_on>true</always_on>
            <update_rate>100</update_rate>
            <visualize>false</visualize>
            <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
                <ros>
                    <remapping>~/out:=imu/data</remapping>
                </ros>
                <initial_orientation_as_reference>false</initial_orientation_as_reference>
                <frame_name>imu_link</frame_name>
            </plugin>
            <pose>0 0 0 0 0 0</pose>
        </sensor>
    </gazebo>

    <!-- RGBD Camera Plugin -->
    <gazebo reference="camera_link">
        <sensor type="depth" name="camera">
            <always_on>true</always_on>
            <update_rate>20.0</update_rate>
            <camera name="sim_camera">
                <horizontal_fov>${60.0*M_PI/180.0}</horizontal_fov>
                <image>
                    <width>640</width>
                    <height>480</height>
                    <format>R8G8B8</format>
                </image>
                <clip>
                    <near>0.1</near>
                    <far>10.0</far>
                </clip>
                <noise>
                    <type>gaussian</type>
                    <mean>0.0</mean>
                    <stddev>0.007</stddev>
                </noise>
            </camera>
            <plugin name="camera_plugin" filename="libgazebo_ros_camera.so">
                <ros>
                    <remapping>image_raw:=rgb/image_rect_color</remapping>
                    <remapping>depth/image_raw:=depth/image_rect_raw</remapping>
                    <remapping>camera_info:=rgb/camera_info</remapping>
                    <remapping>depth/camera_info:=depth/camera_info</remapping>
                    <remapping>points:=depth/color/points</remapping>
                </ros>
                <camera_name>camera</camera_name>
                <frame_name>camera_depth_optical_frame</frame_name>
                <min_depth>0.1</min_depth>
                <max_depth>10.0</max_depth>
                <hack_baseline>0.0</hack_baseline>
            </plugin>
        </sensor>
    </gazebo>

<!-- *** ADD LASER SCANNER PLUGIN *** -->
    <gazebo reference="laser_link">
        <sensor type="ray" name="laser_sensor">
            <pose>0 0 0 0 0 0</pose>
            <visualize>true</visualize> <!-- Set to true to see rays in Gazebo -->
            <update_rate>10</update_rate> <!-- Scan rate Hz -->
            <ray>
                <scan>
                    <horizontal>
                        <samples>360</samples> <!-- Number of rays per scan -->
                        <resolution>1</resolution> <!-- Should be 1 -->
                        <min_angle>-${M_PI}</min_angle> <!-- -180 degrees -->
                        <max_angle>${M_PI}</max_angle>  <!-- +180 degrees -->
                    </horizontal>
                </scan>
                <range>
                    <min>0.10</min> <!-- Minimum range -->
                    <max>12.0</max> <!-- Maximum range -->
                    <resolution>0.01</resolution> <!-- Range resolution -->
                </range>
                <noise>
                    <type>gaussian</type>
                    <mean>0.0</mean>
                    <stddev>0.01</stddev>
                </noise>
            </ray>
            <plugin name="gazebo_ros_laser_controller" filename="libgazebo_ros_ray_sensor.so">
                <ros>
                    <!-- Use 'scan' as the topic name -->
                    <remapping>~/out:=scan</remapping>
                </ros>
                <!-- Set the frame ID to the laser link -->
                <frame_name>laser_link</frame_name>
                <!-- Set output type to LaserScan -->
                <output_type>sensor_msgs/LaserScan</output_type>
            </plugin>
        </sensor>
    </gazebo>
    <!-- *** END LASER SCANNER PLUGIN *** -->



</robot>
```

## Launch Files

Create these launch files:

1. **Robot Simulation (`robot_simulation.launch.py`):**

    ```python
    import os
    from ament_index_python.packages import get_package_share_directory
    from launch import LaunchDescription
    from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
    from launch_ros.actions import Node
    from launch_ros.substitutions import FindPackageShare
    import xacro

    def generate_launch_description():

        # Package paths
        pkg_share = FindPackageShare(package='rtabmap_diff_drive_tutorial')
        pkg_gazebo_ros = FindPackageShare(package='gazebo_ros')

        # Launch arguments
        use_sim_time = LaunchConfiguration('use_sim_time', default='true')
        # Default world path relative to the package
        default_world_path = PathJoinSubstitution([
            pkg_share, 
            'worlds', 
            'test.world'
        ])
        world_path = LaunchConfiguration('world', default=default_world_path) 

        # URDF/Xacro file path
        xacro_file = PathJoinSubstitution([pkg_share, 'urdf', 'robot.urdf.xacro'])
        # Use Command to process xacro file
        robot_description_raw = Command(['xacro ', xacro_file])

        # === Nodes ===

        # Robot State Publisher
        # Takes the URDF and publishes /tf based on joint states provided by Gazebo
        node_robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description_raw,
                'use_sim_time': use_sim_time # Use Gazebo's clock
            }]
        )
        

        # Gazebo Simulation
        # Launches Gazebo server and client
        gzserver_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_gazebo_ros, 'launch', 'gzserver.launch.py'])
            ),
            # Pass the world file and set verbose output
            launch_arguments={'world': world_path, 'verbose': 'true', 'pause': 'false'}.items() # Start Gazebo unpaused
        )

        gzclient_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_gazebo_ros, 'launch', 'gzclient.launch.py'])
            ),
            launch_arguments={'verbose': 'true'}.items()
        )

        # Spawn Entity Node
        # Spawns the robot model into Gazebo from the /robot_description topic
        spawn_entity_node = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', # Subscribe to the processed URDF
                    '-entity', 'diff_drive_robot', # Name of the model in Gazebo
                    '-x', '0.0', # Initial X position
                    '-y', '0.0', # Initial Y position
                    '-z', '0.1', # Initial Z position (slightly above ground)
                    '-Y', '0.0'], # Initial Yaw orientation
            output='screen'
        )

        # === Launch Description ===
        ld = LaunchDescription()

        # Declare launch arguments that can be passed via command line
        ld.add_action(DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock?'))
        ld.add_action(DeclareLaunchArgument(
            'world',
            default_value=default_world_path,
            description='Full path to world file to load'))

        # Add nodes and included launch files to the description
        ld.add_action(gzserver_cmd)            # Start Gazebo server
        ld.add_action(gzclient_cmd)            # Start Gazebo client GUI
        ld.add_action(node_robot_state_publisher) # Publish TF tree
        ld.add_action(spawn_entity_node)       # Spawn robot model

        return ld
    ```

2. **RTAB-Map (`rtabmap.launch.py`):**

    ```python
    import os
    from ament_index_python.packages import get_package_share_directory
    from launch import LaunchDescription
    from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
    from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
    from launch_ros.actions import Node
    from launch_ros.substitutions import FindPackageShare

    def generate_launch_description():

        # Package paths
        pkg_share = FindPackageShare(package='rtabmap_diff_drive_tutorial')
        # rtabmap_ros_pkg_share = get_package_share_directory('rtabmap_ros') # Not used directly anymore

        # Default paths
        default_rviz_config_path = PathJoinSubstitution([pkg_share, 'config', 'rtabmap.rviz'])

        # Launch arguments
        use_sim_time = LaunchConfiguration('use_sim_time', default='true')
        qos = LaunchConfiguration('qos', default='2') # Use reliable QoS for simulation
        rviz_config_path = LaunchConfiguration('rviz_config', default=default_rviz_config_path)

        # Parameters for RTAB-Map node
        # NOTE: Changed string 'true'/'false' to boolean True/False
        rtabmap_params = {
            'subscribe_depth': True,
            'subscribe_rgb': True,
            'subscribe_scan': False,       # No lidar
            'subscribe_odom_info': False, # Corrected: Use /odom topic directly, not /odom_info
            'frame_id': 'base_link',       # Robot base frame (must match URDF)
            'map_frame_id': 'map',         # Frame for the map
            'odom_frame_id': 'odom',       # Odometry frame (must match diff drive plugin)
            'approx_sync': True,       # Sync RGBD topics
            'wait_for_transform': 0.2,  # Corrected type: float
            'use_sim_time': use_sim_time, # LaunchConfiguration handles this correctly
            'qos_image': qos,
            'qos_imu': qos,
            'qos_odom': qos,
            # RTAB-Map Strategy (ICP vs Visual)
            'Reg/Strategy': '1',          # Use string
            'Reg/Force3DoF': 'True',      # Assume planar motion (differential drive)
            # Occupancy Grid Generation
            'Grid/FromDepth': 'True',     # Create occupancy grid from depth image
            'Grid/RangeMax': '5.0',        # Changed back to string due to node error
            # Visual features / Inliers
            'Vis/MinInliers': '12',       # Corrected type: integer
            'queue_size': 30,          # Use integer, not string
            'publish_tf_map': True ,     # Corrected type: boolean
            'database_path': LaunchConfiguration('database_path')
        }

        # Remappings
        # Adjusted to match actual topics published by gazebo_ros_camera plugin
        rtabmap_remaps = [
            ('odom', '/odom'),
            ('imu', '/imu/data'),
            ('rgb/image', '/camera/image_raw'),         # Default from Gazebo
            ('rgb/camera_info', '/camera/camera_info'), # Default from Gazebo
            ('depth/image', '/camera/depth/image_raw'), # Default from Gazebo
            # Add depth camera info remapping if needed by RTAB-Map 
            # (Usually needed if subscribe_depth=True and not approx_sync, but good practice)
            ('depth/camera_info', '/camera/depth/camera_info') 
        ]

        # === Nodes ===

        # RTAB-Map Core Node
        # Performs SLAM
        rtabmap_node = Node(
            package='rtabmap_slam', # Adjusted package name based on find output
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[rtabmap_params],
            remappings=rtabmap_remaps,
            #arguments=['-d'] # Optional: Delete previous database on start
        )

        # RTAB-Map Viz Node
        # Provides visualization data for RViz
        rtabmap_viz_node = Node(
            package='rtabmap_viz', # Adjusted package name based on find output
            executable='rtabmap_viz',
            name='rtabmap_viz',
            output='screen',
            parameters=[{
            'subscribe_odom_info': True,
            'subscribe_scan': False,
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'map_frame_id': 'map',
            'use_sim_time': use_sim_time,
            'qos_image': qos,
            'qos_imu': qos,
            'qos_odom': qos,
            'queue_size': 30, # Use integer, not string
            }],
            # Remap topics for visualization needs
            remappings=rtabmap_remaps # Use the same base remappings
        )

        # RViz Node
        # Visualizes the robot, map, and sensor data
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path], # Load custom RViz config
            parameters=[{'use_sim_time': use_sim_time}]
        )

        # === Launch Description ===
        ld = LaunchDescription()

        # Declare launch arguments
        ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation (Gazebo) clock?'))
        ld.add_action(DeclareLaunchArgument('qos', default_value='2', description='QoS profile: 0=system default, 1=sensor data, 2=reliable'))
        ld.add_action(DeclareLaunchArgument('rviz_config', default_value=default_rviz_config_path, description='Full path to the RVIZ config file to use'))
        ld.add_action(DeclareLaunchArgument('database_path', default_value='~/.ros/rtabmap.db', description='Path to the RTAB-Map database file'))
        # Add nodes
        ld.add_action(rtabmap_node)
        ld.add_action(rtabmap_viz_node)
        ld.add_action(rviz_node)

        return ld
    ```

## Running the Tutorial

**Do not forget to modify the `setup.py` `package.xml` files**

1. Create a basic world file in `worlds/test.world`.
2. Create an RViz config file in `config/rtabmap.rviz`.
3. **Build and source your workspace:**

    ```bash
    colcon build
    source install/setup.bash
    ```

4. **Launch the simulation:**

    ```bash

    ros2 launch rtabmap_diff_drive_tutorial robot_simulation.launch.py
    ```

5. **In a new terminal, launch RTAB-Map:**

    ```bash
    ros2 launch rtabmap_diff_drive_tutorial rtabmap.launch.py
    ```

6. **Control the robot:**

    ```bash
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
    ```

7. Visualize the map in RViz and `rtabmap_viz`.

## Using the Generated Map

### Saving the Map

RTAB-Map automatically saves the database file when it shuts down:

```bash
~/.ros/rtabmap.db
```

To specify a custom save path:

```bash
ros2 launch rtabmap_diff_drive_tutorial rtabmap.launch.py \
  rtabmap_args:="--database_path:=/absolute/path/to/my_map.db"
```

---

### Resume Mapping (Continue SLAM)

To continue SLAM using an existing `.db` file:

```bash
ros2 launch rtabmap_diff_drive_tutorial rtabmap.launch.py \
  rtabmap_args:="--database_path:=/absolute/path/to/my_map.db"
```

No additional parameter changes are needed. It will resume mapping from the last saved state.

---

### Localization Only (No SLAM)

To use the map only for localization, disable new node insertion and loop closures:

#### 1. Launch with localization parameters

```bash
ros2 launch rtabmap_diff_drive_tutorial rtabmap.launch.py \
  rtabmap_args:="--database_path:=/absolute/path/to/my_map.db" \
  use_sim_time:=true \
  qos:=2
```

#### 2. Modify your `rtabmap_params` in the launch file

```python
'Mem/IncrementalMemory': 'false',
'Mem/InitWMWithAllNodes': 'true',
'RGBD/ProximityBySpace': 'false',
'RGBD/LoopClosureReextractFeatures': 'false',
```

This loads the map and only tracks the robot’s pose within the existing environment.

---
**note:**  

```bash
    source /usr/share/gazebo-11/setup.bash

```
