
## Step 1: Installing Required ROS2 Packages

### 1.1 Install Required ROS2 Packages

| Package Name                    | Purpose                                           |
|---------------------------------|---------------------------------------------------|
| ros-humble-navigation2          | Provides AMCL, map_server, and other utilities    |
| ros-humble-nav2-amcl            | Adaptive Monte Carlo Localization (AMCL)          |
| ros-humble-nav2-map-server      | Loads pre-existing maps                           |
| ros-humble-robot-localization   | Extended Kalman Filter (EKF) for sensor fusion    |

Install them with:

```bash
sudo apt install -y ros-humble-navigation2 \
                    ros-humble-nav2-amcl \
                    ros-humble-nav2-map-server \
                    ros-humble-robot-localization \
```

### 1.2 Verify Package Installation

Run the following command to ensure the packages are available:

```bash
ros2 pkg list | grep "nav2\|localization"
```


### 1.3 Build Your ROS2 Workspace (If Needed)

build and source ROS2 workspace `~/AMR_ws` 

```bash
cd ~/AMR_ws
colcon build
source install/setup.bash
```

---

## Step 2: Download and Load a Pre-Existing Map

### 2.1 Prepare Your Map Files

Store your map files in a dedicated folder (e.g., `~/maps`). A typical map directory should include:

```
~/maps/
├── map.yaml
└── map.pgm  (or map.png)
```

Open the `map.yaml` file and verify it includes the following parameters (adjust as necessary):

```yaml
image: map.pgm  # or map.png
resolution: 0.05  # Adjust based on your map scale. here each pixel represents 5 cm in the real world.
origin: [-10.0, -10.0, 0.0]  # Adjust based on the map reference frame. The first two values (x and y) shift the map in the global coordinate system. The third value (theta) represents the rotation in radians (usually 0.0 unless the map is rotated).
occupied_thresh: 0.65  #Defines the threshold for occupied (obstacle) cells in the grayscale image. In a PGM map, pixel values range from 0 (black) to 255 (white). Any pixel darker than 65% (value ≤ 165 in grayscale) is considered occupied.
free_thresh: 0.196
negate: 0 #Indicates whether to invert colors in the map: 0: Black = occupied, white = free (default convention).  1: Black = free, white = occupied (inverts the colors).
```

### 2.2 Launch the ROS2 Map Server

Load the map using the map server:

```bash
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=~/maps/map.yaml -p topic_name:=/map
```
Configure it:
```bash
 ros2 lifecycle set /map_server configure
 ```

Activate it:
```bash
ros2 lifecycle set /map_server activate
```
Verify the map is being published on the `/map` topic:

```bash
ros2 topic echo /map
```

---

## Step 3: Configure and Launch AMCL for Localization

### 3.1 Create an AMCL Configuration File

Create a directory for your robot’s configuration and create an AMCL parameters file:

```bash
mkdir -p ~/AMR_ws/src/my_robot/config
gedit  ~/AMR_ws/src/my_robot/config/amcl_params.yaml
```

Add the following content:

```yaml
amcl:
  ros__parameters:
    use_sim_time: True
    # min_particles: 500
    # max_particles: 2000
    transform_tolerance: 0.1

    # Frame IDs
    base_frame_id: "base_link"
    odom_frame_id: "odom"
    map_frame_id: "map"

    # Scan Matching Parameters:
    # Specifies the sensor model used for laser scan matching.
    # 'likelihood_field' computes the likelihood of a scan observation based on pre-computed distances to obstacles.
    laser_model_type: "likelihood_field"

    # Particle filter settings:
    # Minimum number of particles to maintain the belief representation.
    min_particles: 100
    # Maximum number of particles allowed (adaptive sampling can adjust between min and max).
    max_particles: 5000

    # Update thresholds:
    # 'update_min_a' defines the minimum angular displacement (in radians) required to trigger an update.
    update_min_a: 0.2
    # 'update_min_d' defines the minimum linear displacement (in meters) required to trigger an update.
    update_min_d: 0.25

    # Resample interval:
    # Number of iterations between resampling steps. A lower number means resampling happens more frequently.
    resample_interval: 2

    # Motion model noise parameters (alpha values):
    # These parameters model the noise in the robot's motion. They are used to add uncertainty
    # to the predicted state during the 'prediction' step of the filter.
    alpha1: 0.2  # noise related to rotation during rotation
    alpha2: 0.2  # noise related to rotation during translation
    alpha3: 0.2  # noise related to translation during translation
    alpha4: 0.2  # noise related to translation during rotation
    alpha5: 0.2  # additional noise parameter (often used for modeling unexpected motion)

    # Laser scan topic
    scan_topic: "/scan"
```

### 3.2 Launch AMCL

Start the AMCL node with:

```bash
ros2 run nav2_amcl amcl --ros-args --params-file ~/AMR_ws/src/my_robot/config/amcl_params.yaml
```
```bash
ros2 lifecycle set /amcl configure
```
```bash
ros2 lifecycle set /amcl activate
```
---

## Step 4: Configure the EKF Sensor Fusion

### 4.1 Create an EKF Configuration File

Create the EKF configuration file:

```bash
gedit ~/AMR_ws/src/my_robot/config/ekf_localization.yaml
```

Add the following content:

```yaml
ekf_localization_node:
  ros__parameters:
    use_sim_time: True
    frequency: 30
    two_d_mode: true  # Set true for robots operating in a 2D plane (discards roll, pitch, and Z-axis values)

    # Odometry input
    odom0: /odom
    odom0_config: [true, true, false,  false, false, true,  false, false, false, false, false, false]


    # IMU input
    imu0: /imu/data
    imu0_config: [false, false, false,  false, false, false,  true, true, false]
```
### **Breakdown of `odom0_config`**
- This array defines which parts of the odometry data will be used.
- `[true, true, true,  true, true, true,  false, false, false, false, false, false]`
  - The **first six values (`true`)** mean:
    - X, Y, and Z position (`true, true, false`)
    - Roll, pitch, and yaw orientation (`false, false, true`)
  - The **last six values (`false`)** mean:
    - Linear and angular velocity on the X, Y, and Z axes are ignored.


### **Breakdown of `imu0_config`**
- `[false, false, false,  false, false, false,  true, true, false]`
  - The **first six values (`false`)** mean:
    - Ignore IMU-provided position and orientation.
  - The **last three values (`true`)** mean:
    - Use angular velocity (gyro) readings for roll, pitch, and yaw rate.



## **Key Considerations**
- If the robot has **wheel odometry**, it provides **position and velocity**, but might drift over time.
- If the robot has an **IMU**, it provides **angular velocity**, which helps correct heading errors in odometry.
- The EKF **fuses** both sensor inputs to generate a more reliable and drift-resistant pose estimate.

---


### 4.2 Launch the EKF Node

Run the EKF node with:

```bash
ros2 run robot_localization ekf_node --ros-args --params-file ~/AMR_ws/src/my_robot/config/ekf_localization.yaml
```

---

## Step 5: Visualizing Localization in RViz2

### 5.1 Start RViz2

```bash
ros2 run rviz2 rviz2
```

### 5.2 Configure RViz2 Displays



1. Configure displays:
   - **Map** (topic: `/map`)
   - **Laser Scan** (topic: `/scan`)
   - **Particle Cloud** (topic: `/particlecloud`)
   - **Robot Model** and **TF**
2. Use **2D Pose Estimate** tool in RViz2 and click on the robot’s approximate location.
3. Drive the robot and observe the **particle cloud** and **TF updates**.
4. you may need to manually st the map - odom TF 
  ```bash
   ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1 map odom
  ```

  ```bash
    ros2 topic pub /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{
      header: {
        stamp: { sec: 0, nanosec: 0 },
        frame_id: 'map'
      },
      pose: {
        pose: {
          position: { x: 0.0, y: 0.0, z: 0.0 },
          orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }
        },
        covariance: [0.1, 0, 0, 0, 0, 0,  0, 0.1, 0, 0, 0, 0,  0, 0, 0.1, 0, 0, 0,  0, 0, 0, 0.1, 0, 0,  0, 0, 0, 0, 0.1, 0,  0, 0, 0, 0, 0, 0.1]
      }
    }"
  ```