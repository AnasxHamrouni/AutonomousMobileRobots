# Setting up SLAM with a Differential Drive Robot

#### ROS2 Packages
Ensure you have the Navigation2 stack and SLAM toolbox installed:

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox ros-humble-gazebo-ros-pkgs
```

#### Existing Robot Description & Bringup Package
- URDF/Xacro files defining the robot.
- Sensor plugins for IMU, LiDAR, and differential drive.
- A launch file to spawn the robot in Gazebo.

#### Nav2 and slam_toolbox Configuration Files
- YAML files for Nav2 and SLAM toolkit configuration.

### Robot Sensor Suite

| Sensor           | Purpose |
|-----------------|---------|
| **Wheel Odometry** | Estimates robot movement, prone to drift. |
| **IMU (Inertial Measurement Unit)** | Tracks orientation, helps correct odometry drift. |
| **2D LiDAR (Laser Scanner)** | Provides distance readings for SLAM and navigation. |

---

### 1. Launching the Robot in Gazebo

```bash
ros2 launch my_robot_bringup my_robot_bringup.launch.py 
```

### 2. Verifying Sensor Topics

Check active topics:
```bash
ros2 topic list
```
Check odometry:
```bash
ros2 topic echo /odom 
```
Check IMU data:
```bash
ros2 topic echo /imu/data 
```
Check LiDAR scan:
```bash
ros2 topic echo /scan 
```
Check TF frames:
```bash
ros2 run tf2_tools view_frames
```

---

## Integrating Nav2 for SLAM

### Overview
- **SLAM Toolbox** provides the map and localization (replacing AMCL).
- **Nav2** uses the map and odometry for path planning.

### Configuring Nav2 for a Differential Drive Robot

Create `nav2_params.yaml` with:
```yaml
local_costmap:
  ros__parameters:
    # List of sensors/sources that provide obstacle data to the local costmap
    observation_sources: laser
    
    laser:
      topic: /scan                   # Topic name where the LiDAR data is published
      max_obstacle_height: 1.5       # Obstacles taller than 1.5 m are ignored (good for tables/chairs legs filtering)
      clearing: true                 # This sensor can clear (free) cells when no obstacle is seen
      marking: true                  # This sensor can mark (occupy) cells when obstacles are detected
      data_type: LaserScan           # Type of message (LaserScan = 2D lidar)

controller_server:
  ros__parameters:
    controller_frequency: 20.0       # How often the controller runs [Hz] — 20 Hz is common
    min_x_velocity_threshold: 0.001  # Below this linear speed → considered stopped (prevents jitter)
    min_theta_velocity_threshold: 0.001  # Below this angular speed → considered stopped

    # Plugins used to check if the robot is making progress or has reached the goal
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"

    # List of controller plugins to load (usually just one for local planning)
    controller_plugins: ["FollowPath"]
    
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"   # The actual local planner being used (DWB = Dynamic Window Based)

      # ────────────────────────────────────────────────
      #                  Debug & Visualization
      # ────────────────────────────────────────────────
      debug_trajectory_details: True    # Publishes more detailed trajectory info (useful for rviz debugging)

      # ────────────────────────────────────────────────
      #             Robot Kinematic Limits
      # ────────────────────────────────────────────────
      min_vel_x: 0.0                    # Can move backwards? (0.0 = no)
      min_vel_y: 0.0                    # Holonomic? (0.0 = differential drive / non-holonomic)
      max_vel_x: 0.26                   # Maximum forward linear velocity [m/s]
      max_vel_y: 0.0
      max_vel_theta: 1.0                # Maximum angular velocity [rad/s]

      min_speed_xy: 0.0                 # Minimum acceptable xy speed (usually 0)
      max_speed_xy: 0.26                # sqrt(vx² + vy²) limit — same as max_vel_x here
      min_speed_theta: 0.0

      # Acceleration and deceleration limits (important for realistic motion)
      acc_lim_x: 2.5                    # max linear acceleration [m/s²]
      acc_lim_y: 0.0
      acc_lim_theta: 3.2                # max angular acceleration [rad/s²]

      decel_lim_x: -2.5                 # max linear deceleration [m/s²]
      decel_lim_y: 0.0
      decel_lim_theta: -3.2

      # ────────────────────────────────────────────────
      #           Trajectory Sampling Parameters
      # ────────────────────────────────────────────────
      vx_samples: 20                    # How many linear velocity samples to try
      vtheta_samples: 40                # How many angular velocity samples to try (more = finer turning)

      sim_time: 1.7                     # How far into the future to simulate trajectories [seconds]
      linear_granularity: 0.05          # Distance resolution between points on simulated path [m]
      angular_granularity: 0.025        # Angular resolution between points [rad]

      transform_tolerance: 0.1          # How much tf delay is acceptable [seconds]

      # ────────────────────────────────────────────────
      #               Cost Function Critics
      # (in order — higher scale = more important)
      # ────────────────────────────────────────────────
      critics: 
        - RotateToGoal       # Special critic: rotate to final orientation when near goal
        - Oscillation        # Penalizes oscillating behavior
        - BaseObstacle       # Avoids collisions with obstacles
        - GoalAlign          # Align robot with final goal orientation
        - PathAlign          # Keep robot aligned with the global path
        - PathDist           # Stay close to the global path
        - GoalDist           # Get closer to the goal

      # Critic weights (scale = importance multiplier)
      RotateToGoal:
        scale: 32.0
      Oscillation:
        scale: 10.0
        oscillation_reset_dist: 0.05   # Distance traveled to reset oscillation counter [m]
      BaseObstacle:
        scale: 10.0
      GoalAlign:
        scale: 24.0
      PathAlign:
        scale: 32.0                    # Very high → robot tries hard to stay parallel to path
      PathDist:
        scale: 32.0                    # Very high → stay very close to reference path
      GoalDist:
        scale: 24.0                    # Encourage progress toward goal

```



### Launching Nav2 and SLAM

Launch Navigation2:
```bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True params_file:=/path/to/nav2_params.yaml
```

Launch SLAM Toolbox:
```bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True
```

---

## Running SLAM and Navigating

### 1. Drive the Robot to Build the Map
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 2. Save the Map
```bash
ros2 run nav2_map_server map_saver_cli -f ~/map
```

---

## Python Scripting for Automation

Create `launch_slam_nav2.py`:
```python
import subprocess
import time

sim_proc = subprocess.Popen(["ros2", "launch", "my_robot_bringup", "my_robot_bringup.launch.py ", "use_sim_time:=True"])
time.sleep(5)

nav2_proc = subprocess.Popen(["ros2", "launch", "nav2_bringup", "navigation_launch.py", "use_sim_time:=True", "params_file:=/path/to/nav2_params.yaml"])
time.sleep(3)

slam_proc = subprocess.Popen(["ros2", "launch", "slam_toolbox", "online_async_launch.py", "use_sim_time:=True"])
time.sleep(3)


try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    for proc in [rviz_proc, slam_proc, nav2_proc, sim_proc]:
        proc.terminate()
```
Run it with:
```bash
python3 launch_slam_nav2.py
```
---

### Next Steps
- Use the saved map for localization with AMCL.
- Explore autonomous exploration algorithms.
- Deploy SLAM and Nav2 on a real robot.
- also :
```yaml
- enable backward movement "add suitable critic for it
- check other planner instead of DWB
```
