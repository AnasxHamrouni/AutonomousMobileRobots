# Launch Localization with AMCL
Since you've already saved the map, Start AMCL with the saved map:

```bash
ros2 launch nav2_bringup localization_launch.py use_sim_time:=True map:=/path/to/saved_map.yaml
```
Replace `/path/to/saved_map.yaml` with the actual path where you saved your map.

---

# Start Navigation
Now, start the Nav2 stack:

```bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True params_file:=/path/to/nav2_params.yaml
```

---

# Verify Localization
Check if AMCL is providing pose estimates:

```bash
ros2 topic echo /amcl_pose
```
If you see no output, ensure that:

- The `/map` frame exists by running:

  ```bash
  ros2 run tf2_tools view_frames
  ```

- AMCL is correctly receiving scan data from your LiDAR:

  ```bash
  ros2 topic list | grep scan
  ```

---

# Send a Navigation Goal
Now, you can send a goal for the robot to navigate in the saved map.

## Method 1: Using RViz

1. Set the **Fixed Frame** to `map`.
2. Use the **2D Pose Estimate** tool to set the robot's estimated initial position.
3. Click on the map where the robot is located.
4. Drag in the direction the robot is facing.
5. Use the **2D Nav Goal** tool to send a navigation goal.

## Method 2: Using the Command Line
Send a goal using the ROS2 action:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
```
Change `x` and `y` coordinates to match a valid location on your map.

---

# Debugging Issues
If the robot does not move:

- Check if AMCL is publishing localization data:
  ```bash
  ros2 topic echo /amcl_pose
  ```
- Ensure that `map -> odom` transform is available:
  ```bash
  ros2 run tf2_ros tf2_echo map odom
  ```
- Confirm that Nav2 costmaps are working correctly:
  ```bash
  ros2 topic echo /global_costmap/costmap
  ```

---

# Set the Initial Pose Correctly
You need to provide the initial pose estimate in the **map** frame. Here's how:
### Manually Set the Initial Pose via CLI
You can manually send an initial pose estimate in the **map** frame using the ROS2 topic:

```bash
ros2 topic pub /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{
    header: {frame_id: 'map'},
    pose: {
      pose: {
        position: {x: 0.0, y: 0.0, z: 0.0},
        orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
      },
      covariance: [0.25, 0, 0, 0, 0, 0,
                   0, 0.25, 0, 0, 0, 0,
                   0, 0, 0.25, 0, 0, 0,
                   0, 0, 0, 0.0685389, 0, 0,
                   0, 0, 0, 0, 0.0685389, 0,
                   0, 0, 0, 0, 0, 0.0685389]
    }
  }"
```
Change `x: 0.0, y: 0.0` to match the robot's actual starting position on the saved map.

---

# Send a Navigation Goal
Try sending a navigation goal:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

This will command the robot to navigate to the specified position in the saved map.
