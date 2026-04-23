# Homework: RTAB-Map with Stereo Camera

Implement 3D SLAM using RTAB-Map with a **stereo camera** on a simulated differential drive robot in Gazebo (ROS 2 Humble).

Unlike the lab which used a single RGB-D depth camera, here you estimate depth from **two standard RGB cameras** separated by a 12 cm baseline.

**Deliverable:** Submit a `.zip` file named after yourself containing this package and a screen recording of the working mapping process.

---

## Prerequisites

```bash
sudo apt update
sudo apt install \
  ros-humble-rtabmap-ros \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-teleop-twist-keyboard \
  python3-colcon-common-extensions
```

---

## Package Structure

```
rtabmap_diff_drive/
├── config/
│   └── stereo_calibration.yaml     # PROVIDED — stereo camera calibration
├── launch/
│   ├── stereo_robot_simulation.launch.py   # TODO: fill in
│   └── rtabmap_stereo.launch.py            # TODO: fill in
├── urdf/
│   ├── robot.urdf.xacro            # PROVIDED — lab depth-camera robot (reference)
│   └── stereo_robot.urdf.xacro     # TODO: implement
├── worlds/
│   └── test.world                  # PROVIDED — Gazebo world with obstacles
├── package.xml
├── setup.py
└── README.md
```

---

## What Is Already Provided

| File | Status |
|---|---|
| `config/stereo_calibration.yaml` | Provided — use as-is |
| `worlds/test.world` | Provided — use as-is |
| `urdf/robot.urdf.xacro` | Provided — lab reference file, do not modify |
| `launch/stereo_robot_simulation.launch.py` | Skeleton — **you fill it in** |
| `launch/rtabmap_stereo.launch.py` | Skeleton — **you fill it in** |
| `urdf/stereo_robot.urdf.xacro` | Skeleton — **you implement it** |

---

## Tasks

### Task 1 — Robot URDF (`urdf/stereo_robot.urdf.xacro`)

Modify `urdf/robot.urdf.xacro` (the lab depth-camera robot) to replace the single depth camera with two RGB cameras separated by a **12 cm baseline**.

Requirements:
- Keep `base_link`, wheels, casters, IMU, and diff-drive plugin unchanged
- Remove the single depth camera link/joint/Gazebo plugin
- Add a `camera_mount_link` at the front of the robot
- Add `left_camera_link` and `right_camera_link` offset ±6 cm along Y from the mount
- Add `left_camera_optical_frame` and `right_camera_optical_frame`
- Use Gazebo `sensor type="camera"` (not `type="depth"`) for both cameras
- Publish topics under namespace `stereo`:
  - `/stereo/left/image_raw` and `/stereo/left/camera_info`
  - `/stereo/right/image_raw` and `/stereo/right/camera_info`
- Set `hack_baseline="0.0"` on the left camera plugin
- Set `hack_baseline="0.12"` on the right camera plugin

> **Note on `hack_baseline`:** This tells the Gazebo renderer the stereo offset so it can correctly simulate the disparity between the two views.

---

### Task 2 — Simulation Launch File (`launch/stereo_robot_simulation.launch.py`)

Fill in the skeleton to launch:
- Gazebo server and client (load `worlds/test.world`)
- Robot State Publisher (using `stereo_robot.urdf.xacro`)
- Joint State Publisher
- Spawn the robot entity into Gazebo

---

### Task 3 — RTAB-Map Stereo Launch File (`launch/rtabmap_stereo.launch.py`)

Fill in the skeleton to configure and launch the following four nodes (in order):

1. **`stereo_odometry_node`** — from package `rtabmap_odom`, executable `stereo_odometry`
   - Subscribe to the stereo image topics
   - Set `frame_id: base_link`, `odom_frame_id: odom`
   - Remap left/right image and camera_info topics to `/stereo/left/...` and `/stereo/right/...`

2. **`rtabmap_node`** — from package `rtabmap_slam`, executable `rtabmap`
   - Set `subscribe_stereo: True`, `subscribe_depth: False`
   - Set `frame_id: base_link`, `map_frame_id: map`, `odom_frame_id: odom`
   - Use the same stereo topic remappings

3. **`rtabmap_viz_node`** — from package `rtabmap_viz`, executable `rtabmap_viz`

4. **`rviz_node`** — from package `rviz2`, executable `rviz2`

> **Hint:** Use `Reg/Force3DoF: 'True'` to constrain RTAB-Map to planar motion (suitable for a differential drive robot).

---

## Build and Run

```bash
cd ~/your_ws
colcon build --packages-select rtabmap_diff_drive
source install/setup.bash
```

**Terminal 1 — Simulation:**
```bash
source /usr/share/gazebo/setup.bash
ros2 launch rtabmap_diff_drive stereo_robot_simulation.launch.py
```

**Terminal 2 — RTAB-Map:**
```bash
ros2 launch rtabmap_diff_drive rtabmap_stereo.launch.py
```

**Terminal 3 — Teleoperation:**
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Drive the robot around to build the map. Record your screen while mapping.

---

## Notes

- The stereo approach does **not** require a depth camera — depth is estimated from the disparity between the two RGB images
- Feature-rich environments (with visible objects/walls) improve stereo odometry and mapping quality
- In a real robot, `stereo_calibration.yaml` values must be obtained by physically calibrating the camera rig with a checkerboard pattern
