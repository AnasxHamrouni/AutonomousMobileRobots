# AMR Exam — Ackermann Steering Robot

## Robot Description

You are working with an **Ackermann (car-like) steering robot** in a Gazebo simulation.

- `cmd_vel.angular.z` → **steering angle** in radians (clamped to `[-0.6, 0.6]`)
- `cmd_vel.linear.x`  → **forward speed** in m/s
- The robot **cannot rotate in place** — it always needs forward motion to steer.

### Node Architecture

Controllers do **not** publish directly to the drive node. A safety layer sits in between:

```text
Controller  →  /cmd_vel_nav  →  obstacle_avoidance  →  /cmd_vel  →  ackermann_drive_node
                                        ↑
                                      /scan  (LiDAR)
```

The `obstacle_avoidance` node must be running before you start a path controller.
When the path ahead is clear it passes commands through unchanged.
When an obstacle is detected it overrides the command with an avoidance maneuver.

---

## Setup

### Option A: Docker

#### 1. Allow GUI forwarding

```bash
xhost +local:docker
```

#### 2. First time — build the image and start

```bash
cd exam/
docker compose build
docker compose up -d
```

Gazebo opens with the robot, 4 colored waypoint discs, and 3 yellow obstacle boxes.

#### 3. Open a shell inside the running container

```bash
docker exec -it amr_exam /entrypoint.sh bash
```

#### 4. After editing any file — rebuild inside the container then re-run your nodes in the container shell.


#### 5. Stop the simulation

```bash
docker compose down
```

Force-kill if frozen:

```bash
docker kill amr_exam
```

> **No NVIDIA GPU?** Remove the `deploy` section from `docker-compose.yml` before running.

---

### Option B: Native ROS2 Humble

Required packages:

```text
ros-humble-gazebo-ros-pkgs  ros-humble-gazebo-ros2-control
ros-humble-ros2-control     ros-humble-ros2-controllers
ros-humble-xacro            ros-humble-robot-state-publisher
ros-humble-tf2-ros
```

#### 1. Copy the package into your workspace

```bash
cp -r exam/ ~/AMR_ws/src/
```

#### 2. Build

#### 3. Launch the simulation

```bash
ros2 launch exam ackermann_bringup.launch.py
```

---

## Environment

### Waypoints

Your controller must navigate the robot through these waypoints **in order**:

| # | Position | Marker color |
|---|----------|--------------|
| 1 | (3, 0)   | Red          |
| 2 | (6, 4)   | Green        |
| 3 | (3, 4)   | Orange       |
| 4 | (0, 0)   | Purple       |

### Obstacles

Three yellow box obstacles (0.5 × 0.5 × 0.6 m) are placed between waypoints:

| Obstacle | Position   | Blocks path  |
|----------|------------|--------------|
| 1        | (1.5, 0.5) | Start → WP 1 |
| 2        | (5.0, 2.0) | WP 1 → WP 2  |
| 3        | (4.5, 4.2) | WP 2 → WP 3  |

## Tasks

All controller files are in `exam/exam/`.
Run each in a **separate terminal** inside the container (simulation must be running first).

---

### Task 1 — Reactive Obstacle Avoidance (`obstacle_avoidance.py`)

This node acts as a **safety layer** between your path controller and the drive node.
It subscribes to `/cmd_vel_nav` (controller output) and `/scan` (LiDAR),
and publishes to `/cmd_vel` (drive node input).

**Run first, before any path controller:**

```bash
ros2 run exam obstacle_avoidance
```

Complete the **5 TODOs**:

| # | What to do |
| --- | ---------- |
| 1 | Split `ranges` into `front_right_ranges` and `front_left_ranges` |
| 2 | Compute `min_left`, `min_right`, `min_front` — filter `inf`/`nan`/`<= 0` before taking min |
| 3 | Set `obstacle_detected = min_front < OBSTACLE_THRESHOLD` |
| 4 | Build `avoidance_cmd`: speed = `AVOIDANCE_SPEED`; steer away from the closer side |
| 5 | Publish `avoidance_cmd` if obstacle detected, else pass through `self.latest_nav_cmd` |

---

### Task 2 — P Controller (`ackermann_p_controller.py`)

**Run:**

```bash
ros2 run exam ackermann_p_controller
```

### Task 3 — Pure Pursuit Controller (`ackermann_pure_pursuit.py`)

**Run:**

```bash
ros2 run exam ackermann_pure_pursuit
```

### Task 4 — Stanley Controller (`ackermann_stanley.py`)

**Run:**

```bash
ros2 run exam ackermann_stanley
```

---

**SUBMIT YOUR SOLUTION IN A ZIP FILE CONTAINING ALL THE NODES YOU WROTE**  

**GOOD LUCK** 

