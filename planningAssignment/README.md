# Planning Assignment - Bug Navigation Algorithms


## Setup Instructions

### 1. Build the local workspace

```bash
cd ~/Documents/AutonomousMobileRobots/planningAssignment
./build_workspace.sh
```

### 2. Launch the System (3 terminals)

**Terminal 1 - Robot Simulation:**
```bash
cd ~/Documents/AutonomousMobileRobots/planningAssignment
./run_sim.sh
```

**Terminal 2 - SLAM / Map Generation:**
```bash
cd ~/Documents/AutonomousMobileRobots/planningAssignment
./run_slam.sh
```

**Terminal 3 - Teleop during SLAM:**
```bash
cd ~/Documents/AutonomousMobileRobots/planningAssignment
./run_teleop.sh
```

Drive the robot around the world in SLAM mode until the map is good enough, then stop the SLAM and teleop terminals and use the generated DB file for localization.

**After mapping, run Bug 0:**
```bash
cd ~/Documents/AutonomousMobileRobots/planningAssignment
./run_bug0.sh
```

**After mapping, switch to localization:**
```bash
cd ~/Documents/AutonomousMobileRobots/planningAssignment
./run_localization.sh maps/current_world.db
```

### 3. Set Goals in RViz

In RViz, click the **"2D Goal Pose"** button


## Reproducing Results & Metrics

To reproduce the quantitative results and collect metrics :

1. **Run each planner with scenario tagging:**
   - Set the `SCENARIO_NAME` environment variable before running each planner script. Example:
     ```bash
     SCENARIO_NAME=convex_01 ./run_bug0.sh
     SCENARIO_NAME=convex_01 ./run_bug1.sh
     SCENARIO_NAME=convex_01 ./run_astar.sh
     ```
   - This will tag all metrics for that run with the scenario name.

2. **Metrics Logging:**
   - Each planner logs its run statistics to CSV files in the `metrics/` directory:
     - `metrics/bug0_metrics.csv`
     - `metrics/bug1_metrics.csv`
     - `metrics/astar_metrics.csv`
   - Metrics include: scenario, success, time, path length, wall follow/circumnavigation counts, recovery events.

3. **Summarize Results:**
     ```bash
     python3 tools/metrics_summary.py --metrics-dir metrics --output metrics/summary.md
     ```
   - This will create `metrics/summary.md` with a quantitative comparison table for all planners and scenarios.

4. **A* Path Screenshots:**
   - Screenshots of A* planned paths are saved in the `metrics/` folder (`astar_path_convex_01.png`).
