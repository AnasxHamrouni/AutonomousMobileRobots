# Obstacle Avoidance Using LiDAR in ROS2

## **Introduction**
In this tutorial, we will create a basic **obstacle avoidance behavior** for our robot using **LiDAR data**. 

The idea is to have the robot drive forward and **automatically avoid obstacles** detected by the **RPLIDAR’s laser scan**.

We will write a **Python node** that subscribes to the LiDAR topic, processes the scan data, and publishes velocity commands.

### **Scenario**
We assume a **differential drive robot** with a **LIDAR** simulated in **Gazebo Classic** (from the previous tutorial). Instead of **teleoperation**, we will use an **autonomous avoidance node** that reads `/scan` data and moves accordingly.

---

## **Step 1: Create a Node to Read LiDAR Data**

### **1.1 Node Structure**
We will create a **ROS2 Python node** (`obstacle_avoider.py`) that:
- **Subscribes** to `/scan` (`sensor_msgs/LaserScan`).
- **Publishes** velocity commands (`geometry_msgs/Twist`) to `/cmd_vel`.

### **1.2 Important Considerations**
- **Disable any teleoperation node** while running this avoidance node to prevent conflicts on `/cmd_vel`.
- Only **one node** should control the robot at a time.

---

## **Step 2: Obstacle Avoidance Strategy**

### **2.1 Basic Reactive Avoidance (Rule-Based)**
1. **Normal movement:** The robot moves **forward** at a constant speed.
2. **Obstacle detection:** If an obstacle is **too close**, the robot turns **left or right**.
3. **Decision making:**
   - **Turn Right** if the obstacle is closer on the left.
   - **Turn Left** if the obstacle is closer on the right.
   - If obstacles are **equally placed**, turn **left** by default.

### **2.2 Processing LaserScan Data**
- The **LaserScan** message provides a **360-degree array of distances**.
- We will analyze **front 60° to the left** and **front 60° to the right**.
- Compare **minimum distances** in each sector:
  - **Left sector:** -60° to 0°
  - **Right sector:** 0° to +60°
- If **both sides are clear**, move **forward**.
- If **too close**, turn **away from the obstacle**.

---

## **Step 3: Implement the Obstacle Avoidance Node**

### **3.1 Create the Node**
Create `obstacle_avoider.py` inside your **ROS2 package** (`lidar_tester`).

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')
        
        # Subscribe to the LiDAR scan data
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10  # QoS history depth
        )
        
        # Publisher for velocity commands
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Parameters
        self.declare_parameter('safe_distance', 0.5)  # Safe threshold in meters
        self.safe_distance = self.get_parameter('safe_distance').value

    def scan_callback(self, scan: LaserScan):
        ranges = scan.ranges
        num_points = len(ranges)
        if num_points == 0:
            return  # No data

        # Determine indices for front-left and front-right sectors
        mid_index = num_points // 2
        sector_width = int((60.0/360.0) * num_points)  # 60° sector on each side
        left_start = mid_index
        left_end = min(num_points, mid_index + sector_width)
        right_end = mid_index
        right_start = max(0, mid_index - sector_width)

        # Extract ranges for each sector
        left_sector = [r for r in ranges[left_start:left_end] if not math.isinf(r)]
        right_sector = [r for r in ranges[right_start:right_end] if not math.isinf(r)]

        # Find minimum distances in each sector
        min_left = min(left_sector) if left_sector else float('inf')
        min_right = min(right_sector) if right_sector else float('inf')

        # Prepare a Twist message
        cmd = Twist()
        forward_speed = 0.2  # m/s
        turn_speed = 0.5     # rad/s

        # Check if obstacles are within the safe distance
        if min_left < self.safe_distance or min_right < self.safe_distance:
            cmd.linear.x = 0.0  # Stop moving forward
            if min_left < min_right:
                cmd.angular.z = -turn_speed  # Turn right
                self.get_logger().info(f"Obstacle on left at {min_left:.2f} m -> turning right")
            elif min_right < min_left:
                cmd.angular.z = turn_speed  # Turn left
                self.get_logger().info(f"Obstacle on right at {min_right:.2f} m -> turning left")
            else:
                cmd.angular.z = turn_speed  # Default turn left
                self.get_logger().info(f"Obstacle ahead -> turning left")
        else:
            cmd.linear.x = forward_speed  # Move forward
            cmd.angular.z = 0.0
            self.get_logger().info("Path clear -> moving forward")

        # Publish the command
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### **3.2 Explanation**
- **Subscribes** to `/scan`, **processes** LiDAR data.
- **Splits** the scan into **left** and **right** front sectors.
- **Finds the closest obstacle** in each sector.
- **Decides to move forward or turn** based on obstacle positions.

---

## **Step 4: Test the Obstacle Avoidance Behavior**

### **4.1 Build and Run the Node**
```bash
colcon build --packages-select lidar_tester
source install/setup.bash
ros2 run lidar_tester obstacle_avoider
```

### **4.2 Observe the Robot in Simulation**
1. The robot should start **moving forward**.
2. **Place an obstacle** in Gazebo in front of the robot.
3. If the obstacle is **detected within 0.5m**, the robot should **turn away**.
4. Once the path is **clear**, the robot moves **forward again**.

### **4.3 Visualize in RViz (Optional)**
```bash
rviz2
```
- Add **LaserScan** display (`/scan`).
- Observe **LiDAR points** and verify avoidance behavior.

---

## **Conclusion**
✅ **Implemented autonomous obstacle avoidance using LiDAR**.  
✅ **Tested and verified the robot’s ability to avoid obstacles**.  
✅ **Learned how to process LiDAR data and publish velocity commands**.  
