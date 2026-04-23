# Autonomous Mobile Robotics - S24
**Instructor : Karam Almaghout** , 
**TA : Ghadeer Issa**

# Part 2: Setting Up ROS 2 Workspace and Understanding Nodes

In this part of the lab, we will:
1. Set up a ROS 2 workspace.
2. Understand ROS 2 packages and nodes.
3. Create a simple package with a publisher and subscriber node.
4. Visualize the node connections using `rqt_graph`.

---

## 1. Setting Up a ROS 2 Workspace

1. **Create the workspace directory**:
   ```bash
   mkdir -p ~/AMR_ws/src
   cd ~/AMR_ws
   ```

2. **Initialize the workspace**:
   ```bash
   cd ~/AMR_ws
   colcon build
   ```

3. **Source the workspace**:
   ```bash
   source install/setup.bash
   ```

4. **Make sourcing automatic**:
   Add the following line to your `~/.bashrc` file to ensure the workspace is sourced every time you open a terminal:
   ```bash
   echo "source ~/AMR_ws/install/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```
5. **Install Auto-Complete Tool**
   ```bash
   sudo apt install python3-argcomplete
   
   source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
   ```


## 2. ROS 2 Package Structure and Nodes

A **ROS 2 package** organizes code, launch files, and configuration into a single unit. It typically includes:
- **`package.xml`**: Metadata about the package and its dependencies.
- **`setup.py`**: For Python packages, this defines how code is installed.
- **`src/` directory**: Contains Python scripts for nodes.
- **Launch files**: Used to start one or more nodes.

A **node** is a program that communicates with other nodes using topics, services, or actions. Nodes use:
- **Publishers** to send data (e.g., a temperature sensor publishing readings).
- **Subscribers** to receive data (e.g., a display node showing those readings).

---

## 3. Create a ROS 2 Package with a Publisher and Subscriber

### 3.1 Create the Package
1. Navigate to the workspace `src` directory:
   ```bash
   cd ~/AMR_ws/src
   ```

2. Create a new ROS 2 Python package:
   ```bash
   ros2 pkg create --build-type ament_python amr_tutorial
   ```

3. This generates the following structure:
   ```
   amr_tutorial/
   ├── package.xml
   ├── setup.cfg
   ├── setup.py
   └── amr_tutorial
       └── __init__.py
   ```

---

### 3.2 Add Dependencies

Open `amr_tutorial/package.xml` and add dependencies for `rclpy` and `std_msgs`:
```xml
<exec_depend>rclpy</exec_depend>
<exec_depend>std_msgs</exec_depend>
```

---

### 3.3 Create a Publisher Node

1. Inside `amr_tutorial/`, create a new file for the publisher:
   ```bash
   touch amr_tutorial/simple_publisher.py
   ```

2. Add the following code to `simple_publisher.py`:
   ```python
      #!/usr/bin/env python3

      import rclpy
      from rclpy.node import Node
      from std_msgs.msg import String

      class SimplePublisher(Node):
         # Define a class that inherits from Node to create a ROS 2 publisher node.

         def __init__(self):

            super().__init__('simple_publisher')
            # Call the parent Node class's constructor with the node name 'simple_publisher'.

            self.publisher_ = self.create_publisher(String, 'topic', 10)
            # Create a publisher that publishes String messages to the 'topic' topic.
            # The '10' refers to the message queue size (depth).

            self.timer = self.create_timer(1.0, self.publish_message)
            # Create a timer that triggers the publish_message method every 1 second.

            self.counter = 0
            # Initialize a counter variable to track the number of messages published.

         def publish_message(self):

            msg = String()

            msg.data = f"Hello, ROS 2! Count: {self.counter}"

            self.publisher_.publish(msg)

            self.get_logger().info(f"Publishing: {msg.data}")

            self.counter += 1


      def main(args=None):
         # Entry point for the program.

         rclpy.init(args=args)
         # Initialize the rclpy library

         node = SimplePublisher()
         # Create an instance of the SimplePublisher node.

         try:
            rclpy.spin(node)
            # Keep the node running and listening for callbacks.
         except KeyboardInterrupt:
            pass
         finally:
            node.destroy_node()
            # Clean up the node and release resources.

            rclpy.shutdown()
            # Shut down the rclpy library.

      if __name__ == '__main__':
         main()

   ```

3. Make the file executable:
   ```bash
   chmod +x amr_tutorial/simple_publisher.py
   ```

---

### 3.4 Create a Subscriber Node

1. Inside `amr_tutorial/`, create a new file for the subscriber:
   ```bash
   touch amr_tutorial/simple_subscriber.py
   ```

2. Add the following code to `simple_subscriber.py`:
   ```python
   #!/usr/bin/env python3

   import rclpy
   from rclpy.node import Node
   from std_msgs.msg import String

   class SimpleSubscriber(Node):
       def __init__(self):
           super().__init__('simple_subscriber')
           self.subscription = self.create_subscription(
               String,
               'topic',
               self.listener_callback,
               10)
           self.subscription  # prevent unused variable warning

       def listener_callback(self, msg):
           self.get_logger().info(f"Received: {msg.data}")

   def main(args=None):
       rclpy.init(args=args)
       node = SimpleSubscriber()
       try:
           rclpy.spin(node)
       except KeyboardInterrupt:
           pass
       finally:
           node.destroy_node()
           rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

3. Make the file executable:
   ```bash
   chmod +x amr_tutorial/simple_subscriber.py
   ```

---

### 3.5 Update `setup.py`

Open `amr_tutorial/setup.py` and add the entry points for the publisher and subscriber:
```python
from setuptools import setup

package_name = 'amr_tutorial'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='YourName',
    maintainer_email='yourname@example.com',
    description='Simple publisher and subscriber example for ROS 2',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_publisher = amr_tutorial.simple_publisher:main',
            'simple_subscriber = amr_tutorial.simple_subscriber:main',
        ],
    },
)
```

---

### 3.6 Build and Test the Package

1. Build the workspace:
   ```bash
   cd ~/AMR_ws
   colcon build
   ```

2. Source the workspace:
   ```bash
   source install/setup.bash
   ```

3. Run the publisher:
   ```bash
   ros2 run amr_tutorial simple_publisher
   ```

4. Open a new terminal, source the workspace, and run the subscriber:
   ```bash
   source ~/AMR_ws/install/setup.bash
   ros2 run amr_tutorial simple_subscriber
   ```

You should see the publisher sending messages and the subscriber receiving them.

---

## 4. Visualizing with `rqt_graph`

### 4.1 Install `rqt_graph`
If not already installed, use the following command:
```bash
sudo apt install ros-humble-rqt-graph
```

### 4.2 Launch `rqt_graph`

1. Open a new terminal and source your workspace:
   ```bash
   source ~/AMR_ws/install/setup.bash
   ```

2. Start `rqt_graph`:
   ```bash
   rqt_graph
   ```

3. You will see a graphical representation of the nodes and topics:
   - **Nodes**: Rectangles representing `simple_publisher` and `simple_subscriber`.
   - **Topics**: Arrows connecting the nodes, labeled with the topic name (e.g., `/topic`).

### 4.3 Explanation of `rqt_graph`

- **Publisher Node (`simple_publisher`)**:
  - Sends messages on the `/topic`.
- **Subscriber Node (`simple_subscriber`)**:
  - Receives messages from the `/topic`.


---

## Summary

- **You have created** a ROS 2 workspace, a package, and simple publisher/subscriber nodes.
- **You visualized** the node graph using `rqt_graph`.
- **Next steps** will involve creating robot descriptions (URDF/Xacro) and integrating Gazebo simulations.

Proceed to **Part 3** to create and simulate robot models.
