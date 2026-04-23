# Autonomous Mobile Robotics - S24
**Instructor : Karam Almaghout** , 
**TA : Ghadeer Issa**



# Part 1: Installing ROS 2 Humble and Gazebo on Ubuntu 22.04

This section guides you through installing **ROS 2 Humble** on **Ubuntu 22.04**, along with essential dependencies and **Gazebo**.

---

## 1. Set Up Your System and Sources

1. **Ensure your system is up to date**:
   ```bash
   sudo apt update
   sudo apt upgrade
   ```
2. **Install required dependencies**:
   ```bash
   sudo apt install curl gnupg lsb-release
   ```
3. **Add the ROS 2 apt repository**:
   ```bash
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
   ```
   ```bash
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null 
   ```

## 2. Install ROS 2 Humble Desktop

1. **Update your apt index** with the new repository:
   ```bash
   sudo apt update
   sudo apt upgrade
   ```
2. **Install the ROS 2 Humble Desktop package** (includes common tools like RViz):
   ```bash
   sudo apt install ros-humble-desktop
   ```
   > *Alternatively, you can install the `ros-humble-ros-base` package if you want a lighter install (without GUI tools), but for this lab, `ros-humble-desktop` is recommended.*
3. **Install Development tools: Compilers and other tools to build ROS packages**
   ```bash
   sudo apt install ros-dev-tools
   ```

4. **Source ROS 2** ( in every new terminal):
   ```bash
   source /opt/ros/humble/setup.bash
   ```

---

## 3. Install Additional ROS 2 Tools

1. **colcon** (build tool for ROS 2):
   ```bash
   sudo apt install python3-colcon-common-extensions
   ```
2. **(Optional) ROS 2 command line tools** for convenience:
   ```bash
   sudo apt install python3-argcomplete
   ```
   > This helps auto-complete ROS 2 commands in the terminal.

---

## 4. Install Gazebo

For **ROS 2 Humble**, the recommended Gazebo version is typically **Gazebo Fortress**. You can install it via the OSRF packages:

1. **Add the Gazebo package repository**:
   ```bash
   sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.osrfoundation.org/gazebo/ubuntu-stable    $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list'
   ```
2. **Add the Gazebo GPG key**:
   ```bash
   wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
   ```
3. **Update and install Gazebo Fortress**:
   ```bash
   sudo apt update
   sudo apt install gazebo  
   sudo apt install libgazebo-dev
   ```
   <!-- > *Note:* Depending on the repository updates, you may see `gazebo11` or `gazebo-fortress`. -->

4. **Install ROS 2 Gazebo integration packages**:
   ```bash
   sudo apt install ros-humble-gazebo-ros-pkgs
   ```
   This provides the ROS <-> Gazebo bridge (e.g. `gazebo_ros_pkgs`).

---

## 5. Verify Your Installation

1. **Open a new terminal** and source ROS 2 again:
   ```bash
   source /opt/ros/humble/setup.bash
   ```
2. **Check ROS 2 version**:
   ```bash
   echo $ROS_DISTRO
   ```
   You should see something like `humble`.
3. **Test a simple Gazebo launch**:
   ```bash
   gazebo
   ```
   Gazebo should open with an empty world.

If everything launches without errors, you are set up with ROS 2 Humble and Gazebo!

---

## 6. Summary

- **You have installed** ROS 2 Humble (Desktop), the `colcon` build system, and a compatible version of Gazebo.
- **Next steps** will involve creating a ROS 2 workspace, building custom packages, and integrating Gazebo simulations.

Proceed to **Part 2** of the lab to set up your workspace and start creating robot description packages.
