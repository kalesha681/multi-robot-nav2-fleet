#!/usr/bin/env bash
set -e

echo "=================================================================="
echo "Installing Multi-Robot Nav2 Fleet System & ROS 2 Dependencies"
echo "Target: Ubuntu 22.04 LTS | ROS 2 Humble | Gazebo Sim | Nav2 MPPI"
echo "=================================================================="

# 1. Update Package Indices & Add OSRF Repository for Gazebo Harmonic
sudo apt-get update
sudo apt-get install -y curl lsb-release gnupg

# Add OSRF Gazebo Harmonic Apt Key & Repository
sudo curl -sSL https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update

# 2. Install Core Build Tools & Python Modules
sudo apt-get install -y \
  python3-pip \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  python3-numpy \
  python3-yaml \
  python3-matplotlib \
  python3-scipy \
  python3-transforms3d \
  git

# 3. Install Gazebo Harmonic (Version 8.x) & ROS 2 Harmonic Bridges
sudo apt-get install -y \
  gz-harmonic \
  ros-humble-ros-gzharmonic \
  ros-humble-ros-gzharmonic-sim \
  ros-humble-ros-gzharmonic-bridge \
  ros-humble-ros-gzharmonic-interfaces \
  ros-humble-ros-gzharmonic-image \
  ros-humble-actuator-msgs

# 4. Install Nav2 Ecosystem & MPPI Controller
sudo apt-get install -y \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-nav2-mppi-controller \
  ros-humble-nav2-smac-planner \
  ros-humble-nav2-navfn-planner \
  ros-humble-nav2-smoother \
  ros-humble-nav2-costmap-2d \
  ros-humble-nav2-behavior-tree \
  ros-humble-nav2-bt-navigator \
  ros-humble-nav2-controller \
  ros-humble-nav2-planner \
  ros-humble-nav2-behaviors \
  ros-humble-nav2-waypoint-follower \
  ros-humble-nav2-velocity-smoother

# 5. Install SLAM Toolbox & Transforms
sudo apt-get install -y \
  ros-humble-slam-toolbox \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-xacro \
  ros-humble-tf2-ros \
  ros-humble-tf2-geometry-msgs \
  ros-humble-tf2-sensor-msgs

# 6. Install Teleop, Multiplexing & Visualization
sudo apt-get install -y \
  ros-humble-twist-mux \
  ros-humble-teleop-twist-keyboard \
  ros-humble-rviz2 \
  ros-humble-rosbag2 \
  ros-humble-rosbag2-storage-mcap

# 7. Initialize & Run Rosdep
echo "------------------------------------------------------------------"
echo "Running rosdep package resolution..."
sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y

echo "=================================================================="
echo "All dependencies successfully installed!"
echo "You can now run: colcon build --symlink-install"
echo "=================================================================="
