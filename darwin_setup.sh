#!/bin/bash
# Setup script for Darwin-OP in ROS1 Noetic (Ubuntu 20.04 WSL)

echo "Setting up Darwin-OP workspace in ROS1 Noetic..."

# Source ROS1 Noetic
source /opt/ros/noetic/setup.bash

# Create workspace
cd ~
mkdir -p ros-darwin/src
cd ros-darwin/src

# Clone Darwin packages
echo "Cloning Darwin-OP packages..."
git clone https://github.com/HumaRobotics/darwin_description.git
git clone https://github.com/HumaRobotics/darwin_control.git
git clone https://github.com/HumaRobotics/darwin_gazebo.git

# Build workspace
cd ~/ros-darwin
echo "Building workspace..."
catkin_make

# Setup environment
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
echo "source ~/ros-darwin/devel/setup.bash" >> ~/.bashrc

source devel/setup.bash

echo "Darwin-OP setup complete!"
echo "Run: roslaunch darwin_gazebo darwin_gazebo.launch"
