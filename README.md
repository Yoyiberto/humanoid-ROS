# Darwin-OP Humanoid Robot Simulation

Darwin-OP robot simulation in Gazebo using ROS1 Noetic on Ubuntu 20.04 WSL.

<p align="center">
  <img src="https://github.com/Yoyiberto/humanoid-ROS/blob/7772364973da9af2d3a4862e6a7574c88b1563f5/humanoid.gif" alt="Simulation GIF" width="60%">
</p>

## Prerequisites

- Windows with WSL2
- X Server (VcXsrv/X410/WSLg)

## Installation

### 1. Install Ubuntu 20.04 WSL
```bash
wsl --install -d Ubuntu-20.04
```

### 2. Install ROS1 Noetic
```bash
wsl -d Ubuntu-20.04
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl -y
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-desktop-full -y
```

### 3. Install Dependencies
```bash
sudo apt install -y \
  ros-noetic-gazebo-ros-pkgs \
  ros-noetic-gazebo-ros-control \
  ros-noetic-effort-controllers \
  ros-noetic-joint-state-controller \
  ros-noetic-joint-state-publisher \
  python3-catkin-tools \
  git
```

### 4. Create Workspace & Build
```bash
mkdir -p ~/ros-darwin/src && cd ~/ros-darwin/src
git clone https://github.com/HumaRobotics/darwin_description.git
git clone https://github.com/HumaRobotics/darwin_control.git
git clone https://github.com/HumaRobotics/darwin_gazebo.git

cd ~/ros-darwin
source /opt/ros/noetic/setup.bash
catkin_make
```

### 5. Fix Python3 Compatibility
```bash
sudo ln -s /usr/bin/python3 /usr/bin/python
sed -i 's|#!/usr/bin/env python|#!/usr/bin/env python3|' ~/ros-darwin/src/darwin_gazebo/scripts/walker_demo.py
sed -i "s|print j,\"p\",self.pfn\[j\],\"a\",self.afn\[j\]|print(j,\"p\",self.pfn[j],\"a\",self.afn[j])|" ~/ros-darwin/src/darwin_gazebo/scripts/walker.py
sed -i 's|print "cmdvel",msg|print("cmdvel",msg)|' ~/ros-darwin/src/darwin_gazebo/scripts/walker.py
```

## Usage

### Launch Gazebo Simulation
**Terminal 1:**
```bash
wsl -d Ubuntu-20.04 bash -c "cd ~/ros-darwin && source devel/setup.bash && roslaunch darwin_gazebo darwin_gazebo.launch"
```

### Run Walking Demo
**Terminal 2** (wait for Gazebo to load):
```bash
wsl -d Ubuntu-20.04 bash -c "cd ~/ros-darwin && source devel/setup.bash && rosrun darwin_gazebo walker_demo.py"
```

### Manual Joint Control
```bash
# Pan head
rostopic pub -1 /darwin/j_pan_position_controller/command std_msgs/Float64 'data: 0.5'

# Velocity control
rostopic pub /darwin/cmd_vel geometry_msgs/Twist '[1,0,0]' '[0,0,0]'
```

## Troubleshooting

**Gazebo slow startup in WSL:**
- WSL has limited OpenGL acceleration
- Consider using native Ubuntu or Docker with GPU passthrough
- Wait 5-10 minutes for full initialization

**Controllers not spawning:**
- Wait for Gazebo physics engine to initialize
- Look for "Loaded controllers:" message

## Credits

- Tutorial: [GenerationRobots Darwin-OP Tutorial](https://www.generationrobots.com/en/content/83-carry-out-simulations-and-make-your-darwin-op-walk-with-gazebo-and-ros)
- Packages: [HumaRobotics Darwin repositories](https://github.com/HumaRobotics)
