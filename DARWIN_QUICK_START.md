#!/bin/bash
# Darwin-OP Quick Start Script
# Run this in Ubuntu-20.04 WSL

echo "=== Darwin-OP Tutorial Quick Start ==="
echo ""
echo "Terminal 1 - Launch Gazebo with Darwin:"
echo "  cd ~/ros-darwin && source devel/setup.bash && roslaunch darwin_gazebo darwin_gazebo.launch"
echo ""
echo "Terminal 2 - Run walking demo:"
echo "  cd ~/ros-darwin && source devel/setup.bash && rosrun darwin_gazebo walker_demo.py"
echo ""
echo "Or test manual control:"
echo "  rostopic pub -1 /darwin/j_pan_position_controller/command std_msgs/Float64 'data: 0.5'"
echo "  rostopic pub /darwin/cmd_vel geometry_msgs/Twist '[1,0,0]' '[0,0,0]'"
echo ""
