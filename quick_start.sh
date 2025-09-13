#!/bin/bash

echo "ROS2 Autonomous Navigation Demo - Quick Start"
echo "============================================="

if [ -z "$ROS_DISTRO" ]; then
    echo "❌ ROS2 not found. Please source ROS2 first:"
    echo "   source /opt/ros/humble/setup.bash"
    exit 1
fi

echo "ROS2 $ROS_DISTRO detected"

echo "Building workspace..."
colcon build --symlink-install

if [ $? -ne 0 ]; then
    echo "❌ Build failed. Please check the error messages above."
    exit 1
fi

echo "Build successful"

echo "Sourcing workspace..."
source install/setup.bash

echo "🎯 Launching complete demo..."
echo "   - Gazebo simulation with warehouse world"
echo "   - Husky robot with LiDAR"
echo "   - Navigation stack (perception, planning, control)"
echo "   - RViz visualization"
echo ""
echo "The robot will navigate from (0,0) to (5,3)"
echo "Press Ctrl+C to stop the demo"
echo ""

ros2 launch path_ros2 demo.launch.py
