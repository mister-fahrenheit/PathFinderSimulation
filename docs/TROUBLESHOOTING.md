# Troubleshooting Guide

This guide helps you diagnose and fix common issues with the autonomous navigation system.

## Common Issues

### Linux Issues

#### 1. Gazebo not starting
**Symptoms:**
- Gazebo window doesn't open
- Error messages about missing models
- Simulation fails to launch

**Solutions:**
```bash
# Check Gazebo installation
gazebo --version

# Install missing models
sudo apt install ros-humble-gazebo-ros-pkgs

# Check GPU drivers
nvidia-smi  # For NVIDIA GPUs
glxinfo | grep "OpenGL version"  # For general OpenGL

# Restart with verbose output
ros2 launch path_ros2 simulation.launch.py --ros-args --log-level debug
```

#### 2. Robot not moving
**Symptoms:**
- Robot spawns but doesn't move
- No velocity commands being published
- Control node not responding

**Solutions:**
```bash
# Check if control node is running
ros2 node list | grep control

# Check velocity commands
ros2 topic echo /cmd_vel

# Check robot pose
ros2 topic echo /robot_pose

# Check planned path
ros2 topic echo /planned_path

# Restart control node
ros2 launch path_ros2 navigation.launch.py
```

#### 3. No path planned
**Symptoms:**
- Planner node running but no path generated
- Robot stays at start position
- No path visualization in RViz

**Solutions:**
```bash
# Check if planner node is running
ros2 node list | grep planner

# Check goal pose
ros2 topic echo /goal_pose

# Check obstacles detection
ros2 topic echo /obstacles

# Check robot pose
ros2 topic echo /robot_pose

# Verify goal is reachable
ros2 run path_ros2 goal_publisher 3.0 2.0 0.0
```

#### 4. RViz not showing data
**Symptoms:**
- RViz opens but shows no data
- "No data" messages in displays
- Empty visualization

**Solutions:**
```bash
# Check frame transforms
ros2 run tf2_tools view_frames

# Check topic connections
ros2 topic list

# Check topic data
ros2 topic echo /scan
ros2 topic echo /obstacles

# Verify RViz config file
ros2 run rviz2 rviz2 -d src/path_ros2/rviz/navigation.rviz
```

### Windows Issues

#### 1. Gazebo not starting
**Symptoms:**
- Gazebo window doesn't open
- Error messages about missing dependencies
- Simulation fails to launch

**Solutions:**
```cmd
REM Check Gazebo installation
gazebo --version

REM Check Visual Studio Build Tools
where cl

REM Check Python installation
python --version

REM Restart with verbose output
ros2 launch path_ros2 simulation.launch.py --ros-args --log-level debug
```

#### 2. Robot not moving
**Symptoms:**
- Robot spawns but doesn't move
- No velocity commands being published
- Control node not responding

**Solutions:**
```cmd
REM Check if control node is running
ros2 node list | findstr control

REM Check velocity commands
ros2 topic echo /cmd_vel

REM Check robot pose
ros2 topic echo /robot_pose

REM Check planned path
ros2 topic echo /planned_path

REM Restart control node
ros2 launch path_ros2 navigation.launch.py
```

#### 3. No path planned
**Symptoms:**
- Planner node running but no path generated
- Robot stays at start position
- No path visualization in RViz

**Solutions:**
```cmd
REM Check if planner node is running
ros2 node list | findstr planner

REM Check goal pose
ros2 topic echo /goal_pose

REM Check obstacles detection
ros2 topic echo /obstacles

REM Check robot pose
ros2 topic echo /robot_pose

REM Verify goal is reachable
ros2 run path_ros2 goal_publisher 3.0 2.0 0.0
```

#### 4. RViz not showing data
**Symptoms:**
- RViz opens but shows no data
- "No data" messages in displays
- Empty visualization

**Solutions:**
```cmd
REM Check frame transforms
ros2 run tf2_tools view_frames

REM Check topic connections
ros2 topic list

REM Check topic data
ros2 topic echo /scan
ros2 topic echo /obstacles

REM Verify RViz config file
ros2 run rviz2 rviz2 -d src\path_ros2\rviz\navigation.rviz
```

#### 5. Build errors on Windows
**Symptoms:**
- Colcon build fails
- Missing dependencies
- Compilation errors

**Solutions:**
```cmd
REM Ensure Visual Studio Build Tools are installed
where cl

REM Check Python version (should be 3.10)
python --version

REM Clean and rebuild
colcon build --cmake-clean-cache
colcon build --symlink-install

REM Check for missing packages
ros2 pkg list
```

## Performance Issues

### 1. Slow Path Planning
**Symptoms:**
- Long delays before path is generated
- High CPU usage during planning
- System becomes unresponsive

**Solutions:**
```bash
# Increase grid size for faster planning
ros2 launch path_ros2 demo.launch.py grid_size:=0.2

# Reduce obstacle detection frequency
ros2 param set /perception_node obstacle_threshold 0.8

# Use simpler robot model
ros2 launch path_ros2 demo.launch.py robot_name:=turtlebot3
```

### 2. High CPU Usage
**Symptoms:**
- System becomes slow
- High CPU usage in task manager
- Simulation runs slowly

**Solutions:**
```bash
# Reduce control frequency
ros2 param set /control_node control_frequency 5.0

# Reduce LiDAR update rate
# Modify world file to reduce LiDAR frequency

# Use lower resolution grid
ros2 launch path_ros2 demo.launch.py grid_size:=0.2
```

### 3. Memory Issues
**Symptoms:**
- System runs out of memory
- Simulation crashes
- High memory usage

**Solutions:**
```bash
# Reduce obstacle detection range
ros2 param set /perception_node obstacle_threshold 0.3

# Use smaller world file
ros2 launch path_ros2 demo.launch.py world_file:=simple_world.world

# Restart simulation periodically
```

## Error Messages

### Common Error Messages

#### 1. "Package not found"
```
[ERROR] [launch]: Caught exception in launch: package 'path_ros2' not found
```
**Solution:**
```bash
# Build the workspace
colcon build --symlink-install

# Source the workspace
source install/setup.bash  # Linux
call install\setup.bat     # Windows
```

#### 2. "Node not found"
```
[ERROR] [launch]: Caught exception in launch: executable 'perception_node' not found
```
**Solution:**
```bash
# Check if package is built
ros2 pkg list | grep path_ros2

# Rebuild the workspace
colcon build --symlink-install

# Check executable exists
ls install/path_ros2/lib/path_ros2/
```

#### 3. "Topic not found"
```
[ERROR] [perception_node]: Failed to subscribe to topic '/scan'
```
**Solution:**
```bash
# Check if topic exists
ros2 topic list | grep scan

# Check if Gazebo is running
ros2 node list | grep gazebo

# Restart simulation
ros2 launch path_ros2 simulation.launch.py
```

#### 4. "Transform not found"
```
[ERROR] [tf2_ros]: Could not find transform from 'base_link' to 'map'
```
**Solution:**
```bash
# Check frame transforms
ros2 run tf2_tools view_frames

# Check if robot is spawned
ros2 topic echo /odom

# Restart simulation
ros2 launch path_ros2 simulation.launch.py
```

## Debugging Tools

### 1. Topic Monitoring
```bash
# Monitor all topics
ros2 topic list

# Monitor specific topic
ros2 topic echo /cmd_vel

# Check topic frequency
ros2 topic hz /scan

# Check topic info
ros2 topic info /scan
```

### 2. Node Monitoring
```bash
# List all nodes
ros2 node list

# Get node info
ros2 node info /perception_node

# Check node parameters
ros2 param list /perception_node
```

### 3. System Monitoring
```bash
# Check system resources
htop  # Linux
# Task Manager  # Windows

# Check ROS2 processes
ps aux | grep ros2  # Linux
tasklist | findstr ros2  # Windows
```

### 4. Log Analysis
```bash
# Check node logs
ros2 log level /perception_node debug

# Monitor all logs
ros2 log level / debug
```

## Diagnostic Commands

### System Health Check
```bash
#!/bin/bash
echo "=== ROS2 System Health Check ==="

# Check ROS2 installation
echo "ROS2 Version:"
ros2 --version

# Check workspace
echo "Workspace Status:"
colcon list

# Check packages
echo "Built Packages:"
ros2 pkg list | grep path_ros2

# Check topics
echo "Active Topics:"
ros2 topic list

# Check nodes
echo "Running Nodes:"
ros2 node list

# Check transforms
echo "Frame Transforms:"
ros2 run tf2_tools view_frames
```

### Performance Monitoring
```bash
#!/bin/bash
echo "=== Performance Monitoring ==="

# Check topic frequencies
echo "Topic Frequencies:"
ros2 topic hz /scan
ros2 topic hz /cmd_vel

# Check CPU usage
echo "CPU Usage:"
top -bn1 | grep "Cpu(s)"

# Check memory usage
echo "Memory Usage:"
free -h
```

## Recovery Procedures

### 1. Complete System Reset
```bash
# Stop all ROS2 processes
pkill -f ros2  # Linux
taskkill /f /im ros2.exe  # Windows

# Clean build
rm -rf build/ install/  # Linux
rmdir /s build install  # Windows

# Rebuild
colcon build --symlink-install

# Restart
ros2 launch path_ros2 demo.launch.py
```

### 2. Partial Reset
```bash
# Restart specific node
ros2 launch path_ros2 navigation.launch.py

# Restart simulation
ros2 launch path_ros2 simulation.launch.py

# Restart RViz
ros2 run rviz2 rviz2 -d src/path_ros2/rviz/navigation.rviz
```

### 3. Parameter Reset
```bash
# Reset all parameters to defaults
ros2 param set /perception_node obstacle_threshold 0.5
ros2 param set /planner_node grid_size 0.1
ros2 param set /control_node lookahead_distance 0.5
```

## Getting Help

### 1. Check Documentation
- Review this troubleshooting guide
- Check the [Architecture Guide](ARCHITECTURE.md)
- Review ROS2 documentation

### 2. Community Support
- ROS2 Discourse: https://discourse.ros.org/
- ROS2 GitHub Issues
- Stack Overflow with `ros2` tag

### 3. Create Issue
When creating an issue, include:
- Operating system and version
- ROS2 version
- Error messages
- Steps to reproduce
- System specifications

### 4. Debug Information
```bash
# Collect debug information
ros2 --version
ros2 pkg list
ros2 topic list
ros2 node list
ros2 param list
```

---
