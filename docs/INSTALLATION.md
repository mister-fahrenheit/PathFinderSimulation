# Installation Guide

This guide provides detailed installation instructions for both Linux and Windows platforms.

## Dependencies

### Linux (Ubuntu 22.04)

#### Required ROS2 Packages
```bash
sudo apt update
sudo apt install -y \
    ros-humble-gazebo-ros \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-rviz2 \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-nav2-common \
    python3-numpy
```

#### Python Dependencies
```bash
pip3 install numpy
```

### Windows

#### Prerequisites
1. **Install ROS2 Humble**:
   - Download from: https://github.com/ros2/ros2/releases
   - Follow the Windows installation guide: https://docs.ros.org/en/humble/Installation/Windows-Install-Binary.html

2. **Install Visual Studio Build Tools** (if not already installed):
   - Download from: https://visualstudio.microsoft.com/downloads/
   - Install "C++ build tools" workload

3. **Install Python 3.10**:
   - Download from: https://www.python.org/downloads/
   - Ensure Python is added to PATH

#### Required ROS2 Packages
```cmd
# Open Command Prompt as Administrator
# Source ROS2 environment
call C:\opt\ros\humble\setup.bat

# Install required packages using vcpkg
vcpkg install --triplet x64-windows \
    gazebo \
    gazebo-ros \
    robot-state-publisher \
    joint-state-publisher \
    rviz2 \
    tf2-ros \
    tf2-geometry-msgs
```

#### Python Dependencies
```cmd
pip install numpy
```

## Installation

### Linux

1. **Create the workspace**:
   ```bash
   mkdir -p ~/path_ros2_ws/src
   cd ~/path_ros2_ws/src
   # Copy this workspace to src/
   ```

2. **Build the workspace**:
   ```bash
   cd ~/path_ros2_ws
   colcon build --symlink-install
   ```

3. **Source the workspace**:
   ```bash
   source install/setup.bash
   ```

### Windows

1. **Create the workspace**:
   ```cmd
   mkdir C:\path_ros2_ws\src
   cd C:\path_ros2_ws\src
   REM Copy this workspace to src/
   ```

2. **Build the workspace**:
   ```cmd
   cd C:\path_ros2_ws
   colcon build --symlink-install
   ```

3. **Source the workspace**:
   ```cmd
   call install\setup.bat
   ```

## Verification

### Test Installation
```bash
# Check if packages are built
ros2 pkg list | grep path_ros2

# Check if nodes are available
ros2 run path_ros2 goal_publisher --help
```

### Test Dependencies
```bash
# Check Gazebo
gazebo --version

# Check RViz
rviz2 --help

# Check Python packages
python3 -c "import numpy; print('NumPy version:', numpy.__version__)"
```

## Platform-Specific Notes

### Linux
- **Recommended**: Ubuntu 22.04 LTS with ROS2 Humble
- **Graphics**: Ensure proper GPU drivers for Gazebo rendering
- **Permissions**: May need to run `chmod +x quick_start.sh` for script execution

### Windows
- **ROS2 Installation**: Use the official Windows binary distribution
- **Build Tools**: Visual Studio Build Tools 2019 or 2022 required
- **Python**: Version 3.10 recommended for compatibility
- **Path Issues**: Ensure ROS2 and Python are in system PATH
- **Antivirus**: May need to whitelist ROS2 directories

## Common Installation Issues

### Linux Issues
1. **Missing dependencies**:
   ```bash
   sudo apt update && sudo apt upgrade
   sudo apt install -f  # Fix broken packages
   ```

2. **Permission errors**:
   ```bash
   sudo chown -R $USER:$USER ~/path_ros2_ws
   ```

3. **Build failures**:
   ```bash
   colcon build --cmake-clean-cache
   ```

### Windows Issues
1. **Python not found**:
   - Ensure Python 3.10 is installed and in PATH
   - Check with `python --version`

2. **Build tools missing**:
   - Install Visual Studio Build Tools
   - Ensure C++ build tools workload is selected

3. **ROS2 not found**:
   - Source ROS2 environment: `call C:\opt\ros\humble\setup.bat`
   - Check installation with `ros2 --version`

## System Requirements

### Minimum Requirements
- **CPU**: 2+ cores
- **RAM**: 4GB
- **Storage**: 2GB free space
- **OS**: Ubuntu 22.04 LTS or Windows 10/11

### Recommended Requirements
- **CPU**: 4+ cores
- **RAM**: 8GB
- **Storage**: 5GB free space
- **GPU**: Dedicated graphics card for better Gazebo performance

## Updating

### Update ROS2 Packages
```bash
# Linux
sudo apt update && sudo apt upgrade

# Windows
# Reinstall ROS2 Humble from official releases
```

### Update Workspace
```bash
# Pull latest changes
git pull origin main

# Rebuild workspace
colcon build --symlink-install
```

## Getting Help

If you encounter installation issues:

1. Check the [Troubleshooting Guide](TROUBLESHOOTING.md)
2. Verify system requirements
3. Check ROS2 installation documentation

---
