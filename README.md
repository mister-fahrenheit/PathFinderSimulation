# ROS2 Autonomous Industrial Vehicle Navigation

A comprehensive ROS2 Humble workspace demonstrating autonomous industrial vehicle navigation entirely in simulation using Gazebo (Ignition), featuring LiDAR-based obstacle detection, A* path planning, and pure pursuit control.

## Quick Start

### Linux
```bash
# Build and run the complete demo
chmod +x quick_start.sh
./quick_start.sh
```

### Windows
```cmd
REM Build and run the complete demo
quick_start.bat
```

### Manual Launch
```bash
# Build the workspace
colcon build --symlink-install
source install/setup.bash  # Linux
# call install\setup.bat   # Windows

# Launch complete demo
ros2 launch path_ros2 demo.launch.py
```

## System Overview

```
┌──────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Gazebo         │    │   Perception    │    │   Planner       │
│   Simulation     │──▶│   Node          │───▶│   Node          │
│                  │    │                 │    │                 │
│ • Robot Model    │    │ • LiDAR Scan    │    │ • A* Algorithm  │
│ • LiDAR Sensor   │    │ • Obstacle      │    │ • Path Planning │
│ • World/Obstacles│    │   Detection     │    │ • Collision     │
└──────────────────┘    └─────────────────┘    │   Avoidance     │
         │                       │             └─────────────────┘
         │                       │                       │
         ▼                       ▼                       ▼
┌──────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Control        │    │   RViz          │    │   Topics        │
│   Node           │    │   Visualization │    │                 │
│                  │    │                 │    │ • /scan         │
│ • Pure Pursuit   │    │ • Robot Pose    │    │ • /obstacles    │
│ • Velocity       │    │ • LiDAR Data    │    │ • /planned_path │
│   Commands       │    │ • Path Display  │    │ • /cmd_vel      │
└──────────────────┘    └─────────────────┘    └─────────────────┘
```

## Key Features

- **Simulation Environment**: Gazebo (Ignition) with warehouse world
- **Robot Models**: Support for Husky and Turtlebot3 differential-drive robots
- **Perception**: LiDAR-based obstacle detection with clustering
- **Path Planning**: A* algorithm for collision-free navigation
- **Control**: Pure pursuit controller for smooth path following
- **Visualization**: RViz configuration for real-time monitoring
- **Modular Design**: Easy to extend and modify components

## Project Structure

```
path_ros2/
├── src/                          # Source packages
│   ├── perception_node/          # LiDAR obstacle detection
│   ├── planner_node/             # A* path planning
│   └── control_node/             # Pure pursuit control
├── launch/                       # Launch files
├── worlds/                       # Gazebo world files
├── rviz/                         # RViz configurations
├── docs/                         # Detailed documentation
│   ├── README.md                # Documentation index
│   ├── INSTALLATION.md          # Setup instructions
│   ├── USAGE.md                 # Usage guide
│   ├── CONFIGURATION.md         # Parameters and settings
│   ├── ARCHITECTURE.md          # System design
│   ├── API.md                   # API reference
│   └── TROUBLESHOOTING.md       # Problem solving
├── scripts/                      # Utility scripts
├── quick_start.sh               # Linux quick start
├── quick_start.bat              # Windows quick start
└── README.md                     # This file
```

## Documentation

- **[Installation Guide](docs/INSTALLATION.md)** - Detailed setup for Linux and Windows
- **[Usage Guide](docs/USAGE.md)** - How to run and customize the system
- **[Configuration](docs/CONFIGURATION.md)** - Parameters and settings
- **[Architecture](docs/ARCHITECTURE.md)** - System design and algorithms
- **[Troubleshooting](docs/TROUBLESHOOTING.md)** - Common issues and solutions
- **[API Reference](docs/API.md)** - Node interfaces and topics

## Basic Usage

### Default Demo
```bash
ros2 launch path_ros2 demo.launch.py
# Robot navigates from (0,0) to (5,3)
```

### Custom Configuration
```bash
ros2 launch path_ros2 demo.launch.py \
    start_x:=0.0 start_y:=0.0 \
    goal_x:=5.0 goal_y:=3.0 \
    robot_name:=husky
```

## Quick Configuration

| Component | Key Parameter | Default | Description |
|-----------|---------------|---------|-------------|
| Perception | `obstacle_threshold` | 0.5m | Max distance for obstacle detection |
| Planner | `grid_size` | 0.1m | A* grid resolution |
| Control | `lookahead_distance` | 0.5m | Pure pursuit lookahead |

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/scan` | `sensor_msgs/LaserScan` | LiDAR sensor data |
| `/obstacles` | `visualization_msgs/MarkerArray` | Detected obstacles |
| `/planned_path` | `nav_msgs/Path` | Planned path |
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands |

## Extending the System

- **New Planners**: Implement `plan_path(start, goal)` method
- **New Controllers**: Add `compute_velocity_commands()` method
- **New Sensors**: Create perception nodes for additional sensors

## Project Information

This project demonstrates autonomous navigation concepts using ROS2 Humble. It includes complete simulation setup, navigation algorithms, and visualization tools for educational and research purposes.

## Getting Help

- Check the [Troubleshooting Guide](docs/TROUBLESHOOTING.md)
- Review ROS2 documentation for general issues
- Consult the [Architecture Guide](docs/ARCHITECTURE.md) for system understanding

---

*For detailed information, see the [docs/](docs/) folder.*
