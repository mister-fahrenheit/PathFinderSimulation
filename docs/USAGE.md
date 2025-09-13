# Usage Guide

This guide covers how to run and use the autonomous navigation system.

## Quick Start

### Option 1: Complete Demo (Recommended)
```bash
# Linux
./quick_start.sh

# Windows
quick_start.bat
```

### Option 2: Manual Launch
```bash
# Build and source workspace
colcon build --symlink-install
source install/setup.bash  # Linux
# call install\setup.bat   # Windows

# Launch complete demo
ros2 launch path_ros2 demo.launch.py
```

## Usage Examples

### Linux

#### Basic Navigation
```bash
# Start with default warehouse world
ros2 launch path_ros2 demo.launch.py

# The robot will automatically navigate from (0,0) to (5,3)
```

#### Custom Robot and World
```bash
# Use Turtlebot3 instead of Husky
ros2 launch path_ros2 demo.launch.py robot_name:=turtlebot3

# Custom start and goal positions
ros2 launch path_ros2 demo.launch.py \
    start_x:=-2.0 start_y:=-2.0 \
    goal_x:=4.0 goal_y:=4.0
```

#### Manual Goal Setting
```bash
# Launch simulation and navigation
ros2 launch path_ros2 simulation.launch.py &
ros2 launch path_ros2 navigation.launch.py &

# Set goal interactively in RViz using "2D Nav Goal" tool
# Or publish goal programmatically:
ros2 run path_ros2 goal_publisher 3.0 2.0 1.57
```

### Windows

#### Basic Navigation
```cmd
REM Start with default warehouse world
ros2 launch path_ros2 demo.launch.py

REM The robot will automatically navigate from (0,0) to (5,3)
```

#### Custom Robot and World
```cmd
REM Use Turtlebot3 instead of Husky
ros2 launch path_ros2 demo.launch.py robot_name:=turtlebot3

REM Custom start and goal positions
ros2 launch path_ros2 demo.launch.py ^
    start_x:=-2.0 start_y:=-2.0 ^
    goal_x:=4.0 goal_y:=4.0
```

#### Manual Goal Setting
```cmd
REM Launch simulation and navigation (use separate command prompts)
start cmd /k "ros2 launch path_ros2 simulation.launch.py"
start cmd /k "ros2 launch path_ros2 navigation.launch.py"

REM Set goal interactively in RViz using "2D Nav Goal" tool
REM Or publish goal programmatically:
ros2 run path_ros2 goal_publisher 3.0 2.0 1.57
```

## Launch File Options

### Demo Launch (`demo.launch.py`)
Complete end-to-end demo with all components.

**Parameters:**
- `world_file`: Gazebo world file (default: `simple_warehouse.world`)
- `robot_name`: Robot model (default: `husky`)
- `start_x`, `start_y`, `start_yaw`: Initial robot position
- `goal_x`, `goal_y`, `goal_yaw`: Target goal position

**Example:**
```bash
ros2 launch path_ros2 demo.launch.py \
    robot_name:=turtlebot3 \
    start_x:=0.0 start_y:=0.0 start_yaw:=0.0 \
    goal_x:=5.0 goal_y:=3.0 goal_yaw:=0.0
```

### Simulation Launch (`simulation.launch.py`)
Launches only Gazebo simulation and robot spawning.

**Parameters:**
- `world_file`: Gazebo world file
- `robot_name`: Robot model
- `x`, `y`, `z`, `yaw`: Robot spawn position

**Example:**
```bash
ros2 launch path_ros2 simulation.launch.py \
    world_file:=simple_warehouse.world \
    robot_name:=husky \
    x:=0.0 y:=0.0 z:=0.0 yaw:=0.0
```

### Navigation Launch (`navigation.launch.py`)
Launches only the navigation stack (perception, planning, control).

**Parameters:**
- `use_sim_time`: Use simulation time (default: `true`)

**Example:**
```bash
ros2 launch path_ros2 navigation.launch.py
```

## Interactive Usage

### RViz Controls
1. **2D Nav Goal**: Click and drag to set new goals
2. **2D Pose Estimate**: Set initial robot pose
3. **Publish Point**: Get coordinates of clicked points

### Command Line Tools
```bash
# Publish custom goal
ros2 run path_ros2 goal_publisher 3.0 2.0 1.57

# Monitor topics
ros2 topic echo /cmd_vel
ros2 topic echo /planned_path
ros2 topic echo /obstacles

# List all topics
ros2 topic list

# Check node status
ros2 node list
```

## Step-by-Step Launch

### Terminal 1: Simulation
```bash
ros2 launch path_ros2 simulation.launch.py
```

### Terminal 2: Navigation Stack
```bash
ros2 launch path_ros2 navigation.launch.py
```

### Terminal 3: Goal Publisher
```bash
ros2 run path_ros2 goal_publisher 5.0 3.0 0.0
```

### Terminal 4: RViz
```bash
ros2 run rviz2 rviz2 -d src/path_ros2/rviz/navigation.rviz
```

## Advanced Usage

### Custom World Files
1. Create new world file in `worlds/` directory
2. Launch with custom world:
   ```bash
   ros2 launch path_ros2 demo.launch.py world_file:=my_custom_world.world
   ```

### Multiple Robots
1. Modify launch files to spawn multiple robots
2. Use different namespaces for each robot
3. Adjust topic remapping as needed

### Parameter Tuning
```bash
# Launch with custom parameters
ros2 launch path_ros2 demo.launch.py \
    obstacle_threshold:=0.3 \
    grid_size:=0.05 \
    lookahead_distance:=0.8
```

## Monitoring and Debugging

### Topic Monitoring
```bash
# Monitor robot movement
ros2 topic echo /cmd_vel

# Check path planning
ros2 topic echo /planned_path

# Monitor obstacle detection
ros2 topic echo /obstacles

# Check robot pose
ros2 topic echo /robot_pose
```

### Node Status
```bash
# List running nodes
ros2 node list

# Get node info
ros2 node info /perception_node
ros2 node info /planner_node
ros2 node info /control_node
```

### Performance Monitoring
```bash
# Check topic frequency
ros2 topic hz /scan
ros2 topic hz /cmd_vel

# Monitor CPU usage
htop  # Linux
# Task Manager  # Windows
```

## Tips and Best Practices

### Performance Optimization
- Use simpler robot models for better performance
- Reduce grid resolution for faster planning
- Adjust control frequency based on system capabilities

### Debugging
- Use RViz to visualize all data streams
- Check topic connections if nodes aren't communicating
- Monitor log output for error messages

### Customization
- Modify world files for different environments
- Adjust algorithm parameters for different scenarios
- Add new sensors or modify existing ones

## Common Usage Issues

### Robot Not Moving
1. Check if control node is running: `ros2 node list | grep control`
2. Monitor velocity commands: `ros2 topic echo /cmd_vel`
3. Verify goal is set: `ros2 topic echo /goal_pose`

### No Path Planned
1. Check planner node: `ros2 node list | grep planner`
2. Verify obstacles are detected: `ros2 topic echo /obstacles`
3. Check goal pose: `ros2 topic echo /goal_pose`

### RViz Not Showing Data
1. Check frame transforms: `ros2 run tf2_tools view_frames`
2. Verify topic connections: `ros2 topic list`
3. Check RViz configuration file path

---
