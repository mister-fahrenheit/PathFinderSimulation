# Configuration Guide

This guide covers all configurable parameters and settings for the autonomous navigation system.

## Node Parameters

### Perception Node Parameters

| Parameter | Default | Description | Range |
|-----------|---------|-------------|-------|
| `obstacle_threshold` | 0.5m | Maximum distance to consider as obstacle | 0.1 - 5.0m |
| `min_obstacle_size` | 0.1m | Minimum obstacle size to detect | 0.05 - 1.0m |
| `max_obstacle_size` | 2.0m | Maximum obstacle size to detect | 0.5 - 10.0m |
| `cluster_tolerance` | 0.2m | Distance for clustering nearby points | 0.05 - 1.0m |

**Example Configuration:**
```bash
ros2 launch path_ros2 demo.launch.py \
    obstacle_threshold:=0.3 \
    min_obstacle_size:=0.05 \
    max_obstacle_size:=1.5 \
    cluster_tolerance:=0.15
```

### Planner Node Parameters

| Parameter | Default | Description | Range |
|-----------|---------|-------------|-------|
| `grid_size` | 0.1m | A* grid resolution | 0.01 - 0.5m |
| `robot_radius` | 0.3m | Robot collision radius | 0.1 - 1.0m |

**Example Configuration:**
```bash
ros2 launch path_ros2 demo.launch.py \
    grid_size:=0.05 \
    robot_radius:=0.25
```

### Control Node Parameters

| Parameter | Default | Description | Range |
|-----------|---------|-------------|-------|
| `lookahead_distance` | 0.5m | Pure pursuit lookahead distance | 0.1 - 2.0m |
| `max_linear_vel` | 0.5 m/s | Maximum linear velocity | 0.1 - 2.0 m/s |
| `max_angular_vel` | 1.0 rad/s | Maximum angular velocity | 0.1 - 3.0 rad/s |
| `control_frequency` | 10.0 Hz | Control loop frequency | 1.0 - 50.0 Hz |

**Example Configuration:**
```bash
ros2 launch path_ros2 demo.launch.py \
    lookahead_distance:=0.8 \
    max_linear_vel:=0.8 \
    max_angular_vel:=1.5 \
    control_frequency:=20.0
```

## Launch File Parameters

### Demo Launch Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `world_file` | `simple_warehouse.world` | Gazebo world file |
| `robot_name` | `husky` | Robot model (husky/turtlebot3) |
| `start_x` | 0.0 | Initial x position |
| `start_y` | 0.0 | Initial y position |
| `start_yaw` | 0.0 | Initial yaw angle |
| `goal_x` | 5.0 | Target x position |
| `goal_y` | 3.0 | Target y position |
| `goal_yaw` | 0.0 | Target yaw angle |

### Simulation Launch Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `world_file` | `simple_warehouse.world` | Gazebo world file |
| `robot_name` | `husky` | Robot model |
| `x` | 0.0 | Spawn x position |
| `y` | 0.0 | Spawn y position |
| `z` | 0.0 | Spawn z position |
| `yaw` | 0.0 | Spawn yaw angle |

## World Configuration

### Creating Custom Worlds

1. **Create world file** in `worlds/` directory:
   ```xml
   <?xml version="1.0" ?>
   <sdf version="1.6">
     <world name="my_custom_world">
       <!-- Add your world content here -->
     </world>
   </sdf>
   ```

2. **Launch with custom world**:
   ```bash
   ros2 launch path_ros2 demo.launch.py world_file:=my_custom_world.world
   ```

### World Elements

#### Ground Plane
```xml
<include>
  <uri>model://ground_plane</uri>
</include>
```

#### Walls
```xml
<model name="wall_1">
  <pose>5 0 1 0 0 0</pose>
  <static>true</static>
  <link name="link">
    <collision name="collision">
      <geometry>
        <box>
          <size>0.2 10 2</size>
        </box>
      </geometry>
    </collision>
  </link>
</model>
```

#### Obstacles
```xml
<model name="box_1">
  <pose>2 1 0.5 0 0 0</pose>
  <static>true</static>
  <link name="link">
    <collision name="collision">
      <geometry>
        <box>
          <size>1 1 1</size>
        </box>
      </geometry>
    </collision>
  </link>
</model>
```

## Robot Configuration

### Robot Models

#### Husky Robot
- **Size**: 1.0m × 0.6m × 0.2m
- **Wheel Separation**: 0.6m
- **Wheel Diameter**: 0.2m
- **LiDAR**: 720 samples, 30m range

#### Turtlebot3 Robot
- **Size**: 0.2m diameter × 0.2m height
- **Wheel Separation**: 0.16m
- **Wheel Diameter**: 0.066m
- **LiDAR**: 360 samples, 30m range

### Custom Robot Models

1. **Create robot description** in launch file
2. **Modify parameters** for your robot:
   - Wheel separation
   - Wheel diameter
   - Robot dimensions
   - LiDAR specifications

## Topic Configuration

### Topic Remapping

```bash
# Remap topics for different namespaces
ros2 launch path_ros2 demo.launch.py \
    --ros-args \
    -r /scan:=/robot1/scan \
    -r /cmd_vel:=/robot1/cmd_vel
```

### QoS Settings

```python
# In node code, modify QoS settings
qos_profile = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10
)
```

## RViz Configuration

### Custom RViz Config

1. **Modify existing config** in `rviz/navigation.rviz`
2. **Add new displays**:
   - Robot model
   - LiDAR scans
   - Path visualization
   - Obstacle markers

### Display Settings

| Display | Topic | Type | Description |
|---------|-------|------|-------------|
| RobotModel | `/robot_description` | Robot Model | Robot visualization |
| LaserScan | `/scan` | LaserScan | LiDAR data |
| Path | `/planned_path` | Path | Planned trajectory |
| Obstacles | `/obstacles` | MarkerArray | Detected obstacles |
| PathMarkers | `/path_markers` | MarkerArray | Path visualization |

## Performance Tuning

### Planning Performance

| Parameter | Impact | Recommendation |
|-----------|--------|----------------|
| `grid_size` | Smaller = slower, more accurate | 0.1m for balance |
| `robot_radius` | Larger = safer, less efficient | Match actual robot size |
| `obstacle_threshold` | Larger = fewer obstacles | 0.5m for warehouse |

### Control Performance

| Parameter | Impact | Recommendation |
|-----------|--------|----------------|
| `lookahead_distance` | Larger = smoother, less precise | 0.5m for balance |
| `max_linear_vel` | Higher = faster, less stable | 0.5 m/s for safety |
| `control_frequency` | Higher = smoother, more CPU | 10 Hz for balance |

### Perception Performance

| Parameter | Impact | Recommendation |
|-----------|--------|----------------|
| `cluster_tolerance` | Larger = fewer clusters | 0.2m for balance |
| `min_obstacle_size` | Larger = fewer false positives | 0.1m for accuracy |
| `max_obstacle_size` | Smaller = more detailed | 2.0m for warehouse |

## Configuration Files

### Parameter Files

Create YAML parameter files:

```yaml
# config/navigation_params.yaml
perception_node:
  ros__parameters:
    obstacle_threshold: 0.5
    min_obstacle_size: 0.1
    max_obstacle_size: 2.0
    cluster_tolerance: 0.2

planner_node:
  ros__parameters:
    grid_size: 0.1
    robot_radius: 0.3

control_node:
  ros__parameters:
    lookahead_distance: 0.5
    max_linear_vel: 0.5
    max_angular_vel: 1.0
    control_frequency: 10.0
```

### Using Parameter Files

```bash
ros2 launch path_ros2 demo.launch.py \
    --ros-args --params-file config/navigation_params.yaml
```

## Environment-Specific Configurations

### Warehouse Environment
```bash
ros2 launch path_ros2 demo.launch.py \
    obstacle_threshold:=0.5 \
    grid_size:=0.1 \
    lookahead_distance:=0.5
```

### Cluttered Environment
```bash
ros2 launch path_ros2 demo.launch.py \
    obstacle_threshold:=0.3 \
    grid_size:=0.05 \
    lookahead_distance:=0.3
```

### Open Environment
```bash
ros2 launch path_ros2 demo.launch.py \
    obstacle_threshold:=1.0 \
    grid_size:=0.2 \
    lookahead_distance:=1.0
```

## Dynamic Reconfiguration

### Runtime Parameter Changes

```bash
# Change parameters while running
ros2 param set /perception_node obstacle_threshold 0.3
ros2 param set /planner_node grid_size 0.05
ros2 param set /control_node lookahead_distance 0.8
```

### Parameter Monitoring

```bash
# List all parameters
ros2 param list

# Get parameter value
ros2 param get /perception_node obstacle_threshold

# Describe parameter
ros2 param describe /perception_node obstacle_threshold
```

---
