# API Reference

This document provides detailed API reference for all nodes, topics, and services in the autonomous navigation system.

## Topics

### Input Topics

#### `/scan`
- **Type**: `sensor_msgs/LaserScan`
- **Publisher**: Gazebo LiDAR sensor
- **Subscriber**: `perception_node`
- **Description**: LiDAR sensor data from the robot
- **Frequency**: 40 Hz

**Message Structure:**
```python
Header header
float32 angle_min
float32 angle_max
float32 angle_increment
float32 time_increment
float32 scan_time
float32 range_min
float32 range_max
float32[] ranges
float32[] intensities
```

#### `/odom`
- **Type**: `nav_msgs/Odometry`
- **Publisher**: Gazebo differential drive plugin
- **Subscriber**: `odom_to_pose` node
- **Description**: Robot odometry information
- **Frequency**: 50 Hz

**Message Structure:**
```python
Header header
string child_frame_id
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist
```

#### `/goal_pose`
- **Type**: `geometry_msgs/PoseStamped`
- **Publisher**: `goal_publisher` node
- **Subscriber**: `planner_node`
- **Description**: Target goal pose for navigation
- **Frequency**: On demand

**Message Structure:**
```python
Header header
geometry_msgs/Pose pose
```

### Output Topics

#### `/obstacles`
- **Type**: `visualization_msgs/MarkerArray`
- **Publisher**: `perception_node`
- **Subscriber**: RViz, `planner_node`
- **Description**: Detected obstacles as visualization markers
- **Frequency**: 10 Hz

**Message Structure:**
```python
visualization_msgs/Marker[] markers
```

#### `/robot_pose`
- **Type**: `geometry_msgs/PoseStamped`
- **Publisher**: `odom_to_pose` node
- **Subscriber**: `planner_node`, `control_node`
- **Description**: Current robot pose
- **Frequency**: 10 Hz

**Message Structure:**
```python
Header header
geometry_msgs/Pose pose
```

#### `/planned_path`
- **Type**: `nav_msgs/Path`
- **Publisher**: `planner_node`
- **Subscriber**: `control_node`, RViz
- **Description**: Planned navigation path
- **Frequency**: On demand

**Message Structure:**
```python
Header header
geometry_msgs/PoseStamped[] poses
```

#### `/path_markers`
- **Type**: `visualization_msgs/MarkerArray`
- **Publisher**: `planner_node`
- **Subscriber**: RViz
- **Description**: Path visualization markers
- **Frequency**: On demand

**Message Structure:**
```python
visualization_msgs/Marker[] markers
```

#### `/cmd_vel`
- **Type**: `geometry_msgs/Twist`
- **Publisher**: `control_node`
- **Subscriber**: Gazebo differential drive plugin
- **Description**: Velocity commands for robot movement
- **Frequency**: 10 Hz

**Message Structure:**
```python
geometry_msgs/Vector3 linear
geometry_msgs/Vector3 angular
```

## Nodes

### Perception Node

#### Class: `PerceptionNode`
- **Package**: `perception_node`
- **Executable**: `perception_node`
- **Description**: Processes LiDAR data to detect obstacles

#### Parameters
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `obstacle_threshold` | float | 0.5 | Maximum distance to consider as obstacle (m) |
| `min_obstacle_size` | float | 0.1 | Minimum obstacle size to detect (m) |
| `max_obstacle_size` | float | 2.0 | Maximum obstacle size to detect (m) |
| `cluster_tolerance` | float | 0.2 | Distance for clustering nearby points (m) |

#### Methods
```python
def __init__(self)
    """Initialize the perception node"""

def laser_callback(self, msg: LaserScan)
    """Process incoming laser scan data"""

def detect_obstacles(self, scan_msg: LaserScan) -> List[dict]
    """Detect obstacles from laser scan data"""

def cluster_points(self, points: List[Tuple[float, float]], tolerance: float) -> List[List[Tuple[float, float]]]
    """Cluster nearby points into obstacles"""

def publish_obstacles(self, obstacles: List[dict], header: Header)
    """Publish detected obstacles as markers"""
```

### Planner Node

#### Class: `PlannerNode`
- **Package**: `planner_node`
- **Executable**: `planner_node`
- **Description**: Plans collision-free paths using A* algorithm

#### Parameters
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `grid_size` | float | 0.1 | A* grid resolution (m) |
| `robot_radius` | float | 0.3 | Robot collision radius (m) |

#### Methods
```python
def __init__(self)
    """Initialize the planner node"""

def pose_callback(self, msg: PoseStamped)
    """Update current robot pose"""

def goal_callback(self, msg: PoseStamped)
    """Update goal pose and trigger planning"""

def plan_if_ready(self)
    """Plan path if we have both current pose and goal"""

def publish_path(self, path_points: List[Tuple[float, float]])
    """Publish path as nav_msgs/Path"""

def publish_path_markers(self, path_points: List[Tuple[float, float]])
    """Publish path as visualization markers"""
```

#### A* Planner Class: `AStarPlanner`
```python
def __init__(self, grid_size: float = 0.1, robot_radius: float = 0.3)
    """Initialize A* planner"""

def set_obstacles(self, obstacles: List[dict])
    """Set the list of obstacles for planning"""

def heuristic(self, a: Tuple[float, float], b: Tuple[float, float]) -> float
    """Calculate heuristic distance between two points"""

def is_collision(self, point: Tuple[float, float]) -> bool
    """Check if a point collides with any obstacle"""

def get_neighbors(self, point: Tuple[float, float]) -> List[Tuple[float, float]]
    """Get valid neighboring points"""

def plan_path(self, start: Tuple[float, float], goal: Tuple[float, float]) -> Optional[List[Tuple[float, float]]]
    """Plan a path from start to goal using A* algorithm"""
```

### Control Node

#### Class: `ControlNode`
- **Package**: `control_node`
- **Executable**: `control_node`
- **Description**: Converts planned paths into velocity commands

#### Parameters
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `lookahead_distance` | float | 0.5 | Pure pursuit lookahead distance (m) |
| `max_linear_vel` | float | 0.5 | Maximum linear velocity (m/s) |
| `max_angular_vel` | float | 1.0 | Maximum angular velocity (rad/s) |
| `control_frequency` | float | 10.0 | Control loop frequency (Hz) |

#### Methods
```python
def __init__(self)
    """Initialize the control node"""

def pose_callback(self, msg: PoseStamped)
    """Update current robot pose"""

def path_callback(self, msg: Path)
    """Update planned path"""

def control_callback(self)
    """Main control loop"""

def publish_stop_command(self)
    """Publish stop command"""
```

#### Pure Pursuit Controller Class: `PurePursuitController`
```python
def __init__(self, lookahead_distance: float = 0.5, max_linear_vel: float = 0.5, max_angular_vel: float = 1.0)
    """Initialize pure pursuit controller"""

def set_pose(self, pose: PoseStamped)
    """Update current robot pose"""

def set_path(self, path: List[Tuple[float, float]])
    """Set the path to follow"""

def get_lookahead_point(self) -> Optional[Tuple[float, float]]
    """Get the lookahead point on the path"""

def compute_velocity_commands(self) -> Optional[Twist]
    """Compute velocity commands using pure pursuit"""

def quaternion_to_yaw(self, quaternion) -> float
    """Convert quaternion to yaw angle"""

def normalize_angle(self, angle: float) -> float
    """Normalize angle to [-pi, pi]"""
```

### Utility Nodes

#### Odom to Pose Node
- **Package**: `path_ros2`
- **Executable**: `odom_to_pose`
- **Description**: Converts odometry messages to pose stamped messages

#### Goal Publisher Node
- **Package**: `path_ros2`
- **Executable**: `goal_publisher`
- **Description**: Publishes goal poses for navigation

**Command Line Arguments:**
```bash
ros2 run path_ros2 goal_publisher <goal_x> <goal_y> <goal_yaw>
```

## Launch Files

### Demo Launch
- **File**: `launch/demo.launch.py`
- **Description**: Complete end-to-end demo

**Parameters:**
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `world_file` | string | `simple_warehouse.world` | Gazebo world file |
| `robot_name` | string | `husky` | Robot model |
| `start_x` | float | 0.0 | Initial x position |
| `start_y` | float | 0.0 | Initial y position |
| `start_yaw` | float | 0.0 | Initial yaw angle |
| `goal_x` | float | 5.0 | Target x position |
| `goal_y` | float | 3.0 | Target y position |
| `goal_yaw` | float | 0.0 | Target yaw angle |

### Simulation Launch
- **File**: `launch/simulation.launch.py`
- **Description**: Gazebo simulation and robot spawning

**Parameters:**
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `world_file` | string | `simple_warehouse.world` | Gazebo world file |
| `robot_name` | string | `husky` | Robot model |
| `x` | float | 0.0 | Spawn x position |
| `y` | float | 0.0 | Spawn y position |
| `z` | float | 0.0 | Spawn z position |
| `yaw` | float | 0.0 | Spawn yaw angle |

### Navigation Launch
- **File**: `launch/navigation.launch.py`
- **Description**: Navigation stack (perception, planning, control)

**Parameters:**
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `use_sim_time` | bool | true | Use simulation time |

## Services

Currently, no custom services are implemented. The system uses standard ROS2 services for:
- Parameter setting/getting
- Node lifecycle management
- TF2 transforms

## Message Types

### Custom Message Types
No custom message types are defined. The system uses standard ROS2 message types.

### Standard Message Types Used
- `sensor_msgs/LaserScan`
- `geometry_msgs/PoseStamped`
- `geometry_msgs/Twist`
- `nav_msgs/Path`
- `nav_msgs/Odometry`
- `visualization_msgs/MarkerArray`
- `std_msgs/Header`

## Extensibility

### Adding New Nodes
1. Create new package in `src/`
2. Implement node class inheriting from `Node`
3. Add to launch files
4. Update documentation

### Adding New Topics
1. Define message type (if custom)
2. Add publisher/subscriber in nodes
3. Update launch files if needed
4. Update documentation

### Adding New Parameters
1. Declare parameter in node constructor
2. Use parameter in node logic
3. Update launch files
4. Update documentation

## Examples

### Basic Node Template
```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        
        # Declare parameters
        self.declare_parameter('my_param', 1.0)
        
        # Create subscribers
        self.subscription = self.create_subscription(
            String,
            'my_topic',
            self.callback,
            10
        )
        
        # Create publishers
        self.publisher = self.create_publisher(
            String,
            'my_output_topic',
            10
        )
    
    def callback(self, msg):
        # Process message
        self.get_logger().info(f'Received: {msg.data}')
        
        # Publish response
        response = String()
        response.data = f'Processed: {msg.data}'
        self.publisher.publish(response)

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Launch File Template
```python
#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            executable='my_node',
            name='my_node',
            parameters=[{'my_param': 2.0}]
        )
    ])
```

---
