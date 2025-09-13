# Architecture Guide

This guide explains the system architecture, algorithms, and design decisions.

## System Architecture

### High-Level Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                    ROS2 Autonomous Navigation System            │
├─────────────────────────────────────────────────────────────────┤
│  Simulation Layer  │  Perception Layer  │  Planning Layer      │
│  ┌──────────────┐  │  ┌──────────────┐  │  ┌──────────────┐    │
│  │   Gazebo     │  │  │   LiDAR      │  │  │   A*         │    │
│  │   World      │  │  │   Processing │  │  │   Planner    │    │
│  │   Robot      │  │  │   Clustering │  │  │   Grid Map   │    │
│  └──────────────┘  │  └──────────────┘  │  └──────────────┘    │
├─────────────────────────────────────────────────────────────────┤
│  Control Layer     │  Visualization Layer │  Integration Layer  │
│  ┌──────────────┐  │  ┌──────────────┐  │  ┌──────────────┐    │
│  │   Pure       │  │  │   RViz       │  │  │   Launch     │    │
│  │   Pursuit    │  │  │   Markers    │  │  │   Files      │    │
│  │   Controller │  │  │   Displays   │  │  │   Topics     │    │
│  └──────────────┘  │  └──────────────┘  │  └──────────────┘    │
└─────────────────────────────────────────────────────────────────┘
```

### Component Interaction

```
┌─────────────┐    ┌─────────────┐    ┌─────────────┐    ┌─────────────┐
│   Gazebo    │    │ Perception  │    │  Planner    │    │  Control    │
│             │    │   Node      │    │   Node      │    │   Node      │
│ • Robot     │───▶│             │───▶│             │───▶│             │
│ • LiDAR     │    │ • Scan      │    │ • Path      │    │ • Velocity  │
│ • World     │    │ • Clustering│    │ • A*        │    │ • Commands  │
│ • Physics   │    │ • Markers   │    │ • Grid      │    │ • Pure      │
└─────────────┘    └─────────────┘    └─────────────┘    └─────────────┘
       │                   │                   │                   │
       ▼                   ▼                   ▼                   ▼
┌─────────────┐    ┌─────────────┐    ┌─────────────┐    ┌─────────────┐
│   Topics    │    │   Topics    │    │   Topics    │    │   Topics    │
│             │    │             │    │             │    │             │
│ /scan       │    │ /obstacles  │    │ /planned_   │    │ /cmd_vel    │
│ /odom       │    │ /robot_pose │    │ path        │    │ /robot_pose │
│ /cmd_vel    │    │             │    │ /path_      │    │             │
└─────────────┘    └─────────────┘    └─────────────┘    └─────────────┘
```

## Algorithm Details

### 1. Obstacle Detection Algorithm

#### Input Processing
```python
def detect_obstacles(self, scan_msg):
    # Convert polar to cartesian coordinates
    points = []
    for i, range_val in enumerate(scan_msg.ranges):
        if (range_val < scan_msg.range_max and 
            range_val > scan_msg.range_min and 
            range_val < obstacle_threshold):
            
            angle = scan_msg.angle_min + i * scan_msg.angle_increment
            x = range_val * math.cos(angle)
            y = range_val * math.sin(angle)
            points.append((x, y))
```

#### Clustering Algorithm
```python
def cluster_points(self, points, tolerance):
    clusters = []
    used_points = set()
    
    for i, point in enumerate(points):
        if i in used_points:
            continue
            
        cluster = [point]
        used_points.add(i)
        
        # Find nearby points
        for j, other_point in enumerate(points):
            if j in used_points:
                continue
                
            distance = math.sqrt(
                (point[0] - other_point[0])**2 + 
                (point[1] - other_point[1])**2
            )
            
            if distance <= tolerance:
                cluster.append(other_point)
                used_points.add(j)
        
        clusters.append(cluster)
    
    return clusters
```

#### Bounding Box Calculation
```python
def calculate_bounding_box(self, cluster):
    x_coords = [p[0] for p in cluster]
    y_coords = [p[1] for p in cluster]
    
    min_x, max_x = min(x_coords), max(x_coords)
    min_y, max_y = min(y_coords), max(y_coords)
    
    width = max_x - min_x
    height = max_y - min_y
    size = max(width, height)
    
    center_x = (min_x + max_x) / 2
    center_y = (min_y + max_y) / 2
    
    return {
        'center': (center_x, center_y),
        'size': size,
        'width': width,
        'height': height
    }
```

### 2. A* Path Planning Algorithm

#### Grid-Based Search
```python
def plan_path(self, start, goal):
    # Priority queue: (f_cost, point)
    open_set = [(0, start)]
    came_from = {}
    g_cost = {start: 0}
    f_cost = {start: self.heuristic(start, goal)}
    
    while open_set:
        current = heapq.heappop(open_set)[1]
        
        if self.heuristic(current, goal) < self.grid_size:
            # Reached goal
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path
        
        for neighbor in self.get_neighbors(current):
            tentative_g = g_cost[current] + self.heuristic(current, neighbor)
            
            if neighbor not in g_cost or tentative_g < g_cost[neighbor]:
                came_from[neighbor] = current
                g_cost[neighbor] = tentative_g
                f_cost[neighbor] = tentative_g + self.heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_cost[neighbor], neighbor))
    
    return None
```

#### Heuristic Function
```python
def heuristic(self, a, b):
    # Euclidean distance
    return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
```

#### Collision Detection
```python
def is_collision(self, point):
    for obstacle in self.obstacles:
        center = obstacle['center']
        size = obstacle['size']
        
        distance = math.sqrt(
            (point[0] - center[0])**2 + 
            (point[1] - center[1])**2
        )
        
        if distance < (size / 2 + self.robot_radius):
            return True
    
    return False
```

### 3. Pure Pursuit Control Algorithm

#### Lookahead Point Selection
```python
def get_lookahead_point(self):
    current_pos = (self.current_pose.pose.position.x, 
                   self.current_pose.pose.position.y)
    
    # Find closest point on path
    min_distance = float('inf')
    closest_index = 0
    
    for i, point in enumerate(self.path):
        distance = math.sqrt(
            (point[0] - current_pos[0])**2 + 
            (point[1] - current_pos[1])**2
        )
        if distance < min_distance:
            min_distance = distance
            closest_index = i
    
    # Find lookahead point
    for i in range(closest_index, len(self.path)):
        point = self.path[i]
        distance = math.sqrt(
            (point[0] - current_pos[0])**2 + 
            (point[1] - current_pos[1])**2
        )
        if distance >= self.lookahead_distance:
            return point
    
    return self.path[-1] if self.path else None
```

#### Velocity Command Generation
```python
def compute_velocity_commands(self):
    lookahead_point = self.get_lookahead_point()
    if lookahead_point is None:
        return None
    
    current_pos = (self.current_pose.pose.position.x, 
                   self.current_pose.pose.position.y)
    
    # Calculate distance to lookahead point
    distance = math.sqrt(
        (lookahead_point[0] - current_pos[0])**2 + 
        (lookahead_point[1] - current_pos[1])**2
    )
    
    if distance < 0.1:  # Close to goal
        return Twist()  # Stop
    
    # Calculate angle to lookahead point
    angle_to_point = math.atan2(
        lookahead_point[1] - current_pos[1],
        lookahead_point[0] - current_pos[0]
    )
    
    # Get current robot orientation
    current_yaw = self.quaternion_to_yaw(self.current_pose.pose.orientation)
    
    # Calculate angular error
    angular_error = self.normalize_angle(angle_to_point - current_yaw)
    
    # Compute velocities
    linear_vel = min(self.max_linear_vel, distance * 0.5)
    angular_vel = angular_error * 2.0  # Proportional control
    
    # Limit angular velocity
    angular_vel = max(-self.max_angular_vel, 
                      min(self.max_angular_vel, angular_vel))
    
    # Create twist message
    twist = Twist()
    twist.linear.x = linear_vel
    twist.angular.z = angular_vel
    
    return twist
```

## Data Flow

### Topic Flow Diagram

```
┌─────────────┐    /scan        ┌─────────────┐    /obstacles    ┌─────────────┐
│   Gazebo    │────────────────▶│ Perception  │────────────────▶│   RViz      │
│   LiDAR     │                 │   Node      │                 │   Display   │
└─────────────┘                 └─────────────┘                 └─────────────┘
                                        │
                                        ▼
┌─────────────┐    /robot_pose  ┌─────────────┐    /planned_    ┌─────────────┐
│   Odometry  │────────────────▶│  Planner    │────────────────▶│   RViz      │
│   Node      │                 │   Node      │                 │   Display   │
└─────────────┘                 └─────────────┘                 └─────────────┘
                                        │
                                        ▼
┌─────────────┐    /cmd_vel     ┌─────────────┐    /robot_pose  ┌─────────────┐
│   Gazebo    │◀────────────────│  Control    │◀────────────────│   Odometry  │
│   Robot     │                 │   Node      │                 │   Node      │
└─────────────┘                 └─────────────┘                 └─────────────┘
```

### Message Types

| Topic | Message Type | Description | Frequency |
|-------|--------------|-------------|-----------|
| `/scan` | `sensor_msgs/LaserScan` | LiDAR sensor data | 40 Hz |
| `/obstacles` | `visualization_msgs/MarkerArray` | Detected obstacles | 10 Hz |
| `/robot_pose` | `geometry_msgs/PoseStamped` | Current robot pose | 10 Hz |
| `/goal_pose` | `geometry_msgs/PoseStamped` | Target goal pose | On demand |
| `/planned_path` | `nav_msgs/Path` | Planned trajectory | On demand |
| `/path_markers` | `visualization_msgs/MarkerArray` | Path visualization | On demand |
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands | 10 Hz |
| `/odom` | `nav_msgs/Odometry` | Robot odometry | 50 Hz |

## Design Patterns

### 1. Node-Based Architecture
- **Separation of Concerns**: Each node handles one specific task
- **Loose Coupling**: Nodes communicate via topics
- **Scalability**: Easy to add/remove nodes

### 2. Publisher-Subscriber Pattern
- **Asynchronous Communication**: Nodes don't block waiting for data
- **Decoupling**: Publishers and subscribers are independent
- **Flexibility**: Multiple subscribers can receive same data

### 3. Parameter-Based Configuration
- **Runtime Configuration**: Parameters can be changed without recompilation
- **Environment Adaptation**: Different parameters for different scenarios
- **Debugging**: Easy to tune parameters for testing

### 4. Launch File Orchestration
- **System Integration**: Launch files coordinate multiple nodes
- **Environment Management**: Different launch files for different scenarios
- **Parameter Passing**: Launch files can override default parameters

## Extensibility Points

### 1. Adding New Planners
```python
class NewPlanner:
    def plan_path(self, start, goal):
        # Implement your planning algorithm
        pass

# In planner node
self.planner = NewPlanner()
```

### 2. Adding New Controllers
```python
class NewController:
    def compute_velocity_commands(self):
        # Implement your control algorithm
        pass

# In control node
self.controller = NewController()
```

### 3. Adding New Sensors
```python
class NewSensorNode(Node):
    def __init__(self):
        super().__init__('new_sensor_node')
        # Add sensor processing
        pass
```

### 4. Adding New Worlds
```xml
<!-- Create new world file -->
<world name="new_world">
    <!-- Add world content -->
</world>
```

## Performance Considerations

### Computational Complexity

| Algorithm | Time Complexity | Space Complexity | Notes |
|-----------|----------------|------------------|-------|
| Obstacle Detection | O(n) | O(n) | n = number of LiDAR points |
| Clustering | O(n²) | O(n) | Can be optimized with spatial indexing |
| A* Planning | O(b^d) | O(b^d) | b = branching factor, d = depth |
| Pure Pursuit | O(m) | O(1) | m = path length |

### Optimization Strategies

1. **Spatial Indexing**: Use k-d trees for faster neighbor searches
2. **Grid Caching**: Cache collision checks for repeated queries
3. **Path Smoothing**: Post-process paths to reduce waypoints
4. **Adaptive Resolution**: Use different grid sizes for different areas

## Safety Considerations

### 1. Collision Avoidance
- **Robot Radius**: Always maintain safe distance from obstacles
- **Emergency Stop**: Stop robot if no valid path is found
- **Velocity Limits**: Enforce maximum velocities for safety

### 2. Error Handling
- **Invalid Data**: Handle corrupted sensor data gracefully
- **Node Failures**: Implement node health monitoring
- **Communication Loss**: Handle topic communication failures

### 3. Parameter Validation
- **Range Checking**: Validate all parameters are within safe ranges
- **Type Checking**: Ensure parameters are correct types
- **Default Values**: Provide safe default values for all parameters

---
