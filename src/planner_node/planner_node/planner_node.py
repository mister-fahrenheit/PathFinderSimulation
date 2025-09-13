#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, ColorRGBA
import math
import heapq
from typing import List, Tuple, Optional


class AStarPlanner:
    def __init__(self, grid_size: float = 0.1, robot_radius: float = 0.3):
        self.grid_size = grid_size
        self.robot_radius = robot_radius
        self.obstacles = []
    
    def set_obstacles(self, obstacles: List[dict]):
        self.obstacles = obstacles
    
    def heuristic(self, a: Tuple[float, float], b: Tuple[float, float]) -> float:
        return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
    
    def is_collision(self, point: Tuple[float, float]) -> bool:
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
    
    def get_neighbors(self, point: Tuple[float, float]) -> List[Tuple[float, float]]:
        neighbors = []
        directions = [
            (self.grid_size, 0), (-self.grid_size, 0),
            (0, self.grid_size), (0, -self.grid_size),
            (self.grid_size, self.grid_size), (-self.grid_size, -self.grid_size),
            (self.grid_size, -self.grid_size), (-self.grid_size, self.grid_size)
        ]
        
        for dx, dy in directions:
            neighbor = (point[0] + dx, point[1] + dy)
            if not self.is_collision(neighbor):
                neighbors.append(neighbor)
        
        return neighbors
    
    def plan_path(self, start: Tuple[float, float], goal: Tuple[float, float]) -> Optional[List[Tuple[float, float]]]:
        if self.is_collision(start) or self.is_collision(goal):
            return None
        
        open_set = [(0, start)]
        came_from = {}
        g_cost = {start: 0}
        f_cost = {start: self.heuristic(start, goal)}
        
        while open_set:
            current = heapq.heappop(open_set)[1]
            
            if self.heuristic(current, goal) < self.grid_size:
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


class PlannerNode(Node):
    def __init__(self):
        super().__init__('planner_node')
        
        self.declare_parameter('grid_size', 0.1)
        self.declare_parameter('robot_radius', 0.3)
        
        grid_size = self.get_parameter('grid_size').value
        robot_radius = self.get_parameter('robot_radius').value
        self.planner = AStarPlanner(grid_size, robot_radius)
        
        self.current_pose = None
        self.goal_pose = None
        self.obstacles = []
        self.current_path = None
        
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/robot_pose',
            self.pose_callback,
            10
        )
        
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )
        
        self.path_pub = self.create_publisher(
            Path,
            '/planned_path',
            10
        )
        
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/path_markers',
            10
        )
        
        self.get_logger().info('Planner node initialized')
        self.get_logger().info('Subscribing to /robot_pose and /goal_pose')
        self.get_logger().info('Publishing to /planned_path and /path_markers')
    
    def pose_callback(self, msg):
        self.current_pose = msg
        self.plan_if_ready()
    
    def goal_callback(self, msg):
        self.goal_pose = msg
        self.get_logger().info(f'New goal received: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')
        self.plan_if_ready()
    
    def plan_if_ready(self):
        if self.current_pose is None or self.goal_pose is None:
            return
        
        start = (self.current_pose.pose.position.x, self.current_pose.pose.position.y)
        goal = (self.goal_pose.pose.position.x, self.goal_pose.pose.position.y)
        
        self.planner.set_obstacles(self.obstacles)
        
        path_points = self.planner.plan_path(start, goal)
        
        if path_points:
            self.current_path = path_points
            self.publish_path(path_points)
            self.publish_path_markers(path_points)
            self.get_logger().info(f'Path planned with {len(path_points)} waypoints')
        else:
            self.get_logger().warn('No valid path found to goal')
    
    def publish_path(self, path_points: List[Tuple[float, float]]):
        path_msg = Path()
        path_msg.header = Header()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'
        
        for point in path_points:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        
        self.path_pub.publish(path_msg)
    
    def publish_path_markers(self, path_points: List[Tuple[float, float]]):
        marker_array = MarkerArray()
        
        clear_marker = Marker()
        clear_marker.header.frame_id = 'map'
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)
        
        if len(path_points) > 1:
            path_marker = Marker()
            path_marker.header.frame_id = 'map'
            path_marker.header.stamp = self.get_clock().now().to_msg()
            path_marker.id = 0
            path_marker.type = Marker.LINE_STRIP
            path_marker.action = Marker.ADD

            path_marker.scale.x = 0.05
            
            path_marker.color = ColorRGBA()
            path_marker.color.r = 0.0
            path_marker.color.g = 1.0
            path_marker.color.b = 0.0
            path_marker.color.a = 1.0
            
            for point in path_points:
                p = Point()
                p.x = point[0]
                p.y = point[1]
                p.z = 0.1
                path_marker.points.append(p)
            
            marker_array.markers.append(path_marker)
        
        if path_points:
            start_marker = Marker()
            start_marker.header.frame_id = 'map'
            start_marker.header.stamp = self.get_clock().now().to_msg()
            start_marker.id = 1
            start_marker.type = Marker.SPHERE
            start_marker.action = Marker.ADD
            
            start_marker.pose.position.x = path_points[0][0]
            start_marker.pose.position.y = path_points[0][1]
            start_marker.pose.position.z = 0.2
            start_marker.pose.orientation.w = 1.0
            
            start_marker.scale.x = 0.2
            start_marker.scale.y = 0.2
            start_marker.scale.z = 0.2
            
            start_marker.color = ColorRGBA()
            start_marker.color.r = 0.0
            start_marker.color.g = 0.0
            start_marker.color.b = 1.0
            start_marker.color.a = 1.0
            
            marker_array.markers.append(start_marker)
            
            goal_marker = Marker()
            goal_marker.header.frame_id = 'map'
            goal_marker.header.stamp = self.get_clock().now().to_msg()
            goal_marker.id = 2
            goal_marker.type = Marker.SPHERE
            goal_marker.action = Marker.ADD
            
            goal_marker.pose.position.x = path_points[-1][0]
            goal_marker.pose.position.y = path_points[-1][1]
            goal_marker.pose.position.z = 0.2
            goal_marker.pose.orientation.w = 1.0
            
            goal_marker.scale.x = 0.2
            goal_marker.scale.y = 0.2
            goal_marker.scale.z = 0.2
            
            goal_marker.color = ColorRGBA()
            goal_marker.color.r = 1.0
            goal_marker.color.g = 0.0
            goal_marker.color.b = 0.0
            goal_marker.color.a = 1.0
            
            marker_array.markers.append(goal_marker)
        
        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    
    planner_node = PlannerNode()
    
    try:
        rclpy.spin(planner_node)
    except KeyboardInterrupt:
        pass
    finally:
        planner_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
