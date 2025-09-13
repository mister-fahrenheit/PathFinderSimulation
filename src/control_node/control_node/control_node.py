#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
import math
import numpy as np
from typing import List, Tuple, Optional


class PurePursuitController:
    def __init__(self, lookahead_distance: float = 0.5, max_linear_vel: float = 0.5, max_angular_vel: float = 1.0):
        self.lookahead_distance = lookahead_distance
        self.max_linear_vel = max_linear_vel
        self.max_angular_vel = max_angular_vel
        self.current_pose = None
        self.path = []
        self.current_waypoint_index = 0
    
    def set_pose(self, pose: PoseStamped):
        self.current_pose = pose
    
    def set_path(self, path: List[Tuple[float, float]]):
        self.path = path
        self.current_waypoint_index = 0
    
    def get_lookahead_point(self) -> Optional[Tuple[float, float]]:
        if not self.path or self.current_pose is None:
            return None
        
        current_pos = (self.current_pose.pose.position.x, self.current_pose.pose.position.y)
        
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
        
        for i in range(closest_index, len(self.path)):
            point = self.path[i]
            distance = math.sqrt(
                (point[0] - current_pos[0])**2 + 
                (point[1] - current_pos[1])**2
            )
            if distance >= self.lookahead_distance:
                return point
        
        return self.path[-1] if self.path else None
    
    def compute_velocity_commands(self) -> Optional[Twist]:
        if not self.path or self.current_pose is None:
            return None
        
        lookahead_point = self.get_lookahead_point()
        if lookahead_point is None:
            return None
        
        current_pos = (self.current_pose.pose.position.x, self.current_pose.pose.position.y)
        
        distance = math.sqrt(
            (lookahead_point[0] - current_pos[0])**2 + 
            (lookahead_point[1] - current_pos[1])**2
        )
        
        if distance < 0.1:
            return Twist()
        
        angle_to_point = math.atan2(
            lookahead_point[1] - current_pos[1],
            lookahead_point[0] - current_pos[0]
        )
        
        current_yaw = self.quaternion_to_yaw(self.current_pose.pose.orientation)
        
        angular_error = self.normalize_angle(angle_to_point - current_yaw)
        
        linear_vel = min(self.max_linear_vel, distance * 0.5)
        angular_vel = angular_error * 2.0  # Proportional control
        
        angular_vel = max(-self.max_angular_vel, min(self.max_angular_vel, angular_vel))
        
        twist = Twist()
        twist.linear.x = linear_vel
        twist.angular.z = angular_vel
        
        return twist
    
    def quaternion_to_yaw(self, quaternion) -> float:
        x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return yaw
    
    def normalize_angle(self, angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')

        self.declare_parameter('lookahead_distance', 0.5)
        self.declare_parameter('max_linear_vel', 0.5)
        self.declare_parameter('max_angular_vel', 1.0)
        self.declare_parameter('control_frequency', 10.0)

        lookahead = self.get_parameter('lookahead_distance').value
        max_lin = self.get_parameter('max_linear_vel').value
        max_ang = self.get_parameter('max_angular_vel').value
        self.controller = PurePursuitController(lookahead, max_lin, max_ang)

        self.current_pose = None
        self.current_path = []

        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/robot_pose',
            self.pose_callback,
            10
        )
        
        self.path_sub = self.create_subscription(
            Path,
            '/planned_path',
            self.path_callback,
            10
        )
        
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        control_freq = self.get_parameter('control_frequency').value
        self.control_timer = self.create_timer(
            1.0 / control_freq,
            self.control_callback
        )
        
        self.get_logger().info('Control node initialized')
        self.get_logger().info('Subscribing to /robot_pose and /planned_path')
        self.get_logger().info('Publishing to /cmd_vel')
    
    def pose_callback(self, msg):
        self.current_pose = msg
        self.controller.set_pose(msg)
    
    def path_callback(self, msg):
        path_points = []
        for pose_stamped in msg.poses:
            path_points.append((pose_stamped.pose.position.x, pose_stamped.pose.position.y))
        
        self.current_path = path_points
        self.controller.set_path(path_points)
        
        if path_points:
            self.get_logger().info(f'New path received with {len(path_points)} waypoints')
    
    def control_callback(self):
        if not self.current_path or self.current_pose is None:
            self.publish_stop_command()
            return
        
        twist = self.controller.compute_velocity_commands()
        
        if twist is not None:
            self.cmd_vel_pub.publish(twist)
        else:
            self.publish_stop_command()
    
    def publish_stop_command(self):
        twist = Twist()
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    
    control_node = ControlNode()
    
    try:
        rclpy.spin(control_node)
    except KeyboardInterrupt:
        pass
    finally:
        control_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
