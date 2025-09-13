#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import sys
import math


class GoalPublisher(Node):
    def __init__(self, goal_x=5.0, goal_y=3.0, goal_yaw=0.0):
        super().__init__('goal_publisher')
        
        self.goal_x = float(goal_x)
        self.goal_y = float(goal_y)
        self.goal_yaw = float(goal_yaw)
        
        self.goal_pub = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            10
        )
        
        self.timer = self.create_timer(1.0, self.publish_goal)
        self.published = False
        
        self.get_logger().info(f'Goal publisher initialized with goal: ({self.goal_x}, {self.goal_y}, {self.goal_yaw})')
    
    def publish_goal(self):
        if not self.published:
            goal_msg = PoseStamped()
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            goal_msg.header.frame_id = 'map'
            
            goal_msg.pose.position.x = self.goal_x
            goal_msg.pose.position.y = self.goal_y
            goal_msg.pose.position.z = 0.0
            
            goal_msg.pose.orientation.w = math.cos(self.goal_yaw / 2.0)
            goal_msg.pose.orientation.x = 0.0
            goal_msg.pose.orientation.y = 0.0
            goal_msg.pose.orientation.z = math.sin(self.goal_yaw / 2.0)
            
            self.goal_pub.publish(goal_msg)
            self.published = True
            
            self.get_logger().info(f'Goal published: ({self.goal_x}, {self.goal_y}, {self.goal_yaw})')
            
            self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    
    goal_x = 5.0
    goal_y = 3.0
    goal_yaw = 0.0
    
    if len(sys.argv) >= 4:
        goal_x = float(sys.argv[1])
        goal_y = float(sys.argv[2])
        goal_yaw = float(sys.argv[3])
    
    goal_publisher = GoalPublisher(goal_x, goal_y, goal_yaw)
    
    try:
        rclpy.spin(goal_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        goal_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
