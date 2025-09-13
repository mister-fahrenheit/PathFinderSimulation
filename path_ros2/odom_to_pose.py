#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


class OdomToPose(Node):
    def __init__(self):
        super().__init__('odom_to_pose')
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/robot_pose',
            10
        )
        
        self.get_logger().info('Odom to pose converter initialized')
    
    def odom_callback(self, msg):
        pose_msg = PoseStamped()
        pose_msg.header = msg.header
        pose_msg.pose = msg.pose.pose
        
        self.pose_pub.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    
    odom_to_pose = OdomToPose()
    
    try:
        rclpy.spin(odom_to_pose)
    except KeyboardInterrupt:
        pass
    finally:
        odom_to_pose.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
