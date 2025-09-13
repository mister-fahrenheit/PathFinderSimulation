#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import Header, ColorRGBA
import numpy as np
import math


class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        
        self.declare_parameter('obstacle_threshold', 0.5)  # meters
        self.declare_parameter('min_obstacle_size', 0.1)   # meters
        self.declare_parameter('max_obstacle_size', 2.0)   # meters
        self.declare_parameter('cluster_tolerance', 0.2)   # meters
        
        self.laser_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )
        
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/obstacles',
            10
        )
        
        self.get_logger().info('Perception node initialized')
        self.get_logger().info('Subscribing to /scan')
        self.get_logger().info('Publishing to /obstacles')

    def laser_callback(self, msg):
        try:
            obstacles = self.detect_obstacles(msg)
            self.publish_obstacles(obstacles, msg.header)
        except Exception as e:
            self.get_logger().error(f'Error processing laser scan: {str(e)}')

    def detect_obstacles(self, scan_msg):
        obstacles = []
        obstacle_threshold = self.get_parameter('obstacle_threshold').value
        min_size = self.get_parameter('min_obstacle_size').value
        max_size = self.get_parameter('max_obstacle_size').value
        cluster_tolerance = self.get_parameter('cluster_tolerance').value
        
        points = []
        for i, range_val in enumerate(scan_msg.ranges):
            if (range_val < scan_msg.range_max and 
                range_val > scan_msg.range_min and 
                range_val < obstacle_threshold):
                
                angle = scan_msg.angle_min + i * scan_msg.angle_increment
                x = range_val * math.cos(angle)
                y = range_val * math.sin(angle)
                points.append((x, y))
        
        clusters = self.cluster_points(points, cluster_tolerance)
        
        for cluster in clusters:
            if len(cluster) >= 3:
                x_coords = [p[0] for p in cluster]
                y_coords = [p[1] for p in cluster]
                
                min_x, max_x = min(x_coords), max(x_coords)
                min_y, max_y = min(y_coords), max(y_coords)
                
                width = max_x - min_x
                height = max_y - min_y
                size = max(width, height)
                
                if min_size <= size <= max_size:
                    center_x = (min_x + max_x) / 2
                    center_y = (min_y + max_y) / 2
                    obstacles.append({
                        'center': (center_x, center_y),
                        'size': size,
                        'width': width,
                        'height': height
                    })
        
        return obstacles

    def cluster_points(self, points, tolerance):
        if not points:
            return []
        
        clusters = []
        used_points = set()
        
        for i, point in enumerate(points):
            if i in used_points:
                continue
                
            cluster = [point]
            used_points.add(i)
            
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

    def publish_obstacles(self, obstacles, header):
        marker_array = MarkerArray()
        
        clear_marker = Marker()
        clear_marker.header = header
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)
        
        for i, obstacle in enumerate(obstacles):
            marker = Marker()
            marker.header = header
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            marker.pose.position.x = obstacle['center'][0]
            marker.pose.position.y = obstacle['center'][1]
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = obstacle['width']
            marker.scale.y = obstacle['height']
            marker.scale.z = 0.5
            
            marker.color = ColorRGBA()
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.7
            
            marker_array.markers.append(marker)
        
        self.marker_pub.publish(marker_array)
        
        if obstacles:
            self.get_logger().info(f'Detected {len(obstacles)} obstacles')


def main(args=None):
    rclpy.init(args=args)
    
    perception_node = PerceptionNode()
    
    try:
        rclpy.spin(perception_node)
    except KeyboardInterrupt:
        pass
    finally:
        perception_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
