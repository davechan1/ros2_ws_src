#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_publisher')
        self.publisher_ = self.create_publisher(Path, 'waypoints', 10)
        
        # Wait a bit for subscribers to connect
        self.timer = self.create_timer(1, self.publish_path)
        self.published = False

    def publish_path(self):
        if self.published:
            return
            
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'

        # Create some dummy poses
        for i in range(10):
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'map'
            
            pose.pose.position.x = i * 0.5
            pose.pose.position.y = i * 0.3
            pose.pose.position.z = 0.0
            
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0
            
            path_msg.poses.append(pose)

        self.publisher_.publish(path_msg)
        self.get_logger().info(f'Published path with {len(path_msg.poses)} poses')
        self.published = True
        
        # Cancel the timer after publishing
        self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    path_publisher = PathPublisher()
    rclpy.spin(path_publisher)
    path_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

