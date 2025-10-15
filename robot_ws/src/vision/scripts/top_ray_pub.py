#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class RayPublisher(Node):
    def __init__(self):
        super().__init__('ray_publisher')
        self.publisher = self.create_publisher(Marker, 'visualization_marker', QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            ))
        self.timer = self.create_timer(0.1, self.publish_ray)

    def publish_ray(self):
        marker = Marker()
        marker.header.frame_id = "CameraTop"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.01  # Line width
        
        # Start and end points of the ray
        start_point = Point(x=0.0, y=0.0, z=0.0)
        end_point = Point(x=5.0, y=0.0, z=0.0)  # 5 meters in x direction
        marker.points = [start_point, end_point]
        
        # Red color
        marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        print("Publishing marker")
        self.publisher.publish(marker)

def main():
    rclpy.init()
    node = RayPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()