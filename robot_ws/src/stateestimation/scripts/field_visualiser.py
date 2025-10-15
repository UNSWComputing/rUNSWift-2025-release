import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from math import cos, sin

LINE_WIDTH = 0.05


class SoccerFieldPublisher(Node):
    def __init__(self):
        super().__init__('soccer_field_publisher')

        # Publisher
        self.publisher_ = self.create_publisher(Marker, 'soccer_field', 10)
        self.timer = self.create_timer(1, self.publish_field_markers)

    def publish_field_markers(self):
        # Soccer field dimensions in meters
        field_length = 9.0
        field_width = 6.0
        border_strip = 0.7
        goal_area_length = 0.6
        goal_area_width = 2.2
        penalty_area_length = 1.65
        penalty_area_width = 4.0
        center_circle_radius = 1.5/2.0

        # Create field boundary marker
        field_marker = self.create_rectangle_marker(
            -field_length / 2, -field_width / 2, field_length / 2, field_width / 2, 'field', 0, 1.0, 1.0, 1.0
        )

        # Goal area markers
        left_goal_area = self.create_rectangle_marker(
            -field_length / 2, -goal_area_width / 2,
            -field_length / 2 + goal_area_length, goal_area_width / 2,
            'goal_area', 1, 1.0, 1.0, 1.0
        )
        right_goal_area = self.create_rectangle_marker(
            field_length / 2 - goal_area_length, -goal_area_width / 2,
            field_length / 2, goal_area_width / 2,
            'goal_area', 2, 1.0, 1.0, 1.0
        )

        # Penalty area markers
        left_penalty_area = self.create_rectangle_marker(
            -field_length / 2, -penalty_area_width / 2,
            -field_length / 2 + penalty_area_length, penalty_area_width / 2,
            'penalty_area', 3, 1.0, 1.0, 1.0
        )
        right_penalty_area = self.create_rectangle_marker(
            field_length / 2 - penalty_area_length, -penalty_area_width / 2,
            field_length / 2, penalty_area_width / 2,
            'penalty_area', 4, 1.0, 1.0, 1.0
        )

        # Center circle marker
        center_circle = self.create_circle_marker(
            0.0, 0.0, center_circle_radius, 'center_circle', 5, 1.0, 1.0, 1.0
        )

        # Field floor marker
        field_floor = self.create_floor_marker(
            -field_length / 2 - border_strip, -field_width / 2 - border_strip, field_length / 2+border_strip, field_width / 2+border_strip, 'field_floor', 6, 0.0, 0.5, 0.0
        )

        # Publish markers
        self.publisher_.publish(field_marker)
        self.publisher_.publish(left_goal_area)
        self.publisher_.publish(right_goal_area)
        self.publisher_.publish(left_penalty_area)
        self.publisher_.publish(right_penalty_area)
        self.publisher_.publish(center_circle)
        self.publisher_.publish(field_floor)

    def create_rectangle_marker(self, x_min, y_min, x_max, y_max, namespace, marker_id, r, g, b):
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.pose.orientation.w = 1.0

        marker.scale.x = LINE_WIDTH  # Line width

        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = 1.0

        # Define the rectangle corners
        points = [
            Point(x=x_min, y=y_min, z=0.0),
            Point(x=x_min, y=y_max, z=0.0),
            Point(x=x_max, y=y_max, z=0.0),
            Point(x=x_max, y=y_min, z=0.0),
            Point(x=x_min, y=y_min, z=0.0)
        ]
        marker.points = points

        return marker

    def create_circle_marker(self, x, y, radius, namespace, marker_id, r, g, b):
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.pose.orientation.w = 1.0

        marker.scale.x = LINE_WIDTH  # Line width

        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = 1.0

        # Define the circle points
        num_points = 100
        points = []
        for i in range(num_points + 1):
            angle = 2.0 * 3.14159265359 * i / num_points
            points.append(Point(x=x + radius * cos(angle), y=y + radius * sin(angle), z=0.0))

        marker.points = points

        return marker

    def create_floor_marker(self, x_min, y_min, x_max, y_max, namespace, marker_id, r, g, b):
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.pose.position.x = (x_min + x_max) / 2
        marker.pose.position.y = (y_min + y_max) / 2
        marker.pose.position.z = -0.01 # in the floor a bit so the lines are clearly on top
        marker.pose.orientation.w = 1.0

        marker.scale.x = x_max - x_min
        marker.scale.y = y_max - y_min
        marker.scale.z = 0.01  # Very thin to represent the floor

        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = 1.0

        return marker

def main(args=None):
    rclpy.init(args=args)
    node = SoccerFieldPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
