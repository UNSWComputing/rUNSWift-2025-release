#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nao_lola_command_msgs.msg import ChestLed
from std_msgs.msg import ColorRGBA
import math

class ChestLedControl(Node):
    def __init__(self):
        super().__init__('chest_led_control')
        self.get_logger().info('Chest LED Control Node started')
        
        # Publisher for controlling chest LED
        self.chest_publisher = self.create_publisher(
            ChestLed, 
            '/effectors/chest_led', 
            10
        )
        
        self.animation_step = 0
        
        # Create timer for animation updates (50ms for smooth animation)
        self.timer = self.create_timer(0.1, self.update_pattern)

    def _create_color(self, r, g, b, a=1.0):
        """Helper function to create a ColorRGBA object."""
        color = ColorRGBA()
        color.r = float(max(0.0, min(1.0, r)))
        color.g = float(max(0.0, min(1.0, g)))
        color.b = float(max(0.0, min(1.0, b)))
        color.a = a
        return color

    def get_color_for_position(self, position):
        """Generate smooth color transitions."""
        # Faster color transitions for more noticeable changes
        pos = position * 0.2  # Increased from 0.1 for faster changes
        
        # Create a breathing rainbow effect
        r = math.sin(pos) * 0.5 + 0.5
        g = math.sin(pos + math.pi/1.5) * 0.5 + 0.5
        b = math.sin(pos + math.pi) * 0.5 + 0.5
        return r, g, b

    def update_pattern(self):
        # Get the current color
        r, g, b = self.get_color_for_position(self.animation_step)
        
        # Create and publish the message
        msg = ChestLed()
        msg.color = self._create_color(r, g, b)
        self.chest_publisher.publish(msg)
        
        # Move animation forward
        self.animation_step += 2  # Bigger step for faster animation

def main():
    rclpy.init()
    node = ChestLedControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()