#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nao_lola_command_msgs.msg import RightEyeLeds
from std_msgs.msg import ColorRGBA
import math

class EyeAnimationControl(Node):
    def __init__(self):
        super().__init__('eye_animation_control')
        self.get_logger().info('eye animation control Node started')
        
        # Publisher for controlling right eye LEDs
        self.right_eye_publisher = self.create_publisher(
            RightEyeLeds, 
            '/effectors/right_eye_leds', 
            10
        )
        
        self.num_leds = RightEyeLeds.NUM_LEDS
        self.animation_step = 0
        
        # Create timer for animation updates (50ms for smooth animation)
        self.timer = self.create_timer(0.03, self.update_pattern)
        # Initialize LED colors
        self.led_colors = [self._create_color(0.0, 0.0, 0.0) for _ in range(self.num_leds)]

    def _create_color(self, r, g, b, a=1.0):
        """Helper function to create a ColorRGBA object."""
        color = ColorRGBA()
        color.r = float(max(0.0, min(1.0, r)))  # Clamp between 0 and 1
        color.g = float(max(0.0, min(1.0, g)))
        color.b = float(max(0.0, min(1.0, b)))
        color.a = a
        return color

    def get_color_for_position(self, position):
        """Generate smooth color transitions based on position."""
        # Use sine waves with different phases for each color channel
        # This creates a smooth, continuous color cycle
        r = math.sin(position * 0.1) * 0.5 + 0.5
        g = math.sin(position * 0.1 + 2 * math.pi / 3) * 0.5 + 0.5
        b = math.sin(position * 0.1 + 4 * math.pi / 3) * 0.5 + 0.5
        return r, g, b

    def update_pattern(self):
        # Update each LED
        for i in range(self.num_leds):
            # Calculate position in the color cycle for this LED
            position = self.animation_step + i * 4
            r, g, b = self.get_color_for_position(position)
            
            # Update LED color
            self.led_colors[i] = self._create_color(r, g, b)
        
        # Move animation forward
        self.animation_step += 2
        
        # Create and publish the message
        msg = RightEyeLeds()
        msg.colors = self.led_colors
        self.right_eye_publisher.publish(msg)

def main():
    rclpy.init()
    node = EyeAnimationControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()