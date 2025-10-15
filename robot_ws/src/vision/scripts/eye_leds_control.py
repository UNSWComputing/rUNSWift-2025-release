#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nao_lola_command_msgs.msg import RightEyeLeds
from std_msgs.msg import ColorRGBA
from runswift_interfaces.msg import VisionBalls
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class EyeLedsControl(Node):
    def __init__(self):
        super().__init__('eye_leds_control')
        self.get_logger().info('Eye Leds Control Node started')

        # Publisher for controlling right eye LEDs
        self.right_eye_publisher = self.create_publisher(RightEyeLeds, '/effectors/right_eye_leds', QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            ))

        # Subscriber to the vision topic containing ball detection info
        self.subscriber = self.create_subscription(
            VisionBalls,  
            'vision/VBalls',
            self.vision_cb,
            10
        )

        # Timer for cooling down logic
        self.timer = self.create_timer(1, self.cooling_down)

        # Initialize state variables
        self.receive_time = time.time()
        self.number_of_bars = 5  # Number of detected balls

        # Predefine LED colors
        self.led_colors = [self._create_color(0.0, 0.0, 0.0) for _ in range(RightEyeLeds.NUM_LEDS)]

    def _create_color(self, r, g, b, a=1.0):
        """Helper function to create a ColorRGBA object."""
        color = ColorRGBA()
        color.r = r
        color.g = g
        color.b = b
        color.a = a  # Alpha is ignored but set for consistency
        return color

    def vision_cb(self, msg):
        # Process incoming ball detection messages
        

        # Update the number of active LEDs based on the ball count
        self.number_of_bars = min(self.number_of_bars+1, RightEyeLeds.NUM_LEDS)  # Limit to the maximum number of LEDs
        self.update_leds()

        # Record the time the message was received
        self.receive_time = time.time()

    def cooling_down(self):
        # Gradually turn off LEDs if no new messages are received
        if time.time()- self.receive_time > 0.5:
            self.number_of_bars = 0
            self.update_leds()
            return
        if self.number_of_bars > 0:
            self.number_of_bars -= 1  # Decrease the LED bars gradually
            self.get_logger().info(f"Cooling down: Reducing LEDs to {self.number_of_bars}")
            self.update_leds()

    def update_leds(self):
        # Update the colors of the predefined LEDs
        for i in range(RightEyeLeds.NUM_LEDS):
            if i < self.number_of_bars:
                # Active LEDs: Set to red
                self.led_colors[i].r = 1.0
                self.led_colors[i].g = 0.0
                self.led_colors[i].b = 0.0
            else:
                # Inactive LEDs: Turn off
                self.led_colors[i].r = 1.0
                self.led_colors[i].g = 1.0
                self.led_colors[i].b = 1.0

        # Create and publish the RightEyeLeds message
        msg = RightEyeLeds()
        msg.colors = self.led_colors
        self.right_eye_publisher.publish(msg)

def main():
    rclpy.init()
    node = EyeLedsControl()  # Initialize your custom node
    rclpy.spin(node)
    node.get_logger().info('Shutting down Eye Leds Control Node')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()