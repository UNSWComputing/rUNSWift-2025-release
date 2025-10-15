#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nao_lola_command_msgs.msg import HeadLeds
from std_msgs.msg import ColorRGBA

import time

class HeadLedsLedsControl(Node):
    def __init__(self):
        super().__init__('head_leds_control')
        self.get_logger().info('Head Leds Control Node started')

        # Publisher for controlling right eye LEDs
        self.head_publisher = self.create_publisher(HeadLeds, '/effectors/head_leds', 10)


        # Timer for cooling down logic
        self.timer = self.create_timer(0.2, self.cooling_down)

        self.wave1 = 0


    def cooling_down(self):
        # Gradually turn off LEDs if no new messages are received   
        self.update_leds()

    def update_leds(self):
        out = HeadLeds()
        # Update the colors of the predefined LEDs
        for i in range(HeadLeds.NUM_LEDS):
            if i+1 == (self.wave1) % HeadLeds.NUM_LEDS \
                or i-1 == self.wave1 % HeadLeds.NUM_LEDS \
                or i+1 == (self.wave1 + int(HeadLeds.NUM_LEDS)/2) % HeadLeds.NUM_LEDS \
                or i-1 == (self.wave1 + int(HeadLeds.NUM_LEDS)/2) % HeadLeds.NUM_LEDS:
                intensity = 0.2
            elif i== (self.wave1) % HeadLeds.NUM_LEDS \
                or i== (self.wave1 + int(HeadLeds.NUM_LEDS)/2) % HeadLeds.NUM_LEDS:
                intensity = 1.0
            else:
                intensity = 0.0
            out.intensities[i] = intensity
        self.wave1 += 1
        print(out)
        self.head_publisher.publish(out)
        


def main():
    rclpy.init()
    node = HeadLedsLedsControl()  # Initialize your custom node
    rclpy.spin(node)
    node.get_logger().info('Shutting down Head Leds Control Node')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()