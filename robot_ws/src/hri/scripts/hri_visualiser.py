#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nao_lola_command_msgs.msg import ChestLed
from std_msgs.msg import ColorRGBA, String
import cv2
import numpy as np


class LedDisplaySim(Node):
    def __init__(self):
        super().__init__('led_display_sim')

        self.chest_sub = self.create_subscription(
            ChestLed, 
            '/effectors/chest_led',
            self.chest_button_display_cb, 
            10
        )

    def chest_button_display_cb(self, msg):
        color = msg.color
        r, g, b = int(color.r * 255), int(color.g * 255), int(color.b * 255)
        
        img = np.zeros((300, 300, 3), dtype=np.uint8)
        
        center = (150, 150)
        radius = 100
        thickness = 20  # Thickness of the circle outline
        cv2.circle(img, center, radius, (b, g, r), thickness)
        
        cv2.imshow('Chest LED Simulator', img)
        cv2.waitKey(1)  # Refresh the window
        
        self.get_logger().info(f'Displaying LED color: R={r}, G={g}, B={b}')
def main():
    rclpy.init()
    node = LedDisplaySim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()