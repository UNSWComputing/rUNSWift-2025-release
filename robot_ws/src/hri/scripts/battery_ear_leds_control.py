#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nao_lola_command_msgs.msg import RightEarLeds
from nao_lola_sensor_msgs.msg import Battery
from std_msgs.msg import ColorRGBA
from math import ceil
import time

class BatteryEarLedsControl(Node):
    def __init__(self):
        super().__init__('battery_ear_leds_control')
        self.get_logger().info('Battery Ear Leds Control Node started')

        # Publisher for controlling right eye LEDs
        self.light_publisher = self.create_publisher(RightEarLeds, '/effectors/right_ear_leds', 10)


        self.battery_sub = self.create_subscription(Battery, '/sensors/battery', self.battery_callback, 1)
        self.battery_sub = None
        self.battery = 100.0
        self.charging = False

        # Not working yet. Idea is to create subscriber when needed and remove afterwards
        #self.timer = self.create_timer(30, self.create_battery_sub)

    # def create_battery_sub(self):
    #     # Subscribe to the battery topic
    #     self.battery_sub = self.create_subscription(Battery, '/sensors/battery', self.battery_callback, 1)
    #     self.get_logger().info('Battery subscriber created')
    
    def battery_callback(self, msg):
        self.battery = msg.charge
        self.charging = msg.charging
        print(f"Received battery message: {self.battery}")
        self.update_leds()
    
    
    def update_leds(self):
        out = RightEarLeds()
        # find numebr of bars based on battery percentage and always round up
        bar = min(ceil(self.battery/10), RightEarLeds.NUM_LEDS)
        
        itensity = bar/10 if not self.charging else 1.0
        
        
        for i in range(bar):
            out.intensities[(i + RightEarLeds.NUM_LEDS//2) % RightEarLeds.NUM_LEDS] = itensity
        
        self.light_publisher.publish(out)
            



    
    

def main():
    rclpy.init()
    node = BatteryEarLedsControl()  # Initialize your custom node
    rclpy.spin(node)
    node.get_logger().info('Shutting down battery display Node')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()