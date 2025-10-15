#!/usr/bin/env python3

import numpy
numpy.float = float
import rclpy
from rclpy.node import Node
from runswift_interfaces.msg import CommsRCGCRRD, BehavioursRobotInfo

class RobotCommPublisher(Node):
    """
    A ROS2 node to subscribe to the topic with the information needed to send to game controller, and publish it to
    game_controller_spl node so it can send it to the game controller.
    """
    behaviours_msg = None
    def __init__(self):
        super().__init__("robot_communication_publisher")

        self.TICK_RATE = 1  # in Hz

        # add subscriptions
        self._behaviours_subscriber = self.create_subscription(BehavioursRobotInfo, "/robot_comms", self._behaviours_callback, 10)

        self.gc_return_data = self.create_publisher(CommsRCGCRRD, "/gc/robot_return_data", 10)

    def publish_to_gc(self):
        if self.behaviours_msg is not None:
            msg = CommsRCGCRRD()
            msg.behaviours_info = self.behaviours_msg  
            self.gc_return_data.publish(msg)
            self.get_logger().debug("Published data to /gc/robot_return_data.")
        else:
            self.get_logger().warn("No behaviours message received yet, cannot publish.")
        
    def _behaviours_callback(self, msg):
        self.behaviours_msg = msg
        self.publish_to_gc()


def main(args=None):
    rclpy.init(args=args)

    game_controller_publisher = RobotCommPublisher()

    rclpy.spin(game_controller_publisher)

    game_controller_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
