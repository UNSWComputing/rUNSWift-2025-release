#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from runswift_interfaces.msg import CommsWhistle
from whistle_detector_functions import SoundReceiverModule
from runswift_interfaces.msg import CommsRCGCD


class WhistleDetector(Node):
    def __init__(self):
        super().__init__("whistle_detector")
        self.publisher = self.create_publisher(CommsWhistle, "/whistle_info", 10)
        self.sub = self.create_subscription(CommsRCGCD, "/gc/data", self.gc_data_callback, 10)
        self.latest_game_state = None
        self.previous_timestamp = None
        self.detector = SoundReceiverModule(self)
        self.get_logger().info("Initialised Whistle Detector Node")
        while True:
            while self.latest_game_state is None or self.latest_game_state not in (
                CommsRCGCD.STATE_SET,
                CommsRCGCD.STATE_PLAYING,
            ):
                rclpy.spin_once(self, timeout_sec=0.1)
            self.detector.listen_forever()

    def gc_data_callback(self, msg):
        self.latest_game_state = msg.state
        self.get_logger().debug(f"Received game state: {msg.state}")


def main(args=None):

    rclpy.init(args=args)
    node = WhistleDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
