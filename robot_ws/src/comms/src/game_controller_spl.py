#!/usr/bin/env python3


"""
This script implements a ROS2 node (`GCSPL`) that facilitates communication with the SPL GameController
using UDP. The node listens for data from the GameController, converts it to ROS messages, and publishes
them to a topic. It also listens for return data from ROS topics and sends it back to the GameController.

This implementation was referenced from another source and adapted for this specific use case. Original source:
https://gamecontroller-spl.readthedocs.io/en/latest/index.html
"""

import socket
from threading import Thread

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from runswift_interfaces.msg import CommsRobotPlayerInfo, CommsRemainingPackets

PACKET_LIMIT = 1200
DEFAULT_TEAM_NUMBER = 18
BASE_PORT = 10000
BROADCAST_IP = '10.0.255.255'
GAMECONTROLLER_RETURN_PORT = 3939 
GAMECONTROLLER_DATA_PORT = 3838

class GCSPL(Node):
    """Node that runs on the robot to communicate with SPL GameController."""

    _loop_thread = None
    _std_client = None
    _robot_client = None
    _publisher = None
    _host = None
    _robot_host = None
    _RCGCD = None
    team_number = DEFAULT_TEAM_NUMBER
    remaining_packets = PACKET_LIMIT

    def __init__(self, node_name="game_controller_spl", **kwargs):
        super().__init__(node_name, **kwargs)

        # Declare parameters
        self.declare_parameter("return_port", GAMECONTROLLER_RETURN_PORT)

        # Read and log parameters
        return_port = self.get_parameter("return_port").value
        self.get_logger().debug("return_port: %s" % return_port)

        self._setup_methods()

        # Setup publisher
        self._publisher = self.create_publisher(self.RCGCD, "gc/data", 10)
        self._packet_publisher = self.create_publisher(CommsRemainingPackets, "gc/remaining_packets", 10)
        self._team_publisher = self.create_publisher(self.RCGCTD, "gc/team_data", 10)

        # Setup subscriber
        self._return_data_subscriber = self.create_subscription(self.RCGCRD, "gc/return_data", self._rcgcrd_callback, 10)
        self._robot_return_data_subscriber = self.create_subscription(self.RCGCRRD, "/gc/robot_return_data", self._rcgcrrd_callback, 10)
        self._robot_info_subscriber = self.create_subscription(CommsRobotPlayerInfo, "/robot_info", self._robot_info_callback, 10)

        # UDP Client - adapted from https://github.com/ninedraft/python-udp/blob/master/client.py
        self._std_client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)  # UDP
        self._std_client.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._std_client.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self._std_client.bind(("", GAMECONTROLLER_DATA_PORT))  # Bind to port 3838 for receiving standard messages
        self._std_client.settimeout(0.1)

        # Create a second socket to send and receive on port 10018 (default) with IP 0.0.0.0
        self._robot_client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)  # UDP
        self._robot_client.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self._robot_client.bind(("", BASE_PORT + self.team_number))  # Bind to port 10018 for sending and receiving
        self._robot_client.settimeout(0.1)

        # Start thread to continuously poll
        self._loop_thread = Thread(target=self._loop)
        self._loop_thread.start()

    def _setup_methods(self):
        # RCGCD
        from runswift_interfaces.msg import CommsRCGCD as RCGCD
        from rcgcd.robocup_game_control_data import GAMECONTROLLER_DATA_PORT
        from rcgcd.conversion import rcgcd_data_to_msg

        self.RCGCD = RCGCD
        self.GAMECONTROLLER_DATA_PORT = GAMECONTROLLER_DATA_PORT
        self.rcgcd_data_to_msg = rcgcd_data_to_msg
        
        # RCGCTD
        from runswift_interfaces.msg import CommsRCGCRRD as RCGCRRD
        from rcgctd.conversion import rcgctd_data_to_msg

        self.RCGCTD = RCGCRRD
        self.rcgctd_data_to_msg = rcgctd_data_to_msg

        # RCGCRD
        from runswift_interfaces.msg import CommsRCGCRD as RCGCRD
        from rcgcrd.conversion import rcgcrd_msg_to_data
        from rcgcrd.robocup_game_control_return_data import GAMECONTROLLER_RETURN_PORT

        self.RCGCRD = RCGCRD
        self.rcgcrd_msg_to_data = rcgcrd_msg_to_data
        self.GAMECONTROLLER_RETURN_PORT = GAMECONTROLLER_RETURN_PORT
        
        from runswift_interfaces.msg import CommsRCGCRRD as RCGCRRD
        from rcgcrrd.conversion import rcgcrrd_msg_to_data
        
        self.RCGCRRD = RCGCRRD
        self.rcgcrrd_msg_to_data = rcgcrrd_msg_to_data

    def _loop(self):
        while rclpy.ok():
            try:
                data, (self._host, _) = self._std_client.recvfrom(1024)
                self.get_logger().debug('From standard received: "%s"' % data)

                # Convert data to ROS msg
                msg = self.rcgcd_data_to_msg(data)

                # Publish it
                if (msg.teams[0].team_number == self.team_number):
                    self.remaining_packets = msg.teams[0].message_budget
                else:
                    self.remaining_packets = msg.teams[1].message_budget
                self._packet_publisher.publish(CommsRemainingPackets(remaining_packets=self.remaining_packets))
                
                self._publisher.publish(msg)
                
                robot_data, (self._robot_host, _) = self._robot_client.recvfrom(1024)
                self.get_logger().debug('From robot received: "%s"' % robot_data)

                # Convert data to ROS msg
                robot_msg = self.rcgctd_data_to_msg(robot_data)

                # Publish it
                self._team_publisher.publish(robot_msg)
            except TimeoutError:
                pass
            
    def _robot_info_callback(self, msg):
        self.team_number = msg.team_number

    def _rcgcrd_callback(self, msg):

        if self._host is None:
            self.get_logger().debug(
                "Not returning RoboCupGameControlReturnData, as GameController" " host address is not known yet."
            )
            return

        data = self.rcgcrd_msg_to_data(msg)
        # Return data directly to the GameController's address and return port
        self._std_client.sendto(data, (self._host, self.GAMECONTROLLER_RETURN_PORT))
        
    def _rcgcrrd_callback(self, msg):
        if (self.remaining_packets > 0):
            data = self.rcgcrrd_msg_to_data(msg)
            # Return data directly to the GameController's address and return port
            self._robot_client.sendto(data, (BROADCAST_IP, BASE_PORT + self.team_number))
        else: 
            self.get_logger().debug(
                "Packet Limit reached!"
            )


def main(args=None):
    rclpy.init(args=args)
    game_controller_spl = GCSPL()
    rclpy.spin(game_controller_spl)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

