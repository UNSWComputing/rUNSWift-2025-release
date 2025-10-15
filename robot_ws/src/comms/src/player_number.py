#!/usr/bin/env python3

import os
import re
import configparser
import rclpy
from rclpy.node import Node
from runswift_interfaces.msg import CommsRobotPlayerInfo, CommsRCGCD

NUM_PLAYERS = 7
DEFAULT_PLAYERS = [1, 2, 3, 4, 5, 6, 7]

class PlayerNumber(Node):
    def __init__(self):
        super().__init__("player_number")
        self.publisher = self.create_publisher(CommsRobotPlayerInfo, "/robot_info", 10)
        self.subscriber = self.create_subscription(CommsRCGCD, "gc/data", self.gc_callback, 10)
        
        self.publish_timer = self.create_timer(1, self.publish)
        self.parse_config_timer = self.create_timer(1, self.get_player_and_team_number)
        self.player_number = 0
        self.team_number = 0
        self.current_players = DEFAULT_PLAYERS
        self.gc_data_players = [0] * NUM_PLAYERS
        self._dynamic_player_number = 0
        self.gc_data = CommsRCGCD()

    def publish(self):
        if self.player_number is None and self.team_number is None:
            return
        msg = CommsRobotPlayerInfo()
        msg.player_number = self.player_number
        msg.team_number = self.team_number
        msg.dynamic_player_number = int(self._dynamic_player_number)
        msg.current_players = self.current_players
        self.publisher.publish(msg)

    def get_player_and_team_number(self):
        config_path = self.get_config_path()
        config = configparser.ConfigParser()
        config.read(config_path)

        namespace = self.get_namespace().strip("/")
        match = re.search(r"robot(\d+)", namespace)
        self.get_dynamic_player_number()
        if match:
            self._player_number = int(match.group(1))
            return

        try:
            self.player_number = config.getint("player", "number")
            self.team_number = config.getint("player", "team")
        except (KeyError, ValueError) as e:
            raise ValueError(f"Error reading player number from config: {e}")

    def get_config_path(self):
        robot_name = self.get_robot_name()
        robot_path = f"/home/nao/data/configs/{robot_name}.cfg"
        dev_path = f"/workspace/image/home/nao/data/configs/{robot_name}.cfg"
        if os.path.exists(robot_path):
            return robot_path
        elif os.path.exists(dev_path):
            return dev_path
        else:
            raise FileNotFoundError(f"Path not found for {robot_name}")

    def get_robot_name(self):
        """Reads the robot's hostname from /etc/hostname.

        Returns:
            str: The robot's hostname.

        NOTE: Defaults to 'robotnotfound' if not on a NAO robot.
        """
        try:
            with open("/etc/hostname", "r") as f:
                robot_name = f.read().strip()
        except FileNotFoundError:
            raise FileNotFoundError("The /etc/hostname file is missing. Unable to determine robot name.")
        except Exception as e:
            raise RuntimeError(f"Error reading /etc/hostname: {e}")

        if not self.is_nao_robot():
            robot_name = "robotnotfound"

        return robot_name

    def is_nao_robot(self):
        return os.path.exists("/etc/naoqi") or os.path.exists("/home/nao")

    def get_dynamic_player_number(self):
        valid_player_numbers = []
        # if no gc just return default players and player number
        if (self.gc_data is None):
            return
            
        # we know our player number but based on how many robots are in the field we may be in a different position
        if (self.gc_data.teams[0].team_number == self.team_number):
            self.gc_data_players = self.gc_data.teams[0].players
        else:
            self.gc_data_players = self.gc_data.teams[1].players
        for i in range(len(self.gc_data_players)):
            if self.gc_data_players[i].penalty == 0:
                valid_player_numbers.append(i + 1)     
        # Pad with zeros if needed
        while len(valid_player_numbers) < NUM_PLAYERS:
            valid_player_numbers.append(0)   
             
        self.current_players = valid_player_numbers[:NUM_PLAYERS] 
        
        for i in range(len(self.current_players)):
            if valid_player_numbers[i] == self.player_number:
                self._dynamic_player_number = i + 1
                return
            
        self._dynamic_player_number = self.player_number
        return 
        
    def gc_callback(self, msg):
        self.gc_data = msg
        
        

def main(args=None):
    rclpy.init(args=args)
    node = PlayerNumber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
