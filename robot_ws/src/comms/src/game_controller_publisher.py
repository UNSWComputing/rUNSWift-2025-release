#!/usr/bin/env python3

import numpy
numpy.float = float
import rclpy
from rclpy.node import Node
from runswift_interfaces.msg import (
    SeBallsAbsolute,
    CommsRCGCRD,
    MotionCommand,
    CommsRobotPlayerInfo,
)
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener
from tf_transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import math

class GameControllerPublisher(Node):
    """
    A ROS2 node to subscribe to the topic with the information needed to send to game controller, and publish it to
    game_controller_spl node so it can send it to the game controller.
    """

    def __init__(self):
        super().__init__("game_controller_publisher")

        self.TICK_RATE = 1  # in Hz

        self.ball = [0.0, 0.0]
        self.ball_age = 0.0
        self.fallen = False
        self.player_number = None
        self.team_number = None
        self.pose = [0.0, 0.0, 0.0]

        self.robot_info_sub = self.create_subscription(CommsRobotPlayerInfo, "/robot_info", self.callback_robot_info, 10)
        self.ball_world_sub = self.create_subscription(SeBallsAbsolute, "/ball_world", self.callback_ball_world, 10)
        self.motion_status_sub = self.create_subscription(
            MotionCommand, "/motion_status", self.callback_motion_status, 10
        )

        self.gc_return_data = self.create_publisher(CommsRCGCRD, "/gc/return_data", 10)

        self.tick_timer = self.create_timer(1.0 / self.TICK_RATE, self.publish_to_gc)

        # self._tf2_buffer = Buffer(cache_time=Duration(seconds=0.3))
        # self._tf2_listener = TransformListener(self._tf2_buffer, self)
        self.robot_sub = self.create_subscription(PoseStamped, "/stateestimation/self", self.callback_robot, QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=1
            ))

    def callback_robot(self, msg):
        self.pose = [msg.pose.position.x, msg.pose.position.y, euler_from_quaternion([
            msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w
        ])[2] % (2 * math.pi)]

    def callback_ball_world(self, msg):
        self.ball = msg.balls_absolute.sort(key=lambda msg: msg.confidence, reverse=True)

    def callback_motion_status(self, msg):
        # TODO: Implement with motion properly
        self.fallen = "getup" in msg.body_command.action_type

    def get_pose(self):
        if self.pose is not None:
            return self.pose
        else:
            return [0.0, 0.0, 0.0]
        # Timeout added in case tf needs to catch up and load the buffer becuase the default timeout is very long
        try:
            transform = self._tf2_buffer.lookup_transform(
                "world", "base_footprint", rclpy.time.Time(seconds=0), timeout=rclpy.duration.Duration(seconds=0.0)
            )
            orientation_q = transform.transform.rotation
            _, _, yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
            self.pose = [transform.transform.translation.x, transform.transform.translation.y, yaw % (2 * math.pi)]
            return self.pose
        except Exception as e:
            self.get_logger().error(f"Error looking up transform: {e}")
            return [0.0, 0.0, 0.0]


    def callback_robot_info(self, msg):
        self.player_number = msg.player_number
        self.team_number = msg.team_number

    def publish_to_gc(self):
        if self.player_number is None or self.team_number is None:
            return

        msg = CommsRCGCRD()
        msg.player_num = self.player_number
        msg.team_num = self.team_number
        msg.fallen = self.fallen
        msg.pose = self.get_pose()
        msg.ball_age = self.ball_age
        msg.ball = self.ball

        self.gc_return_data.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    game_controller_publisher = GameControllerPublisher()

    rclpy.spin(game_controller_publisher)

    game_controller_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
