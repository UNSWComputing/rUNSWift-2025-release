import math
import time
import random
import rclpy
from rclpy.time import Time
from rclpy.node import Node

from src1.Ball import Ball
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Header


class Robot(Node):
    RADIUS = 200  # Robot base radius (used for collisions)
    MIN_KICK_TIME = 0  # Minimum time taken for the robot to kick (seconds)
    KICK_TIME_VARIATION = 1  # Maximum extra time take for the robot to kick (seconds)

    def __init__(self, player_number, initial_position=(0.0, 0.0), initial_heading=0.0):
        """
        Represents a robot in the simulation.

        :param player_number: Unique number identifying the robot.
        :param initial_position: Tuple (x, y) for initial position on the field.
        :param initial_heading: Initial heading (in radians).
        """

        super().__init__('robot_node')
        self.player_number = player_number
        self.position = list(initial_position)  # [x, y]
        self.heading = initial_heading  # in radians
        self.velocity = [0.0, 0.0, 0.0]  # [forward, left, turn]
        self.power = 0
        self.yaw = 0
        self.action_type = ""
        self.percieved_ball_position = [0.0, 0.0]
        self.pos_publisher = self.create_publisher(PoseStamped, "/stateestimation/self", 2)


    def set_motion_command(self, motion_command):
        """
        Update the robot's motion command.
        :param motion_command: MotionCommand message (e.g., move, rotate, kick).
        """
        if motion_command.body_command.action_type in ["stand", "sit"]:
            self.velocity[0] = 0.0
            self.velocity[1] = 0.0
            self.velocity[2] = 0.0
            self.action_type = motion_command.body_command.action_type

        if motion_command.body_command.action_type == "walk":
            self.velocity[0] = motion_command.body_command.twist.linear.x
            self.velocity[1] = motion_command.body_command.twist.linear.y
            self.velocity[2] = motion_command.body_command.twist.angular.z
            self.action_type = "walk"

        if motion_command.body_command.action_type == "kick":
            self.velocity[0] = 0.0
            self.velocity[1] = 0.0
            self.velocity[2] = 0.0
            self.power = motion_command.body_command.power
            self.action_type = "kick"

        self.yaw = motion_command.head_command.yaw

        self.action_type = motion_command.body_command.action_type

    def tick(self, delta_time):
        """
        Update the robot's position and heading based on its velocity and motion commands.
        :param delta_time: Time elapsed since the last tick (seconds).
        """
        self.heading += self.velocity[2] * delta_time

        # Normalize heading to stay within [-pi, pi]
        self.heading = (self.heading + math.pi) % (2 * math.pi) - math.pi

        forward_global = self.velocity[0] * math.cos(self.heading)
        left_global = self.velocity[1] * math.sin(self.heading)

        self.position[0] += (forward_global - left_global) * delta_time
        self.position[1] += (
            self.velocity[0] * math.sin(self.heading) + self.velocity[1] * math.cos(self.heading)
        ) * delta_time

    def handle_ball_collision(self, ball):
        """
        Check and handle collision with the ball.

        :param ball: The ball object to check for collision.
        """
        # Calculate the distance between the robot and the ball
        dx = ball.position[0] - self.position[0]
        dy = ball.position[1] - self.position[1]
        distance = math.sqrt(dx**2 + dy**2)

        # Check if the distance is less than the sum of the radii (collision threshold)
        if distance < Robot.RADIUS + Ball.RADIUS:
            # Compute collision angle
            collision_angle = math.atan2(dy, dx)
            collision_angle = (collision_angle + math.pi) % (2 * math.pi) - math.pi

            if self.action_type == "kick":
                time.sleep(Robot.MIN_KICK_TIME + (Robot.KICK_TIME_VARIATION * random.random()))
                self.kick(ball, collision_angle)
                self.action_type = "stand"
                self.power = 0
            else:
                # Normal collision handling (apply force to the ball)
                relative_velocity_x = self.velocity[0] * math.cos(self.heading) - ball.velocity[0]
                relative_velocity_y = self.velocity[0] * math.sin(self.heading) - ball.velocity[1]
                relative_velocity_magnitude = math.sqrt(relative_velocity_x**2 + relative_velocity_y**2)

                # Apply a force proportional to the relative velocity
                ball.kick(relative_velocity_magnitude * 1.5, collision_angle)

    def kick(self, ball, collision_angle):
        """
        Kick the ball in the direction the robot is facing with a side component if offset.
        :param ball: The ball object to kick.
        :param collision_angle: The angle of collision (in radians).
        """
        # Forward velocity component (aligned with the robot's heading)
        forward_velocity = self.power * math.cos(self.heading)

        # Side velocity component (perpendicular to the robot's heading)
        side_offset = math.sin(collision_angle - self.heading)
        side_velocity = self.power * side_offset

        # Compute resulting ball velocity
        ball_velocity_x = forward_velocity + side_velocity * math.cos(self.heading + math.pi / 2)
        ball_velocity_y = side_velocity * math.sin(self.heading + math.pi / 2)

        # Apply the kick to the ball
        ball.kick((ball_velocity_x + ball_velocity_y)*3000, collision_angle)

    def publish_transform(self):
        """
        Publish the robot's transform to the TF tree.
        """
        msg = PoseStamped()
        header = Header()
        header.stamp = Time().to_msg()
        header.frame_id = "/world"


        pose = Pose()
        pose.position.x = self.position[0] / 1000
        pose.position.y = self.position[1] / 1000
        pose.position.z = 0.0

        quaternion = self.euler_to_quaternion(0.0, 0.0, self.heading)
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]

        msg.pose = pose
        msg.header = header
        self.pos_publisher.publish(msg)



    @staticmethod
    def euler_to_quaternion(roll, pitch, yaw):
        """
        Convert Euler angles to quaternion.
        :param roll: Roll angle (radians)
        :param pitch: Pitch angle (radians)
        :param yaw: Yaw angle (radians)
        :return: Quaternion as (x, y, z, w)
        """
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(
            pitch / 2
        ) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(
            pitch / 2
        ) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(
            pitch / 2
        ) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(
            pitch / 2
        ) * math.sin(yaw / 2)
        return (qx, qy, qz, qw)

    def main(self):
        rclpy.init()
        robot = Robot()
        rclpy.spin(robot)
