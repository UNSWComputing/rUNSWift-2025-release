import os
import sys
import argparse
import subprocess
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from runswift_interfaces.msg import (
    SeBallsAbsolute,
    SeBallsRelative,
    SeBall,
    CommsGamestate,
    MotionCommand,
    SimulationState,
    SimulationRobotState,
    VisionBalls,
    VisionBallFeature,
)
import math
import threading
from tf2_ros import TransformBroadcaster
from src.BehaviourNode import BehaviourNode
from src1.Robot import Robot
from src1.Ball import Ball


class BehaviourSimulator(Node):
    """
    A ROS2 node that simulates robot behaviours and ball dynamics.
    Subscribes to motion commands and publishes simulation states.
    """

    def __init__(self):
        super().__init__("behaviour_simulator")

        # Simulation entities
        self.robots = {}  # Dictionary of Robot objects (player number as keys)
        self.behaviour_processes = {}  # Dictionary to track process by player number
        self.motion_status_publishers = {}  # Dictionary to track motion status publishers
        self.ball = None  # Ball object
        self.simulation_time = 0.0

        self.simulation_state_publisher = self.create_publisher(SimulationState, "/simulation/state", 10)

        # ROS2 publishers
        self.gamestate_publisher = self.create_publisher(CommsGamestate, "/gamestate", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer for simulation ticks
        self.tick_rate = 30  # Ticks per second
        self.create_timer(1.0 / self.tick_rate, self.tick)

    def add_robot(self, robot):
        """
        Add a robot to the simulation and create a corresponding BehaviourNode with remapped topics.
        :param robot: Robot object to add.
        """
        self.robots[robot.player_number] = robot

        # Dynamically subscribe to the robot's motion_command topic
        self.create_subscription(
            MotionCommand,
            f"/robot{robot.player_number}/motion_command",
            lambda msg, player_number=robot.player_number: self.update_robot_motion(player_number, msg),
            10,
        )

        # Create a publisher for the robot's motion_status
        self.motion_status_publishers[robot.player_number] = self.create_publisher(
            MotionCommand, f"/robot{robot.player_number}/motion_status", 10
        )

        # Start the BehaviourNode for this robot
        self.start_behaviour_node(robot.player_number)

    def remove_robot(self, player_number):
        """
        Remove a robot from the simulator.
        This stops its BehaviourNode and deletes the robot from the internal state.
        """
        if player_number in self.robots:
            # Stop the associated BehaviourNode
            self.stop_behaviour_node(player_number)
            # Remove the robot from the simulator
            del self.robots[player_number]
            print(f"Removed robot {player_number}")
        else:
            print(f"No robot found with player number {player_number}")

    def start_behaviour_node(self, player_number):
        """
        Start a BehaviourNode using its launch file with the specified player number.
        """
        launch_command = [
            "ros2",
            "launch",
            "behaviour_simulator",
            "behaviour_node.launch.xml",
            f"player_number:={player_number}",
        ]
        process = subprocess.Popen(launch_command)
        self.behaviour_processes[player_number] = process
        print(f"Started BehaviourNode for player {player_number}")

    def stop_behaviour_node(self, player_number):
        """
        Stop the BehaviourNode for the specified player number.
        """
        process = self.behaviour_processes.get(player_number)
        if process:
            process.terminate()  # Send SIGTERM to stop the process
            process.wait()  # Wait for the process to exit
            del self.behaviour_processes[player_number]
            print(f"Stopped BehaviourNode for player {player_number}")
        else:
            print(f"No BehaviourNode running for player {player_number}")

    def set_ball(self, ball):
        """
        Set the ball for the simulation.
        :param ball: Ball object to add.
        """
        self.ball = ball

    def update_robot_motion(self, player_number, motion_command):
        """
        Update the motion command for the robot with the given player number.
        :param player_number: The player number of the robot.
        :param motion_command: The MotionCommand message.
        """
        robot = self.robots.get(player_number)
        if robot:
            robot.set_motion_command(motion_command)

    def publish_motion_status(self, robot):
        """
        Publish the current motion status of the robot to the motion_status topic.
        :param player_number: The player number of the robot.
        :param robot: The Robot object whose motion status is being published.
        """
        motion_status = MotionCommand()
        motion_status.body_command.action_type = robot.action_type
        motion_status.body_command.twist.linear.x = robot.velocity[0]
        motion_status.body_command.twist.linear.y = robot.velocity[1]
        motion_status.body_command.twist.angular.z = robot.velocity[2]
        motion_status.head_command.yaw = float(robot.yaw)

        # Publish to the robot's motion_status topic
        self.motion_status_publishers[robot.player_number].publish(motion_status)

    def tick(self):
        """
        Perform a single simulation tick.
        """
        delta_time = 1.0 / self.tick_rate  # Fixed time step for simplicity

        if self.ball:
            self.ball.tick(delta_time)

        for robot in self.robots.values():
            robot.tick(delta_time)
            self.publish_motion_status(robot)
            if self.ball:
                robot.handle_ball_collision(self.ball)

        self.publish_ball_position()
        self.publish_transforms()
        self.publish_gamestate()

        self.simulation_time += delta_time
        self.publish_simulation_state()
        # self.log_state()

    def publish_ball_position(self):
        """Publish the ball's current position to each robot only if it's within its field of view."""
        if not self.ball:
            return

        for robot in self.robots.values():
            # Calculate the ball's position relative to the robot
            dx = self.ball.position[0] - robot.position[0]
            dy = self.ball.position[1] - robot.position[1]
            distance = math.sqrt(dx**2 + dy**2)
            angle_to_ball = math.atan2(dy, dx)

            # Combine heading and yaw to calculate effective vision direction
            # Since yaw is negative to the left, we subtract it from the heading
            effective_heading = robot.heading - robot.yaw

            # Normalize the effective heading to be within [-pi, pi]
            effective_heading = (effective_heading + math.pi) % (2 * math.pi) - math.pi

            # Normalize the angle to the ball to be within [-pi, pi]
            angle_to_ball = (angle_to_ball - effective_heading + math.pi) % (2 * math.pi) - math.pi

            # Check if the ball is within the robot's field of view
            FOV_ANGLE = math.radians(56.3)  # Field of view angle
            if -FOV_ANGLE / 2 <= angle_to_ball <= FOV_ANGLE / 2:  # Ball is within the FOV
                # Publish relative ball position
                rel_msg = SeBallsRelative()
                rel_msg.balls_relative.append(
                    SeBall(pos_x=dx, pos_y=dy, vel_x=self.ball.velocity[0], vel_y=self.ball.velocity[1], confidence=1.0)
                )

                relative_publisher = self.create_publisher(
                    SeBallsRelative, f"/robot{robot.player_number}/ball_base", 10
                )
                relative_publisher.publish(rel_msg)

                # Publish world ball position
                world_msg = SeBallsAbsolute()
                world_msg.balls_absolute.append(
                    SeBall(
                        pos_x=self.ball.position[0],
                        pos_y=self.ball.position[1],
                        vel_x=self.ball.velocity[0],
                        vel_y=self.ball.velocity[1],
                        confidence=1.0,
                    )
                )

                world_publisher = self.create_publisher(SeBallsAbsolute,
                                                        f"/robot{robot.player_number}/stateestimation/SeBallsAbsolute", 10)
                world_publisher.publish(world_msg)

                vballs_msg = VisionBalls()
                vball_feature = VisionBallFeature()
                vball_feature.ball_coordinates.x = dx
                vball_feature.ball_coordinates.y = dy
                vballs_msg.ball_features.append(vball_feature)

                vballs_publisher = self.create_publisher(VisionBalls, f"/robot{robot.player_number}/VBalls", 10)
                vballs_publisher.publish(vballs_msg)

                robot.percieved_ball_position[0] = self.ball.position[0]
                robot.percieved_ball_position[1] = self.ball.position[1]

    def publish_gamestate(self):
        """
        Publish the game state to the /gamestate topic.
        """
        msg = CommsGamestate()

        # TODO: Add more game state data as needed

        self.gamestate_publisher.publish(msg)

    def publish_simulation_state(self):
        """
        Publish the aggregated simulation state to the renderer.
        """
        msg = SimulationState()

        # Add simulation time
        msg.simulation_time = self.simulation_time

        # Add ball state
        if self.ball:
            msg.ball_pos_x = self.ball.position[0]
            msg.ball_pos_y = self.ball.position[1]
            msg.ball_vel_x = self.ball.velocity[0]
            msg.ball_vel_y = self.ball.velocity[1]
        else:
            msg.ball_pos_x = 0.0
            msg.ball_pos_y = 0.0
            msg.ball_vel_x = 0.0
            msg.ball_vel_y = 0.0

        # Add robot states
        for robot in self.robots.values():
            robot_state = SimulationRobotState()
            robot_state.player_number = robot.player_number
            robot_state.action_type = robot.action_type
            robot_state.pos_x = robot.position[0]
            robot_state.pos_y = robot.position[1]
            robot_state.heading = robot.heading
            robot_state.vel_forward = robot.velocity[0]
            robot_state.vel_left = robot.velocity[1]
            robot_state.vel_turn = robot.velocity[2]
            robot_state.yaw = float(robot.yaw)
            robot_state.ball_pos_x = robot.percieved_ball_position[0]
            robot_state.ball_pos_y = robot.percieved_ball_position[1]
            msg.robots.append(robot_state)

        # Publish the message
        self.simulation_state_publisher.publish(msg)

    def publish_transforms(self):
        for robot in self.robots.values():
            robot.publish_transform()

    def log_state(self):
        """
        Log the current state of the simulation for debugging purposes.
        """
        self.get_logger().info(f"Simulation Time: {self.simulation_time:.2f}s")

        if self.ball:
            self.get_logger().info(f"Ball Position: {self.ball.position}, Velocity: {self.ball.velocity}")

        for robot in self.robots.values():
            self.get_logger().info(
                f"Robot {robot.player_number}: Position {robot.position}, "
                f"Heading {robot.heading:.2f}, Velocity {robot.velocity}"
            )


def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(description="Run a simulation with a specified scenario script.")
    parser.add_argument(
        "--scenario",
        type=str,
        required=True,
        help="Name of the scenario script inside the simulation_scripts folder.",
    )
    parsed_args = parser.parse_args()

    # Construct the path to the simulation_scripts directory
    base_dir = os.path.dirname(os.path.abspath(__file__))
    scenarios_dir = os.path.join(base_dir, "../simulation_scripts")
    script_path = os.path.join(scenarios_dir, parsed_args.scenario)

    # Validate scenario script path
    if not os.path.isfile(script_path):
        print(f"Error: Scenario script '{script_path}' does not exist.")
        sys.exit(1)

    simulator = BehaviourSimulator()

    # Load and execute the scenario script
    try:
        with open(script_path, "r") as script_file:
            script_code = script_file.read()
            exec(script_code, {"simulator": simulator, "Robot": Robot, "Ball": Ball})
    except Exception as e:
        print(f"Error while executing the scenario script: {e}")
        sys.exit(1)

    # Run the simulator
    rclpy.spin(simulator)

    rclpy.shutdown()
