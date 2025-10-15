#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import math
import time
import os
from nao_lola_command_msgs.msg import JointPositions, JointStiffnesses, JointIndexes
from nao_lola_sensor_msgs.msg import JointPositions as JointPositionsSensor

STIFFNESS = 0.6


class JointMovementTester(Node):
    def __init__(self):
        super().__init__("joint_movement_tester")

        # Declare ROS2 parameters
        self.declare_parameter("move_to_initial", True)

        # Publishers - matching the topics used in motion_adapter
        self.joint_positions_pub = self.create_publisher(JointPositions, "/effectors/joint_positions", 10)
        self.joint_stiffnesses_pub = self.create_publisher(JointStiffnesses, "/effectors/joint_stiffnesses", 10)

        # Subscriber for joint feedback
        self.joint_positions_sub = self.create_subscription(
            JointPositionsSensor, "/sensors/joint_positions", self.joint_positions_callback, 10
        )

        # Joint data storage - initialized to zeros
        self.current_positions = [0.0] * JointIndexes.NUMJOINTS
        self.positions_received = False

        # Configuration parameters
        self.move_to_initial = self.get_parameter("move_to_initial").get_parameter_value().bool_value

        # Error tracking
        self.error_messages = []

        # Joint names in the order they appear in the arrays
        self.joint_names = [
            "HeadYaw",
            "HeadPitch",
            "LShoulderPitch",
            "LShoulderRoll",
            "LElbowYaw",
            "LElbowRoll",
            "LWristYaw",
            "LHipYawPitch",
            "LHipRoll",
            "LHipPitch",
            "LKneePitch",
            "LAnklePitch",
            "LAnkleRoll",
            "RHipRoll",
            "RHipPitch",
            "RKneePitch",
            "RAnklePitch",
            "RAnkleRoll",
            "RShoulderPitch",
            "RShoulderRoll",
            "RElbowYaw",
            "RElbowRoll",
            "RWristYaw",
            "LHand",
            "RHand",
        ]

        # Joint limits - not in aldebaran's order. we test top-down
        self.joint_limits = [
            ("HeadYaw", [-119.5, 119.5]),
            ("HeadPitch", [-38.5, 29.5]),
            ("LShoulderPitch", [-119.5, 119.5]),
            ("RShoulderPitch", [-119.5, 119.5]),
            ("LShoulderRoll", [-18.0, 76]),
            ("RShoulderRoll", [-76.0, 18.0]),
            ("LElbowYaw", [-119.5, 119.5]),
            ("RElbowYaw", [-119.5, 119.5]),
            ("LElbowRoll", [-88.5, -2]),
            ("RElbowRoll", [2.0, 88.5]),
            ("LWristYaw", [-104.5, 104.5]),
            ("RWristYaw", [-104.5, 104.5]),
            # don't test these as they may be taped
            # ("LHand", [0.0, 57]),
            # ("RHand", [0.0, 57]),
            ("LHipYawPitch", [-65.62, 42.44]),
            ("LHipPitch", [-88.0, 27.73]),
            ("RHipPitch", [-88.0, 27.73]),
            ("LHipRoll", [-21.74, 45.29]),
            ("RHipRoll", [-45.29, 21.74]),
            ("LKneePitch", [-5.29, 121.04]),
            ("RKneePitch", [-5.90, 121.47]),
            ("LAnklePitch", [-68.15, 52.86]),
            ("RAnklePitch", [-67.97, 53.40]),
            ("LAnkleRoll", [-22.79, 44.06]),
            ("RAnkleRoll", [-44.06, 22.80]),
        ]

    def joint_positions_callback(self, msg):
        """Update current joint positions from sensor feedback"""
        # Sensor message has fixed NUMJOINTS-element array, not sparse updates
        # Copy all positions from the sensor message
        if len(msg.positions) == JointIndexes.NUMJOINTS:
            self.current_positions = list(msg.positions)
        else:
            self.get_logger().warn(
                f"Unexpected sensor message length: {len(msg.positions)}, expected {JointIndexes.NUMJOINTS}"
            )
        self.positions_received = True

    def wait_for_joint_data(self, timeout=5.0):
        """Wait for joint position data to be received"""
        start_time = time.time()
        while not self.positions_received and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.positions_received

    def set_joint_stiffness(self, joint_name, stiffness):
        """Set stiffness for a specific joint"""
        joint_index = self.joint_names.index(joint_name)
        msg = JointStiffnesses()
        msg.indexes = [joint_index]
        msg.stiffnesses = [stiffness]
        self.joint_stiffnesses_pub.publish(msg)

    def set_joint_position(self, joint_name, position):
        """Set position for a specific joint"""
        joint_index = self.joint_names.index(joint_name)
        msg = JointPositions()
        msg.indexes = [joint_index]
        msg.positions = [position]
        self.joint_positions_pub.publish(msg)

    def get_joint_position(self, joint_name):
        """Get current position of a joint"""
        joint_index = self.joint_names.index(joint_name)
        return self.current_positions[joint_index]

    def move_and_check_joint(self, joint_name, target_degrees, duration=1.0):
        """Move joint to target position and check if it reached there"""
        target = math.radians(target_degrees)
        start_time = time.time()
        end_time = start_time + duration
        initial_position = self.get_joint_position(joint_name)

        while time.time() < end_time:
            current_time = time.time()
            progress = (current_time - start_time) / duration
            position = initial_position + progress * (target - initial_position)

            print(f"{joint_name}: {position}")
            self.set_joint_stiffness(joint_name, STIFFNESS)
            self.set_joint_position(joint_name, position)

            # Spin to handle incoming messages
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.01)  # Small delay to avoid overwhelming the system

        # Check final position
        time.sleep(0.1)  # Allow time for the robot to reach position
        rclpy.spin_once(self, timeout_sec=0.1)  # Get latest joint positions

        actual_position = self.get_joint_position(joint_name)
        diff = abs(target - actual_position)

        if diff > math.radians(5):
            error_msg = (
                f"!!!!Joint problem: {joint_name} target {target_degrees}° "
                f"actual {math.degrees(actual_position):.2f}° "
                f"diff {math.degrees(diff):.2f}°"
            )
            self.get_logger().error(error_msg)
            # Store error data in a structured format for table display
            self.error_messages.append(
                {
                    "joint": joint_name,
                    "target": target_degrees,
                    "actual": math.degrees(actual_position),
                    "diff": math.degrees(diff),
                }
            )

    def run_test(self):
        """Main test routine"""
        print("Please make sure the robot is connected and LoLA is running")
        print("Place robot flat on ground, eyes up")

        # Wait for initial joint data
        self.get_logger().info("Waiting for joint position data...")
        if not self.wait_for_joint_data():
            self.get_logger().error("Failed to receive joint position data within timeout")
            return
        self.get_logger().info("Joint position data received, starting test")

        try:
            # First, move all joints to zero if enabled
            if self.move_to_initial:
                self.get_logger().info("Moving all joints to initial (0) positions...")
                for joint, limits in self.joint_limits:
                    self.move_and_check_joint(joint, 0)

            # Then test each joint's range
            for joint, limits in self.joint_limits:
                self.get_logger().info(f"Testing {joint}")
                print(f"checking {joint}")

                # Use espeak for audio feedback (if available)
                try:
                    os.system(f'espeak -a 100 -vf5 -p75 -g20 -m "{joint}" 2>/dev/null')
                except:
                    pass  # Ignore if espeak is not available

                # Special handling for shoulder joints to avoid collisions
                if joint == "LShoulderPitch":
                    self.move_and_check_joint("LShoulderRoll", 10)
                elif joint == "RShoulderPitch":
                    self.move_and_check_joint("RShoulderRoll", -10)

                # Test joint limits
                self.move_and_check_joint(joint, limits[0])
                self.move_and_check_joint(joint, limits[1])
                self.move_and_check_joint(joint, 0)

                # Reset shoulder rolls
                if joint == "LShoulderPitch":
                    self.move_and_check_joint("LShoulderRoll", 0)
                elif joint == "RShoulderPitch":
                    self.move_and_check_joint("RShoulderRoll", 0)

        except KeyboardInterrupt:
            self.get_logger().info("Test interrupted by user")
        except Exception as e:
            self.get_logger().error(f"Test failed with error: {e}")
        finally:
            # Turn off all stiffness
            self.get_logger().info("Disabling all joint stiffness...")
            for joint_name in self.joint_names:
                self.set_joint_stiffness(joint_name, 0.0)
            time.sleep(0.1)  # Give time for the commands to be sent

            # Print all errors at the end
            if self.error_messages:
                self.get_logger().info("Summary of errors encountered during testing:")

                # Create a formatted table
                header = f"{'Joint':<16} {'Target (°)':<10} {'Actual (°)':<10} {'Diff (°)':<10}"
                separator = "-" * len(header)

                self.get_logger().info(separator)
                self.get_logger().info(header)
                self.get_logger().info(separator)

                for error in self.error_messages:
                    row = (
                        f"{error['joint']:<16} {error['target']:<10.1f} {error['actual']:<10.2f} {error['diff']:<10.2f}"
                    )
                    self.get_logger().info(row)

                self.get_logger().info(separator)
                self.get_logger().info(f"Total errors found: {len(self.error_messages)}")
            else:
                self.get_logger().info("No errors encountered during testing")

            self.get_logger().info("Test completed, stiffness disabled")


def main(args=None):
    rclpy.init(args=args)
    tester = JointMovementTester()

    try:
        tester.run_test()
    except KeyboardInterrupt:
        tester.get_logger().info("Interrupted by user")
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
