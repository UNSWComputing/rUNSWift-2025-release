import os
import sys
import rclpy
from rclpy.node import Node
from runswift_interfaces.msg import SeBallsAbsolute, SeBallsRelative, CommsGamestate, CommsTeamstate
from src.Blackboard import Blackboard


# Mock Node
class MockNode(Node):
    def __init__(self):
        super().__init__("mock_node")


# ANSI color codes for terminal output
GREEN = "\033[92m"
RED = "\033[91m"
RESET = "\033[0m"


# Test Functions
def test_singleton_behavior():
    rclpy.init()
    node = MockNode()
    instance1 = Blackboard(node)
    instance2 = Blackboard(node)
    assert instance1 is instance2, "Singleton behaviour test failed"
    rclpy.shutdown()
    print(f"{GREEN}[SUCCESS] Singleton behaviour test passed{RESET}")


def test_robot_world():
    rclpy.init()
    node = MockNode()
    blackboard = Blackboard(node)

    # Mock a tf2 buffer
    class MockBuffer:
        def lookup_transform(self, target_frame, source_frame, time, timeout):
            class Transform:
                class TransformData:
                    class Translation:
                        x, y, z = 1.0, 2.0, 3.0

                    class Rotation:
                        x, y, z, w = 0.0, 0.0, 0.0, 1.0

                    translation = Translation()
                    rotation = Rotation()

                transform = TransformData()

            return Transform()

    blackboard._tf2_buffer = MockBuffer()
    robot_pose = blackboard.robot_world

    assert robot_pose.point.x == 1.0, "Unexpected x position"
    assert robot_pose.point.y == 2.0, "Unexpected y position"
    assert robot_pose.point.z == 3.0, "Unexpected z position"
    assert round(robot_pose.heading, 2) == 0.0, "Unexpected yaw value"
    rclpy.shutdown()
    print(f"{GREEN}[SUCCESS] Robot world test passed{RESET}")


def test_ball_world_setter_and_getter():
    rclpy.init()
    node = MockNode()
    blackboard = Blackboard(node)

    # Mock a valid SeBallsAbsolute object
    ball_world_mock = SeBallsAbsolute()

    # Test setter
    blackboard.ball_world = ball_world_mock

    # Test getter
    assert blackboard.ball_world == ball_world_mock, "ball_world getter did not return the expected value"

    # Test invalid assignment
    try:
        blackboard.ball_world = "invalid_value"
    except ValueError:
        print(f"{GREEN}[SUCCESS] ball_world setter correctly raised ValueError{RESET}")
    else:
        assert False, "ball_world setter did not raise ValueError for invalid assignment"

    rclpy.shutdown()
    print(f"{GREEN}[SUCCESS] ball_world setter and getter test passed{RESET}")


def test_ball_base_setter_and_getter():
    rclpy.init()
    node = MockNode()
    blackboard = Blackboard(node)

    # Mock a valid SeBallsRelative object
    ball_base_mock = SeBallsRelative()

    # Test setter
    blackboard.ball_base = ball_base_mock

    # Test getter
    assert blackboard.ball_base == ball_base_mock, "ball_base getter did not return the expected value"

    # Test invalid assignment
    try:
        blackboard.ball_base = "invalid_value"
    except ValueError:
        print(f"{GREEN}[SUCCESS] ball_base setter correctly raised ValueError{RESET}")
    else:
        assert False, "ball_base setter did not raise ValueError for invalid assignment"

    rclpy.shutdown()
    print(f"{GREEN}[SUCCESS] ball_base setter and getter test passed{RESET}")


def test_gamestate_setter_and_getter():
    rclpy.init()
    node = MockNode()
    blackboard = Blackboard(node)

    # Mock a valid CommsGamestate object
    gamestate_mock = CommsGamestate()

    # Test setter
    blackboard.gamestate = gamestate_mock

    # Test getter
    assert blackboard.gamestate == gamestate_mock, "gamestate getter did not return the expected value"

    # Test invalid assignment
    try:
        blackboard.gamestate = "invalid_value"
    except ValueError:
        print(f"{GREEN}[SUCCESS] gamestate setter correctly raised ValueError{RESET}")
    else:
        assert False, "gamestate setter did not raise ValueError for invalid assignment"

    rclpy.shutdown()
    print(f"{GREEN}[SUCCESS] gamestate setter and getter test passed{RESET}")


def test_teamstate_setter_and_getter():
    rclpy.init()
    node = MockNode()
    blackboard = Blackboard(node)

    # Mock a valid CommsTeamstate object
    teamstate_mock = CommsTeamstate()

    # Test setter
    blackboard.teamstate = teamstate_mock

    # Test getter
    assert blackboard.teamstate == teamstate_mock, "teamstate getter did not return the expected value"

    # Test invalid assignment
    try:
        blackboard.teamstate = "invalid_value"
    except ValueError:
        print(f"{GREEN}[SUCCESS] teamstate setter correctly raised ValueError{RESET}")
    else:
        assert False, "teamstate setter did not raise ValueError for invalid assignment"

    rclpy.shutdown()
    print(f"{GREEN}[SUCCESS] teamstate setter and getter test passed{RESET}")


# Main Runner
if __name__ == "__main__":
    try:
        test_singleton_behavior()
        test_robot_world()
        test_ball_world_setter_and_getter()
        test_ball_base_setter_and_getter()
        test_gamestate_setter_and_getter()
        test_teamstate_setter_and_getter()
    except AssertionError as e:
        print(f"{RED}[ERROR] {e}{RESET}")