import rclpy
from rclpy.node import Node
from runswift_interfaces.msg import SeBall, SeBallsAbsolute, SeBallsRelative, CommsGamestate, CommsTeamstate
from src.BehaviourNode import BehaviourNode
from time import sleep


class TestBehaviourNode:
    """
    A test suite for the BehaviourNode class to verify live interactions with its subscribed topics.
    """

    GREEN = "\033[92m"  # ANSI code for green
    RED = "\033[91m"  # ANSI code for red
    RESET = "\033[0m"  # ANSI code to reset color

    def __init__(self):
        rclpy.init()
        self.node = BehaviourNode()
        self.executor = rclpy.executors.SingleThreadedExecutor()
        self.executor.add_node(self.node)

    def spin_once(self, timeout=0.1):
        """Spin the executor to process callbacks."""
        self.executor.spin_once(timeout_sec=timeout)

    def test_se_balls_absolute_subscription(self):
        """Test that SeBallsAbsolute messages update the blackboard."""
        publisher = self.node.create_publisher(SeBallsAbsolute, "/ball_world", 10)

        # Create a mock SeBallsAbsolute message
        msg = SeBallsAbsolute()
        msg.balls_absolute = [SeBall(pos_x=1.0, pos_y=2.0, vel_x=0.5, vel_y=0.0, confidence=0.125)]
        # Publish the message
        publisher.publish(msg)

        # Spin the node to process the message
        for _ in range(5):
            self.spin_once()
            sleep(0.1)

        # Assert that the blackboard was updated
        assert self.node.blackboard.ball_world == msg, "Blackboard not updated with SeBallsAbsolute message"

    def test_se_balls_relative_subscription(self):
        """Test that SeBallsRelative messages update the blackboard."""
        publisher = self.node.create_publisher(SeBallsRelative, "/ball_base", 10)

        # Create a mock SeBallsRelative message
        msg = SeBallsRelative()
        msg.balls_relative = [SeBall(pos_x=0.5, pos_y=1.0, vel_x=0.25, vel_y=0.5, confidence=0.5)]
        # Publish the message
        publisher.publish(msg)

        # Spin the node to process the message
        for _ in range(5):
            self.spin_once()
            sleep(0.1)

        # Assert that the blackboard was updated
        assert self.node.blackboard.ball_base == msg, "Blackboard not updated with SeBallsRelative message"

    def test_comms_gamestate_subscription(self):
        """Test that CommsGamestate messages update the blackboard."""
        publisher = self.node.create_publisher(CommsGamestate, "/gamestate", 10)

        # Create a mock CommsGamestate message
        msg = CommsGamestate()

        # Publish the message
        publisher.publish(msg)

        # Spin the node to process the message
        for _ in range(5):
            self.spin_once()
            sleep(0.1)

        # Assert that the blackboard was updated
        assert self.node.blackboard.gamestate == msg, "Blackboard not updated with CommsGamestate message"

    def test_comms_teamstate_subscription(self):
        """Test that CommsTeamstate messages update the blackboard."""
        publisher = self.node.create_publisher(CommsTeamstate, "/teamstate", 10)

        # Create a mock CommsTeamstate message
        msg = CommsTeamstate()

        # Publish the message
        publisher.publish(msg)

        # Spin the node to process the message
        for _ in range(5):
            self.spin_once()
            sleep(0.1)

        # Assert that the blackboard was updated
        assert self.node.blackboard.teamstate == msg, "Blackboard not updated with CommsTeamstate message"

    def print_result(self, test_name, success):
        """Print the test result with color."""
        if success:
            print(f"{self.GREEN}[SUCCESS] {test_name}{self.RESET}")
        else:
            print(f"{self.RED}[FAIL] {test_name}{self.RESET}")

    def teardown(self):
        """Clean up the node and executor."""
        self.executor.shutdown()
        self.node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    tester = TestBehaviourNode()
    tests = [
        ("SeBallsAbsolute subscription", tester.test_se_balls_absolute_subscription),
        ("SeBallsRelative subscription", tester.test_se_balls_relative_subscription),
        ("CommsGamestate subscription", tester.test_comms_gamestate_subscription),
        ("CommsTeamstate subscription", tester.test_comms_teamstate_subscription),
    ]

    try:
        for test_name, test_func in tests:
            try:
                test_func()
                tester.print_result(test_name, True)
            except AssertionError as e:
                tester.print_result(test_name, False)
                print(f"    {e}")
    finally:
        tester.teardown()