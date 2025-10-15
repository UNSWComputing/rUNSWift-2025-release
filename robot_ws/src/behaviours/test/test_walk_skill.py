import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Assumes the action command uses a String message
from src.skills.leaf_skills.Walk import Walk
from runswift_interfaces.msg import MotionCommand
from src.BehaviourNode import BehaviourNode
from time import sleep

"""
This test is a stub, it only tests use_shuffle...
"""
class TestWalkSkill:
    """
    A test suite for the Walk skill to verify it publishes correct commands.
    """

    GREEN = "\033[92m"  # ANSI code for green
    RED = "\033[91m"  # ANSI code for red
    RESET = "\033[0m"  # ANSI code to reset color

    def __init__(self):
        rclpy.init()
        self.node = Node("test_walk_skill")
        self.behaviour_node = BehaviourNode()
        self.skill = Walk(parent=None, ctx=self.behaviour_node)  # Initialize Walk skill with the node
        self.executor = rclpy.executors.SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.received_messages = []

        # Subscriber to verify what the Walk skill publishes
        self.subscription = self.node.create_subscription(MotionCommand, "/motion_command", self.callback, 10)

    def callback(self, msg):
        """Store received messages in a list."""
        self.received_messages.append(msg.body_command.action_type)

    def spin_once(self, timeout=0.1):
        """Spin the executor to process callbacks."""
        self.executor.spin_once(timeout_sec=timeout)

    def test_walk_publish(self):
        """Test that the Walk skill publishes the correct walk command."""
        self.received_messages.clear()

        self.skill._reset()
        # Call the walk skill's tick method
        self.skill._tick(forward=1.0, left=0.0, turn=0.5, speed=1.0)

        # Spin the node to process the message
        for _ in range(5):
            self.spin_once()
            sleep(0.1)

        # Assert that the message was published correctly
        expected_message = "walk"
        assert len(self.received_messages) > 0, "No messages received from Walk skill"
        assert (
            self.received_messages[-1] == expected_message
        ), f"Expected '{expected_message}', got '{self.received_messages[-1]}'"

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
    tester = TestWalkSkill()
    tests = [
        ("Walk skill publishes correct command", tester.test_walk_publish),
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