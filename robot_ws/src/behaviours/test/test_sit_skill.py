import rclpy
from rclpy.node import Node
from runswift_interfaces.msg import MotionCommand
from src.skills.leaf_skills.Sit import Sit
from src.BehaviourNode import BehaviourNode
from time import sleep

"""
This test is a stub, it only tests use_shuffle...
"""
class TestSitSkill:
    """
    A test suite for the Sit skill to verify it publishes correct commands.
    """

    GREEN = "\033[92m"  # ANSI code for green
    RED = "\033[91m"  # ANSI code for red
    RESET = "\033[0m"  # ANSI code to reset color

    def __init__(self):
        rclpy.init()
        self.node = Node("test_sit_skill")
        self.behaviour_node = BehaviourNode()
        self.skill = Sit(parent=None, ctx=self.behaviour_node)  # Initialize Sit skill with the node
        self.executor = rclpy.executors.SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.received_messages = []

        # Subscriber to verify what the Sit skill publishes
        self.subscription = self.node.create_subscription(MotionCommand, "/motion_command", self.callback, 10)

    def callback(self, msg):
        """Store received messages in a list."""
        self.received_messages.append(msg.body_command.action_type)

    def spin_once(self, timeout=0.1):
        """Spin the executor to process callbacks."""
        self.executor.spin_once(timeout_sec=timeout)

    def test_sit_publish(self):
        """Test that the Sit skill publishes the correct sit command."""
        self.received_messages.clear()

        # Call the Sit skill's reset and tick methods
        self.skill._reset()  # Reset the timer
        for _ in range(30):  # Simulate time before sitting
            self.skill._tick()
            self.spin_once()
            sleep(0.1)

        # Assert that the message was published correctly
        # Adjust the expected_message based on the correct implementation of `Sit`
        expected_message = "sit"
        assert len(self.received_messages) > 0, "No messages received from Sit skill"
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
    tester = TestSitSkill()
    tests = [
        ("Sit skill publishes correct command", tester.test_sit_publish),
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