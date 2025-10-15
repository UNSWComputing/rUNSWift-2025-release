import rclpy
from src.utils.Timer import Timer  # Adjust the import based on your file structure

class TestTimer:
    """
    A test suite for the Timer class with color-coded results.
    """

    GREEN = "\033[92m"  # ANSI code for green
    RED = "\033[91m"    # ANSI code for red
    RESET = "\033[0m"   # ANSI code to reset color

    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("test_timer_node")

    def print_result(self, test_name, success):
        """
        Print the result of a test with color.
        """
        if success:
            print(f"{self.GREEN}[SUCCESS] {test_name}{self.RESET}")
        else:
            print(f"{self.RED}[FAIL] {test_name}{self.RESET}")

    def test_elapsed_time(self):
        """
        Test that the timer measures elapsed time correctly.
        """
        try:
            timer = Timer(time_target=2.0)
            timer.restart()
            self.print_result("Timer restarted", True)

            rclpy.spin_once(self.node, timeout_sec=1.0)  # Wait for 1 second
            elapsed = timer.elapsed_seconds()
            if 0.9 <= elapsed <= 1.1:
                self.print_result("Elapsed time after 1 second", True)
            else:
                raise AssertionError(f"Expected ~1.0s, but got {elapsed:.2f}s")

            rclpy.spin_once(self.node, timeout_sec=2.0)  # Wait for 2 more seconds
            elapsed = timer.elapsed_seconds()
            if 2.9 <= elapsed <= 3.1:
                self.print_result("Elapsed time after 3 seconds", True)
            else:
                raise AssertionError(f"Expected ~3.0s, but got {elapsed:.2f}s")
        except Exception as e:
            self.print_result("Elapsed time test", False)
            print(f"    {e}")

    def test_timer_finished(self):
        """
        Test that the timer finishes correctly after the target duration.
        """
        try:
            timer = Timer(time_target=2.0)
            timer.restart()
            self.print_result("Timer restarted", True)

            rclpy.spin_once(self.node, timeout_sec=1.0)  # Wait for 1 second
            if not timer.finished():
                self.print_result("Timer not finished after 1 second", True)
            else:
                raise AssertionError("Timer finished prematurely at 1 second")

            rclpy.spin_once(self.node, timeout_sec=2.0)  # Wait for 2 more seconds
            if timer.finished():
                self.print_result("Timer finished after 3 seconds", True)
            else:
                raise AssertionError("Timer did not finish after 3 seconds")
        except Exception as e:
            self.print_result("Timer finished test", False)
            print(f"    {e}")

    def teardown(self):
        """
        Clean up the ROS2 node.
        """
        self.node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    tester = TestTimer()
    tests = [
        ("Test Elapsed Time", tester.test_elapsed_time),
        ("Test Timer Finished", tester.test_timer_finished),
    ]

    try:
        for test_name, test_func in tests:
            print(f"Running {test_name}...")
            test_func()
    finally:
        tester.teardown()