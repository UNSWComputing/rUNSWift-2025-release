import rclpy
from rclpy.clock import Clock
from rclpy.duration import Duration

class Timer:
    def __init__(self, time_target=0.0):
        """
        Initializes a Timer.

        :param time_target: The target duration in seconds for the timer.
        """
        self.time_target = Duration(seconds=time_target)
        self.clock = Clock()  # Create a Clock instance
        self.start_time = self.clock.now()
        self.running = False

    def restart(self):
        """
        Restarts the timer by resetting the start time.
        """
        self.start_time = self.clock.now()
        self.running = True
        return self

    def elapsed(self):
        """
        Gets the elapsed time in nanoseconds since the timer started.

        :return: Elapsed time in nanoseconds.
        """
        now = self.clock.now()
        return (now - self.start_time).nanoseconds

    def elapsed_seconds(self):
        """
        Gets the elapsed time in seconds since the timer started.

        :return: Elapsed time in seconds.
        """
        return self.elapsed() / 1e9

    def finished(self):
        """
        Checks if the timer has reached the target duration.

        :return: True if the timer has finished, False otherwise.
        """
        elapsed_duration = Duration(nanoseconds=self.elapsed())
        return elapsed_duration >= self.time_target