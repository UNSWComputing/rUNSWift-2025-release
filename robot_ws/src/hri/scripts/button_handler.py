#!/usr/bin/env python3

import math
import subprocess
import rclpy
from rclpy.node import Node
from nao_lola_sensor_msgs.msg import Buttons
from runswift_interfaces.msg import ButtonEvent

TAP_THRESHOLD = 1.0  # seconds
TAP_WARNING_THRESHOLD = 10  # presses
LONG_PRESS_THRESHOLD = 2.0  # seconds
RELEASE_THRESHOLD = 1.0  # seconds


class ButtonState:
    def __init__(self, name):
        self.name = name
        self.pressed = False
        self.press_start_time = 0
        self.tap_count = 0
        self.last_release_time = 0
        self.is_long_press_reported = False
        self.last_beep_time = 0

    def handle_press(self, clock):
        """Handle button press event"""
        self.pressed = True
        self.press_start_time = clock.now().nanoseconds / 1e9
        self.last_beep_time = self.press_start_time
        self.is_long_press_reported = False

    def handle_release(self, clock):
        """Handle button release event"""
        self.pressed = False
        current_time = clock.now().nanoseconds / 1e9
        press_duration = current_time - self.press_start_time

        if press_duration < TAP_THRESHOLD:
            self.tap_count += 1
            self.last_release_time = current_time
            if self.tap_count == TAP_WARNING_THRESHOLD:
                subprocess.Popen(["flite", "-t", "Pressed 10 times, stop it!"])
        else:
            self.tap_count = 0

    def check_status(self, clock, logger, p):
        """Check button status for long press and tap sequence"""
        current_time = clock.now().nanoseconds / 1e9

        if self.pressed:
            press_duration = current_time - self.press_start_time
            time_since_last_beep = current_time - self.last_beep_time

            if time_since_last_beep >= LONG_PRESS_THRESHOLD:
                subprocess.Popen(["flite", "-t", f"{math.floor(press_duration)}"])
                logger.info(f"Beep at {press_duration:.2f} seconds of holding")
                self.last_beep_time = current_time

                button_event = ButtonEvent()
                button_event.button_name = self.name
                button_event.duration = press_duration
                button_event.press_count = -1
                p.publish(button_event)

        elif not self.pressed and self.tap_count > 0:
            time_since_release = current_time - self.last_release_time
            if time_since_release >= RELEASE_THRESHOLD:
                subprocess.Popen(["flite", "-t", f"{self.tap_count} taps detected"])
                logger.info(f"Button {self.name} was tapped {self.tap_count} times")

                button_event = ButtonEvent()
                button_event.button_name = self.name
                button_event.press_count = self.tap_count
                p.publish(button_event)
                self.tap_count = 0


class ButtonHandler(Node):
    def __init__(self):
        super().__init__("chest_button_handler")

        self.chest_button = ButtonState("chest")

        self.current_state = False

        self.subscription = self.create_subscription(Buttons, "/sensors/buttons", self.callback, 1)
        self.button_event_pub = self.create_publisher(ButtonEvent, "/hri/button_event", 1)

        self.get_logger().info("Chest button handler initialized and listening to 'buttons' topic")

    def callback(self, msg):
        """Process button message"""
        if msg.chest != self.current_state:
            if msg.chest:
                self.chest_button.handle_press(self.get_clock())
            else:
                self.chest_button.handle_release(self.get_clock())

            self.current_state = msg.chest

        # Always check for long press and delayed tap reporting on every message
        self.chest_button.check_status(self.get_clock(), self.get_logger(), self.button_event_pub)


def main(args=None):
    rclpy.init(args=args)

    handler = ButtonHandler()

    try:
        rclpy.spin(handler)
    except KeyboardInterrupt:
        pass
    finally:
        handler.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
