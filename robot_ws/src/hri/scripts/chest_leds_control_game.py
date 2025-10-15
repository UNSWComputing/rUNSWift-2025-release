#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nao_lola_command_msgs.msg import ChestLed
from std_msgs.msg import ColorRGBA, String, Bool
import itertools
from runswift_interfaces.msg import BehavioursGamestate


GAMESTATE_COLOR_MAPPING = {
    "Unstiff": (ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0), "blinking"),  # Blue-Blinking
    "Initial": (ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0,),"solid"),  # Off
    "Standby": (ColorRGBA(r=0.0, g=1.0, b=1.0, a=1.0),"solid"),  # Cyan
    "Ready": (ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0),"solid"),    # Blue
    "Set": (ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0),"solid"),      # Yellow
    "Playing": (ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0),"solid"),  # Green
    "Penalized": (ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0),"solid"), # Red
    "Finished": (ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0),"solid"), # Off
    "Calibration": (ColorRGBA(r=0.5, g=0.0, b=0.5, a=1.0),"solid") # Purple
}

m = BehavioursGamestate()
msg_int_to_gamestate_string = {
    m.STATE_INITIAL: "Initial",
    m.STATE_READY: "Ready",
    m.STATE_SET: "Set",
    m.STATE_PLAYING: "Playing",
    m.STATE_FINISHED: "Finished",
    m.STATE_STANDBY: "Standby",
    m.STATE_PENALIZED: "Penalized",
}
class ChestLedControl(Node):
    def __init__(self):
        super().__init__('chest_led_control')
        self.get_logger().info('Chest LED Control Node started')
        self.is_stiffened = False
        self.current_color = None
        self.current_mode = None
        intensity_values = [0.0, 0.25, 0.5, 0.75, 1.0, 0.75, 0.5, 0.25]
        self.circular_iterator = itertools.cycle(intensity_values)

        # Publisher for controlling chest LED
        self.chest_publisher = self.create_publisher(
            ChestLed,
            '/effectors/chest_led',
            10
        )

        self.behaviours_gamestate_sub = self.create_subscription(
            BehavioursGamestate,
            '/behaviours_gamestate',
            self.behaviours_gamestate_callback,
            10
        )

        self.robot_is_stiffened_sub = self.create_subscription(
            Bool,
            '/robot_stiffness',
            self.robot_stiffness_callback,
            10
        )
        self.timer = self.create_timer(0.05, self.timer_cb)
    def robot_stiffness_callback(self, msg):
        self.is_stiffened = msg.data
    def behaviours_gamestate_callback(self, msg):
        gamestate = msg.state
        if msg_int_to_gamestate_string[gamestate] in GAMESTATE_COLOR_MAPPING:
            self.current_color, self.current_mode = GAMESTATE_COLOR_MAPPING[msg_int_to_gamestate_string[gamestate]]
        else:
            self.current_mode = "unknown" # this will cause it to falsh white so we know we have an unknown gamestate

    def timer_cb(self):
        if self.is_stiffened:
            current_color, current_mode = self.current_color, self.current_mode
        else:
            current_color, current_mode = GAMESTATE_COLOR_MAPPING["Unstiff"]
        msg = ChestLed()
        if current_mode == "solid":
            msg.color = current_color
        elif current_mode == "blinking":
            intensity = next(self.circular_iterator)
            msg.color = ColorRGBA(
                r=current_color.r * intensity,
                g=current_color.g * intensity,
                b=current_color.b * intensity,
                a=1.0
            )
        else:
            # log
            self.get_logger().error(f"Unknown mode: {current_mode}")
            intensity = next(self.circular_iterator)
            # unknow mode white flashing
            msg.color = ColorRGBA(
                r=1.0*intensity,
                g=1.0*intensity,
                b=1.0*intensity,
                a=1.0*intensity
            )
        self.chest_publisher.publish(msg)

def main():
    rclpy.init()
    node = ChestLedControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
