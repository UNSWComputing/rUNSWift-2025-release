#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import pyttsx3
from std_msgs.msg import String


class SpeechControl(Node):
    def __init__(self):
        super().__init__('speech_control')
        self.get_logger().info('speech Control Node started')
        self.subscriber = self.create_subscription(String, "speech_topic", self.speech_cb, 1)
        self.speech_engine = pyttsx3.init()
        self.speech_engine.setProperty('rate', 130)
        self.speech_engine.setProperty('volume', 0.9)

        # Test the audio at startup
        self.get_logger().info('Testing audio...')
        self.speech_engine.say("Audio test")
        self.speech_engine.runAndWait()

    def speech_cb(self, msg):
        print(msg.data)
        self.speech_engine.say(msg.data)
        self.speech_engine.runAndWait()




def main():
    rclpy.init()
    node = SpeechControl()  # Initialize your custom node
    rclpy.spin(node)
    node.get_logger().info('Shutting down speech Control Node')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
