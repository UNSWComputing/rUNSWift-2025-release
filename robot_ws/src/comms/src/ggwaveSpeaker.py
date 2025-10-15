#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
import pyaudio
import ggwave
from runswift_interfaces.msg import BehavioursRobotInfo

class AudioDataSpeaker(Node):
    def __init__(self):
        super().__init__('AudioDataSpeaker')
        self.pyaudio = pyaudio.PyAudio()
        self.subscriber = self.create_subscription(BehavioursRobotInfo, '/comms_audio_message_input', self.callback, 10)

    def callback(self, msg: BehavioursRobotInfo):

        try:
            robot_x = msg.robot_pos_x
            robot_y = msg.robot_pos_y
            robot_angle = msg.heading
            num = msg.player_number
            ball_x = msg.ball_pos_x
            ball_y = msg.ball_pos_y

            if (robot_x >= abs(4500+300) or robot_y >= abs(3000+300) 
                or ball_x >= abs(4500+300) or ball_y >= abs(3000+300)
                or num < 0 or num > 7 or robot_angle < -3.14 or robot_angle > 3.14):
                self.get_logger().warn("Values out of range 1, not transmitting")
                return
            #make sure all values are non-negative
            robot_angle = round(robot_angle + 3.14, 2)
            robot_x += 4800
            robot_y += 3300
            ball_x += 4800
            ball_y += 3300

            if (robot_x >= (4500*2+300) or robot_x < 0 
                or robot_y >= (3000*2+300) or robot_y < 0 
                or ball_x >= (4500*2+300) or ball_x < 0
                or ball_y >= (3000*2+300) or ball_y < 0
                or num < 0 or num > 7 or robot_angle < 0 or robot_angle > (3.14*2)):
                self.get_logger().warn("Values out of range 2, not transmitting")
                return
        except Exception as e:
            self.get_logger().warn(f"Error processing message: {e}")
            return

        # Convert the message to a tuple of 6 numbers
        tup = (robot_x, robot_y, robot_angle, num, ball_x, ball_y)

        self.transmit(tup)
        

    def encodeMessage(self, numbers: tuple) -> str:
        if (len(numbers) != 6):
            print("Invalid Numbers len not 6")
            return
        robot_x = numbers[0]
        robot_y = numbers[1]
        robot_angle = numbers[2]
        num = numbers[3]
        ball_x = numbers[4]
        ball_y = numbers[5]

        x1 = int(round(robot_x / 25))   # 9 bits
        y1 = int(round(robot_y / 25))   # 8 bits 

        # Encode heading as 3+7 bits: int + 2 decimal digits (0.01 steps)
        heading = robot_angle
        heading_int = int(heading)
        heading_frac = int(round((heading - heading_int) * 100))
        if heading_frac > 99:
            heading_frac = 99
        if heading_int > 7:
            heading_int = 7
            heading_frac = 99
        heading_bits = (heading_int << 7) | heading_frac  # total 10 bits


        x2 = int(round(ball_x / 25))    # 9 bits
        y2 = int(round(ball_y / 25))    # 8 bits

        bits = 0
        bits = (bits << 9) | x1
        bits = (bits << 8) | y1
        bits = (bits << 10) | heading_bits
        bits = (bits << 3) | num
        bits = (bits << 9) | x2
        bits = (bits << 8) | y2
        bits = (bits << 1) | 0  # Pad with 1 unused bit (47 + 1 = 48)


        print("Original Values:", robot_x, robot_y, robot_angle, num, ball_x, ball_y)
        ascii85_chars = ''.join(chr(i) for i in range(33, 118))
        def int_to_ascii85(n):
            if n == 0:
                return ascii85_chars[0]
            encoded = []
            while n > 0:
                n, rem = divmod(n, 85)
                encoded.append(ascii85_chars[rem])
            return ''.join(reversed(encoded))

        encoded_str = int_to_ascii85(bits)
        print("Encoded:", encoded_str)
        return encoded_str

            
    def transmit(self, message):
        compressedMessage = self.encodeMessage(message)
        self.get_logger().info(f"Transmitting message: {message} as {compressedMessage}")
        waveform = ggwave.encode(compressedMessage, protocolId = 1, volume = 20)
        stream = self.pyaudio.open(format=pyaudio.paFloat32, channels=1, rate=48000, output=True, frames_per_buffer=4096)
        stream.write(waveform, len(waveform)//4)
        stream.stop_stream()
        stream.close()  
        
        
def main(args=None):
    rclpy.init(args=args)
    speaker = AudioDataSpeaker()
    try:
        while rclpy.ok():
            #speaker.run()
            rclpy.spin(speaker)
    except KeyboardInterrupt as e:
        pass
    finally:
        speaker.get_logger().info("Shutting down AudioDataSpeaker...")
        speaker.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
