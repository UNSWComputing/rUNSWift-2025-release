#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import time
import pyaudio
import ggwave
from runswift_interfaces.msg import BehavioursRobotInfo

class AudioReader(Node):
    SAMPLE_RATE = 48000
    WINDOW_SIZE = 1024

    def __init__(self):
        super().__init__('audio_reader_threaded')
        self.publisher_ = self.create_publisher(BehavioursRobotInfo, '/comms_audio_message_output', 10)
        self.p = pyaudio.PyAudio()
        self.stream = self.p.open(
            format=pyaudio.paFloat32,
            channels=1,
            rate=self.SAMPLE_RATE,
            input=True,
            frames_per_buffer=self.WINDOW_SIZE,
            input_device_index=None,  # Or set to specific device
            stream_callback=None,
            start=True
        )

        self.ggwave_instance = ggwave.init()
        self.running = True
        self.decode_event = threading.Event()

        self.get_logger().info("Starting GGWave audio listener thread...")
        self.listener_thread = threading.Thread(target=self.listen_loop)
        self.listener_thread.daemon = True
        self.listener_thread.start()

    def listen_loop(self):
        self.get_logger().info("STARTING AUDIO LISTENER")
        while self.running and rclpy.ok():
            #self.get_logger().info("IN LOOP")
            try:
                data = self.stream.read(self.WINDOW_SIZE, exception_on_overflow=False)
                # self.get_logger().info("READING AUDIO DATA")
                if not data:
                    self.get_logger().warn("Empty data received.")
                    continue
                
                decode_thread = threading.Thread(target=self.decode_audio, args=(data,))
                self.decode_event.clear()
                decode_thread.start()
                decode_thread.join(timeout=2)
            except Exception as e:
                self.get_logger().warn(f"Audio read error: {e}")
            time.sleep(0.01)  # Give CPU a breath

    def decodeMessage(self, encoded_str: str) -> int:
        behaviourData = BehavioursRobotInfo()
        ascii85_chars = ''.join(chr(i) for i in range(33, 118))
        char_to_val = {c: i for i, c in enumerate(ascii85_chars)}

        decoded_bits = 0
        for c in encoded_str:
            decoded_bits = decoded_bits * 85 + char_to_val[c]

        # Skip the last (unused) 1 bit
        decoded_bits >>= 1

        y2 = decoded_bits & 0xFF
        decoded_bits >>= 8

        x2 = decoded_bits & 0x1FF
        decoded_bits >>= 9

        num = decoded_bits & 0x7
        decoded_bits >>= 3

        heading_bits = decoded_bits & 0x3FF  # 10 bits
        decoded_bits >>= 10

        heading_int = (heading_bits >> 7) & 0x7
        heading_frac = heading_bits & 0x7F
        heading_radians = heading_int + (heading_frac / 100.0)

        y1 = decoded_bits & 0xFF
        decoded_bits >>= 8

        x1 = decoded_bits & 0x1FF

        behaviourData.player_number = num
        behaviourData.robot_pos_x = x1 * 25 - 4800.0
        behaviourData.robot_pos_y = y1 * 25 - 3300.0
        behaviourData.heading = round(heading_radians - 3.14, 2)  # radians, centered to [-pi, pi]
        behaviourData.ball_pos_x = x2 * 25 - 4800.0
        behaviourData.ball_pos_y = y2 * 25 - 3300.0

        self.get_logger().info(f"Decoded: x={behaviourData.robot_pos_x}, y={behaviourData.robot_pos_y}, "
                            f"heading={behaviourData.heading}, num={num}, "
                            f"ball_x={behaviourData.ball_pos_x}, ball_y={behaviourData.ball_pos_y}")


        return behaviourData

    def decode_audio(self, data):
        try: 
            result = ggwave.decode(self.ggwave_instance, data)
            if result is not None:
                decoded = result.decode("utf-8")
                uncompressedNumbers = self.decodeMessage(decoded)
                self.publisher_.publish(uncompressedNumbers)
                self.decode_event.set()
        except Exception as e:
            self.get_logger().warn(f"Failed to decode: {e}")

    def destroy_node(self):
        self.get_logger().info("Shutting down listener...")
        self.running = False
        self.listener_thread.join()
        ggwave.free(self.ggwave_instance)
        self.stream.stop_stream()
        self.stream.close()
        self.p.terminate()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AudioReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
