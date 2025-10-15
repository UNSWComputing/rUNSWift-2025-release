#!/usr/bin/env python3

import rclpy
import rclpy.logging
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from runswift_interfaces.msg import VisionImageWithFieldBoundry, VisionPipelineSwitchAsk, VisionPipelineSwitchResult
from sensor_msgs.msg import Image
import os
import numpy as np
import time
import onnxruntime
import subprocess
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nao_lola_command_msgs.msg import LeftEyeLeds
from std_msgs.msg import ColorRGBA

class Visionpipeline_switch_node(Node):
    def __init__(self):
        super().__init__('vision_pipeline_switch_node')
        options = onnxruntime.SessionOptions()

        # This reduces the cpu consumption of the node from ~ 100% to 65% of a core.
        options.inter_op_num_threads = 1
        options.intra_op_num_threads = 1
        options.execution_mode = onnxruntime.ExecutionMode.ORT_SEQUENTIAL  # Disable inter-op parallelism
        # Disable thread spinning
        #options.add_session_config_entry("session.intra_op.allow_spinning", "0")
        #options.add_session_config_entry("session.inter_op.allow_spinning", "0")
        options.graph_optimization_level = onnxruntime.GraphOptimizationLevel.ORT_ENABLE_ALL

        # Create CV bridge
        self.bridge = CvBridge()
        robot_path_1 = "/home/nao/vision_models/field_detector (1).onnx"
        dev_path_1 = "/workspace/vision_models/field_detector (1).onnx" # in dev docker container
        robot_path_2 = "/home/nao/vision_models/single_thunder_init8.onnx"
        dev_path_2 = "/workspace/vision_models/single_thunder_init8.onnx" # in dev docker container
        if os.path.exists(robot_path_1):
           self.fieldline_detector = onnxruntime.InferenceSession(robot_path_1,sess_options=options)
        elif os.path.exists(dev_path_1):
            self.fieldline_detector = onnxruntime.InferenceSession(dev_path_1, sess_options=options)


        # Create CV bridge
        if os.path.exists(robot_path_2):
            self.skeleton_detector = onnxruntime.InferenceSession(robot_path_2,sess_options=options)
        elif os.path.exists(dev_path_2):
            self.skeleton_detector = onnxruntime.InferenceSession(dev_path_2, sess_options=options)

        #Test
        self.check_mode = 1
        self.vision_cb = self.fbline
        self.subscription = self.create_subscription(VisionPipelineSwitchAsk , "vision/ref_switch", self.pipeline_switching_cb, 2)

        self.refpub = self.create_publisher(VisionPipelineSwitchResult, 'vision/ref_result', 2)
        #

        # Create subscription to vision info with QOS best effort
        self.subscription = self.create_subscription(
            Image,
            'camera/top/raw_image',
            self.vision_callback,
            QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=1
            )
        )

        self.repub = self.create_publisher(VisionImageWithFieldBoundry, 'vision/image_with_boundry', QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=2
            ))
        self.yellow_eye_msg = LeftEyeLeds()
        yellow_color = ColorRGBA()
        yellow_color.r = 1.0
        yellow_color.g = 1.0
        yellow_color.b = 0.0
        yellow_color.a = 1.0  # Alpha is ignored but set for consistency
        green_color = ColorRGBA()
        green_color.r = 0.0
        green_color.g = 1.0
        green_color.b = 0.0
        green_color.a = 1.0  # Alpha is ignored but set for consistency

        off_color = ColorRGBA()
        off_color.r = 0.0
        off_color.g = 0.0
        off_color.b = 0.0
        off_color.a = 1.0  # Alpha is ignored but set for consistency

        self.yellow_eye_msg.colors = [yellow_color] * 8
        self.green_eye_msg = LeftEyeLeds()
        self.green_eye_msg.colors = [green_color] * 8
        self.off_eye_msg = LeftEyeLeds()
        self.off_eye_msg.colors = [off_color] * 8

        # EYE LEDS PUB
        self.eye_publisher = self.create_publisher(LeftEyeLeds, '/effectors/left_eye_leds', QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=1
            ))



        # self.debug_publisher = self.create_publisher(Image, 'fieldline_detector/debug', QoSProfile(
        #         reliability=ReliabilityPolicy.BEST_EFFORT,
        #         history=HistoryPolicy.KEEP_LAST,
        #         depth=2
        #     ))

        # self.debug_publisher = self.create_publisher(Image, 'skeleton_detector/debug', QoSProfile(
        #         reliability=ReliabilityPolicy.BEST_EFFORT,
        #         history=HistoryPolicy.KEEP_LAST,
        #         depth=2
        #     ))

    #Test
    def pipeline_switching_cb(self, msg):
        self.vision_cb = self.skeleton_cb if msg.is_ref else self.fbline
        self.check_mode = msg.check_mode

        if not msg.is_ref:
            self.eye_publisher.publish(self.off_eye_msg)

    def vision_callback(self, msg):
        self.vision_cb(msg)

    def fbline(self,msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='yuv422_yuy2')
        # debug_image = cv_image.copy()

        height, width = cv_image.shape[0], cv_image.shape[1]

        # Create output array for YUV format (3 channels)
        output = np.zeros((30, 40, 3), dtype=np.uint8)

        # Flatten input for easier indexing
        input_data = cv_image.reshape(-1)

        # Calculate ratios for resizing
        height_ratio = height / 30
        width_ratio = width / 40

        # Process each pixel in output (40x30)
        for y in range(30):
            for x in range(40):
                # Find corresponding position in input image
                in_y = int(y * height_ratio)
                in_x = int(x * width_ratio)

                # Calculate input index (each YUYV macropixel is 4 bytes for 2 pixels)
                in_idx = (in_y * width + (in_x & ~1)) * 2

                # Extract Y
                output[y, x, 0] = input_data[in_idx + (in_x % 2) * 2]  # Y1 or Y2

                # Extract U and V (shared between 2 pixels)
                output[y, x, 1] = input_data[in_idx + 1]  # U
                output[y, x, 2] = input_data[in_idx + 3]  # V

        #Convert image to tensor
        input_tensor = output.transpose((2, 0, 1))
        input_tensor = np.expand_dims(input_tensor, axis=0).astype(np.float32)

        #Run inference
        output = self.fieldline_detector.run(None, {'input': input_tensor})[0][0]

        if hasattr(self.fieldline_detector, 'close'):
            self.fieldline_detector.close()

        out_msg = VisionImageWithFieldBoundry()
        # adding 0.03 to the output to make the field line use the bottom of the detected pixel
        output = output + 0.03
        out_msg.field_boundary = output.tolist()
        out_msg.image = msg
        self.repub.publish(out_msg)

        # resize image
        # debug_image = cv2.resize(debug_image, (40, 30))
        # debug_image = cv2.cvtColor(debug_image, cv2.COLOR_YUV2BGR_YUY2)
        # for i in range(40):
        #     cv2.circle(debug_image, (i, int(output[i]*30)), 0, (0, 255, 0), 1)
        # msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
        # self.debug_publisher.publish(msg)


    def skeleton_cb(self,msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='yuv422_yuy2')

        # convert yuv422_yuy2 to rgb
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_YUV2BGR_YUY2)


        # crop image to 480x480 with the center of the image
        cv_image = cv_image[0:480, 80:560]

        # resize image to 256x256
        cv_image = cv2.resize(cv_image, (256, 256))

        # convert to rgb
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

        # put the image in the correct format for the model
        input_tensor = np.expand_dims(cv_image, axis=0).astype(np.int32)

        #Run inference
        output = self.skeleton_detector.run(None, {'input': input_tensor})[0][0]

        if hasattr(self.skeleton_detector, 'close'):
            self.skeleton_detector.close()

        # Publish debug image
        # self.get_logger().info(f"{output.shape}")

        # print conf for debug
        # self.get_logger().info(f"Confidence: {output[0][5][2]}")
        # self.get_logger().info(f"Confidence: {output[0][6][2]}")
        # self.get_logger().info(f"Confidence: {output[0][9][2]}")
        # self.get_logger().info(f"Confidence: {output[0][10][2]}")

        # Prep msg to be sent back to behaviors regarding the referee's hand signal
        out_msg = VisionPipelineSwitchResult()

        if self.check_mode == 1:

            if output[0][6][0] > output[0][10][0] and output[0][5][0] > output[0][9][0]:
                # print out the red points

                # cv2.circle(cv_image, (int(output[0][5][1]*256), int(output[0][5][0]*256)), 5, (0, 0, 255), -1)
                # cv2.circle(cv_image, (int(output[0][6][1]*256), int(output[0][6][0]*256)), 5, (0, 0, 255), -1)
                # cv2.circle(cv_image, (int(output[0][9][1]*256), int(output[0][9][0]*256)), 5, (0, 0, 255), -1)
                # cv2.circle(cv_image, (int(output[0][10][1]*256), int(output[0][10][0]*256)), 5, (0, 0, 255), -1)
                # # print out red text message
                # cv2.putText(cv_image, "Standby Detected", (int(output[0][6][1]*256), int(output[0][6][0]*256)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
                # subprocess.Popen(["flite", "-t", "Up"])

                out_msg.referee_signal = 1
                self.refpub.publish(out_msg)
                self.eye_publisher.publish(self.green_eye_msg)  # GREEN LEDs for success up.

            else:
                # cv2.circle(cv_image, (int(output[0][5][1]*256), int(output[0][5][0]*256)), 5, (0, 255, 0), -1)
                # cv2.circle(cv_image, (int(output[0][6][1]*256), int(output[0][6][0]*256)), 5, (0, 255, 0), -1)
                # cv2.circle(cv_image, (int(output[0][9][1]*256), int(output[0][9][0]*256)), 5, (0, 255, 0), -1)
                # cv2.circle(cv_image, (int(output[0][10][1]*256), int(output[0][10][0]*256)), 5, (0, 255, 0), -1)
                # subprocess.Popen(["flite", "-t", "No"])

                out_msg.referee_signal = 0
                self.refpub.publish(out_msg)

                self.eye_publisher.publish(self.yellow_eye_msg)  # YELLOW LEDs for waiting for sig

        elif self.check_mode == 2:
            # Get coordinates for left and right body parts
            left_shoulder_x = output[0][5][1]  # Left shoulder X
            left_shoulder_y = output[0][5][0]  # Left shoulder Y
            right_shoulder_x = output[0][6][1] # Right shoulder X
            right_shoulder_y = output[0][6][0] # Right shoulder Y
            left_wrist_x = output[0][9][1]     # Left wrist X
            left_wrist_y = output[0][9][0]     # Left wrist Y
            right_wrist_x = output[0][10][1]   # Right wrist X
            right_wrist_y = output[0][10][0]   # Right wrist Y

            # Calculate center point between shoulders to determine body center
            body_center_x = (left_shoulder_x + right_shoulder_x) / 2

            # Calculate which wrist is furthest from body center horizontally
            left_wrist_distance_from_center = abs(left_wrist_x - body_center_x)
            right_wrist_distance_from_center = abs(right_wrist_x - body_center_x)

            # Determine which wrist is more extended
            if left_wrist_distance_from_center > right_wrist_distance_from_center:
                extended_wrist_x = left_wrist_x
                arm_extension = left_wrist_distance_from_center
            else:
                extended_wrist_x = right_wrist_x
                arm_extension = right_wrist_distance_from_center

            # Set thresholds
            extension_threshold = 0.16  # Minimum extension from body center to consider pointing
            directional_threshold = 0.08  # Minimum horizontal distance from body center for left/right detection

            if arm_extension > extension_threshold:
                # Check if the extended wrist is pointing left or right from robot's perspective
                horizontal_distance = abs(extended_wrist_x - body_center_x)

                if horizontal_distance > directional_threshold:
                    if extended_wrist_x < body_center_x:
                        # Wrist is to the left of body center = pointing left from robot's perspective
                        out_msg.referee_signal = 2
                        # subprocess.Popen(["flite", "-t", "Left"])
                        cv2.putText(cv_image, "Left Detected", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
                    else:
                        # Wrist is to the right of body center = pointing right from robot's perspective
                        out_msg.referee_signal = 3
                        # subprocess.Popen(["flite", "-t", "Right"])
                        cv2.putText(cv_image, "Right Detected", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
                else:
                    # Extended but not clearly left or right (pointing forward/center)
                    out_msg.referee_signal = 0
                    # subprocess.Popen(["flite", "-t", "No"])
                    cv2.putText(cv_image, "Pointing Forward/Center", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
            else:
                # No clear extension detected
                out_msg.referee_signal = 0
                # subprocess.Popen(["flite", "-t", "No"])
                cv2.putText(cv_image, "No Clear Direction", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)

            self.refpub.publish(out_msg)

            # Draw keypoints for debugging
            cv2.circle(cv_image, (int(left_shoulder_x*256), int(left_shoulder_y*256)), 5, (255, 0, 0), -1)   # Blue for left shoulder
            cv2.circle(cv_image, (int(right_shoulder_x*256), int(right_shoulder_y*256)), 5, (0, 255, 0), -1) # Green for right shoulder
            cv2.circle(cv_image, (int(left_wrist_x*256), int(left_wrist_y*256)), 5, (255, 0, 255), -1)       # Magenta for left wrist
            cv2.circle(cv_image, (int(right_wrist_x*256), int(right_wrist_y*256)), 5, (0, 255, 255), -1)     # Cyan for right wrist

            # Draw body center point
            cv2.circle(cv_image, (int(body_center_x*256), int((left_shoulder_y + right_shoulder_y)/2*256)), 8, (255, 255, 255), -1)  # White for body center

            # Draw lines from body center to wrists to show extension
            cv2.line(cv_image,
                     (int(body_center_x*256), int((left_shoulder_y + right_shoulder_y)/2*256)),
                     (int(left_wrist_x*256), int(left_wrist_y*256)),
                     (255, 0, 255), 2)  # Magenta line to left wrist
            cv2.line(cv_image,
                     (int(body_center_x*256), int((left_shoulder_y + right_shoulder_y)/2*256)),
                     (int(right_wrist_x*256), int(right_wrist_y*256)),
                     (0, 255, 255), 2)  # Cyan line to right wrist

            # Add text labels for each joint
            cv2.putText(cv_image, "L Shoulder",
                       (int(left_shoulder_x*256) + 8, int(left_shoulder_y*256) - 8),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv2.LINE_AA)
            cv2.putText(cv_image, "R Shoulder",
                       (int(right_shoulder_x*256) + 8, int(right_shoulder_y*256) - 8),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv2.LINE_AA)
            cv2.putText(cv_image, "L Wrist",
                       (int(left_wrist_x*256) + 8, int(left_wrist_y*256) - 8),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv2.LINE_AA)
            cv2.putText(cv_image, "R Wrist",
                       (int(right_wrist_x*256) + 8, int(right_wrist_y*256) - 8),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv2.LINE_AA)
            cv2.putText(cv_image, "Body Center",
                       (int(body_center_x*256) + 8, int((left_shoulder_y + right_shoulder_y)/2*256) - 8),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv2.LINE_AA)


        else:
            #Case for irregular data
            pass


        # msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='rgb8')
        # self.debug_publisher.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = Visionpipeline_switch_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
