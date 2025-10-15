#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from runswift_interfaces.msg import VisionProcessedImageData
from sensor_msgs.msg import Image
import os
import numpy as np
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
class VisionDebugNode(Node):
    def __init__(self):
        super().__init__('vision_debug_cca')
        
        # Create CV bridge
        self.bridge = CvBridge()
        
        # Create subscription to vision info
        self.subscription = self.create_subscription(
            VisionProcessedImageData,
            '/vision/vision_info_in',
            self.vision_callback,
            2
        )

        time_now = int(time.time())
        os.makedirs(f'/workspace/vision_models/datasets/t{time_now}', exist_ok=True)
        self.saving_dir = f'/workspace/vision_models/datasets/t{time_now}'
        self.image_saver = self.create_subscription(
            Image,
            '/vision/image_save',
            self.image_saver_callback,
            2
        )
        self.count = 0 
        self.saved_img_idx = 0
        self.publisher = self.create_publisher(
            Image,
            '/vision/debug/annotated_image',
            QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            )
        )
        self.publisher2 = self.create_publisher(
            Image,
            '/vision/debug/annotated_image_ball_roi_region',
            QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            )
        )
        
        self.publisher3 = self.create_publisher(
            Image,
            '/vision/debug/annotated_image_roi_region_all',
            QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            )
        )

        self.random_pub = self.create_publisher(
            Image,
            '/vision/debug/random_image',
            QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            )
        )

        self.random_pub2 = self.create_publisher(
            Image,
            '/vision/debug/random_image2',
            QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            )
        )

        self.raw_pub = self.create_publisher(
            Image,
            '/vision/debug/raw_image',
            QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            )
        )

        self.get_logger().info('Vision Debug Node started')
    def image_saver_callback(self, msg):
        print(f"Saving image {self.saved_img_idx}")
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        cv2.imwrite(f'{self.saving_dir}/{self.saved_img_idx}.png', cv_image)
        self.saved_img_idx += 1
    def vision_callback(self, msg):
        # republish msg.image_raw
        self.raw_pub.publish(msg.image_raw)
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg.image_binarised, desired_encoding='mono8')
            
            # Convert to BGR for colored annotations
            colored_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
            
            colored_image2 = colored_image.copy()

            colored_image3 = colored_image.copy()

            colored_image4 = colored_image.copy()

            innercount = 0
            print(f"Image {self.count} has {len(msg.image_rois)} rois")
            print(f"Image {self.count} has {len(msg.roi_regions)} roi regions")
            print(f"Image {self.count} has {len(msg.full_regions)} full regions")
            print(f"Image {self.count} has {len(msg.ball_roi_regions)} ball roi regions")
            

            
            for i in range(len(msg.image_rois)):
                innercount += 1

                # roi bbox
                
                # save the roi which is already a image
                cv_image_roi = self.bridge.imgmsg_to_cv2(msg.image_rois[i], desired_encoding='mono8')
                # print if any pixel is not 0 or 255
                colored_image_roi = cv2.cvtColor(cv_image_roi, cv2.COLOR_GRAY2BGR)


                # publish to the topic random 
                cv2.putText(colored_image_roi, f"I {innercount}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 1)
                random_image = self.bridge.cv2_to_imgmsg(colored_image_roi, encoding='bgr8')
                random_image.header = msg.header

                bbox = msg.roi_regions[i]
                
                center_x = int(bbox.center.position.x)
                center_y = int(bbox.center.position.y)
                width = int(bbox.size_x)
                height = int(bbox.size_y)
                
                # Calculate rectangle corners
                top_left = (center_x - width//2, center_y - height//2)
                bottom_right = (center_x + width//2, center_y + height//2)

                cv2.rectangle(colored_image3, top_left, bottom_right, (255, 0, 0), 2)
                cv2.putText(colored_image3, f"I {innercount}", (center_x, center_y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 1)
                self.random_pub.publish(random_image)
                
                
            # Draw ROI regions
            for roi in msg.roi_regions:
                # Extract center and size from the ROI
                center_x = int(roi.center.position.x)
                center_y = int(roi.center.position.y)
                width = int(roi.size_x)
                height = int(roi.size_y)
                
                # Calculate rectangle corners
                top_left = (center_x - width//2, center_y - height//2)
                bottom_right = (center_x + width//2, center_y + height//2)
                
                # Draw rectangle in green
                cv2.rectangle(colored_image, top_left, bottom_right, (0, 255, 0), 2)
                
                # Draw center point in red
                cv2.circle(colored_image, (center_x, center_y), 3, (0, 0, 255), -1)

            for roi_full in msg.ball_roi_regions:
                # Extract center and size from the ROI
                center_x = int(roi_full.center.position.x)
                center_y = int(roi_full.center.position.y)
                width = int(roi_full.size_x)
                height = int(roi_full.size_y)
                
                # Calculate rectangle corners
                top_left = (center_x - width//2, center_y - height//2)
                bottom_right = (center_x + width//2, center_y + height//2)
                
                # Draw rectangle in green
                cv2.rectangle(colored_image2, top_left, bottom_right, (255, 0, 0), 2)
                
                # Draw center point in red
                cv2.circle(colored_image2, (center_x, center_y), 3, (0, 0, 255), -1)

            for roi_full in msg.full_regions:
                # Extract center and size from the ROI
                center_x = int(roi_full.center.position.x)
                center_y = int(roi_full.center.position.y)
                width = int(roi_full.size_x)
                height = int(roi_full.size_y)
                
                # Calculate rectangle corners
                top_left = (center_x - width//2, center_y - height//2)
                bottom_right = (center_x + width//2, center_y + height//2)
                
                #Draw rectangle in green
                cv2.rectangle(colored_image4, top_left, bottom_right, (255, 0, 0), 2)
                
                # Draw center point in red
                cv2.circle(colored_image4, (center_x, center_y), 3, (0, 0, 255), -1)

            # Convert back to ROS Image message
            annotated_img_msg = self.bridge.cv2_to_imgmsg(colored_image, encoding='bgr8')
            annotated_img_msg.header = msg.header  # Preserve the original header

            annotated_img_msg2 = self.bridge.cv2_to_imgmsg(colored_image2, encoding='bgr8')
            annotated_img_msg2.header = msg.header  # Preserve the original header

            annotated_img_msg3 = self.bridge.cv2_to_imgmsg(colored_image4, encoding='bgr8')
            annotated_img_msg3.header = msg.header  # Preserve the original header
            
            # Publish annotated image
            self.publisher.publish(annotated_img_msg)
            self.publisher2.publish(annotated_img_msg2)
            self.publisher3.publish(annotated_img_msg3)

            # convert to ros image
            msg_colored_image3 = self.bridge.cv2_to_imgmsg(colored_image3, encoding='bgr8')
            self.random_pub2.publish(msg_colored_image3)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = VisionDebugNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
