#!/usr/bin/env python3

import rclpy
import rclpy.logging
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import os
import cv2
import time
class Image_saver_node(Node):
    def __init__(self):
        super().__init__('save_images')
        cv2.namedWindow('top')
        cv2.namedWindow('bottom')
        cv2.setMouseCallback('top', self.top_save)
        cv2.setMouseCallback('bottom', self.bottom_save)

        # time in seconds
        t = int(time.time())
        if not os.path.exists('saved-images'):
            os.mkdir('saved-images')
        os.mkdir(f'saved-images/{t}')
        self.top_dir = f'saved-images/{t}/top'
        self.bottom_dir = f'saved-images/{t}/bottom'
        self.top_count = 0
        self.bottom_count = 0
        self.top_img = None
        self.bottom_img = None
        os.mkdir(self.top_dir)
        os.mkdir(self.bottom_dir)
        self.get_logger().info(f'Saving images to saved-images/{t}')
        # Create CV bridge
        self.bridge = CvBridge()
        self.subscriptiontop = self.create_subscription(
            Image,
            'camera/top/raw_image',
            self.top_callback,
            2
        )

        self.subscriptionbot = self.create_subscription(
            Image,
            'camera/bot/raw_image',
            self.bottom_callback,
            2
        )

        

    def top_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='yuv422_yuy2') 
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_YUV2BGR_YUY2)
        self.top_img = cv_image
        cv2.imshow('top', cv_image)
        cv2.waitKey(1)

    def bottom_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='yuv422_yuy2')
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_YUV2BGR_YUY2)
        self.bottom_img = cv_image
        cv2.imshow('bottom', cv_image)
        cv2.waitKey(1)
    def top_save(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            cv2.imwrite(f'{self.top_dir}/{self.top_count}.png', self.top_img)
            self.top_count += 1
            self.get_logger().info(f'Saved top image {self.top_count}')
    
    def bottom_save(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            cv2.imwrite(f'{self.bottom_dir}/{self.bottom_count}.png', self.bottom_img)
            self.bottom_count += 1
            self.get_logger().info(f'Saved bot image {self.bottom_count}')
        

        
        


        

def main(args=None):
    rclpy.init(args=args)
    node = Image_saver_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
