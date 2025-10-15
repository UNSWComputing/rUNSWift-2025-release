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
from runswift_interfaces.msg import VisionFieldFeatures, VisionFieldFeature, VisionBallFeature, VisionBalls, VisionImageWithFieldBoundry
from vision_msgs.msg import BoundingBox2D

class visionPlotting(Node):
    def __init__(self):
        super().__init__("vision_plotting")
        self.bridge = CvBridge()
        self.ball_publisher = self.create_publisher(Image, 'vision/ball_image', 2)
        self.field_publisher = self.create_publisher(Image, 'vision/field_image', 2)
        self.image = None
        cv2.setUseOptimized(True)
        self.bot_image = None
        # TODO: Get Ball and figure out which frame to use(in header)
        self.ball_subscription = self.create_subscription(
            VisionBalls,
            'vision/VBalls',
            self.ball_callback,
            2
        )

        self.top_image_sub = self.create_subscription(
            Image,
            '/camera/top/raw_image',
            self.image_callback,
            2
        )
        self.bot_image_sub = self.create_subscription(
            Image,
            '/camera/bot/raw_image',
            self.bot_image_callback,
            2
        )
        self.region_of_interest_sub = self.create_subscription(
            VisionProcessedImageData,
            'vision/vision_info_in',
            self.region_of_interest_callback,
            2
        )

        self.field_sub= self.create_subscription(
            VisionImageWithFieldBoundry,
            'vision/image_with_boundry',
            self.field_callback,
            2)

        self.timer_sub = self.create_timer(0.01, self.timer_callback)
    def timer_callback(self):
        cv2.waitKey(1)

    
    def region_of_interest_callback(self, msg):
        img = msg.image_raw
        img = self.bridge.imgmsg_to_cv2(img, desired_encoding='yuv422_yuy2')
        img = cv2.cvtColor(img, cv2.COLOR_YUV2BGR_YUY2)
        r = msg.ratio
        # construct a black image with the same dimensions as the original image
        black_board = np.zeros((img.shape[0], img.shape[1]), np.uint8)
        for idx, region in enumerate(msg.roi_regions):
            print(region.center)
            x = int(region.center.position.x * r)
            y = int(region.center.position.y * r)
            w = int(region.size_x * r)
            h = int(region.size_y * r)
            top_left = (x - w//2, y - h//2)

            print(f"ROI: {w}x{h}")
            print("=====")
            cv2.rectangle(img, (x - w//2, y - h//2), (x + w//2, y + h//2), (255, 255, 255), 3)
        cv2.imshow('ROI', img)
            
    def field_callback(self, msg):
        img = msg.image
        img = self.bridge.imgmsg_to_cv2(img, desired_encoding='yuv422_yuy2')
        img = cv2.cvtColor(img, cv2.COLOR_YUV2BGR_YUY2)
        boundry = msg.field_boundary
        # resize to 40x30
        debug_image = cv2.resize(img, (40, 30))
        for i in range(40):
            cv2.circle(debug_image, (i, int(boundry[i]*30)), 0, (0, 255, 0), 1)
        cv2.imshow('Field Boundry', debug_image)

            


        
    def image_callback(self, msg):
        self.image = msg
        self.get_logger().info("Got top image")
    def bot_image_callback(self, msg):
        self.bot_image = msg
        self.get_logger().info("Got bot image")
    def ball_callback(self, msg):
        if self.image == None or self.bot_image == None:
            return
        frame = msg.header.frame_id
        image = None
        if frame == "base_footprint_from_top":
            image = self.bridge.imgmsg_to_cv2(self.image, desired_encoding='yuv422_yuy2')
            window = 'Top Balls'
        elif frame == "base_footprint_from_bottom":
            image = self.bridge.imgmsg_to_cv2(self.bot_image, desired_encoding='yuv422_yuy2')
            window = 'Bottom Balls'
        else:
            self.get_logger().error(f"Unknown frame: {frame}")
            return
        for ball in msg.ball_features:
            x = int(ball.ball_pixel_coordinates.x)
            y = int(ball.ball_pixel_coordinates.y)
            radius = int(ball.pixel_radius)
            cv2.circle(image, (x, y), radius, (0, 255, 0), -1)
            cv2.putText(image, "Ball", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
            bgr_image = cv2.cvtColor(image, cv2.COLOR_YUV2BGR_YUY2)
            # Create a named window and show the image
            cv2.namedWindow(window, cv2.WINDOW_NORMAL)
            cv2.imshow(window, bgr_image)

if __name__ == '__main__':
    rclpy.init()
    node = visionPlotting()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


    