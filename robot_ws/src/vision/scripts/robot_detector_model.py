#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import os
import numpy as np
import onnxruntime
import time #
# Import the new service and message types
from runswift_interfaces.srv import DetectRobots
from runswift_interfaces.msg import RobotDetection 

class RobotDetectionNode(Node):
    def __init__(self):
        super().__init__('robot_detector_model')

        # --- Configuration Loading ---
        #robot_config_path = "/home/nao/vision_models/4_anchor_boxes_20230629-220730.cfg"
        #dev_config_path = "/workspace/vision_models/4_anchor_boxes_20230629-220730.cfg" # Or wherever your config is in dev env
        
        #config_path = None
        #if os.path.exists(robot_config_path):
        #   config_path = robot_config_path
        #elif os.path.exists(dev_config_path):
        #    config_path = dev_config_path
        #else:
        #    self.get_logger().error("Config file not found at standard locations!")
        #    raise FileNotFoundError("Config not found")
        self.config = self.parse_config()
        self.get_logger().info(f"Config loaded successfully: {self.config}")
        # Check required config keys
        required_keys = ['inputHeight', 'inputWidth', 'inputChannels',
                            'outputHeight', 'outputWidth', 'outputAnchors',
                            'paramsPerAnchor', 'predictFallen',
                            'confidenceIndex', 'yMidIndex', 'xMidIndex',
                            'heightIndex', 'widthIndex', 'anchors']
        for key in required_keys:
            if key not in self.config:
                raise ValueError(f"Missing required key '{key}' in config file.")

        # Ensure fallenClassIndex is present if predictFallen is true
        if self.config['predictFallen'] and 'fallenClassIndex' not in self.config:
                self.get_logger().warning("Config key 'fallenClassIndex' is missing but 'predictFallen' is true. Fallen status will not be decoded.")
                self.config['fallenClassIndex'] = -1 # Treat as not available

        # Check if anchor count matches
        if len(self.config['anchors']) != self.config['outputAnchors']:
                self.get_logger().warning(f"Number of anchors in config ({len(self.config['anchors'])}) does not match outputAnchors ({self.config['outputAnchors']}). This might cause issues.")

        # Store relevant config values for easy access
        self.input_height = self.config['inputHeight']
        self.input_width = self.config['inputWidth']
        self.input_channels = self.config['inputChannels'] # Should be 3 for YUV
        self.output_height = self.config['outputHeight']
        self.output_width = self.config['outputWidth']
        self.output_anchors = self.config['outputAnchors']
        self.params_per_anchor = self.config['paramsPerAnchor']
        self.predict_fallen = self.config['predictFallen']
        self.confidence_idx = self.config['confidenceIndex']
        self.y_mid_idx = self.config['yMidIndex']
        self.x_mid_idx = self.config['xMidIndex']
        self.height_idx = self.config['heightIndex']
        self.width_idx = self.config['widthIndex']
        self.fallen_class_idx = self.config['fallenClassIndex']

        # Post-processing Thresholds
        self.confidence_threshold = 0.6 # Minimum confidence to consider a detection
        self.nms_threshold = 0.3      # IoU threshold for Non-Maximum Suppression
        self.fallen_threshold = 0.55   # Minimum confidence for fallen class to classify as fallen

        # --- Model Loading ---
        options = onnxruntime.SessionOptions()
        # Optimize for throughput on platforms like NAO
        options.inter_op_num_threads = 1 # Keep single thread per operation
        options.intra_op_num_threads = 1 # Keep single thread within operation
        #options.graph_optimization_level = onnxruntime.GraphOptimizationLevel.ORT_ENABLE_ALL

        robot_model_path = "/home/nao/vision_models/4_anchor_boxes_model_no_activation_20230629-220730.onnx"
        dev_model_path = "/workspace/vision_models/4_anchor_boxes_model_no_activation_20230629-220730.onnx"

        model_path = None
        if os.path.exists(robot_model_path):
           model_path = robot_model_path
        elif os.path.exists(dev_model_path):
            model_path = dev_model_path
        else:
            self.get_logger().error("ONNX model file not found at standard locations!")
            raise FileNotFoundError("ONNX model not found")

        self.get_logger().info(f"Loading ONNX model from: {model_path}")
        try:
            self.model = onnxruntime.InferenceSession(model_path, sess_options=options)
            input_details = self.model.get_inputs()[0]
            output_details = self.model.get_outputs()[0]
            self.get_logger().info(f"Model Input: Name={input_details.name}, Shape={input_details.shape}, Type={input_details.type}")
            self.get_logger().info(f"Model Output: Name={output_details.name}, Shape={output_details.shape}, Type={output_details.type}")

            # Verify model input shape against config
            model_input_shape = input_details.shape
            # Expecting (Batch, H, W, C) or (Batch, C, H, W)
            input_shape_ok = False
            if len(model_input_shape) == 4:
                 # Check NHWC format: (Batch, H, W, C)
                if model_input_shape[1] == self.input_height and \
                    model_input_shape[2] == self.input_width and \
                    model_input_shape[3] == self.input_channels:
                    self.input_format = 'NHWC'
                    input_shape_ok = True
                # Check NCHW format: (Batch, C, H, W)
                elif model_input_shape[1] == self.input_channels and \
                    model_input_shape[2] == self.input_height and \
                    model_input_shape[3] == self.input_width:
                    self.input_format = 'NCHW'
                    input_shape_ok = True

            if not input_shape_ok:
                self.get_logger().warning(f"Model input shape {model_input_shape} does not strictly match expected config shape ({self.input_height}, {self.input_width}, {self.input_channels}) in NHWC or NCHW format. Preprocessing must ensure correct input shape and format.")
                # For robustness, we should determine the actual model input format (NHWC/NCHW) from input_details
                if len(model_input_shape) == 4:
                    if model_input_shape[1] == self.input_height and model_input_shape[2] == self.input_width:
                        # Assume NHWC if H and W match, even if channels differ (logged above)
                        self.input_format = 'NHWC'
                        self.get_logger().info("Assuming model input format is NHWC based on height/width match.")
                    elif model_input_shape[2] == self.input_height and model_input_shape[3] == self.input_width:
                        # Assume NCHW if H and W match in different positions
                        self.input_format = 'NCHW'
                        self.get_logger().info("Assuming model input format is NCHW based on height/width match.")
                    else:
                        self.get_logger().warning(f"Could not confidently determine model input format (NHWC/NCHW) from shape {model_input_shape}.")
                        self.input_format = None # Unknown format
                else:
                    self.get_logger().warning(f"Model input shape {model_input_shape} is not 4D. Cannot determine format.")
                    self.input_format = None

            # Verify model output shape against config
            model_output_shape = output_details.shape
             # Assuming ONNX output is (Batch, H_out, W_out, Anchors * Params) or (Batch, Anchors * Params, H_out, W_out)
            if len(model_output_shape) == 5 and \
                model_output_shape[1] == self.output_height and \
                model_output_shape[2] == self.output_width and \
                model_output_shape[3] == self.output_anchors and \
                model_output_shape[4] == self.params_per_anchor:
                    self.output_format = 'NHWAP'
                    self.get_logger().info(f"Model output format identified as 5D NHWAP: {model_output_shape}")


            self.input_name = input_details.name
            self.get_logger().info(f"Model output format assumed: {self.output_format}")


        except Exception as e:
            self.get_logger().error(f"Failed to load or verify ONNX model: {str(e)}")
            raise e

        self.bridge = CvBridge()

        # --- Service Definition ---
        self.run_model_service = self.create_service(
            DetectRobots, 
            'detect_robots_srv', 
            self.model_service_callback
        )
        self.get_logger().info(f"'/detect_robots_srv' service ready for robot detection.")

        # --- Visualization Publishers ---
        # Publisher for the combined overlay image with bounding boxes
        self.pub_overlay = self.create_publisher(Image, "/vision/debug/robot_overlay_combined", 10)


    def parse_config(self):
        config = {
            'inputHeight': 60,
            'inputWidth': 80,
            'inputChannels': 3,
            'outputHeight': 8,
            'outputWidth': 10,
            'outputAnchors': 4,
            'paramsPerAnchor': 5,
            'predictFallen': False,
            'confidenceIndex': 0,
            'yMidIndex': 1,
            'xMidIndex': 2,
            'heightIndex': 3,
            'widthIndex': 4,
            'fallenClassIndex': -1,
            'anchors': [
                {'x': 0.09658511133978298, 'y': 0.2370820816605168},
                {'x': 0.14000417622375075, 'y': 0.4056380034479811},
                {'x': 0.2177281673006002,  'y': 0.7012438628796819},
                {'x': 0.3954677177611361,  'y': 0.9756357911017483}
            ],
            'sizeConversionFactor': 8.0
        }

        return config


    def sigmoid(self, x):
        """Helper for sigmoid activation"""
        # Clamp input to avoid overflow with large negative numbers
        x = np.clip(x, -500, 500)
        return 1.0 / (1.0 + np.exp(-x))

    def decode_boxes(self, output_tensor, original_img_width, original_img_height):
        """
        Decodes bounding boxes from the model output tensor based on config.
        Each detection includes [x1, y1, x2, y2, confidence] and optionally [fallen_conf].
        """
        detections = [] # List to store decoded boxes: [x1, y1, x2, y2, confidence, fallen_conf (optional)]

        # Squeeze batch dimension
        output_tensor_single = output_tensor[0] # Shape (H_out, W_out, Anchors*Params) or (Anchors*Params, H_out, W_out)

        if self.output_format == 'NCHW':
             # Reshape from (C, H, W) to (H, W, C) for easier processing
             output_tensor_single = np.transpose(output_tensor_single, (1, 2, 0)) # Shape (H_out, W_out, Anchors*Params)

        output_tensor_reshaped = output_tensor_single
        # Scale factors from feature map grid to input image size
        # These are needed to convert grid cell coordinates to image pixel coordinates
        scale_x_to_input = original_img_width / self.output_width
        scale_y_to_input = original_img_height / self.output_height


        for h in range(self.output_height):
            for w in range(self.output_width):
                for a in range(self.output_anchors):
                    anchor_params = output_tensor_reshaped[h, w, a, :]

                    # Ensure we have enough parameters per anchor
                    if len(anchor_params) < self.params_per_anchor:
                        self.get_logger().warning(f"Anchor params length {len(anchor_params)} < expected {self.params_per_anchor} at grid cell ({w},{h}), anchor {a}. Skipping.")
                        continue

                    conf = self.sigmoid(anchor_params[self.confidence_idx])

                    if conf >= self.confidence_threshold:
                        # Decode box parameters
                        # Center (relative to grid cell, then scaled to input image pixels)
                        # The +0.5 adds the offset to the center of the grid cell
                        x_center_feature = self.sigmoid(anchor_params[self.x_mid_idx]) + w + 0.5
                        y_center_feature = self.sigmoid(anchor_params[self.y_mid_idx]) + h + 0.5

                        # Convert feature map coordinates to original image pixel coordinates
                        x_center_abs = x_center_feature * scale_x_to_input
                        y_center_abs = y_center_feature * scale_y_to_input


                        # Dimensions (relative to anchor dimensions, scaled by exp)
                        # Anchors from config are (x, y) which we interpret as (width, height)
                        # relative to the INPUT image size (0-1 range)
                        if a >= len(self.config['anchors']):
                             self.get_logger().warning(f"Anchor index {a} out of bounds for config anchors list ({len(self.config['anchors'])}). Cannot decode dimensions.")
                             continue

                        anchor_w_rel_input = self.config['anchors'][a]['x']
                        anchor_h_rel_input = self.config['anchors'][a]['y']

                        # Clamp exponent input to prevent overflow
                        exp_width_input = np.clip(anchor_params[self.width_idx], -10, 10) # Arbitrary clamp, adjust if needed
                        exp_height_input = np.clip(anchor_params[self.height_idx], -10, 10)


                        # Calculate box width and height in relative-to-input scale
                        box_w_rel_input = np.exp(exp_width_input) * anchor_w_rel_input
                        box_h_rel_input = np.exp(exp_height_input) * anchor_h_rel_input

                        # Convert relative-to-input dimensions to absolute pixel dimensions
                        box_w_abs = box_w_rel_input * original_img_width
                        box_h_abs = box_h_rel_input * original_img_height


                        # Convert center/width/height to (x1, y1, x2, y2) format
                        x1 = x_center_abs - box_w_abs / 2
                        y1 = y_center_abs - box_h_abs / 2
                        x2 = x_center_abs + box_w_abs / 2
                        y2 = y_center_abs + box_h_abs / 2

                        # Clamp coordinates to image bounds (important before NMS)
                        x1 = max(0.0, float(x1))
                        y1 = max(0.0, float(y1))
                        x2 = min(float(original_img_width), float(x2))
                        y2 = min(float(original_img_height), float(y2))


                        detection_data = [x1, y1, x2, y2, conf]

                        # --- Decode fallen status if applicable ---
                        fallen_conf = 0.0 # Default if not predicted or index invalid
                        if self.predict_fallen and \
                           self.fallen_class_idx != -1 and \
                           self.fallen_class_idx < self.params_per_anchor:
                             fallen_conf = self.sigmoid(anchor_params[self.fallen_class_idx])

                        detection_data.append(fallen_conf) # Append fallen_conf (0.0 if not available/predicted)


                        detections.append(detection_data)

        return detections

    def model_service_callback(self, req: DetectRobots.Request, res: DetectRobots.Response):
        self.get_logger().debug("Received service request.")
        start_time = time.time()

        # Initialize response fields
        self.create_empty_response(res) # Start fresh
        res.success = False # Assume failure initially
        res.message = "Processing failed" # Default message

        input_image_msg = req.image
        cv_image_raw_passthrough = None # Raw data from CV bridge
        cv_image_bgr_for_viz = None # BGR conversion for visualization
        cv_image_yuv_for_model = None # YUV 3-channel for model input

        if input_image_msg is None or not input_image_msg.data:
            res.message = "Request does not contain valid image data."
            self.get_logger().error(res.message)
            return res

        original_img_width = input_image_msg.width
        original_img_height = input_image_msg.height
        image_encoding = input_image_msg.encoding # Get the actual encoding

        # 1. Get the raw image data using passthrough
        cv_image_raw_passthrough = self.bridge.imgmsg_to_cv2(input_image_msg, desired_encoding='passthrough')
        self.get_logger().debug(f"Received image with encoding '{image_encoding}' and raw shape {cv_image_raw_passthrough.shape}")

        # --- Preprocessing ---
        # image_encoding == 'yuv422_yuy2'
        # 2. Convert packed YUYV to BGR (for visualization base)
        cv_image_bgr_for_viz = cv2.cvtColor(cv_image_raw_passthrough, cv2.COLOR_YUV2BGR_YUY2)
        self.get_logger().debug(f"Converted YUYV to BGR for visualization: shape {cv_image_bgr_for_viz.shape}")

        # 3. Convert BGR to YUV (full resolution 4:4:4 YUV) for model input
        cv_image_yuv_full_res = cv2.cvtColor(cv_image_bgr_for_viz, cv2.COLOR_BGR2YUV) # COLOR_YUV2BGR_YUY2 when turns off the vis
        self.get_logger().debug(f"Converted BGR to YUV 4:4:4 (full resolution): shape {cv_image_yuv_full_res.shape}")
        # Y, U, V order 
        cv_image_yuv_for_model = cv_image_yuv_full_res

        # --- Resize the YUV image to model input dimensions ---
        if cv_image_yuv_for_model.shape[0] != self.input_height or cv_image_yuv_for_model.shape[1] != self.input_width:
            self.get_logger().debug(f"Resizing YUV image from {cv_image_yuv_for_model.shape[:2]} to model input size ({self.input_height}, {self.input_width}).")
            cv_image_for_model_resized = cv2.resize(cv_image_yuv_for_model, (self.input_width, self.input_height), interpolation=cv2.INTER_LINEAR)
        else:
            self.get_logger().debug("YUV image size matches model input size.")
            cv_image_for_model_resized = cv_image_yuv_for_model

        # 4. Prepare input tensor: Convert to float32 and add batch dimension
        # Assuming model expects float32, raw 0-255 YUV values
        input_tensor = cv_image_for_model_resized.astype(np.float32)

        # Add batch dimension. Order needs to match model input format (NHWC or NCHW)
        if self.input_format == 'NCHW':
            input_batch = np.transpose(input_tensor, (2, 0, 1)) # Transpose HWC -> CHW
            input_batch = np.expand_dims(input_batch, axis=0) # Add batch dimension -> (1, C, H, W)
            self.get_logger().debug("Input tensor prepared in NCHW format.")
        elif self.input_format == 'NHWC':
            input_batch = np.expand_dims(input_tensor, axis=0) # Add batch dimension -> (1, H, W, C)
            self.get_logger().debug("Input tensor prepared in NHWC format.")


        self.get_logger().debug(f"Prepared input batch shape: {input_batch.shape}")

        # --- Run Inference ---
        try:
            ort_inputs = {self.input_name: input_batch}
            inference_start_time = time.time()
            outputs = self.model.run(None, ort_inputs)
            inference_time = time.time() - inference_start_time
            raw_model_output = outputs[0]
            self.get_logger().debug(f"Raw model output shape: {raw_model_output.shape}. Inference time: {inference_time:.4f}s")

        except Exception as e:
            res.message = f"ONNX Runtime Inference Error: {str(e)}"
            self.get_logger().error(res.message, exc_info=True)
            return res

        # --- Post-processing (Decoding and NMS) ---
        try:
            # Decode raw model output into potential bounding boxes
            # This part uses the original image dimensions for scaling results
            decoded_detections = self.decode_boxes(raw_model_output, original_img_width, original_img_height)
            self.get_logger().debug(f"Decoded {len(decoded_detections)} raw detections before NMS.")

            # Prepare data for NMS
            # _decode_boxes returns [x1, y1, x2, y2, confidence, fallen_conf]
            boxes = [d[:4] for d in decoded_detections]
            confidences = [float(d[4]) for d in decoded_detections]

            final_detections_raw = [] # Store detections including fallen_conf before creating ROS msgs
            if len(boxes) > 0:
                indices_to_keep = cv2.dnn.NMSBoxes(boxes, confidences, self.confidence_threshold, self.nms_threshold)
                if isinstance(indices_to_keep, np.ndarray):
                    # NMSBoxes can return a flat array of indices
                    final_detections_raw = [decoded_detections[i] for i in indices_to_keep.flatten()]

                self.get_logger().debug(f"Found {len(final_detections_raw)} robots after NMS.")
            else:
                self.get_logger().debug("No detections above confidence threshold before NMS.")


            # --- Populate Service Response (using the new structure) ---
            detected_robots_list = []
            for det_raw in final_detections_raw:
                # det_raw is [x1, y1, x2, y2, conf, fallen_conf]
                x1, y1, x2, y2, conf = det_raw[:5]

                robot_detection_msg = RobotDetection()
                robot_detection_msg.confidence = float(conf)
                robot_detection_msg.x1 = float(x1)
                robot_detection_msg.y1 = float(y1)
                robot_detection_msg.x2 = float(x2)
                robot_detection_msg.y2 = float(y2)

                # Handle optional fallen status
                if self.predict_fallen and len(det_raw) > 5:
                    fallen_conf = det_raw[5]
                    robot_detection_msg.is_fallen = fallen_conf > self.fallen_threshold
                # else: robot_detection_msg.is_fallen will be False by default

                detected_robots_list.append(robot_detection_msg)

            res.detections = detected_robots_list
            res.success = True # Indicate success after processing
            res.message = f"Processed successfully. Found {len(res.detections)} robots."


            # --- Generate Visualization (if anyone is subscribing) ---
            # Use the BGR image converted earlier for visualization
            if self.pub_overlay.get_subscription_count() > 0 and cv_image_bgr_for_viz is not None:
                 overlay_image = cv_image_bgr_for_viz.copy() # Use the BGR image as base

                 # Draw bounding boxes and confidence scores from the response list
                 for robot_det_msg in res.detections: # Iterate through the final response list
                      x1, y1, x2, y2 = int(robot_det_msg.x1), int(robot_det_msg.y1), int(robot_det_msg.x2), int(robot_det_msg.y2)
                      conf = robot_det_msg.confidence

                      # Determine color based on fallen status
                      box_color = (0, 0, 255) # Default Red for upright/unknown
                      label = f"{conf:.2f}"
                      if hasattr(robot_det_msg, 'is_fallen'): # Check if the field exists in the msg definition
                          if robot_det_msg.is_fallen:
                              box_color = (255, 0, 0) # Blue for Fallen
                              label += " (Fallen)"
                          else:
                              label += " (Upright)"


                      # Draw rectangle
                      cv2.rectangle(overlay_image, (x1, y1), (x2, y2), box_color, 2)
                      # Add label
                      text_y = max(15, y1 - 5) # Position text slightly above the box
                      cv2.putText(overlay_image, label, (x1, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, box_color, 1)

                 # Publish overlay image
                 try:
                    overlay_msg = self.bridge.cv2_to_imgmsg(overlay_image, "bgr8")
                    overlay_msg.header = input_image_msg.header # Use original header
                    self.pub_overlay.publish(overlay_msg)
                 except Exception as pub_e:
                    self.get_logger().error(f"Failed to publish overlay image: {pub_e}")
            elif self.pub_overlay.get_subscription_count() > 0 and cv_image_bgr_for_viz is None:
                 self.get_logger().warning("Cannot publish overlay image because BGR conversion failed or BGR image was not created.")


            total_time = time.time() - start_time
            self.get_logger().debug(f"Service request processed in {total_time:.4f}s.")

        except Exception as e:
            res.message = f"Error during post-processing/visualization: {str(e)}"
            self.get_logger().error(res.message, exc_info=True)
            # res.success is already False from initialization
            return res # Return response with success=False and error message

        return res # Return the populated response object with success=True

    # Helper to initialize response fields (using the new service type)
    def create_empty_response(self, res: DetectRobots.Response):
        res.success = False
        res.message = ""
        res.detections = []
        return res

def main(args=None):
    rclpy.init(args=args)
    robot_detection_node = None
    robot_detection_node = RobotDetectionNode()
    rclpy.spin(robot_detection_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

    