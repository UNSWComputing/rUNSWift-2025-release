#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from vision_msgs.msg import Point2D
import os
from std_msgs.msg import String
from runswift_interfaces.srv import BallCls  # You'll need to modify this service definition
import numpy as np
# import torch
# from torch import nn
# from torch.nn import functional as F
import onnxruntime



# --- Helper Functions ---

def conv1x1(in_planes, out_planes, stride=1):
  """1x1 convolution (pointwise)"""
  return nn.Conv2d(in_planes, out_planes, kernel_size=1, stride=stride, bias=False)

def conv3x3_dw(in_planes, stride=1):
  """3x3 depthwise convolution"""
  return nn.Conv2d(in_planes, in_planes, kernel_size=3, stride=stride, padding=1, groups=in_planes, bias=False)

# --- Inverted Bottleneck Block ---

# class InvertedBottleneck(nn.Module):
#   def __init__(self, in_planes, out_planes, stride=1, expansion=3):
#     super(InvertedBottleneck, self).__init__()
#     self.stride = stride
#     self.use_expansion = (expansion > 1) and (in_planes != out_planes)
#     self.use_skip_connection = self.stride == 1 and in_planes == out_planes

#     if self.use_expansion:
#       expanded_planes = in_planes * expansion
#       self.expand = nn.Sequential(
#           conv1x1(in_planes, expanded_planes),
#           nn.BatchNorm2d(expanded_planes),
#           nn.SiLU(inplace=True)
#       )
#     else:
#         expanded_planes = in_planes

#     self.depthwise = nn.Sequential(
#         conv3x3_dw(expanded_planes, stride=stride),
#         nn.BatchNorm2d(expanded_planes),
#         nn.SiLU(inplace=True)
#     )

#     self.contract = nn.Sequential(
#         conv1x1(expanded_planes, out_planes),
#         nn.BatchNorm2d(out_planes)
#     ) if in_planes != out_planes else nn.Identity()

#     self.silu = nn.SiLU(inplace=True)

#   def forward(self, x):
#     identity = x

#     if self.use_expansion:
#         out = self.expand(x)
#     else:
#         out = x

#     out = self.depthwise(out)
#     out = self.contract(out)

#     if self.use_skip_connection:
#       out += identity
#     else:
#         out = self.silu(out)

#     return out

# # --- Encoder ---

# class Encoder(nn.Module):
#   def __init__(self, in_channels=1, stem_channels=8, output_channels_block1=16, output_channels_block2=32, output_channels_block3=16, output_channels_block4=8):
#     super(Encoder, self).__init__()

#     self.stem = nn.Sequential(
#         conv1x1(in_channels, stem_channels),
#         nn.BatchNorm2d(stem_channels),
#         nn.SiLU(inplace=True),
#         conv3x3_dw(stem_channels, stride=2),
#         nn.BatchNorm2d(stem_channels),
#         nn.SiLU(inplace=True),
#         conv1x1(stem_channels, output_channels_block1),
#         nn.BatchNorm2d(output_channels_block1),
#         nn.SiLU(inplace=True),
#         conv3x3_dw(output_channels_block1, stride=2),
#         nn.BatchNorm2d(output_channels_block1),
#         nn.SiLU(inplace=True)
#     )

#     self.layer1 = self._make_layer(in_planes=output_channels_block1, out_planes=output_channels_block1, num_blocks=2, stride=1, expansion=2)
#     self.layer2 = self._make_layer(in_planes=output_channels_block1, out_planes=output_channels_block2, num_blocks=1, stride=2, expansion=4) # Strided block
#     self.layer3 = self._make_layer(in_planes=output_channels_block2, out_planes=output_channels_block3, num_blocks=2, stride=1, expansion=3)
#     self.layer4 = self._make_layer(in_planes=output_channels_block3, out_planes=output_channels_block4, num_blocks=1, stride=2, expansion=4)
#     self.output_channels = output_channels_block4

#   def _make_layer(self, in_planes, out_planes, num_blocks, stride, expansion):
#     strides = [stride] + [1]*(num_blocks-1)
#     layers = []
#     for stride in strides:
#       layers.append(InvertedBottleneck(in_planes, out_planes, stride, expansion))
#       in_planes = out_planes
#     return nn.Sequential(*layers)

#   def forward(self, x):
#     x = self.stem(x)
#     x = self.layer1(x)
#     x = self.layer2(x)
#     x = self.layer3(x)
#     x = self.layer4(x)
#     return x

# # --- Classification Decoder ---

# class ClassificationDecoder(nn.Module):
#   def __init__(self, in_planes, num_classes=3, num_dw_convs=1):
#     super(ClassificationDecoder, self).__init__()

#     self.conv1 = conv1x1(in_planes, in_planes)
#     self.bn1 = nn.BatchNorm2d(in_planes)
#     self.silu1 = nn.SiLU(inplace=True)
#     self.flatten = nn.Flatten()
#     self.linear = nn.Linear(12,3)
#     # Optional Depthwise Convolutions
#     dw_layers = []
#     for _ in range(num_dw_convs):
#         dw_layers.append(conv3x3_dw(in_planes))
#         dw_layers.append(nn.BatchNorm2d(in_planes))
#         dw_layers.append(nn.SiLU(inplace=True))
#     self.dw_convs = nn.Sequential(*dw_layers)

#     self.conv2 = conv1x1(in_planes, num_classes)

#   def forward(self, x):
#     x = self.conv1(x)
#     x = self.bn1(x)
#     x = self.silu1(x)
#     x = self.dw_convs(x)
#     x = self.conv2(x)
#     x = self.flatten(x)
#     x = self.linear(x)  
#     return x

# # --- Regression Decoder ---

# class RegressionDecoder(nn.Module):
#   def __init__(self, in_planes, num_outputs=3, num_dw_convs=1):
#     super(RegressionDecoder, self).__init__()

#     self.conv1 = conv1x1(in_planes, in_planes)
#     self.bn1 = nn.BatchNorm2d(in_planes)
#     self.silu1 = nn.SiLU(inplace=True)
#     self.flatten = nn.Flatten()
#     self.linear = nn.Linear(12,3) 
#     # Optional Depthwise Convolutions
#     dw_layers = []
#     for _ in range(num_dw_convs):
#         dw_layers.append(conv3x3_dw(in_planes))
#         dw_layers.append(nn.BatchNorm2d(in_planes))
#         dw_layers.append(nn.SiLU(inplace=True))
#     self.dw_convs = nn.Sequential(*dw_layers)

#     self.conv2 = conv1x1(in_planes, num_outputs)

#   def forward(self, x):
#     x = self.conv1(x)
#     x = self.bn1(x)
#     x = self.silu1(x)
#     x = self.dw_convs(x)
#     x = self.conv2(x)
#     x = self.flatten(x)
#     x = self.linear(x) 
#     return x

# # --- Main Model ---

# class LightweightBallDetector3(nn.Module):
#   def __init__(self):
#     super(LightweightBallDetector3, self).__init__()
#     self.encoder = Encoder()
#     self.classification_decoder = ClassificationDecoder(self.encoder.output_channels, num_classes=3)
#     self.regression_decoder = RegressionDecoder(self.encoder.output_channels, num_outputs=3)
#     self.num_classes = 3

#   def forward(self, x):
#     encoded_features = self.encoder(x)
#     class_output = self.classification_decoder(encoded_features)
#     reg_output = self.regression_decoder(encoded_features)

#     # Concatenate class and regression outputs along the channel dimension
#     combined_output = torch.cat([class_output, reg_output], dim=1)


#     return combined_output



class BallDetectorModel(Node):
    def __init__(self):
        super().__init__('ball_detector_model')
        """
        Load the model using torch
        """
        # self.t_model = LightweightBallDetector3()
        # model_dict = torch.load("/home/nao/vision_models/ball_penalty_model (2).pth", map_location=torch.device('cpu')) # on the robot
        # #model_dict = torch.load("/workspace/vision_models/ball_penalty_model (2).pth", map_location=torch.device('cpu')) # on docker
        # self.t_model.load_state_dict(model_dict)
        # self.t_model.eval()

        """
        Load the model using onnxruntime
        """
        options = onnxruntime.SessionOptions()
        options.inter_op_num_threads = 1
        options.intra_op_num_threads = 1
        robot_path = "/home/nao/vision_models/ball_detector (1).onnx"
        dev_path = "/workspace/vision_models/ball_detector (1).onnx" # in dev docker container
        if os.path.exists(robot_path):
           self.t_model = onnxruntime.InferenceSession(robot_path, sess_options=options)
        elif os.path.exists(dev_path):
            self.t_model = onnxruntime.InferenceSession(dev_path, sess_options=options)
        else:
            raise FileNotFoundError("Model not found")
        
        self.bridge = CvBridge()
    
        self.run_model = self.create_service(
            BallCls, 
            'ball_cls_srv', 
            self.image_srv
        )



    def image_srv(self, req, res):
      if len(req.images) == 0:
        return res
      try:
          # Convert all images at once
          cv_images = []
          for img_msg in req.images:
              cv_img = self.bridge.imgmsg_to_cv2(img_msg, 'mono8')
              cv_images.append(cv_img)

          # Preprocess batch
          batch = np.stack(cv_images, axis=0).astype(np.float32) / 255.0
          batch = batch[:, np.newaxis, :, :]  # Add channel dimension

          # Run inference on batch
          ort_inputs = {self.t_model.get_inputs()[0].name: batch}
          outputs = self.t_model.run(None, ort_inputs)[0]

          # Post-process batch
          class_logits = outputs[:, :3]
          class_probs = np.exp(class_logits) / np.sum(np.exp(class_logits), axis=1)[:, np.newaxis]
          
          # Extend the response lists with batch results
          res.cls1_conf.extend(class_probs[:, 0].tolist())
          res.cls2_conf.extend(class_probs[:, 1].tolist())
          res.cls3_conf.extend(class_probs[:, 2].tolist())
          res.xs = outputs[:, 3].tolist()
          res.ys = outputs[:, 4].tolist()
          res.ball_pixel_radius.extend(outputs[:, 5].tolist())

      except Exception as e:
          raise e
          self.get_logger().error(f"Error in batch processing: {str(e)}")
          return res


      return res
    
def main(args=None):
    rclpy.init(args=args)
    feature_detector = BallDetectorModel()
    rclpy.spin(feature_detector)
    feature_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()