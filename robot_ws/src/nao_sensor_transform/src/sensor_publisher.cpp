// Copyright 2021 Kenji Brameld
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "nao_sensor_transform/sensor_publisher.hpp"
#include "nao_sensor_msgs/msg/joint_indexes.hpp"

namespace nao_sensor_transform {
std::vector<std::string> joint_names = {
  "HeadYaw",
  "HeadPitch",
  "LShoulderPitch",
  "LShoulderRoll",
  "LElbowYaw",
  "LElbowRoll",
  "LWristYaw",
  "LHipYawPitch",
  "LHipRoll",
  "LHipPitch",
  "LKneePitch",
  "LAnklePitch",
  "LAnkleRoll",
  "RHipRoll",
  "RHipPitch",
  "RKneePitch",
  "RAnklePitch",
  "RAnkleRoll",
  "RShoulderPitch",
  "RShoulderRoll",
  "RElbowYaw",
  "RElbowRoll",
  "RWristYaw",
  "LHand",
  "RHand",
};

SensorPublisherNode::SensorPublisherNode(const rclcpp::NodeOptions & options)
: rclcpp::Node{"SensorPubisherNode", options}
{
  pub_joint_states = create_publisher<sensor_msgs::msg::JointState>(
    "joint_states", 1);

  sub_joint_positions =
    create_subscription<nao_sensor_msgs::msg::JointPositions>(
    "sensors/joint_positions", 1,
    [this](nao_sensor_msgs::msg::JointPositions::SharedPtr sensor_joint_positions) {
      auto joint_state_msg = std::make_unique<sensor_msgs::msg::JointState>();
    
      // Set joint names and positions
      joint_state_msg->name = joint_names;
      std::vector<double> joint_positions_vector(sensor_joint_positions->positions.begin(), sensor_joint_positions->positions.end());

      joint_state_msg->position = joint_positions_vector;

      // TODO: Set other fields as needed (velocity, effort, etc.)

      joint_state_msg->header.stamp = this->now();

      this->pub_joint_states->publish(std::move(joint_state_msg));
//       RCLCPP_DEBUG(
//         this->get_logger(), ("headyaw " +
//         std::to_string(sensor_joints->positions[nao_sensor_msgs::msg::JointIndexes::HEADYAW]
// )).c_str());
      });

}
SensorPublisherNode::~SensorPublisherNode() {}

}
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nao_sensor_transform::SensorPublisherNode)
