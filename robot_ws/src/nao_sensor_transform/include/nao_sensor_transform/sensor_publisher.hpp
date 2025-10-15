// Copyright 2023 Mikhail Asavkin
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

#ifndef NAO_SENSOR_TRANSFORM__SENSOR_PUBLISHER_HPP_
#define NAO_SENSOR_TRANSFORM__SENSOR_PUBLISHER_HPP_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "nao_sensor_msgs/msg/joint_positions.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/time.hpp"


namespace nao_sensor_transform
{

class SensorPublisherNode : public rclcpp::Node
{
public:
  explicit SensorPublisherNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});
  virtual ~SensorPublisherNode();

private:
  
  rclcpp::Subscription<nao_sensor_msgs::msg::JointPositions>::SharedPtr sub_joint_positions;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_states;
  
  rclcpp::Time begin;
};

}  // namespace nao_sensor_transform

#endif  // NAO_SENSOR_TRANSFORM__SENSOR_PUBLISHER_HPP_
