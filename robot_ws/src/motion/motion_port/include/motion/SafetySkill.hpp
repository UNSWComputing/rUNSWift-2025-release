#pragma once

#include "runswift_interfaces/msg/motion_command.hpp"
#include "types/SensorValues.hpp"
#include <rclcpp/rclcpp.hpp>

using MotionCommand = runswift_interfaces::msg::MotionCommand;

class SafetySkill
{
public:
  SafetySkill(const rclcpp::Node::SharedPtr & node);
  ~SafetySkill();
  void wrapRequest(MotionCommand & request, const SensorValues & s);
  void readParameters();

protected:
  rclcpp::Node::SharedPtr motion_node_;

private:
  float filtered_fsr_sum;
  float sag_angular_velocity;
  float cor_angular_velocity;
  float prev_angles[2];
  int fallingCounter;
  bool useGetups;
  bool simulation;
  bool ukemiEnabled;
};
