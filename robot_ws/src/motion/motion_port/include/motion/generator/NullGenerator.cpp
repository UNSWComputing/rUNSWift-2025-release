#include "motion/generator/NullGenerator.hpp"

NullGenerator::NullGenerator(const rclcpp::Node::SharedPtr & node)
: Generator(node)
{
  RCLCPP_INFO(
    motion_node_->get_logger(), "NullGenerator constructed");
}

NullGenerator::~NullGenerator()
{
  RCLCPP_INFO(
    motion_node_->get_logger(), "NullGenerator destroyed");
}

JointValues NullGenerator::makeJoints(MakeJointsIn & makeJointsIn)
{
  const SensorValues & sensors = makeJointsIn.sensors;
  JointValues j = sensors.joints;
  for (uint8_t i = 0; i < Joints::NUMBER_OF_JOINTS; ++i) {
    j.stiffnesses[i] = 0.0f;
  }
  return j;
}

bool NullGenerator::isActive()
{
  return false;
}

void NullGenerator::reset() {}
