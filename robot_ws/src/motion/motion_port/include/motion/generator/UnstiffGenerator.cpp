#include "motion/generator/UnstiffGenerator.hpp"

UnstiffGenerator::UnstiffGenerator(const rclcpp::Node::SharedPtr & node)
: Generator(node), ticks(0)
{
  RCLCPP_INFO(
    motion_node_->get_logger(), "UnstiffGenerator constructed");
}

UnstiffGenerator::~UnstiffGenerator()
{
  RCLCPP_INFO(
    motion_node_->get_logger(), "UnstiffGenerator destroyed");
}

JointValues UnstiffGenerator::makeJoints(MakeJointsIn & makeJointsIn)
{
  const SensorValues & sensors = makeJointsIn.sensors;

  ++ticks;
  JointValues j = sensors.joints;
  for (uint8_t i = 0; i < Joints::NUMBER_OF_JOINTS; ++i) {
    j.stiffnesses[i] = 0.0f;
  }
  return j;
}

bool UnstiffGenerator::isActive()
{
  return ticks <= MIN_TICKS;
}

void UnstiffGenerator::reset()
{
  ticks = 0;
}
