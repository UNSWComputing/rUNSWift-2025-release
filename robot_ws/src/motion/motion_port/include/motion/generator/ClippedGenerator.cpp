#include "motion/generator/ClippedGenerator.hpp"
#include "utils/body.hpp"
#include "utils/clip.hpp"

ClippedGenerator::ClippedGenerator(const rclcpp::Node::SharedPtr & node, Generator * g)
: Generator(node), generator(g),
  old_exists(false)
{
  RCLCPP_INFO(
    motion_node_->get_logger(), "ClippedGenerator constructed");
}

ClippedGenerator::~ClippedGenerator()
{
  delete generator;
  RCLCPP_INFO(
    motion_node_->get_logger(), "ClippedGenerator destroyed");
}

bool ClippedGenerator::isActive()
{
  return generator->isActive();
}

void ClippedGenerator::reset()
{
  generator->reset();
  old_exists = false;
}

void ClippedGenerator::readParameters()
{
  generator->readParameters();
}

JointValues ClippedGenerator::makeJoints(MakeJointsIn & makeJointsIn)
{
  JointValues j = generator->makeJoints(makeJointsIn);
  for (uint8_t i = 0; i < Joints::NUMBER_OF_JOINTS; ++i) {
    // Clip stifnesses
    if (j.stiffnesses[i] >= 0.0f) {
      j.stiffnesses[i] = CLIP(j.stiffnesses[i], 0.0f, 1.0f);
    } else {
      j.stiffnesses[i] = -1.0f;
    }

    // Clip angles
    if (!std::isnan(j.angles[i])) {
      j.angles[i] = Joints::limitJointRadians(
        Joints::jointCodes[i],
        j.angles[i]);
    }

    // Clip velocities
    if (old_exists) {
      j.angles[i] = CLIP(
        j.angles[i],
        old_j.angles[i] - Joints::Radians::MaxSpeed[i],
        old_j.angles[i] + Joints::Radians::MaxSpeed[i]);
    }
  }
  old_exists = true;
  old_j = j;
  return j;
}
