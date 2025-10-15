#include "motion/generator/RefPickupGenerator.hpp"
#include "motion/generator/ActionGenerator.hpp"
#include "utils/angles.hpp"
#include "utils/body.hpp"

RefPickupGenerator::RefPickupGenerator(const rclcpp::Node::SharedPtr & node)
: Generator(node), t(100), stopping(false), stopped(false)
{
  RCLCPP_INFO(motion_node_->get_logger(), "RefPickupGenerator constructed");
  standGen = (Generator *)(new ActionGenerator(node, "standing-to/standStraight"));
}

RefPickupGenerator::~RefPickupGenerator()
{
  RCLCPP_INFO(motion_node_->get_logger(), "RefPickupGenerator destroyed");
  delete standGen;
}

JointValues RefPickupGenerator::makeJoints(MakeJointsIn & makeJointsIn)
{
  if (stopping) {
    if (!(--t)) {
      stopping = !(stopped = true);
    }
  }
  return standGen->makeJoints(makeJointsIn);
}

bool RefPickupGenerator::isActive()
{
  return !stopped || standGen->isActive();
}

void RefPickupGenerator::reset()
{
  t = 100;
  stopping = stopped = false;
  standGen->reset();
}

void RefPickupGenerator::stop()
{
  standGen->stop();
  stopping = true;
}

void RefPickupGenerator::readParameters()
{
  standGen->readParameters();
}
