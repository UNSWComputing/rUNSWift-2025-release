#pragma once

#include "motion/generator/Generator.hpp"

class RefPickupGenerator : Generator
{
public:
  explicit RefPickupGenerator(const rclcpp::Node::SharedPtr & node);
  ~RefPickupGenerator();
  virtual JointValues makeJoints(MakeJointsIn & makeJointsIn);
  virtual bool isActive();
  void reset();
  void stop();
  void readParameters();

private:
  int t;
  bool stopping;
  bool stopped;
  Generator * standGen;
};
