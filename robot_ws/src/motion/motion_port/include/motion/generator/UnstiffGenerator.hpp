#pragma once

#include "motion/generator/Generator.hpp"

#define MIN_TICKS 100

class UnstiffGenerator : Generator
{
public:
  explicit UnstiffGenerator(const rclcpp::Node::SharedPtr & node);
  ~UnstiffGenerator();
  virtual JointValues makeJoints(MakeJointsIn & makeJointsIn);
  virtual bool isActive();
  void reset();

private:
  int ticks;
};
