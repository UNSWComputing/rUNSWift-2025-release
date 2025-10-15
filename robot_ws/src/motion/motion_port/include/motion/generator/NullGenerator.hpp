#pragma once

#include "motion/generator/Generator.hpp"

class NullGenerator : Generator
{
public:
  explicit NullGenerator(const rclcpp::Node::SharedPtr & node);
  ~NullGenerator();
  virtual JointValues makeJoints(MakeJointsIn & makeJointsIn);
  virtual bool isActive();
  void reset();
};
