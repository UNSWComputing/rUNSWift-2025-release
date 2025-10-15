#pragma once

#include "motion/generator/Generator.hpp"

class ClippedGenerator : Generator
{
public:
  explicit ClippedGenerator(const rclcpp::Node::SharedPtr & node, Generator * g);
  ~ClippedGenerator();
  virtual JointValues makeJoints(MakeJointsIn & makeJointsIn);
  virtual bool isActive();
  void reset();
  void readParameters();

private:
  Generator * generator;
  JointValues old_j;
  bool old_exists;
};
