#pragma once
#include "motion/generator/Generator.hpp"
#include "motion/generator/Walk2014Generator.hpp"
#include "utils/Timer.hpp"

class WalkEnginePreProcessor : Generator
{
public:
  explicit WalkEnginePreProcessor(const rclcpp::Node::SharedPtr & node);
  ~WalkEnginePreProcessor();
  JointValues makeJoints(MakeJointsIn & makeJointsIn);
  bool isActive();
  void readParameters();
  void reset();
  void stop();

private:
  bool isKicking;
  Walk2014Generator * walkEngine;
};
