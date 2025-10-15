#pragma once

#include "motion/generator/Generator.hpp"

/**
 * HeadGenerator - a Generator implementation that only moves the Head.
 * Optionally uses a PID conroller to smooth the head movements.
 */
class HeadGenerator : Generator
{
public:
  explicit HeadGenerator(const rclcpp::Node::SharedPtr & node);
  ~HeadGenerator();
  virtual JointValues makeJoints(MakeJointsIn & makeJointsIn);
  virtual bool isActive();
  void reset();

private:
  float yaw;
  float pitch;
};
