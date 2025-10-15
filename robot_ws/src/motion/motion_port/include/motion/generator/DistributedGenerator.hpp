#pragma once

#include "motion/generator/Generator.hpp"

class DistributedGenerator : Generator
{
public:
  explicit DistributedGenerator(const rclcpp::Node::SharedPtr & node, bool simulation);
  ~DistributedGenerator();
  virtual JointValues makeJoints(MakeJointsIn & makeJointsIn);
  virtual bool isActive();
  void reset();
  void readParameters();
  bool isStopping;

private:
  std::unordered_map<std::string, Generator *> bodyGenerators;
  Generator * headGenerator;
  std::string current_generator;
  std::string prev_generator;
  std::string requestedDive;
};
