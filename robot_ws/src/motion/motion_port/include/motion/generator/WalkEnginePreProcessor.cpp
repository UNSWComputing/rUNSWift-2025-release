#include "motion/generator/WalkEnginePreProcessor.hpp"
#include "runswift_interfaces/msg/body_command.hpp"

using MotionCommand = runswift_interfaces::msg::MotionCommand;
using namespace std;
using BodyCommand = runswift_interfaces::msg::BodyCommand;

void toWalkRequest(MotionCommand * request)
{
  request->body_command.action_type = BodyCommand::WALK;
  request->body_command.power = 0;
  request->body_command.bend = 1;
  request->body_command.speed = 1;
}

WalkEnginePreProcessor::WalkEnginePreProcessor(const rclcpp::Node::SharedPtr & node)
: Generator(node)
{
  walkEngine = new Walk2014Generator(node);
  isKicking = false;
}

WalkEnginePreProcessor::~WalkEnginePreProcessor()
{
  delete walkEngine;
}

JointValues WalkEnginePreProcessor::makeJoints(MakeJointsIn & makeJointsIn)
{
  MotionCommand * request = makeJointsIn.request;
  auto active = request->body_command.action_type;

  if (request->body_command.action_type == BodyCommand::KICK) {
    request->body_command.twist.angular.z = 0;      // 0 the turn used for line up heading adjustments
    request->body_command.speed = 0;     // reverted overloaded param for turn threshold
  }

  JointValues joints = walkEngine->makeJoints(makeJointsIn);

  // walkEngine sets kick to walk2014 after kick finishes
  if (walkEngine->active.action_type == BodyCommand::KICK &&
    request->body_command.action_type == BodyCommand::WALK)
  {
    isKicking = false;
  } else {
    request->body_command.action_type = active;
  }

  return joints;
}

bool WalkEnginePreProcessor::isActive()
{
  return walkEngine->isActive();
}

void WalkEnginePreProcessor::readParameters()
{
  walkEngine->readParameters();
}

void WalkEnginePreProcessor::reset()
{
  walkEngine->reset();
  isKicking = false;
}

void WalkEnginePreProcessor::stop()
{
  walkEngine->stop();
}
