#pragma once
#include <rclcpp/rclcpp.hpp>
#include "types/JointValues.hpp"
#include "types/MakeJointsIn.hpp"

/**
 * Generator - responsible for transforming a MotionCommand into joint angles.
 * May consider feedback from sensors.
 */
class Generator
{
public:
  Generator(const rclcpp::Node::SharedPtr & node)
  : motion_node_(node) {}

  virtual ~Generator() {}
  /**
     * makeJoints - generate joint angles & stiffnesses for next cycle
     * @return Values of joint actuators in next cycle
     */
  virtual JointValues makeJoints(MakeJointsIn & makeJointsIn) = 0;
  /**
     * isActive - informs the parent of whether the robot is in a neutral stance
     *            (i.e., whether a different generator can be switched to)
     */
  virtual bool isActive() = 0;
  /**
     * reset - return to neutral stance and become inactive immediately
     */
  virtual void reset() = 0;

  /**
     * stop - like reset, but time is given to transition to the neutral stance
     */
  virtual void stop()       /* defaults to reset() - can be overridden */
  {
  }
  virtual void readParameters() {}

protected:
  rclcpp::Node::SharedPtr motion_node_;
};
