#pragma once

#include <string>
#include <vector>
#include "motion/generator/Generator.hpp"
#include "motion/MotionOdometry.hpp"

using BodyCommand = runswift_interfaces::msg::BodyCommand;

/* Determine whether the class is just called */
#define NOT_RUNNING -1

/* Hack the stiffness */
#define MAX_STIFF 1.0

class ActionGenerator : public Generator
{
public:
  explicit ActionGenerator(const rclcpp::Node::SharedPtr & node, std::string filename);
  ~ActionGenerator();

  virtual JointValues makeJoints(MakeJointsIn & makeJointsIn);
  virtual bool isActive();
  void reset();
  void stop();
  void readParameters();

private:
  int current_time;
  std::string file_name;
  std::vector<JointValues> joints;
  BodyCommand active;
  MotionOdometry motionOdometry;
  std::string file_path_ = "/home/nao/data/pos/legacy-port";

  /**
     * Determine the duration before the robot
     * actually begins to execute the sequence
     * of poses
     */
  int max_iter;

  /**
     * Interpolate the time that are determined by the
     * duration between the new joints with the previous
     * joints that are read in the file
     * @param newJoint the value of the joint that need to be
     * interpolated with the previous value
     * @param duration if duration = 0, it will do the interpolation
     * between the newJoint value with the joint at MAX_ITER position.
     * Otherwise, it will interpolate between the new joint and the previous
     * joint.
     */
  void interpolate(JointValues newJoint, int duration = 0);

  /**
     * Reading the file from the provided path and construct
     * the pose
     * @param path the directory to read the pose file
     */
  void constructPose(std::string path);
};
