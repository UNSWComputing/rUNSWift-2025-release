#include <fstream>
#include <limits>
#include <cctype>
#include "motion/generator/ActionGenerator.hpp"
#include "utils/angles.hpp"
#include <boost/algorithm/string.hpp>
#include "../MotionDefs.hpp"

using namespace std;

ActionGenerator::ActionGenerator(const rclcpp::Node::SharedPtr & node, std::string filename)
: Generator(node), file_name(filename)
{
  max_iter = 0;
  current_time = NOT_RUNNING;
}

ActionGenerator::~ActionGenerator()
{
  RCLCPP_INFO(motion_node_->get_logger(), "ActionGenerator destroyed");
}

bool ActionGenerator::isActive()
{
  return current_time != NOT_RUNNING;
}

void ActionGenerator::reset()
{
  current_time = 0;
  motionOdometry.reset();
}

void ActionGenerator::stop()
{
}

JointValues ActionGenerator::makeJoints(MakeJointsIn & makeJointsIn)
{
  MotionCommand * request = makeJointsIn.request;
  Odometry * odometry = makeJointsIn.odometry;
  const SensorValues & sensors = makeJointsIn.sensors;

  JointValues j;
  if (current_time == NOT_RUNNING) {
    active = request->body_command;
    j = joints[joints.size() - 1];
  } else {
    JointValues newJoints = sensors.joints;
    for (int i = 0; i < Joints::NUMBER_OF_JOINTS; i++) {
      newJoints.stiffnesses[i] = 1.0f;
    }
    if (current_time == 0) {interpolate(newJoints);}
    j = joints[current_time++];
    if (current_time == (signed int)joints.size()) {  // if we just did last action
      current_time = NOT_RUNNING;
    }
  }

  // Update odometry, (important for gyroscopez)
  *odometry = *odometry + motionOdometry.updateOdometry(sensors, Odometry());

  return j;
}

void ActionGenerator::interpolate(JointValues newJoint, int duration)
{
  if (joints.empty()) {
    max_iter = duration / (1000.0 * MOTION_DT);
    // Reserve space for the interpolation when the generator
    // first called
    for (int i = 0; i < max_iter; i++) {
      joints.push_back(newJoint);
    }
    joints.push_back(newJoint);
  } else {
    int inTime = 0;
    float offset[Joints::NUMBER_OF_JOINTS];

    if (duration != 0) {
      inTime = duration / (1000.0 * MOTION_DT);
      JointValues currentJoint = joints.back();

      // Calculate the difference between new joint and the previous joint
      for (int i = 0; i < Joints::NUMBER_OF_JOINTS; i++) {
        offset[i] = (newJoint.angles[i] - currentJoint.angles[i]) / inTime;
      }

      for (int i = 0; i < inTime; i++) {
        JointValues inJoint;
        for (int j = 0; j < Joints::NUMBER_OF_JOINTS; j++) {
          inJoint.angles[j] = joints.back().angles[j] + offset[j];
          inJoint.stiffnesses[j] = newJoint.stiffnesses[j];
        }
        joints.push_back(inJoint);
      }
    } else {

      JointValues firstJoint = joints.at(max_iter);
      // Calculate the difference between the joint at MAX_ITER position
      // with the new joint
      for (int i = 0; i < Joints::NUMBER_OF_JOINTS; i++) {
        offset[i] = (firstJoint.angles[i] - newJoint.angles[i]) / max_iter;
      }

      joints[0] = newJoint;
      for (int i = 1; i < max_iter; i++) {
        for (int j = 0; j < Joints::NUMBER_OF_JOINTS; j++) {
          joints[i].angles[j] = joints[i - 1].angles[j] + offset[j];
          joints[i].stiffnesses[j] = firstJoint.stiffnesses[j];
        }
      }
    }
  }
}

void ActionGenerator::constructPose(std::string path)
{

  char hostname[HOST_NAME_MAX] = {0};
  gethostname(hostname, HOST_NAME_MAX);

  std::ostringstream buffer;
  for (int i = 0; i < HOST_NAME_MAX; ++i) {
    if (hostname[i]) {buffer << hostname[i];}
  }
  std::string checkHostName = buffer.str();


  ifstream in;
  in.open(string(path + "/" + file_name + ".pos").c_str());

  if (!in.is_open()) {
    RCLCPP_FATAL(
      motion_node_->get_logger(), "ActionGenerator can not open %s", file_name.c_str());
  } else {
    JointValues newJoint(0);

    // Set to default 1 stiffness
    for (unsigned i = 0; i < Joints::NUMBER_OF_JOINTS; ++i) {
      newJoint.stiffnesses[i] = 1.0f;
    }

    while (!in.eof()) {

      std::string line;
      std::getline(in, line);

      if (boost::starts_with(line, "!")) {
        boost::erase_all(line, "!");

        std::istringstream ss(line);

        float angle;
        for (unsigned i = 0; i < Joints::NUMBER_OF_JOINTS; ++i) {
          ss >> angle;
          newJoint.angles[i] = DEG2RAD(angle);
        }

        int duration;
        ss >> duration;

        interpolate(newJoint, duration);

        // Set to default 1 stiffness
        for (unsigned i = 0; i < Joints::NUMBER_OF_JOINTS; ++i) {
          newJoint.stiffnesses[i] = 1.0f;
        }
      }

      if (boost::starts_with(line, "$")) {
        boost::erase_all(line, "$");

        std::istringstream ss(line);

        float stiffness;
        for (unsigned i = 0; i < Joints::NUMBER_OF_JOINTS; ++i) {
          ss >> stiffness;
          newJoint.stiffnesses[i] = stiffness;
        }
      }
    }
    in.close();
  }
  RCLCPP_INFO(
    motion_node_->get_logger(), "ActionGenerator(%s) created", file_name.c_str());
}

void ActionGenerator::readParameters()
{
  file_path_ = motion_node_->get_parameter("pos_file_path").as_string();
  constructPose(file_path_);
}
