#include "motion/generator/HeadGenerator.hpp"
#include "utils/basic_maths.hpp"
#include "utils/clip.hpp"

HeadGenerator::HeadGenerator(const rclcpp::Node::SharedPtr & node)
: Generator(node),
  yaw(0.0f),
  pitch(0.0f)
{
  RCLCPP_INFO(
    motion_node_->get_logger(), "HeadGenerator constructed");
}

HeadGenerator::~HeadGenerator()
{
  RCLCPP_INFO(
    motion_node_->get_logger(), "HeadGenerator destroyed");
}

JointValues HeadGenerator::makeJoints(MakeJointsIn & makeJointsIn)
{
  MotionCommand * request = makeJointsIn.request;
  const SensorValues & sensors = makeJointsIn.sensors;

  // Simple state of the head angles
  float desY = 0.0, desP = 0.0;     // Current requested angles

  // Boundary checking (NAN values)
  if (std::isnan(request->head_command.yaw) || std::isnan(request->head_command.pitch)) {
    RCLCPP_WARN(
      motion_node_->get_logger(), "HeadGenerator: NAN angles - Doing Nothing");
    return sensors.joints;
  }

  if (request->head_command.is_relative) {
    desY = yaw + request->head_command.yaw;
  } else {
    desY = request->head_command.yaw;
  }
  desP = request->head_command.pitch;

  float diffY = desY - yaw;
  float diffP = desP - pitch;
  diffY = CLIP<float>(
    diffY, Joints::Radians::HeadYawSpeed *
    request->head_command.yaw_speed);
  diffP = CLIP<float>(
    diffP, Joints::Radians::HeadPitchSpeed *
    request->head_command.pitch_speed);
  yaw += diffY;
  pitch += diffP;

  // Return JointValues with head angles changed, all others same
  JointValues j = sensors.joints;
  j.angles[Joints::HeadYaw] = yaw;
  j.angles[Joints::HeadPitch] = pitch;
  j.stiffnesses[Joints::HeadYaw] = 0.8f;
  j.stiffnesses[Joints::HeadPitch] = 0.8f;
  return j;
}

bool HeadGenerator::isActive()
{
  return true;
}

void HeadGenerator::reset()
{
  yaw = 0.0f;
  pitch = 0.0f;
}
