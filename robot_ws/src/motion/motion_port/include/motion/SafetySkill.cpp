#include <rclcpp/rclcpp.hpp>
#include "motion/SafetySkill.hpp"
#include "runswift_interfaces/msg/body_command.hpp"
#include "utils/basic_maths.hpp"

#define MIN_STANDING_WEIGHT 0.55f
// FALLING_ANG must be less than FALLEN_ANG
#define FALLEN_ANG 50
#define FUTURE_TIME 0.2
#define FALLING_CONFIRM_FRAMES 3
#define FALLING_ANG_FORWARD 40
#define FALLING_ANG_BACK 35
#define FALLING_ANG_Y 45

using BodyCommand = runswift_interfaces::msg::BodyCommand;

SafetySkill::SafetySkill(const rclcpp::Node::SharedPtr & node)
: motion_node_(node)
{
  filtered_fsr_sum = 5.0;       // assume we start standing
  cor_angular_velocity = 0;
  sag_angular_velocity = 0;
  prev_angles[0] = 0;
  prev_angles[1] = 0;
  fallingCounter = 0;
  RCLCPP_INFO(motion_node_->get_logger(), "SafetySkill constructed");
}

SafetySkill::~SafetySkill()
{
  RCLCPP_INFO(motion_node_->get_logger(), "SafetySkill destroyed");
}

void SafetySkill::readParameters()
{
  simulation = motion_node_->get_parameter("simulation").as_bool();
  useGetups = true;
  ukemiEnabled = true;
}

float calcFutureAngle(float angle, float velocity)
{
  return angle + velocity * FUTURE_TIME;
}

void SafetySkill::wrapRequest(MotionCommand & request, const SensorValues & s)
{
  // If we're in the simulator, don't do ukemi or getups because it goes nuts
  if (simulation) {
    return;
  }

  if (request.body_command.action_type == BodyCommand::MOTION_CALIBRATE) {
    return;
  }

  float fsr_sum = s.sensors[Sensors::LFoot_FSR_FrontLeft] +
    s.sensors[Sensors::LFoot_FSR_FrontRight] +
    s.sensors[Sensors::LFoot_FSR_RearLeft] +
    s.sensors[Sensors::LFoot_FSR_RearRight] +
    s.sensors[Sensors::RFoot_FSR_FrontLeft] +
    s.sensors[Sensors::RFoot_FSR_FrontRight] +
    s.sensors[Sensors::RFoot_FSR_RearLeft] +
    s.sensors[Sensors::RFoot_FSR_RearRight];

  if (!std::isnan(fsr_sum)) {
    filtered_fsr_sum = filtered_fsr_sum + 0.2 * (fsr_sum - filtered_fsr_sum);
  }

  float ang[2] = {RAD2DEG(s.sensors[Sensors::InertialSensor_AngleX]),
    RAD2DEG(s.sensors[Sensors::InertialSensor_AngleY])};

  float futureAng[2] = {
    RAD2DEG(
      calcFutureAngle(
        s.sensors[Sensors::InertialSensor_AngleX],
        s.sensors[Sensors::InertialSensor_GyroscopeX]
      )),
    RAD2DEG(
      calcFutureAngle(
        s.sensors[Sensors::InertialSensor_AngleY],
        s.sensors[Sensors::InertialSensor_GyroscopeY]
    ))
  };

  // choose a safety skill
  // used to publish blinking eye leds for each one, perhaps publish to hri or straight to leds
  if (ang[1] < -FALLEN_ANG) {
    fallingCounter = 0;
    // if fallen back more than FALLEN_ANG
    request.body_command.action_type = (useGetups) ? BodyCommand::GETUP_BACK : BodyCommand::UNSTIFF;
  } else if (ang[1] > FALLEN_ANG) {
    fallingCounter = 0;
    // if fallen forward more than FALLEN_ANG
    request.body_command.action_type =
      (useGetups) ? BodyCommand::GETUP_FRONT : BodyCommand::UNSTIFF;
  } else if (ang[0] > FALLEN_ANG || ang[0] < -FALLEN_ANG) {
    fallingCounter = 0;
    // if fallen on either side
    request.body_command.action_type = BodyCommand::TIP_OVER;
  } else if (futureAng[1] > FALLING_ANG_FORWARD && ukemiEnabled) {
    fallingCounter++;
    if (fallingCounter >= FALLING_CONFIRM_FRAMES) {
      request.body_command.action_type = BodyCommand::UKEMI_FRONT;
    }
  } else if (futureAng[1] < -FALLING_ANG_BACK) {
    fallingCounter++;
    if (fallingCounter >= FALLING_CONFIRM_FRAMES && ukemiEnabled) {
      request.body_command.action_type = BodyCommand::UKEMI_BACK;
    }
  } else if (ABS(futureAng[0]) > FALLING_ANG_Y) {
    fallingCounter++;
    // still falling!
    if (fallingCounter >= FALLING_CONFIRM_FRAMES) {
      request.body_command.action_type = BodyCommand::UNSTIFF;
    }
  } else if (filtered_fsr_sum < MIN_STANDING_WEIGHT &&
    request.body_command.action_type != BodyCommand::GOALIE_STAND)
  {
    fallingCounter = 0;
    // upright but not on the ground
    request.body_command.action_type = BodyCommand::REF_PICKUP;
  } else {
    fallingCounter = 0;
  }
}
