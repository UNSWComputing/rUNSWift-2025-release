#include "motion/motion_adapter.hpp"
#include <bitset>
#include <ctime>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include "nao_lola_sensor_msgs/msg/joint_stiffnesses.hpp"
#include "nao_lola_sensor_msgs/msg/accelerometer.hpp"
#include "nao_lola_sensor_msgs/msg/gyroscope.hpp"
#include "nao_lola_sensor_msgs/msg/angle.hpp"
#include "nao_lola_sensor_msgs/msg/fsr.hpp"
#include "nao_lola_sensor_msgs/msg/touch.hpp"
#include "nao_lola_sensor_msgs/msg/battery.hpp"
#include "nao_lola_sensor_msgs/msg/joint_temperatures.hpp"
#include "nao_lola_sensor_msgs/msg/joint_currents.hpp"
#include "nao_lola_sensor_msgs/msg/buttons.hpp"
#include "nao_lola_sensor_msgs/msg/joint_indexes.hpp"
#include "nao_lola_command_msgs/msg/joint_positions.hpp"
#include "nao_lola_command_msgs/msg/joint_stiffnesses.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "motion/SafetySkill.hpp"
#include "motion/generator/ClippedGenerator.hpp"
#include "motion/generator/DistributedGenerator.hpp"
#include "runswift_interfaces/msg/vision_balls.hpp"
#include "types/AbsCoord.hpp"
#include "types/JointValues.hpp"
#include "types/MakeJointsIn.hpp"
#include "types/SensorValues.hpp"
#include "utils/Timer.hpp"
#include "utils/body.hpp"

#define LOST_STIFFNESS_CYCLES 83

#define BUTTON_PRESS_LENGTH 80
#define BUTTON_TAP_LENGTH 10

using namespace std;
using BodyCommand = runswift_interfaces::msg::BodyCommand;

/*-----------------------------------------------------------------------------
 * Motion thread constructor
 *---------------------------------------------------------------------------*/
MotionAdapter::MotionAdapter(const rclcpp::NodeOptions & options)
: Node("motion_port", options)
{
  RCLCPP_INFO(this->get_logger(), "Constructing MotionAdapter");

  RCLCPP_INFO(this->get_logger(), "MotionAdapater constructed");

  // Default to sit
  active_motion_command_.body_command.action_type = BodyCommand::SIT;
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(
    *this, rclcpp::QoS(
      1).reliability(rclcpp::ReliabilityPolicy::BestEffort));
  // ROS2 start point
  // subscribers
  vision_balls_sub_ = this->create_subscription<runswift_interfaces::msg::VisionBalls>(
    "vision/VBalls", 1,
    std::bind(&MotionAdapter::vision_balls_callback, this, std::placeholders::_1));
  joint_stiffnesses_sub_ =
    this->create_subscription<nao_lola_sensor_msgs::msg::JointStiffnesses>(
    "/sensors/joint_stiffnesses", 1,
    std::bind(&MotionAdapter::joint_stiffnesses_callback, this, std::placeholders::_1));
  accelerometer_sub_ = this->create_subscription<nao_lola_sensor_msgs::msg::Accelerometer>(
    "/sensors/accelerometer", 1,
    std::bind(&MotionAdapter::accelerometer_callback, this, std::placeholders::_1));
  gyroscope_sub_ = this->create_subscription<nao_lola_sensor_msgs::msg::Gyroscope>(
    "/sensors/gyroscope", 1,
    std::bind(&MotionAdapter::gyroscope_callback, this, std::placeholders::_1));
  angle_sub_ = this->create_subscription<nao_lola_sensor_msgs::msg::Angle>(
    "/sensors/angle", 1, std::bind(&MotionAdapter::angle_callback, this, std::placeholders::_1));
  imu_madgwick_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/imu_madgwick", 1,
    std::bind(&MotionAdapter::imu_madgwick_callback, this, std::placeholders::_1));
  fsr_sub_ = this->create_subscription<nao_lola_sensor_msgs::msg::FSR>(
    "/sensors/fsr", 1, std::bind(&MotionAdapter::fsr_callback, this, std::placeholders::_1));
  touch_sub_ = this->create_subscription<nao_lola_sensor_msgs::msg::Touch>(
    "/sensors/touch", 1, std::bind(&MotionAdapter::touch_callback, this, std::placeholders::_1));
  battery_sub_ = this->create_subscription<nao_lola_sensor_msgs::msg::Battery>(
    "/sensors/battery", 1,
    std::bind(&MotionAdapter::battery_callback, this, std::placeholders::_1));
  joint_temperatures_sub_ =
    this->create_subscription<nao_lola_sensor_msgs::msg::JointTemperatures>(
    "/sensors/joint_temperatures", 1,
    std::bind(&MotionAdapter::joint_temperatures_callback, this, std::placeholders::_1));
  joint_currents_sub_ = this->create_subscription<nao_lola_sensor_msgs::msg::JointCurrents>(
    "/sensors/joint_currents", 1,
    std::bind(&MotionAdapter::joint_currents_callback, this, std::placeholders::_1));
  buttons_sub_ = this->create_subscription<nao_lola_sensor_msgs::msg::Buttons>(
    "/sensors/buttons", 1,
    std::bind(&MotionAdapter::buttons_callback, this, std::placeholders::_1));
  joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 1,
    std::bind(&MotionAdapter::joint_states_callback, this, std::placeholders::_1));

  // Publishers
  joint_positions_pub_ = this->create_publisher<nao_lola_command_msgs::msg::JointPositions>(
    "/effectors/joint_positions", 1);
  joint_stiffnesses_pub_ =
    this->create_publisher<nao_lola_command_msgs::msg::JointStiffnesses>(
    "/effectors/joint_stiffnesses", 1);

  robot_is_stiff_pub_ =
    this->create_publisher<std_msgs::msg::Bool>(
    "/robot_stiffness", 1);

  // Before tick happens, default all button and head haptics to False
  sensors_.sensors[Sensors::SensorCodesEnum::Head_Touch_Front] = false;
  sensors_.sensors[Sensors::SensorCodesEnum::Head_Touch_Middle] = false;
  sensors_.sensors[Sensors::SensorCodesEnum::Head_Touch_Rear] = false;
  sensors_.sensors[Sensors::SensorCodesEnum::ChestBoard_Button] = false;
  sensors_.sensors[Sensors::SensorCodesEnum::LFoot_Bumper_Left] = false;
  sensors_.sensors[Sensors::SensorCodesEnum::LFoot_Bumper_Right] = false;
  sensors_.sensors[Sensors::SensorCodesEnum::RFoot_Bumper_Left] = false;
  sensors_.sensors[Sensors::SensorCodesEnum::RFoot_Bumper_Right] = false;

  this->declare_parameter("camera_yaw_bottom", 0.0);
  this->declare_parameter("camera_pitch_bottom", 0.0);
  this->declare_parameter("camera_roll_bottom", 0.0);

  this->declare_parameter("camera_pitch_top_when_looking_left", 0.0f);
  this->declare_parameter("camera_pitch_top_when_looking_straight", 0.0f);
  this->declare_parameter("camera_pitch_top_when_looking_right", 0.0f);
  this->declare_parameter("camera_roll_top_when_looking_left", 0.0f);
  this->declare_parameter("camera_roll_top_when_looking_straight", 0.0f);
  this->declare_parameter("camera_roll_top_when_looking_right", 0.0f);
  this->declare_parameter("camera_yaw_top_when_looking_left", 0.0f);
  this->declare_parameter("camera_yaw_top_when_looking_straight", 0.0f);
  this->declare_parameter("camera_yaw_top_when_looking_right", 0.0f);

  this->declare_parameter("body_pitch", 0.0);

  // [motion]
  this->declare_parameter("getup_speed", "MODERATE");
  this->declare_parameter("walk_speed_cap", 0.0);

  // [kick]
  this->declare_parameter("kick_lean_offset_l", 0.0);
  this->declare_parameter("kick_lean_offset_r", 0.0);
  this->declare_parameter("kick_lean", 20.5);

  // [simulation]
  this->declare_parameter("simulation", false, 
    rcl_interfaces::msg::ParameterDescriptor()
    .set__description("Set to true if running in simulation mode. Only works on startup"));
  if (this->get_parameter("simulation").as_bool()) {
    unstiff_ = false;
  }
  this->declare_parameter(
    "pos_file_path",
    this->get_parameter("simulation").as_bool() ?
    "image/home/nao/data/pos/legacy-port" :
    "/home/nao/data/pos/legacy-port");

  param_sub_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
  param_event_callback_handle_ = param_sub_->add_parameter_event_callback(
    std::bind(&MotionAdapter::param_event_callback, this, std::placeholders::_1)
  );

  // from behaviour
  motion_command_sub_ = this->create_subscription<MotionCommand>(
    "motion_command", rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&MotionAdapter::motion_command_callback, this, std::placeholders::_1));

  motion_status_pub_ = this->create_publisher<MotionCommand>(
    "motion_status", 1);

  motion_odometry_pub_ = this->create_publisher<runswift_interfaces::msg::MotionOdometry>(
    "motion_odometry", 1);

  RCLCPP_INFO(this->get_logger(), "MotionAdapter constructed");
}

void MotionAdapter::init()
{
  // shared_from_this() doesn't work in constructor therefore we use an init funciton
  DistributedGenerator * g =
    new DistributedGenerator(shared_from_this(), this->get_parameter("simulation").as_bool());
  generator = (Generator *) new ClippedGenerator(shared_from_this(), (Generator *) g);
  if (generator == NULL) {
    RCLCPP_FATAL(this->get_logger(), "MotionAdapter: NULL Generator");
  }
  generator->readParameters();

  kinematics = std::make_unique<Kinematics>(shared_from_this());
  kinematics->readParameters();

  safetySkill = std::make_unique<SafetySkill>(shared_from_this());
  if (safetySkill == NULL) {
    RCLCPP_FATAL(this->get_logger(), "MotionAdapter: NULL SafetySkill");
  }
  safetySkill->readParameters();

  timer_ = this->create_wall_timer(12.2ms, std::bind(&MotionAdapter::tick, this));
}

/*-----------------------------------------------------------------------------
 * ROS2 parameter change callback
 *---------------------------------------------------------------------------*/
void MotionAdapter::param_event_callback(const rcl_interfaces::msg::ParameterEvent & event)
{
  RCLCPP_INFO(this->get_logger(), "Parameter change callback triggered");
  bool kinematics_updated = false;
  bool generators_updated = false;

  // Extract changed parameters from the event
  for (const auto & changed_param : event.changed_parameters) {
    if (
      changed_param.name == "camera_pitch_top_when_looking_left"     ||
      changed_param.name == "camera_pitch_top_when_looking_straight" ||
      changed_param.name == "camera_pitch_top_when_looking_right"    ||
      changed_param.name == "camera_roll_top_when_looking_left"      ||
      changed_param.name == "camera_roll_top_when_looking_straight"  ||
      changed_param.name == "camera_roll_top_when_looking_right"     ||
      changed_param.name == "camera_yaw_top_when_looking_left"       ||
      changed_param.name == "camera_yaw_top_when_looking_straight"   ||
      changed_param.name == "camera_yaw_top_when_looking_right"      ||
      changed_param.name == "camera_yaw_bottom" ||
      changed_param.name == "camera_pitch_bottom" ||
      changed_param.name == "camera_roll_bottom" ||
      changed_param.name == "body_pitch")
    {
      kinematics_updated = true;
    } else if (changed_param.name == "getup_speed" ||
      changed_param.name == "pos_file_path" ||
      changed_param.name == "kick_lean_offset_l" ||
      changed_param.name == "kick_lean_offset_r" ||
      changed_param.name == "kick_lean")
    {
      generators_updated = true;
    }
  }

  // Also check newly set parameters
  for (const auto & new_param : event.new_parameters) {
    RCLCPP_WARN(get_logger(), "%s", new_param.name.c_str());

    if (
      new_param.name == "camera_pitch_top_when_looking_left"     ||
      new_param.name == "camera_pitch_top_when_looking_straight" ||
      new_param.name == "camera_pitch_top_when_looking_right"    ||
      new_param.name == "camera_roll_top_when_looking_left"      ||
      new_param.name == "camera_roll_top_when_looking_straight"  ||
      new_param.name == "camera_roll_top_when_looking_right"     ||
      new_param.name == "camera_yaw_top_when_looking_left"       ||
      new_param.name == "camera_yaw_top_when_looking_straight"   ||
      new_param.name == "camera_yaw_top_when_looking_right"      ||
      new_param.name == "camera_yaw_bottom" ||
      new_param.name == "camera_pitch_bottom" ||
      new_param.name == "camera_roll_bottom" ||
      new_param.name == "body_pitch")
    {
      kinematics_updated = true;
    } else if (new_param.name == "getup_speed" ||
      new_param.name == "pos_file_path" ||
      new_param.name == "kick_lean_offset_l" ||
      new_param.name == "kick_lean_offset_r" ||
      new_param.name == "kick_lean")
    {
      generators_updated = true;
    }
  }

  if (kinematics_updated) {
    kinematics->readParameters();
  }

  if (generators_updated) {
    generator->readParameters();
  }
}

/*-----------------------------------------------------------------------------
  * Motion thread destructor
  *---------------------------------------------------------------------------*/
MotionAdapter::~MotionAdapter()
{
  RCLCPP_INFO(this->get_logger(), "Destroying MotionAdapter");

  delete generator;

}

/*-----------------------------------------------------------------------------
 * ROS2 input callbacks
 *---------------------------------------------------------------------------*/
void MotionAdapter::motion_command_callback(const MotionCommand::SharedPtr cmd)
{
  MotionCommand motion_request = *cmd;

  // Reject None action type as it will stand the robot up with stiffness 0,
  // which is dangerous. To stand up from any position is also unsafe as it
  // moves the joints faster than pos files suppose to do.
  // TODO: Investigate why None action is moving the joints !!!

  if (cmd->body_command.action_type == "") {
    return;
  }

  // A lot of the if/else if mapping made sense when MotionCommand mapped to ActionRequest, but is now just boilerplate
  // Map the body command
  if (cmd->body_command.action_type.empty()) {
    motion_request.body_command.action_type = BodyCommand::NULL_GENERATOR;
  } else if (cmd->body_command.action_type == cmd->body_command.WALK) {
    motion_request.body_command.action_type = BodyCommand::WALK;

    // Map the twist for walking
    motion_request.body_command.bend = 1.0;
    motion_request.body_command.speed = this->get_parameter("walk_speed_cap").as_double();

  } else if (cmd->body_command.action_type == cmd->body_command.STAND) {
    if (previous_motion_command_.body_command.action_type == BodyCommand::WALK) {
      motion_request.body_command.action_type = BodyCommand::WALK;
      motion_request.body_command.bend = 0.0;
      motion_request.body_command.twist.linear.x = 0;
      motion_request.body_command.twist.linear.y = 0;
      motion_request.body_command.twist.angular.z = 0;
    } else {
      motion_request.body_command.action_type = BodyCommand::STAND;
    }
  } else if (cmd->body_command.action_type == cmd->body_command.SIT) {
    motion_request.body_command.action_type = BodyCommand::SIT;
  } else if (cmd->body_command.action_type == cmd->body_command.KICK) {
    motion_request.body_command.action_type = BodyCommand::KICK;

    // Map kick parameters
    motion_request.body_command.foot = cmd->body_command.foot;
    motion_request.body_command.power = cmd->body_command.power;
  } else if (cmd->body_command.action_type == cmd->body_command.RAISE_ARM) {
    motion_request.body_command.action_type = BodyCommand::RAISE_ARM;
  } else if (cmd->body_command.action_type == cmd->body_command.REF_PICKUP) {
    motion_request.body_command.action_type = BodyCommand::REF_PICKUP;
  } else if (cmd->body_command.action_type == cmd->body_command.UNSTIFF) {
    motion_request.body_command.action_type = BodyCommand::UNSTIFF;
  } else if (cmd->body_command.action_type == cmd->body_command.GOALIE_SIT) {
    motion_request.body_command.action_type = BodyCommand::GOALIE_SIT;
  } else if (cmd->body_command.action_type == cmd->body_command.GOALIE_FAST_SIT) {
    motion_request.body_command.action_type = BodyCommand::GOALIE_FAST_SIT;
  } else if (cmd->body_command.action_type == cmd->body_command.GOALIE_STAND) {
    motion_request.body_command.action_type = BodyCommand::GOALIE_STAND;
  } else if (cmd->body_command.action_type == cmd->body_command.GOALIE_CENTRE) {
    motion_request.body_command.action_type = BodyCommand::GOALIE_CENTRE;
  } else if (cmd->body_command.action_type == cmd->body_command.GOALIE_UNCENTRE) {
    motion_request.body_command.action_type = BodyCommand::GOALIE_UNCENTRE;
  } else if (cmd->body_command.action_type == cmd->body_command.GOALIE_DIVE_LEFT) {
    motion_request.body_command.action_type = BodyCommand::GOALIE_DIVE_LEFT;
  } else if (cmd->body_command.action_type == cmd->body_command.GOALIE_DIVE_RIGHT) {
    motion_request.body_command.action_type = BodyCommand::GOALIE_DIVE_RIGHT;
  } else if (cmd->body_command.action_type == cmd->body_command.GOALIE_INITIAL) {
    motion_request.body_command.action_type = BodyCommand::GOALIE_INITIAL;
  } else if (cmd->body_command.action_type == cmd->body_command.INITIAL) {
    motion_request.body_command.action_type = BodyCommand::INITIAL;
  } else if (cmd->body_command.action_type == cmd->body_command.DEFENDER_CENTRE) {
    motion_request.body_command.action_type = BodyCommand::DEFENDER_CENTRE;
  } else {
    // For any other action type, set to NONE
    motion_request.body_command.action_type = BodyCommand::NULL_GENERATOR;
    RCLCPP_WARN(
      this->get_logger(), "Unhandled action type: %s", cmd->body_command.action_type.c_str());
  }

  motion_request.head_command.yaw = cmd->head_command.yaw;
  motion_request.head_command.pitch = cmd->head_command.pitch;
  motion_request.head_command.is_relative = cmd->head_command.is_relative;
  motion_request.head_command.yaw_speed = cmd->head_command.yaw_speed;
  motion_request.head_command.pitch_speed = cmd->head_command.pitch_speed;

  RCLCPP_INFO(
    this->get_logger(), "Translated motion command to motion command with action type: %s",
    motion_request.body_command.action_type.c_str());
  active_motion_command_ = motion_request;
}

MotionCommand MotionAdapter::getMotionStatus(const MotionCommand & motion_command)
{
  runswift_interfaces::msg::MotionCommand motion_status = motion_command;

  // logic here to modify the returned action command if flt == 0 and bend

  motion_status.stamp = this->now();

  std::vector<string> known_commands = {
    BodyCommand::DEFENDER_CENTRE,
    BodyCommand::GETUP_BACK,
    BodyCommand::GETUP_FRONT,
    BodyCommand::GOALIE_CENTRE,
    BodyCommand::GOALIE_DIVE_LEFT,
    BodyCommand::GOALIE_DIVE_RIGHT,
    BodyCommand::GOALIE_FAST_SIT,
    BodyCommand::GOALIE_INITIAL,
    BodyCommand::GOALIE_SIT,
    BodyCommand::GOALIE_STAND,
    BodyCommand::GOALIE_UNCENTRE,
    BodyCommand::INITIAL,
    BodyCommand::KICK,
    BodyCommand::MOTION_CALIBRATE,
    BodyCommand::RAISE_ARM,
    BodyCommand::REF_PICKUP,
    BodyCommand::SIT,
    BodyCommand::STAND,
    BodyCommand::TIP_OVER,
    BodyCommand::UKEMI_BACK,
    BodyCommand::UKEMI_FRONT,
    BodyCommand::UNSTIFF,
    BodyCommand::WALK
  };
  auto it = std::find(
    known_commands.begin(),
    known_commands.end(),
    motion_status.body_command.action_type
  );
  if (it == known_commands.end()) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Unknown action type: %s",
      motion_command.body_command.action_type.c_str()
    );
  }

  return motion_status;
}

runswift_interfaces::msg::MotionOdometry MotionAdapter::getMotionOdometry(
  Odometry & odometry)
{
  runswift_interfaces::msg::MotionOdometry motion_odometry;

  motion_odometry.header.stamp = this->now();
  motion_odometry.forward = odometry.forward;
  motion_odometry.left = odometry.left;
  motion_odometry.turn = odometry.turn;

  return motion_odometry;
}

/*-----------------------------------------------------------------------------
 * Motion thread tick function
 *---------------------------------------------------------------------------*/
void MotionAdapter::tick()
{
  Timer t(this->get_logger());


  // 1.0 get sensors/inputs needed for motion
  t.restart();
  SensorValues sensors = sensors_;      // We want motion sensors to be constant
  kinematics->setSensorValues(sensors);
  kinematics->updateDHChain();
  kinematics->publishPoseToTF2(*tf_broadcaster_, lastest_joint_positions_time_, "base_footprint");
  processButtons(sensors);

  // Update the body model
  bodyModel.kinematics = kinematics.get();
  bodyModel.update(sensors);
  // TODO: understand if body model changes the kinematics as to whether it should
  // be published beforehand or afterwards

  // Get the position of the ball in robot relative cartesian coordinates
  AbsCoord ball;
  if (has_latest_ball_) {
    ball = AbsCoord(latest_ball_.x, latest_ball_.y, 0.0f);
  } else {
    ball = AbsCoord(0.0f, 0.0f, 0.0f);
  }
  RCLCPP_DEBUG(this->get_logger(), "sensor get took %d ms", t.elapsed_ms());


  // 2.0 prep motion
  t.restart();
  MotionCommand request = active_motion_command_;

  if (newly_unstiff_) {
    RCLCPP_INFO(this->get_logger(), "Try Sit");
    generator->reset();
    request.body_command.action_type = BodyCommand::SIT;
    active_motion_command_.body_command.action_type = BodyCommand::SIT;
    newly_unstiff_ = false;
  } else if (unstiff_) {
    request.body_command.action_type = BodyCommand::UNSTIFF;
    active_motion_command_.body_command.action_type = BodyCommand::UNSTIFF;
  } else if (newly_stiff_) {
    RCLCPP_INFO(this->get_logger(), "Try Stand");
    generator->reset();
    request.body_command.action_type = BodyCommand::INITIAL;
    active_motion_command_.body_command.action_type = BodyCommand::INITIAL;
    newly_stiff_ = false;
    odometry.clear();
  } else {
    safetySkill->wrapRequest(request, sensors);
  }
  RCLCPP_DEBUG(this->get_logger(), "motion prep took %d ms", t.elapsed_ms());


  // 3.0 do motion
  t.restart();
  MakeJointsIn makeJointsIn(&request, &odometry, sensors, bodyModel, ball.x(), ball.y());
  JointValues joints = generator->makeJoints(makeJointsIn);
  actuate(joints);
  RCLCPP_DEBUG(this->get_logger(), "generator->makeJoints took %d ms", t.elapsed_ms());


  // 4.0 update everyone else with what motion did if necessary. odometry etc
  // TODO: see if any &odometrys get modified to see where it would be best to
  // publish it. same with kinematics
  motion_status_pub_->publish(getMotionStatus(request));
  motion_odometry_pub_->publish(getMotionOdometry(odometry));
  previous_motion_command_ = request;

}

/*-----------------------------------------------------------------------------
* actuation pubs
*---------------------------------------------------------------------------*/

void MotionAdapter::actuate(JointValues joints)
{
  // Joint Positions
  nao_lola_command_msgs::msg::JointPositions joint_positions_msg;
  for (size_t i = 0; i < Joints::NUMBER_OF_JOINTS; ++i) {
    joint_positions_msg.indexes.push_back(i);
    joint_positions_msg.positions.push_back(joints.angles[i]);
  }
  joint_positions_pub_->publish(joint_positions_msg);

  // Joint Stiffnesses
  nao_lola_command_msgs::msg::JointStiffnesses joint_stiffnesses_msg;
  for (size_t i = 0; i < Joints::NUMBER_OF_JOINTS; ++i) {
    joint_stiffnesses_msg.indexes.push_back(i);
    joint_stiffnesses_msg.stiffnesses.push_back(joints.stiffnesses[i]);
  }
  joint_stiffnesses_pub_->publish(joint_stiffnesses_msg);
}

/*-----------------------------------------------------------------------------
* vision input
*---------------------------------------------------------------------------*/
void MotionAdapter::vision_balls_callback(
  const runswift_interfaces::msg::VisionBalls::SharedPtr msg)
{
  if (!msg->ball_features.empty()) {
    // Select the ball with the highest confidence
    auto best = std::max_element(
      msg->ball_features.begin(), msg->ball_features.end(),
      [](const auto & a, const auto & b) {
        return a.confidence_score < b.confidence_score;
      });
    latest_ball_ = best->ball_coordinates;
    has_latest_ball_ = true;
    // RCLCPP_INFO(
    //   this->get_logger(), "Ball detected at: x = %f, y = %f, z = %f, confidence = %u",
    //   latest_ball_.x, latest_ball_.y, latest_ball_.z, best->confidence_score);
  } else {
    has_latest_ball_ = false;
    RCLCPP_WARN(this->get_logger(), "No ball features received in vision message.");
  }
}

/*-----------------------------------------------------------------------------
* joint sensors
*---------------------------------------------------------------------------*/
void MotionAdapter::joint_states_callback(
  const sensor_msgs::msg::JointState::SharedPtr joint_states)
{
  lastest_joint_positions_time_ = joint_states->header.stamp;
  for (size_t i = 0; i < nao_lola_sensor_msgs::msg::JointIndexes::NUMJOINTS; ++i) {
    sensors_.joints.angles[i] = joint_states->position[i];
  }
}

void MotionAdapter::joint_stiffnesses_callback(
  const nao_lola_sensor_msgs::msg::JointStiffnesses::SharedPtr joint_stiffnesses)
{
  for (size_t i = 0; i < nao_lola_sensor_msgs::msg::JointIndexes::NUMJOINTS; ++i) {
    sensors_.joints.stiffnesses[i] = joint_stiffnesses->stiffnesses[i];
  }
}

void MotionAdapter::joint_temperatures_callback(
  const nao_lola_sensor_msgs::msg::JointTemperatures::SharedPtr joint_temperatures)
{
  for (size_t i = 0; i < nao_lola_sensor_msgs::msg::JointIndexes::NUMJOINTS; ++i) {
    sensors_.joints.temperatures[i] = joint_temperatures->temperatures[i];
  }
}

void MotionAdapter::joint_currents_callback(
  const nao_lola_sensor_msgs::msg::JointCurrents::SharedPtr joint_currents)
{
  for (size_t i = 0; i < nao_lola_sensor_msgs::msg::JointIndexes::NUMJOINTS; ++i) {
    sensors_.joints.currents[i] = joint_currents->currents[i];
  }
  // monitorJointHealth();
}

/*-----------------------------------------------------------------------------
* inertial sensors
*---------------------------------------------------------------------------*/
void MotionAdapter::accelerometer_callback(
  const nao_lola_sensor_msgs::msg::Accelerometer::SharedPtr accelerometer)
{
  sensors_.sensors[Sensors::SensorCodesEnum::InertialSensor_AccelerometerX] = accelerometer->x;
  sensors_.sensors[Sensors::SensorCodesEnum::InertialSensor_AccelerometerY] = accelerometer->y;
  sensors_.sensors[Sensors::SensorCodesEnum::InertialSensor_AccelerometerZ] = accelerometer->z;

}

void MotionAdapter::gyroscope_callback(
  const nao_lola_sensor_msgs::msg::Gyroscope::SharedPtr gyroscope)
{
  sensors_.sensors[Sensors::SensorCodesEnum::InertialSensor_GyroscopeX] = gyroscope->x;
  sensors_.sensors[Sensors::SensorCodesEnum::InertialSensor_GyroscopeY] = gyroscope->y;
  // We must invert the gyroscopeZ for the V6
  sensors_.sensors[Sensors::SensorCodesEnum::InertialSensor_GyroscopeZ] = -gyroscope->z;
}

void MotionAdapter::angle_callback(const nao_lola_sensor_msgs::msg::Angle::SharedPtr angle)
{
  sensors_.sensors[Sensors::SensorCodesEnum::InertialSensor_AngleX] = angle->x;
  sensors_.sensors[Sensors::SensorCodesEnum::InertialSensor_AngleY] = angle->y;
}

/*-----------------------------------------------------------------------------
* other sensors to help with motion
*---------------------------------------------------------------------------*/

void MotionAdapter::fsr_callback(const nao_lola_sensor_msgs::msg::FSR::SharedPtr fsr)
{
  sensors_.sensors[Sensors::SensorCodesEnum::LFoot_FSR_FrontLeft] = fsr->l_foot_front_left;
  sensors_.sensors[Sensors::SensorCodesEnum::LFoot_FSR_FrontRight] = fsr->l_foot_front_right;
  sensors_.sensors[Sensors::SensorCodesEnum::LFoot_FSR_RearLeft] = fsr->l_foot_back_left;
  sensors_.sensors[Sensors::SensorCodesEnum::LFoot_FSR_RearRight] = fsr->l_foot_back_right;
  sensors_.sensors[Sensors::SensorCodesEnum::RFoot_FSR_FrontLeft] = fsr->r_foot_front_left;
  sensors_.sensors[Sensors::SensorCodesEnum::RFoot_FSR_FrontRight] = fsr->r_foot_front_right;
  sensors_.sensors[Sensors::SensorCodesEnum::RFoot_FSR_RearLeft] = fsr->r_foot_back_left;
  sensors_.sensors[Sensors::SensorCodesEnum::RFoot_FSR_RearRight] = fsr->r_foot_back_right;
}

/*-----------------------------------------------------------------------------
* human interaction sensors
*---------------------------------------------------------------------------*/
void MotionAdapter::battery_callback(const nao_lola_sensor_msgs::msg::Battery::SharedPtr battery)
{
  sensors_.sensors[Sensors::SensorCodesEnum::Battery_Charge] = battery->charge;
  sensors_.sensors[Sensors::SensorCodesEnum::Battery_Status] = battery->charging;
}

void MotionAdapter::touch_callback(const nao_lola_sensor_msgs::msg::Touch::SharedPtr touch)
{
  sensors_.sensors[Sensors::SensorCodesEnum::Head_Touch_Front] = touch->head_front;
  sensors_.sensors[Sensors::SensorCodesEnum::Head_Touch_Middle] = touch->head_middle;
  sensors_.sensors[Sensors::SensorCodesEnum::Head_Touch_Rear] = touch->head_rear;
}

void MotionAdapter::buttons_callback(const nao_lola_sensor_msgs::msg::Buttons::SharedPtr buttons)
{
  sensors_.sensors[Sensors::SensorCodesEnum::ChestBoard_Button] = buttons->chest;
  sensors_.sensors[Sensors::SensorCodesEnum::LFoot_Bumper_Left] = buttons->l_foot_bumper_left;
  sensors_.sensors[Sensors::SensorCodesEnum::LFoot_Bumper_Right] = buttons->l_foot_bumper_right;
  sensors_.sensors[Sensors::SensorCodesEnum::RFoot_Bumper_Left] = buttons->r_foot_bumper_left;
  sensors_.sensors[Sensors::SensorCodesEnum::RFoot_Bumper_Right] = buttons->r_foot_bumper_right;
}

void MotionAdapter::imu_madgwick_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  // Extract orientation quaternion
  tf2::Quaternion q(
    msg->orientation.w,
    -msg->orientation.z,
    msg->orientation.y,
    -msg->orientation.x);

  // Convert quaternion to roll, pitch, and yaw
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  sensors_.sensors[Sensors::SensorCodesEnum::InertialSensor_IntegratedAngleX] = roll;
  sensors_.sensors[Sensors::SensorCodesEnum::InertialSensor_IntegratedAngleY] = pitch;
}

void MotionAdapter::toggleStiffen()
{
  if (unstiff_) {
    setStiff();
    return;
  }
  setUnstiff();
}

void MotionAdapter::setUnstiff()
{
  if (!unstiff_) {
    newly_unstiff_ = true;
    newly_stiff_ = false;
    auto msg = std_msgs::msg::Bool();
    msg.data = false;
    robot_is_stiff_pub_->publish(msg);
  }
  head_unstiff_ = true;
  unstiff_ = true;
  RCLCPP_INFO(this->get_logger(), "set unstiff");
}

void MotionAdapter::setStiff()
{
  if (unstiff_) {
    newly_stiff_ = true;
    newly_unstiff_ = false;
    auto msg = std_msgs::msg::Bool();
    msg.data = true;
    robot_is_stiff_pub_->publish(msg);
  }
  head_unstiff_ = false;
  unstiff_ = false;
  RCLCPP_INFO(this->get_logger(), "set stiff");
}

inline void MotionAdapter::doButtons(
  bool chest, bool head_front, bool head_middle, bool head_rear
)
{
  static int head_buttons_count = 0;
  if (head_front && head_middle && head_rear) {
    head_buttons_count++;
    if (head_buttons_count == BUTTON_PRESS_LENGTH) {
      toggleStiffen();
    }
  } else if (head_buttons_count >= BUTTON_PRESS_LENGTH) {
    head_buttons_count = 2 * BUTTON_TAP_LENGTH;
  } else if (head_buttons_count > 0) {
    head_buttons_count--;
  }

  static int chest_up = 0, chest_presses = 0, chest_down = 0;
  if (chest_presses > 0) {
    setStiff();
    chest_presses = 0;
  }
  if (chest) {
    if (chest_down >= 0) {
      chest_down++;
    }
    chest_up = 0;
  } else {
    chest_up++;
    if (chest_down > 0) {
      chest_presses++;
      chest_down = 0;
    }
  }
}

void MotionAdapter::processButtons(const SensorValues & sensors)
{
  doButtons(
    static_cast<bool>(sensors.sensors[Sensors::ChestBoard_Button]),
    static_cast<bool>(sensors.sensors[Sensors::Head_Touch_Front]),
    static_cast<bool>(sensors.sensors[Sensors::Head_Touch_Middle]),
    static_cast<bool>(sensors.sensors[Sensors::Head_Touch_Rear])
  );
}

// note to self double check the LoadCache in lolatouch to reimplement lost stiffness counters eventually
/*-----------------------------------------------------------------------------
* handle sensors etc
*---------------------------------------------------------------------------*/
void MotionAdapter::monitorJointHealth()
{
  // This function should be called regularly, perhaps from your sensor callback
  // isn't tested yet

  for (int jointIndex = 0; jointIndex < Joints::NUMBER_OF_JOINTS; ++jointIndex) {
    // Skip wrist joints which can give false positives
    if (jointIndex != Joints::LWristYaw && jointIndex != Joints::RWristYaw) {

      // Get the current joint data from your sensor readings
      float currentCurrent = sensors_.joints.currents[jointIndex];
      float currentStiffness = sensors_.joints.stiffnesses[jointIndex];
      float currentAngle = sensors_.joints.angles[jointIndex];
      float targetAngle = targetAngles_[jointIndex];

      bool noCurrentFlow = (currentCurrent == 0.0f);
      bool stiffnessActive = (currentStiffness > 0.0f);
      bool notReachingTarget = (std::abs(currentAngle - targetAngle) > DEG2RAD(2));

      if (noCurrentFlow && stiffnessActive && notReachingTarget) {
        ++lostStiffnessCounters[jointIndex];

        if (lostStiffnessCounters[jointIndex] > LOST_STIFFNESS_CYCLES) {
          // Use your speech system or logging
          RCLCPP_ERROR(get_logger(), "Cannot move %s", Joints::jointNames[jointIndex].c_str());

          // If you have a text-to-speech system:
          // publishSpeech("cannot move " + Joints::fliteJointNames[jointIndex]);
        }
      } else {
        lostStiffnessCounters[jointIndex] = 0;
      }
    }
  }
}

/*-----------------------------------------------------------------------------
* main function
*---------------------------------------------------------------------------*/
int main(int argc, char * argv[])
{
  // Initialize ROS 2
  rclcpp::init(argc, argv);

  // Spin the MotionAdapter node
  auto node = std::make_shared<MotionAdapter>();
  node->init();
  rclcpp::spin(node);

  // Shutdown ROS 2 and clean up
  rclcpp::shutdown();

  return 0;
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(MotionAdapter)
