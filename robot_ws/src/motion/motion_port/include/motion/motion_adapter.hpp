#pragma once

#include <string>
#include <map>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/parameter_event.hpp"

#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "runswift_interfaces/msg/body_command.hpp"
#include "runswift_interfaces/msg/vision_balls.hpp"
#include "runswift_interfaces/msg/motion_odometry.hpp"

#include "nao_lola_command_msgs/msg/joint_positions.hpp"
#include "nao_lola_command_msgs/msg/joint_stiffnesses.hpp"

#include "nao_lola_sensor_msgs/msg/accelerometer.hpp"
#include "nao_lola_sensor_msgs/msg/angle.hpp"
#include "nao_lola_sensor_msgs/msg/battery.hpp"
#include "nao_lola_sensor_msgs/msg/buttons.hpp"
#include "nao_lola_sensor_msgs/msg/fsr.hpp"
#include "nao_lola_sensor_msgs/msg/gyroscope.hpp"
#include "nao_lola_sensor_msgs/msg/joint_currents.hpp"
#include "nao_lola_sensor_msgs/msg/joint_stiffnesses.hpp"
#include "nao_lola_sensor_msgs/msg/joint_temperatures.hpp"
#include "nao_lola_sensor_msgs/msg/sonar.hpp"
#include "nao_lola_sensor_msgs/msg/touch.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "motion/generator/BodyModel.hpp"
#include "motion/generator/Generator.hpp"
#include "motion/SafetySkill.hpp"
#include "perception/kinematics/Kinematics.hpp"

#include "std_msgs/msg/bool.hpp"

using MotionCommand = runswift_interfaces::msg::MotionCommand;

/**
 * MotionAdapter - interfaces between Motion & rest of system via Blackboard
 *
 * The MotionAdapter is the controlling class for everything Motion. It reads
 * MotionCommands from the Blackboard, and SensorValues from its Touch instance.
 *
 * It then passes AC::Head, AC::Body & SensorValues to a Generator instance,
 * which processes them and determines the correct JointValues for the next DCM
 * cycle. In practice, the Generator will usually be a DistributorGenerator,
 * which will select the most appropriate Generator based on the AC's.
 *
 * These JointValues, plus AC::LED are passed to an Effector instance. The
 * effector actuates the joints and LEDs according to these instructions.
 */
class MotionAdapter : public rclcpp::Node
{
public:
  /* Constructor */
  MotionAdapter(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());  // Removed blackboard from input
  /* init */
  void init();
  /* Destructor */
  ~MotionAdapter();
  /* One cycle of this thread */
  void tick();

  // ROS2 stuff
  using BodyCommand = runswift_interfaces::msg::BodyCommand;
  using ActionType = runswift_interfaces::msg::BodyCommand;
  using MotionCommand = runswift_interfaces::msg::MotionCommand;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

private:
  Odometry odometry;
  /* Motion module instance */
  Generator * generator;
  BodyModel bodyModel;
  std::unique_ptr<Kinematics> kinematics;
  std::unique_ptr<SafetySkill> safetySkill;

  XYZ_Coord com_;

  SensorValues sensors_;

  void toggleStiffen();
  void setUnstiff();
  void setStiff();
  bool head_unstiff_ = false;
  bool unstiff_ = true;
  bool newly_stiff_ = false;
  bool newly_unstiff_ = false;

  std::vector<float> targetAngles_;

  inline void doButtons(
    bool chest, bool head_front, bool head_middle, bool head_rear
  );
  void processButtons(const SensorValues & sensors);

  // TODO (not implemented yet)
  // For detecting joints with lost stiffness
  std::array<uint8_t, Joints::NUMBER_OF_JOINTS> lostStiffnessCounters = {};
  void monitorJointHealth();

  // ROS2
  std::mutex sensors_mutex_;
  void motion_command_callback(const MotionCommand::SharedPtr cmd);
  runswift_interfaces::msg::MotionCommand getMotionStatus(const MotionCommand & motion_command);
  runswift_interfaces::msg::MotionOdometry getMotionOdometry(Odometry & odometry);

  void vision_balls_callback(const runswift_interfaces::msg::VisionBalls::SharedPtr msg);
  void param_event_callback(const rcl_interfaces::msg::ParameterEvent & event);

  void joint_stiffnesses_callback(const nao_lola_sensor_msgs::msg::JointStiffnesses::SharedPtr msg);
  void accelerometer_callback(const nao_lola_sensor_msgs::msg::Accelerometer::SharedPtr msg);
  void gyroscope_callback(const nao_lola_sensor_msgs::msg::Gyroscope::SharedPtr msg);
  void angle_callback(const nao_lola_sensor_msgs::msg::Angle::SharedPtr msg);
  void imu_madgwick_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void sonar_callback(const nao_lola_sensor_msgs::msg::Sonar::SharedPtr msg);
  void fsr_callback(const nao_lola_sensor_msgs::msg::FSR::SharedPtr msg);
  void touch_callback(const nao_lola_sensor_msgs::msg::Touch::SharedPtr msg);
  void battery_callback(const nao_lola_sensor_msgs::msg::Battery::SharedPtr msg);
  void joint_temperatures_callback(
    const nao_lola_sensor_msgs::msg::JointTemperatures::SharedPtr msg);
  void joint_currents_callback(const nao_lola_sensor_msgs::msg::JointCurrents::SharedPtr msg);
  void buttons_callback(const nao_lola_sensor_msgs::msg::Buttons::SharedPtr msg);
  void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void actuate(JointValues joints);

  rclcpp::Subscription<runswift_interfaces::msg::VisionBalls>::SharedPtr vision_balls_sub_;
  geometry_msgs::msg::Point latest_ball_;
  bool has_latest_ball_ = false;

  rclcpp::Subscription<nao_lola_sensor_msgs::msg::JointStiffnesses>::SharedPtr
    joint_stiffnesses_sub_;
  rclcpp::Subscription<nao_lola_sensor_msgs::msg::Accelerometer>::SharedPtr accelerometer_sub_;
  rclcpp::Subscription<nao_lola_sensor_msgs::msg::Gyroscope>::SharedPtr gyroscope_sub_;
  rclcpp::Subscription<nao_lola_sensor_msgs::msg::Angle>::SharedPtr angle_sub_;
  rclcpp::Subscription<nao_lola_sensor_msgs::msg::Sonar>::SharedPtr sonar_sub_;
  rclcpp::Subscription<nao_lola_sensor_msgs::msg::FSR>::SharedPtr fsr_sub_;
  rclcpp::Subscription<nao_lola_sensor_msgs::msg::Touch>::SharedPtr touch_sub_;
  rclcpp::Subscription<nao_lola_sensor_msgs::msg::Battery>::SharedPtr battery_sub_;
  rclcpp::Subscription<nao_lola_sensor_msgs::msg::JointTemperatures>::SharedPtr
    joint_temperatures_sub_;
  rclcpp::Subscription<nao_lola_sensor_msgs::msg::JointCurrents>::SharedPtr joint_currents_sub_;
  rclcpp::Subscription<nao_lola_sensor_msgs::msg::Buttons>::SharedPtr buttons_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_madgwick_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;

  rclcpp::Publisher<nao_lola_command_msgs::msg::JointPositions>::SharedPtr joint_positions_pub_;
  rclcpp::Publisher<nao_lola_command_msgs::msg::JointStiffnesses>::SharedPtr joint_stiffnesses_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr robot_is_stiff_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<MotionCommand>::SharedPtr motion_command_sub_;
  rclcpp::Publisher<MotionCommand>::SharedPtr motion_status_pub_;
  rclcpp::Publisher<runswift_interfaces::msg::MotionOdometry>::SharedPtr motion_odometry_pub_;

  std::shared_ptr<rclcpp::ParameterEventHandler> param_sub_;
  std::shared_ptr<rclcpp::ParameterEventCallbackHandle> param_event_callback_handle_;

  MotionCommand active_motion_command_;
  MotionCommand previous_motion_command_;
  double walk_speed_;

  rclcpp::Time lastest_joint_positions_time_;
};
