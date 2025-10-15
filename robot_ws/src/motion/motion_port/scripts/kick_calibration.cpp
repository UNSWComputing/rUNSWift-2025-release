#include "rcl_interfaces/msg/parameter_event.hpp"
#include "rclcpp/rclcpp.hpp"
#include "runswift_interfaces/msg/body_command.hpp"
#include "runswift_interfaces/msg/motion_command.hpp"
#include "runswift_interfaces/msg/vision_balls.hpp"
#include "utils/Timer.hpp"
#include <chrono>
#include <fstream>

class KickCalibration : public rclcpp::Node
{

public:
  KickCalibration()
  : Node("kick_calibration")
  {
    motion_command_pub_ =
      this->create_publisher<MotionCommand>("motion_command", 1);
    vision_balls_sub_ =
      this->create_subscription<runswift_interfaces::msg::VisionBalls>(
      "vision/VBalls", 1,
      std::bind(
        &KickCalibration::vision_balls_callback, this,
        std::placeholders::_1));

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(12),
      std::bind(&KickCalibration::tick, this));
    goKick_ = false;
    kicking_ = false;
    kicking_foot_ = runswift_interfaces::msg::BodyCommand::LEFT;
    // declare the parameters
    this->declare_parameter("individual_body_config", "");
    this->declare_parameter("kick_foot", "left");
    this->declare_parameter("power", 0);
    this->get_parameter("individual_body_config", individual_body_config_);
    std::string kicking_foot_str_;
    this->get_parameter("kick_foot", kicking_foot_str_);
    kicking_foot_ =
      kicking_foot_str_ == "right" ? BodyCommand::RIGHT : BodyCommand::LEFT;
    param_sub_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
    param_event_callback_handle_ = param_sub_->add_parameter_event_callback(
      std::bind(
        &KickCalibration::param_event_callback, this,
        std::placeholders::_1));
  }

private:
  using BodyCommand = runswift_interfaces::msg::BodyCommand;
  using ActionType = runswift_interfaces::msg::BodyCommand;
  using MotionCommand = runswift_interfaces::msg::MotionCommand;
  rclcpp::Subscription<runswift_interfaces::msg::VisionBalls>::SharedPtr
    vision_balls_sub_;
  rclcpp::Publisher<MotionCommand>::SharedPtr motion_command_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr kick_timer_;
  bool kicking_;
  bool kicking_foot_;
  bool goKick_ = false;
  double kick_power_ = 0;
  std::shared_ptr<rclcpp::ParameterEventHandler> param_sub_;
  std::shared_ptr<rclcpp::ParameterEventCallbackHandle>
  param_event_callback_handle_;
  std::string individual_body_config_;

  void vision_balls_callback(
    const runswift_interfaces::msg::VisionBalls::SharedPtr msg)
  {
    if (kicking_) {
      return;
    }
    if (!msg->ball_features.empty()) {
      // Select the ball with the highest confidence
      auto best =
        std::max_element(
        msg->ball_features.begin(), msg->ball_features.end(),
        [](const auto & a, const auto & b) {
          return a.confidence_score < b.confidence_score;
        });
      if (best->ball_coordinates.x <= 300 && best->ball_coordinates.x >= 0 &&
        best->ball_coordinates.y <= 300 && best->ball_coordinates.y >= -300)
      {
        goKick_ = true;
      }
    }
  }

  void param_event_callback(const rcl_interfaces::msg::ParameterEvent & event)
  {
    // Extract changed parameters from the event
    for (const auto & changed_param : event.changed_parameters) {
      if (changed_param.name == "kick_lean_offset_l" ||
        changed_param.name == "kick_lean_offset_r")
      {

        // log the value
        double value = changed_param.value.double_value;
        RCLCPP_INFO(
          this->get_logger(), "%s changed to %f",
          changed_param.name.c_str(), value);
        // write to the config file under section kick
        write_to_config_file(changed_param.name, value);
      }
      if (changed_param.name == "kick_foot") {
        kicking_foot_ = changed_param.value.string_value ==
          "right" ? BodyCommand::RIGHT : BodyCommand::LEFT;
      }
      if (changed_param.name == "power") {
        kick_power_ = changed_param.value.double_value;
      }
    }
    for (const auto & new_param : event.new_parameters) {
      if (new_param.name == "kick_lean_offset_l" ||
        new_param.name == "kick_lean_offset_r")
      {
        // log the value
        RCLCPP_INFO(
          this->get_logger(), "%s registered to %f",
          new_param.name.c_str(), new_param.value.double_value);
      }
    }
  }

  void tick()
  {
    if (kicking_) {
      return;
    }
    if (goKick_) {
      kicking_ = true;
      MotionCommand motion_command;
      motion_command = runswift_interfaces::msg::MotionCommand();
      motion_command.body_command.foot = kicking_foot_;
      motion_command.head_command.pitch = 0.3;
      motion_command.head_command.pitch_speed = 0.3;
      motion_command.body_command.power = kick_power_;
      motion_command.body_command.action_type = ActionType::KICK;
      kick_timer_ = this->create_wall_timer(
        std::chrono::seconds(2), [this]() {
          kicking_ = false;
          goKick_ = false;
        });
      motion_command_pub_->publish(motion_command);
      return;
    } else {
      MotionCommand motion_command;
      motion_command = runswift_interfaces::msg::MotionCommand();
      motion_command.head_command.pitch = 0.3;
      motion_command.head_command.pitch_speed = 0.3;
      motion_command.body_command.action_type = ActionType::WALK;
      motion_command_pub_->publish(motion_command);
      return;
    }
  }

  void write_to_config_file(const std::string & param_name, double value)
  {
    if (individual_body_config_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Config file path not set");
      return;
    }

    // Map parameter names to config keys
    std::string key;
    if (param_name == "kick_lean_offset_l") {
      key = "leanOffsetL";
    } else if (param_name == "kick_lean_offset_r") {
      key = "leanOffsetR";
    } else if (param_name == "kick_lean") {
      key = "kickLean";
    } else {
      RCLCPP_WARN(
        this->get_logger(), "Unknown parameter: %s",
        param_name.c_str());
      return;
    }
    try {
      // Read file line by line
      std::ifstream infile(individual_body_config_);
      if (!infile.is_open()) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Failed to open config file for reading");
        return;
      }
      std::vector<std::string> lines;
      std::string line;
      bool in_kick_section = false;
      bool updated = false;
      while (std::getline(infile, line)) {
        // Check for section markers
        if (line.find("[kick]") == 0) {
          in_kick_section = true;
          lines.push_back(line);
          continue;
        } else if (!line.empty() && line[0] == '[') {
          in_kick_section = false;
        }
        // If in kick section and line contains our key
        if (in_kick_section) {
          size_t key_pos = line.find(key + "=");
          if (key_pos == 0 || (key_pos != std::string::npos &&
            std::isspace(line[key_pos - 1])))
          {
            // Format value with fixed precision
            std::ostringstream ss;
            ss << key << "=" << std::fixed << std::setprecision(6) << value;
            lines.push_back(ss.str());
            updated = true;
            continue;
          }
        }
        // Keep line as is
        lines.push_back(line);
      }
      infile.close();
      if (!updated) {
        RCLCPP_WARN(
          this->get_logger(), "Key '%s' not found in [kick] section",
          key.c_str());
        return;
      }
      // Write back to file
      std::ofstream outfile(individual_body_config_);
      if (!outfile.is_open()) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Failed to open config file for writing");
        return;
      }
      for (const auto & updated_line : lines) {
        outfile << updated_line << std::endl;
      }
      outfile.close();
      RCLCPP_INFO(
        this->get_logger(), "Updated %s = %f in config file",
        key.c_str(), value);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Error updating config: %s", e.what());
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KickCalibration>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
