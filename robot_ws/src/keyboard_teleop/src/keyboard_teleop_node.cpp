#include "rclcpp/rclcpp.hpp"
#include "runswift_interfaces/msg/motion_command.hpp"
#include "runswift_interfaces/msg/body_command.hpp"
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>

class KeyboardTeleop : public rclcpp::Node
{
public:
  using BodyCommand = runswift_interfaces::msg::BodyCommand;

  KeyboardTeleop()
  : Node("keyboard_teleop")
  {
    publisher_ = this->create_publisher<runswift_interfaces::msg::MotionCommand>(
      "/motion_command",
      10);

    // Initialize message with zeros and empty action type
    msg_ = runswift_interfaces::msg::MotionCommand();
    msg_.body_command.twist.linear.x = 0.0;
    msg_.body_command.twist.linear.y = 0.0;
    msg_.body_command.twist.angular.z = 0.0;
    msg_.body_command.action_type = "";

    // Initialize head command
    msg_.head_command.yaw = 0.0;
    msg_.head_command.pitch = 0.0;
    msg_.head_command.is_relative = true;  // Always use relative movement
    msg_.head_command.yaw_speed = 0.2;     // Set fixed speed to 0.2
    msg_.head_command.pitch_speed = 0.2;   // Set fixed speed to 0.2

    // Set up terminal for non-blocking input
    init_keyboard();

    // Create two timers:
    // 1. For publishing messages continuously
    publish_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(30),  // 30ms = ~33Hz
      std::bind(&KeyboardTeleop::publish_callback, this));

    // 2. For checking keyboard input
    input_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),  // 100ms = 10Hz
      std::bind(&KeyboardTeleop::input_callback, this));

    send_instructions();
  }

  ~KeyboardTeleop()
  {
    // Restore terminal settings
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt_);
  }

private:
  void publish_callback()
  {
    // Print current values before publishing
    RCLCPP_INFO(
      this->get_logger(),
      "Publishing - Linear: [x: %.2f, y: %.2f] Angular: [z: %.2f] Action: [%s]",
      msg_.body_command.twist.linear.x,
      msg_.body_command.twist.linear.y,
      msg_.body_command.twist.angular.z,
      msg_.body_command.action_type.c_str());
    RCLCPP_INFO(this->get_logger(),
      "    Kick power: [%s]",
      msg_.body_command.power == 0.0 ? "None" : std::to_string(msg_.body_command.power).c_str());
    RCLCPP_INFO(this->get_logger(),
      "    Kick foot: [%s]",
      msg_.body_command.foot == BodyCommand::LEFT ? "Left" : "Right");
    publisher_->publish(msg_);
  }

  void input_callback()
  {
    char c = getch();

    if (c == EOF) {return;}

    // head command
    if (c == '\033') {
      // Skip the '['
      getch();
      
      // Get the actual key code
      char arrow = getch();
      
      switch (arrow) {
        case 'A': // Up arrow
          msg_.head_command.pitch += 0.1;  // Increment pitch (look up)
          break;
        case 'B': // Down arrow
          msg_.head_command.pitch -= 0.1;  // Decrement pitch (look down)
          break;
        case 'C': // Right arrow
          msg_.head_command.yaw -= 0.1;    // Decrement yaw (look right)
          break;
        case 'D': // Left arrow
          msg_.head_command.yaw += 0.1;    // Increment yaw (look left)
          break;
      }
      return;
    }

    switch (c) {
      case 'w':
        msg_.body_command.twist.linear.x += 10;
        msg_.body_command.action_type = BodyCommand::WALK;
        break;
      case 's':
        msg_.body_command.twist.linear.x -= 10;
        msg_.body_command.action_type = BodyCommand::WALK;
        break;
      case 'd':
        msg_.body_command.twist.linear.y -= 10;
        msg_.body_command.action_type = BodyCommand::WALK;
        break;
      case 'a':
        msg_.body_command.twist.linear.y += 10;
        msg_.body_command.action_type = BodyCommand::WALK;
        break;
      case 'e':
        msg_.body_command.twist.angular.z -= 0.1;
        msg_.body_command.action_type = BodyCommand::WALK;
        break;
      case 'q':
        msg_.body_command.twist.angular.z += 0.1;
        msg_.body_command.action_type = BodyCommand::WALK;
        break;
      case ' ':
        set_twist_zero();
        msg_.body_command.action_type = "";
        break;
      case 'u':
        set_twist_zero();
        msg_.body_command.action_type = BodyCommand::STAND;
        break;
      case 'j':
        set_twist_zero();
        msg_.body_command.action_type = BodyCommand::SIT;
        break;
      case 'g':
        set_twist_zero();
        msg_.body_command.action_type = BodyCommand::KICK;
        msg_.body_command.power = 0;
        msg_.body_command.foot = BodyCommand::LEFT;
        break;
      case 'h':
        set_twist_zero();
        msg_.body_command.action_type = BodyCommand::KICK;
        msg_.body_command.power = 0;
        msg_.body_command.foot = BodyCommand::RIGHT;
        break;
      case 't':
        set_twist_zero();
        msg_.body_command.action_type = BodyCommand::KICK;
        msg_.body_command.power = 1;
        msg_.body_command.foot = BodyCommand::LEFT;
        break;
      case 'y':
        set_twist_zero();
        msg_.body_command.action_type = BodyCommand::KICK;
        msg_.body_command.power = 1;
        msg_.body_command.foot = BodyCommand::RIGHT;
        break;
      case 'p':
        set_twist_zero();
        msg_.body_command.power += 0.05;
        break;
      case 'o':
        set_twist_zero();
        msg_.body_command.power -= 0.05;
        break;
      case 'k':
        set_twist_zero();
        msg_.body_command.foot = BodyCommand::LEFT;
        msg_.body_command.action_type = BodyCommand::KICK;
        break;
      case 'l':
        set_twist_zero();
        msg_.body_command.foot = BodyCommand::RIGHT;
        msg_.body_command.action_type = BodyCommand::KICK;
        break;
      default:
        break;
    }
  }

  void send_instructions()
  {
    puts("Reading from keyboard");
    puts("---------------------------");
    puts("Use arrow keys to move the nao.");
    puts("---------------------------");
    puts("u: stand");
    puts("j: sit");
    puts("---------------------------");
    puts("w: increases forward speed");
    puts("s: decreases forward speed, and allows going back too");
    puts("a: increases left speed");
    puts("d: decreases left speed, and allows going right too");
    puts("q: increases turn left");
    puts("e: decreases turn left, and allows turning right too");
    puts("---------------------------");
    puts("t: use left foot to kick with power=1");
    puts("y: use right foot to kick with power=1");
    puts("g: use left foot for kick with power=0");
    puts("h: use right foot to kick with power=0");
    puts("k: use left foot for kick with current power");
    puts("l: use right foot for kick with current power");
    puts("---------------------------");
    puts("'SPACE' is set walking speeds to 0");
    puts("---------------------------");
  }

  void init_keyboard()
  {
    // Get the current terminal settings
    tcgetattr(STDIN_FILENO, &oldt_);
    newt_ = oldt_;

    // Disable canonical mode and echo
    newt_.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt_);

    // Set stdin to non-blocking
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
  }

  char getch()
  {
    char ch;
    int n = read(STDIN_FILENO, &ch, 1);
    if (n == 1) {
      return ch;
    }
    return EOF;
  }

  void set_twist_zero()
  {
    msg_.body_command.twist.linear.x = 0.0;
    msg_.body_command.twist.linear.y = 0.0;
    msg_.body_command.twist.angular.z = 0.0;
  }

  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr input_timer_;
  rclcpp::Publisher<runswift_interfaces::msg::MotionCommand>::SharedPtr publisher_;
  runswift_interfaces::msg::MotionCommand msg_;
  struct termios oldt_, newt_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KeyboardTeleop>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
