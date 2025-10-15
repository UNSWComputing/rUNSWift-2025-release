
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8.hpp"
#include <runswift_interfaces/msg/comms_gamestate.hpp>

// add custom imports below
// end custom imports
namespace comms {
class GameInfo : public rclcpp::Node
{
    public:
        GameInfo();
        // add custom public methods below

        // end custom public methods 
    private:
        rclcpp::Publisher<runswift_interfaces::msg::CommsGamestate>::SharedPtr game_info_pub;

        // add custom private methods below

        // end custom private methods
};
}
