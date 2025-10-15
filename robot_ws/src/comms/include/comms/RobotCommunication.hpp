
#include "rclcpp/rclcpp.hpp"
#include <runswift_interfaces/msg/comms_teamstate.hpp>

// add custom imports below
// end custom imports
namespace comms {
class RobotCommunication : public rclcpp::Node
{
    public:
        RobotCommunication();
        // add custom public methods below

        // end custom public methods 
    private:
        rclcpp::Publisher<runswift_interfaces::msg::CommsTeamstate>::SharedPtr robot_info_pub;

        // add custom private methods below

        // end custom private methods
};
}
