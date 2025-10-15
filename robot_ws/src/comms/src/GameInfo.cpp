
#include "comms/GameInfo.hpp"

namespace comms {

GameInfo::GameInfo() : Node("game_info") {
    game_info_pub = create_publisher<runswift_interfaces::msg::CommsGamestate>("GameInfo", 10);

    // add additional constructor functionality below
    
    // end additional constructor functionality
}


// add custom method implementations below

// end custom method implementations
}
int main(int argc, char * argv[])
{
    // customize main function below
    // Note: editing any of the generated code here could break the node.
    // take precaution when editing
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<comms::GameInfo>());
    rclcpp::shutdown();
    return 0;
    // end main customization
}
