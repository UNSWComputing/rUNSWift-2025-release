
#include "rclcpp/rclcpp.hpp"
#include "runswift_interfaces/msg/vision_processed_image_data.hpp"

// add custom imports below
// end custom imports
namespace vision {
class TopCamCalib : public rclcpp::Node
{
    public:
        TopCamCalib();
        // add custom public methods below

        // end custom public methods 
    private:
        void cameratopready_image_callback(const std::shared_ptr<runswift_interfaces::msg::VisionProcessedImageData> msg);
        rclcpp::Subscription<runswift_interfaces::msg::VisionProcessedImageData>::SharedPtr cameratopready_image_sub;

        // add custom private methods below

        // end custom private methods
};
}
