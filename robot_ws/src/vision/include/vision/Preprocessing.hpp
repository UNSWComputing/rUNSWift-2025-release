#include "rclcpp/rclcpp.hpp"
#include "runswift_interfaces/msg/vision_processed_image_data.hpp"
#include "runswift_interfaces/msg/vision_image_with_field_boundry.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
// add custom imports below
// end custom imports
namespace vision {
class Preprocessing : public rclcpp::Node
{
    public:
        Preprocessing();
        // add custom public methods below
        // params for performance
    private:
        rclcpp::Subscription<runswift_interfaces::msg::VisionImageWithFieldBoundry>::SharedPtr camera_raw_sub;
        void imageCallback(const std::shared_ptr<runswift_interfaces::msg::VisionImageWithFieldBoundry> msg);
        cv::Mat makeBinary(
            cv::Mat grayImage,
            int windowSize, 
            int percentage 
        );
        rclcpp::Publisher<runswift_interfaces::msg::VisionProcessedImageData>::SharedPtr publisher;
        // image is a binarisied image
        void getROIs(cv::Mat image, runswift_interfaces::msg::VisionProcessedImageData* processed_image_data, std::vector<float> field_boundry);
        void processImageRegions(const uint8_t *rawImage, uint8_t *output,
                                        const int *integralImg, 
                                        int width, int height, int doubleDensity,
                                        int rowSize, int sHalf, int t);
        struct FieldParams {
            int sample_rows = 30;  // Number of rows to sample for field color
            float value_min = 30.0f;  // Minimum brightness
            float value_max = 220.0f;  // Maximum brightness
        };
        FieldParams params_;
        cv::Mat field_mask_;
        cv::Mat morphology_kernel_;
};
}
