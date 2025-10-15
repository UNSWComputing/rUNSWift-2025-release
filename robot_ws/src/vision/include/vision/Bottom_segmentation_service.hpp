#include <rclcpp/rclcpp.hpp>
#include <runswift_interfaces/srv/bot_model.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <CompiledNN/Model.h>
#include <CompiledNN/CompiledNN.h>
#include <map>
#include <string>
#include <Eigen/Core>
#include <vector>

// Config for each detection class
struct ClassConfig {
    std::string name;
    double threshold;
    cv::Scalar color;
};

class BottomSegmentationService : public rclcpp::Node
{
public:
    BottomSegmentationService(const rclcpp::NodeOptions & options);

private:
    rclcpp::Service<runswift_interfaces::srv::BotModel>::SharedPtr service_;
    std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> debug_publishers_;

    NeuralNetwork::CompiledNN nn;
    int input_height_;
    int input_width_;
    int output_height_;
    int output_width_;
    int num_output_channels_;
    Eigen::Vector2i outputSize;
    Eigen::Vector2i inputSize;
    Eigen::Vector2i scales;
    // Configuration
    std::map<int, ClassConfig> class_config_map_;

    // Core callback functions
    void loadModel();
    void modelServiceCallback(
        const std::shared_ptr<runswift_interfaces::srv::BotModel::Request> request,
        std::shared_ptr<runswift_interfaces::srv::BotModel::Response> response);

    // Post-processing helper functions
    void processBall(const float* data,
                     runswift_interfaces::srv::BotModel::Response& response,
                     const Eigen::Vector2i& scales);

    void processObstacles(const float* data,
                          runswift_interfaces::srv::BotModel::Response& response,
                          const Eigen::Vector2i& scales,
                          cv::Mat* overlay_image); // Pointer to allow modification

    void publishHeatmapVisualizations(const cv::Mat& heatmaps,
                                    const std_msgs::msg::Header& header);
};