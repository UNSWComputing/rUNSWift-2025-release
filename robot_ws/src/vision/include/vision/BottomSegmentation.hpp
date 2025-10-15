#include <rclcpp/rclcpp.hpp>
#include <runswift_interfaces/msg/vision_balls.hpp>
#include <runswift_interfaces/msg/vision_obstacles.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/point2_d.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <CompiledNN/Model.h>
#include <CompiledNN/CompiledNN.h>
#include <optional>
#include <map>
#include <string>

// Use constexpr for type-safe, modern constants
constexpr double MULTIPLY_M_TO_MM = 1000.0;
constexpr double MAX_PIXEL_DISTANCE_BETWEEN_BALLS = 60.0;
constexpr int MIN_BOTTOM_BALL_CONFIDENCE_FOR_MERGE = 200;

// Helper macro for converting degrees to radians
#ifndef DEG2RAD
#define DEG2RAD(x) ((x) * M_PI / 180.0)
#endif

// Struct to hold configuration for each detection class
struct ClassConfig {
    std::string name;
    double threshold;
    cv::Scalar color;
};

namespace vision {

class BotSegmentation : public rclcpp::Node {
public:
    explicit BotSegmentation(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    // --- ROS-related Methods ---
    void onCameraImage(const sensor_msgs::msg::Image::SharedPtr msg);
    std::optional<geometry_msgs::msg::Point> projectRayToGround(
        const vision_msgs::msg::Point2D& pixel_coords,
        const uint img_width,
        const uint img_height,
        const std::string& camera_frame,
        const builtin_interfaces::msg::Time& header_stamp,
        geometry_msgs::msg::TransformStamped transform,
        float ground_plane_z
    );

    // --- Neural Network Methods ---
    void loadModel();
    void processDetections(const sensor_msgs::msg::Image& input_msg, const float* output_data);
    void processBall(const float* data,
                     std::vector<vision_msgs::msg::Point2D>& ball_pixels,
                     std::vector<double>& ball_confidences);
    void processObstacles(const float* data,
                          std::vector<std::pair<vision_msgs::msg::Point2D, vision_msgs::msg::Point2D>>& obstacle_pixel_bases);
    
    // --- Visualization ---
    void publishVisualizations(const sensor_msgs::msg::Image& input_msg, const float* output_data,
                               const std::vector<vision_msgs::msg::Point2D>& ball_pixels,
                               const std::vector<std::pair<vision_msgs::msg::Point2D, vision_msgs::msg::Point2D>>& obstacle_pixel_bases);
    void publishHeatmap(const float* data, int channel_index, const std::string& topic_name, const std_msgs::msg::Header& header);

    // --- ROS Components ---
    rclcpp::Publisher<runswift_interfaces::msg::VisionBalls>::SharedPtr vision_balls_publisher_;
    rclcpp::Publisher<runswift_interfaces::msg::VisionObstacles>::SharedPtr vision_obstacles_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_bot_;
    std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> debug_publishers_;
    
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // --- Neural Network Components ---
    NeuralNetwork::CompiledNN nn_;
    Eigen::Vector2i input_size_;
    Eigen::Vector2i output_size_;
    Eigen::Vector2i scales_;
    int num_output_channels_;
    cv::Mat obstacle_heatmap_; // Pre-allocated for efficiency

    // --- Parameters and Configuration ---
    std::map<int, ClassConfig> class_config_map_;
    double camera_fov_horizontal_;
    double camera_fov_vertical_;
    double ground_plane_z_;
    bool enable_debug_;

    // --- Constants for channel indices ---
    static constexpr int BALL_CHANNEL_INDEX = 0;
    static constexpr int OBSTACLE_CHANNEL_INDEX = 2;
};

} 