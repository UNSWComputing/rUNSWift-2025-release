#include "rclcpp/rclcpp.hpp"
#include "runswift_interfaces/msg/vision_processed_image_data.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include "runswift_interfaces/msg/robot_detection.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#define DEG2RAD(x) ((x) * M_PI / 180.0)

#include <vector>
#include <utility>
#include <cmath>
#include <numeric>

#include "runswift_interfaces/srv/detect_robots.hpp"


namespace vision {


class RobotDetector : public rclcpp::Node
{
    public:
        RobotDetector();

    private: 
        // KINEMATICS CODE START
        double camera_fov_horizontal;
        double camera_fov_vertical;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener;

        geometry_msgs::msg::Point projectRayToGround(const vision_msgs::msg::Point2D& pixel_coords, 
                                               const uint img_width, 
                                               const uint img_height,
                                               const std::string camera_frame,
                                               const builtin_interfaces::msg::Time& header_stamp);


        // END KINEMATICS CODE
        void onCallbackBot(const sensor_msgs::msg::Image::SharedPtr msg);
        rclcpp::Subscription<runswift_interfaces::msg::VisionProcessedImageData>::SharedPtr sub;
        rclcpp::Client<runswift_interfaces::srv::DetectRobots>::SharedPtr cls_client;
        rclcpp::Publisher<runswift_interfaces::msg::RobotDetection>::SharedPtr vision_robots_publisher;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subBot;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_publisher;

};
}
