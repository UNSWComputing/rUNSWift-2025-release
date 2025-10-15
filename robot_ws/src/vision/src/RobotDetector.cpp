#include "vision/RobotDetector.hpp" // Include your header file
#include <functional> // Required for std::bind
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp> // Includes imgproc (drawing) and highgui (display if needed)
#include "rclcpp/rclcpp.hpp"

// For waiting for service
#include <chrono>
#include <future>

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace vision {

RobotDetector::RobotDetector()
    : Node("robot_detector") // Initialize the ROS 2 Node with the name "robot_detector"
{
    // --- Kinematics (placeholders for now) ---
    // camera_fov_horizontal = this->declare_parameter<double>("camera_fov_horizontal", 1.0); // Example param
    // camera_fov_vertical = this->declare_parameter<double>("camera_fov_vertical", 1.0);   // Example param
    // tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    // tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    // --- Service Client Creation ---
    // Create a client for the DetectRobots service
    // Use the correct service name used by the Python node ('detect_robots_srv')
    cls_client = this->create_client<runswift_interfaces::srv::DetectRobots>("detect_robots_srv");

    // Wait for the service to be available (good practice)
    // You can add a timer or do this in a loop during initialization
    RCLCPP_INFO(this->get_logger(), "Waiting for detect_robots_srv service...");
    while (!cls_client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return; // Or throw an exception
        }
        RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
    }
    RCLCPP_INFO(this->get_logger(), "detect_robots_srv service found!");


    // --- Subscribers ---
    // Subscribe to the raw camera image topic
    // Replace "camera_topic" with the actual topic publishing sensor_msgs::msg::Image
    // Use an appropriate QoS profile (e.g., 10)
    subBot = this->create_subscription<sensor_msgs::msg::Image>(
        "camera/top/raw_image",
        1, // QoS history depth
        std::bind(&RobotDetector::onCallbackBot, this, _1) // Bind the callback function
    );
    RCLCPP_INFO(this->get_logger(), "Subscribed to camera_topic");


    // --- Publishers ---
    // Publisher for the debug visualization image
    debug_publisher = this->create_publisher<sensor_msgs::msg::Image>(
        "vision/debug/robot_detector_output",
        10 // QoS history depth
    );
    RCLCPP_INFO(this->get_logger(), "Publishing debug visualization to vision/debug/robot_detector_output");

    // sub for VisionProcessedImageData seems unrelated to this task, not initialized here.
    // vision_robots_publisher for individual RobotDetection msgs seems incorrect based on srv, not initialized here.
}

// Callback for incoming camera images
void RobotDetector::onCallbackBot(const sensor_msgs::msg::Image::SharedPtr msg)
{
    RCLCPP_DEBUG(this->get_logger(), "Received image message. Calling robot detection service.");

    // Check if the service client is ready
    if (!cls_client->service_is_ready()) {
         RCLCPP_WARN(this->get_logger(), "Robot detection service not ready.");
         return;
    }

    // Create the service request
    auto request = std::make_shared<runswift_interfaces::srv::DetectRobots::Request>();
    // Copy the incoming image message into the request
    request->image = *msg; // Dereference the shared pointer to copy the Image message

    // Send the request asynchronously
    // We provide a callback function (lambda) to handle the response when it arrives.
    // We capture 'this' and the original 'msg' shared pointer so they are available in the lambda.
    cls_client->async_send_request(request,
        [this, msg](rclcpp::Client<runswift_interfaces::srv::DetectRobots>::SharedFuture future) {
            // This lambda executes when the service response is received

            auto response = future.get(); // Get the response from the future

            // Check if the service call itself succeeded (communication level)
            if (!response) {
                RCLCPP_ERROR(this->get_logger(), "Robot detection service call failed.");
                // No response received, cannot visualize
                return;
            }

            // Check the 'success' flag from the service response (application level)
            if (!response->success) {
                RCLCPP_WARN(this->get_logger(), "Robot detection service reported failure: %s", response->message.c_str());
                // Service processed the image but found an issue, cannot visualize results
                return;
            }

            RCLCPP_DEBUG(this->get_logger(), "Received robot detection service response with %zu detections.", response->detections.size());

            // --- Visualization ---
            // We need the original image to draw on. We captured the shared pointer 'msg'.
            cv_bridge::CvImagePtr cv_ptr;
            try {
                // Convert the ROS Image message to an OpenCV image (CvImage pointer)
                // Request "bgr8" encoding for standard OpenCV drawing operations
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            } catch (const cv_bridge::Exception& e) {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
                return; // Cannot visualize without a valid OpenCV image
            }

            // Get the OpenCV Mat object from the CvImage pointer
            cv::Mat& image_bgr = cv_ptr->image;

            // Iterate through the detected robots in the response
            for (const auto& detection : response->detections) {
                // Extract bounding box coordinates and confidence
                int x1 = static_cast<int>(detection.x1);
                int y1 = static_cast<int>(detection.y1);
                int x2 = static_cast<int>(detection.x2);
                int y2 = static_cast<int>(detection.y2);
                float confidence = detection.confidence;

                // Clamp coordinates to image bounds just in case
                x1 = std::max(0, x1);
                y1 = std::max(0, y1);
                x2 = std::min(image_bgr.cols, x2);
                y2 = std::min(image_bgr.rows, y2);


                // Determine box color based on fallen status (if available in the message)
                cv::Scalar box_color(0, 0, 255); // Default Red (BGR)
                std::string label_text = std::to_string(static_cast<int>(confidence * 100)) + "%";

                // Check if the 'is_fallen' field exists and is true
                // This requires including runswift_interfaces/msg/RobotDetection.hpp
                // and checking the message structure if it's variable
                // Assuming the RobotDetection message always has 'is_fallen' due to definition:
                if (detection.is_fallen) {
                    box_color = cv::Scalar(255, 0, 0); // Blue (BGR) for fallen
                    label_text += " (Fallen)";
                } else {
                    label_text += " (Upright)";
                }


                // Draw the bounding box rectangle
                cv::rectangle(image_bgr, cv::Point(x1, y1), cv::Point(x2, y2), box_color, 2);

                // Put text label (confidence + status) above the box
                int text_y = std::max(15, y1 - 5); // Position text slightly above the box
                cv::putText(image_bgr, label_text, cv::Point(x1, text_y),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, box_color, 1);

                RCLCPP_DEBUG(this->get_logger(), "Drew box for robot at (%d,%d)-(%d,%d) with conf %.2f, fallen: %s",
                             x1, y1, x2, y2, confidence, detection.is_fallen ? "true" : "false");

            } // End loop through detections

            // --- Publish the visualized image ---
            // Convert the modified OpenCV image back to a ROS Image message
            try {
                // Reuse the header from the original message
                cv_ptr->header = msg->header;
                // Publish the image
                debug_publisher->publish(*cv_ptr->toImageMsg());
                RCLCPP_DEBUG(this->get_logger(), "Published debug visualization image.");
            } catch (const cv_bridge::Exception& e) {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge exception when converting back to Image msg: %s", e.what());
            }

        }); // End of async_send_request lambda

    RCLCPP_DEBUG(this->get_logger(), "Service request sent for image with timestamp %s",
                 std::to_string(msg->header.stamp.sec).c_str());
}
}
// --- Main function to spin the node ---

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<vision::RobotDetector>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

