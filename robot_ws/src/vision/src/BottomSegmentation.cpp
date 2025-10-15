#include "vision/BottomSegmentation.hpp"
#include <filesystem>

namespace vision {

BotSegmentation::BotSegmentation(const rclcpp::NodeOptions & options)
    : Node("BotSegmentation", options)
{
    // --- Initialize Parameters ---
    this->declare_parameter("enable_debug", false);
    this->declare_parameter("camera_fov_horizontal", 56.3);
    this->declare_parameter("camera_fov_vertical", 43.7);

    enable_debug_ = this->get_parameter("enable_debug").as_bool();
    camera_fov_horizontal_ = this->get_parameter("camera_fov_horizontal").as_double();
    camera_fov_vertical_ = this->get_parameter("camera_fov_vertical").as_double();


    RCLCPP_INFO(this->get_logger(), "Debug mode is: %s", enable_debug_ ? "ON" : "OFF");

    // --- Initialize Class Configuration ---
    class_config_map_[BALL_CHANNEL_INDEX] = {"ball", 0.75, cv::Scalar(0, 255, 0)};
    class_config_map_[OBSTACLE_CHANNEL_INDEX] = {"obstacle", 0.85, cv::Scalar(0, 0, 255)};

    // --- Load the Neural Network Model ---
    loadModel();

    // --- Initialize ROS Components ---
    vision_balls_publisher_ = this->create_publisher<runswift_interfaces::msg::VisionBalls>("/vision/VBalls", 1);
    vision_obstacles_publisher_ = this->create_publisher<runswift_interfaces::msg::VisionObstacles>("/vision/VObstacles", 1);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    // tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, false, rclcpp::QoS(1).reliability(rclcpp::ReliabilityPolicy::BestEffort));

    sub_bot_ = this->create_subscription<sensor_msgs::msg::Image>(
        "camera/bot/raw_image", 1, std::bind(&BotSegmentation::onCameraImage, this, std::placeholders::_1));

    if (enable_debug_) {
        debug_publishers_["overlay"] = this->create_publisher<sensor_msgs::msg::Image>("/vision/debug_bottom/overlay", 10);
        for (const auto& pair : class_config_map_) {
            debug_publishers_[pair.second.name] = this->create_publisher<sensor_msgs::msg::Image>("/vision/debug_bottom/heatmap_" + pair.second.name, 10);
        }
    }

    RCLCPP_INFO(this->get_logger(), "Bottom Segmentation node started!");
}

void BotSegmentation::loadModel()
{
    std::string model_path;
    const std::vector<std::string> possible_paths = {"/home/nao/vision_models/net.h5", "/workspace/vision_models/net.h5"};
    for (const auto& path : possible_paths) {
        if (std::filesystem::exists(path)) { model_path = path; break; }
    }
    if (model_path.empty()) {
        RCLCPP_FATAL(this->get_logger(), "Model file (.h5) not found!");
        throw std::runtime_error("Model file could not be found.");
    }

    RCLCPP_INFO(this->get_logger(), "Loading model from: %s", model_path.c_str());
    try {
        NeuralNetwork::Model model(model_path);
        model.setInputUInt8(0);
        nn_.compile(model);

        const auto& input_desc = Eigen::Vector2i(nn_.input(0).dims(1), nn_.input(0).dims(0));
        input_size_ = Eigen::Vector2i(input_desc.x(), input_desc.y());

        const auto& output_desc = Eigen::Vector2i(nn_.output(0).dims(1), nn_.output(0).dims(0));
        output_size_ = Eigen::Vector2i(output_desc.x(), output_desc.y());
        num_output_channels_ = 4; //nn_.output(0).dims(2);

        scales_ = Eigen::Vector2i(input_size_.x() / output_size_.x(), input_size_.y() / output_size_.y());

        // Pre-allocate the obstacle heatmap for efficiency
        obstacle_heatmap_ = cv::Mat(output_size_.y(), output_size_.x(), CV_32FC1);

        RCLCPP_INFO(this->get_logger(), "Model loaded. Input: [%d, %d]. Output: [%d, %d, %d].",
                    input_size_.y(), input_size_.x(), output_size_.y(), output_size_.x(), num_output_channels_);

    } catch (const std::exception& e) {
        RCLCPP_FATAL(this->get_logger(), "Failed to load/compile model: %s", e.what());
        throw;
    }
}

void BotSegmentation::onCameraImage(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try {
        // --- 1. Run Inference ---
        auto start_time = this->get_clock()->now();
        memcpy(nn_.input(0).data(), msg->data.data(), input_size_.x() * input_size_.y() * 2);
        nn_.apply();
        const float* output_data = nn_.output(0).data();
        auto inference_end_time = this->get_clock()->now();
        // RCLCPP_INFO(this->get_logger(), "Inference time: %.2f ms", (inference_end_time - start_time).seconds() * 1000.0);

        // --- 2. Process Detections ---
        processDetections(*msg, output_data);

    } catch (const std::exception& e) {
        // RCLCPP_ERROR(this->get_logger(), "Camera callback failed: %s", e.what());
    }
}

void BotSegmentation::processDetections(const sensor_msgs::msg::Image& input_msg, const float* output_data)
{
    // --- Prepare storage for raw pixel detections ---
    std::vector<vision_msgs::msg::Point2D> ball_pixels;
    std::vector<double> ball_confidences;
    std::vector<std::pair<vision_msgs::msg::Point2D, vision_msgs::msg::Point2D>> obstacle_pixel_bases;

    // --- Run detection logic ---
    auto start_time = this->get_clock()->now();
    processBall(output_data, ball_pixels, ball_confidences);
    auto ball_end_time = this->get_clock()->now();
    // RCLCPP_INFO(this->get_logger(), "Ball process %.2f ms", (ball_end_time - start_time).seconds() * 1000.0);
    start_time = this->get_clock()->now();
    processObstacles(output_data, obstacle_pixel_bases);
    auto obstacle_end_time = this->get_clock()->now();
    // RCLCPP_INFO(this->get_logger(), "Obstacle process %.2f ms", (obstacle_end_time - start_time).seconds() * 1000.0);

    // --- Prepare ROS messages for publishing ---
    runswift_interfaces::msg::VisionBalls output_balls_msg;
    output_balls_msg.header = input_msg.header;
    output_balls_msg.header.frame_id = "base_footprint_from_bottom";

    runswift_interfaces::msg::VisionObstacles output_obstacles_msg;
    output_obstacles_msg.header = input_msg.header;
    output_obstacles_msg.header.frame_id = "base_footprint_from_bottom";

    geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform("base_footprint",  "CameraBottomMotion", tf2::TimePointZero );

    // --- Project ball pixels to ground and populate message ---
    start_time = this->get_clock()->now();
    for (size_t i = 0; i < ball_pixels.size(); ++i) {
        if (auto world_coords_opt = projectRayToGround(ball_pixels[i], input_msg.width, input_msg.height, "CameraBottomMotion", input_msg.header.stamp, transform, 0.05)) { // to add transform hpp, cpp
            runswift_interfaces::msg::VisionBallFeature ball_feature;
            ball_feature.ball_coordinates.x = world_coords_opt->x * MULTIPLY_M_TO_MM;
            ball_feature.ball_coordinates.y = world_coords_opt->y * MULTIPLY_M_TO_MM;
            ball_feature.ball_coordinates.z = world_coords_opt->z * MULTIPLY_M_TO_MM;
            ball_feature.ball_pixel_coordinates.x = ball_pixels[i].x;
            ball_feature.ball_pixel_coordinates.y = ball_pixels[i].y;
            ball_feature.confidence_score = static_cast<uint8_t>(std::max(0.0, std::min(1.0, ball_confidences[i])) * 255.0);
            output_balls_msg.ball_features.push_back(ball_feature);
        }
    }
    auto ball_projection_end_time = this->get_clock()->now();
    // RCLCPP_INFO(this->get_logger(), "Ball projection time: %.2f ms", (ball_projection_end_time - start_time).seconds() * 1000.0);

    start_time = this->get_clock()->now();
    // --- Project obstacle pixels to ground and populate message ---
    for (const auto& base : obstacle_pixel_bases) {
        auto left_world_opt = projectRayToGround(base.first, input_msg.width, input_msg.height, "CameraBottomMotion", input_msg.header.stamp, transform, 0.00);
        auto right_world_opt = projectRayToGround(base.second, input_msg.width, input_msg.height, "CameraBottomMotion", input_msg.header.stamp, transform, 0.00);
        if (left_world_opt && right_world_opt) {
            runswift_interfaces::msg::VisionObstacleFeature obstacle_feature;
            obstacle_feature.leftmost_coordinates = *left_world_opt;
            obstacle_feature.rightmost_coordinates = *right_world_opt;
            output_obstacles_msg.obstacles.push_back(obstacle_feature);
        }
    }
    auto obstacle_projection_end_time = this->get_clock()->now();
    // RCLCPP_INFO(this->get_logger(), "Obstacle projection time: %.2f ms", (obstacle_projection_end_time - start_time).seconds() * 1000.0);

    // --- Publish Results ---
    if (! output_balls_msg.ball_features.empty()){
        vision_balls_publisher_->publish(output_balls_msg);
    }
    if (! output_obstacles_msg.obstacles.empty()){
        vision_obstacles_publisher_->publish(output_obstacles_msg);
    }


    // --- Handle Visualizations (only if enabled) ---
    if (enable_debug_) {
        publishVisualizations(input_msg, output_data, ball_pixels, obstacle_pixel_bases);
    }
}

// ... (projectRayToGround function from your previous code, with std::optional fix) ...
std::optional<geometry_msgs::msg::Point> BotSegmentation::projectRayToGround(
    const vision_msgs::msg::Point2D& pixel_coords, const uint img_width, const uint img_height,
    const std::string& camera_frame, const builtin_interfaces::msg::Time& header_stamp, geometry_msgs::msg::TransformStamped transform, float ground_plane_z_)
{
    try {
        // tf_buffer_->lookupTransform("base_footprint", camera_frame, tf2_ros::fromMsg(header_stamp), tf2::durationFromSec(0.1));
        // geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform("base_footprint", camera_frame, tf2::TimePointZero, tf2::durationFromSec(0.1));

        float horizontal_angle = -static_cast<float>(DEG2RAD(camera_fov_horizontal_)) * (pixel_coords.x - img_width / 2.0f) / img_width;
        float vertical_angle = -static_cast<float>(DEG2RAD(camera_fov_vertical_)) * (pixel_coords.y - img_height / 2.0f) / img_height;
        geometry_msgs::msg::PointStamped camera_point;
        camera_point.point.x = 1.0;
        camera_point.point.y = tan(horizontal_angle);
        camera_point.point.z = tan(vertical_angle);
        geometry_msgs::msg::PointStamped camera_origin_in_base; // move up?
        tf2::doTransform(geometry_msgs::msg::PointStamped(), camera_origin_in_base, transform); // move up?
        geometry_msgs::msg::PointStamped point_on_ray_in_base;
        tf2::doTransform(camera_point, point_on_ray_in_base, transform);
        Eigen::Vector3d base_camera_vector(camera_origin_in_base.point.x, camera_origin_in_base.point.y, camera_origin_in_base.point.z);
        Eigen::Vector3d point_on_ray_vector(point_on_ray_in_base.point.x, point_on_ray_in_base.point.y, point_on_ray_in_base.point.z);
        Eigen::Vector3d direction_vector = (point_on_ray_vector - base_camera_vector).normalized();
        if (std::abs(direction_vector.z()) < 1e-6) return std::nullopt;
        double t = (ground_plane_z_ - base_camera_vector.z()) / direction_vector.z();
        if (t < 0.0) return std::nullopt;
        Eigen::Vector3d intersection = base_camera_vector + t * direction_vector;
        geometry_msgs::msg::Point world_point;
        world_point.x = intersection.x();
        world_point.y = intersection.y();
        world_point.z = ground_plane_z_;
        return world_point;
    } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "TF2 transform failed from %s: %s", camera_frame.c_str(), ex.what());
        return std::nullopt;
    }
}


void BotSegmentation::processBall(const float* data,
                                std::vector<vision_msgs::msg::Point2D>& ball_pixels,
                                std::vector<double>& ball_confidences)
{
    const float* ball_data = data + BALL_CHANNEL_INDEX;
    cv::Point max_loc(-1, -1);
    float max_val = class_config_map_.at(BALL_CHANNEL_INDEX).threshold;

    for (int y = 0; y < output_size_.y(); ++y) {
        for (int x = 0; x < output_size_.x(); ++x) {
            if (*ball_data > max_val) {
                max_val = *ball_data;
                max_loc = cv::Point(x, y);
            }
            ball_data += num_output_channels_;
        }
    }

    if (max_loc.x != -1) {
        // Step 1: Create a temporary Point2D object
        vision_msgs::msg::Point2D new_ball_pixel;

        // Step 2: Explicitly set its members
        new_ball_pixel.x = static_cast<double>((max_loc.x + 0.5) * scales_.x());
        new_ball_pixel.y = static_cast<double>((max_loc.y + 0.5) * scales_.y());

        // Step 3: Push the fully-constructed object into the vector
        ball_pixels.push_back(new_ball_pixel);

        ball_confidences.push_back(max_val);
    }
}

void BotSegmentation::processObstacles(const float* data,
                                     std::vector<std::pair<vision_msgs::msg::Point2D, vision_msgs::msg::Point2D>>& obstacle_pixel_bases)
{
    const float* obstacle_data = data + OBSTACLE_CHANNEL_INDEX;
    for (int y = 0; y < output_size_.y(); ++y) {
        float* row_ptr = obstacle_heatmap_.ptr<float>(y);
        for (int x = 0; x < output_size_.x(); ++x) {
            row_ptr[x] = *obstacle_data;
            obstacle_data += num_output_channels_;
        }
    }

    const auto& config = class_config_map_.at(OBSTACLE_CHANNEL_INDEX);
    cv::Mat binary_map;
    cv::threshold(obstacle_heatmap_, binary_map, config.threshold, 255, cv::THRESH_BINARY);
    binary_map.convertTo(binary_map, CV_8U);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary_map, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (const auto& cnt : contours) {
        cv::Rect box = cv::boundingRect(cnt);

        vision_msgs::msg::Point2D left_base_px, right_base_px;
        double bottom_y_px = (box.y + box.height) * scales_.y();

        left_base_px.x = box.x * scales_.x();
        left_base_px.y = bottom_y_px;
        right_base_px.x = (box.x + box.width) * scales_.x();
        right_base_px.y = bottom_y_px;

        obstacle_pixel_bases.push_back({left_base_px, right_base_px});
    }
}


void BotSegmentation::publishVisualizations(const sensor_msgs::msg::Image& input_msg, const float* output_data,
                                           const std::vector<vision_msgs::msg::Point2D>& ball_pixels,
                                           const std::vector<std::pair<vision_msgs::msg::Point2D, vision_msgs::msg::Point2D>>& obstacle_pixel_bases)
{
    if (debug_publishers_["overlay"]->get_subscription_count() == 0 &&
        debug_publishers_["ball"]->get_subscription_count() == 0 &&
        debug_publishers_["obstacle"]->get_subscription_count() == 0) {
        return; // No one is watching, so don't do any work
    }

    // --- Publish Heatmaps ---
    publishHeatmap(output_data, BALL_CHANNEL_INDEX, "ball", input_msg.header);
    publishHeatmap(output_data, OBSTACLE_CHANNEL_INDEX, "obstacle", input_msg.header);

    // --- Publish Overlay ---
    if (debug_publishers_["overlay"]->get_subscription_count() > 0) {
        cv_bridge::CvImagePtr cv_ptr_bgr = cv_bridge::toCvCopy(input_msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat& overlay_image = cv_ptr_bgr->image;

        // Draw balls
        for(const auto& ball_px : ball_pixels) {
            cv::circle(overlay_image, cv::Point(ball_px.x, ball_px.y), 10, class_config_map_.at(BALL_CHANNEL_INDEX).color, 2);
        }
        // Draw obstacles
        for(const auto& base : obstacle_pixel_bases) {
            cv::Point pt1(base.first.x, base.first.y);
            cv::Point pt2(base.second.x, base.second.y);
            cv::line(overlay_image, pt1, pt2, class_config_map_.at(OBSTACLE_CHANNEL_INDEX).color, 3);
            cv::circle(overlay_image, pt1, 5, class_config_map_.at(OBSTACLE_CHANNEL_INDEX).color, -1);
            cv::circle(overlay_image, pt2, 5, class_config_map_.at(OBSTACLE_CHANNEL_INDEX).color, -1);
        }
        debug_publishers_["overlay"]->publish(*cv_ptr_bgr->toImageMsg());
    }
}

void BotSegmentation::publishHeatmap(const float* data, int channel_index, const std::string& topic_name, const std_msgs::msg::Header& header)
{
    if (debug_publishers_[topic_name]->get_subscription_count() == 0) return;

    cv::Mat heatmap(output_size_.y(), output_size_.x(), CV_32FC1);
    const float* channel_data = data + channel_index;
    for (int y = 0; y < output_size_.y(); ++y) {
        float* row_ptr = heatmap.ptr<float>(y);
        for (int x = 0; x < output_size_.x(); ++x) {
            row_ptr[x] = *channel_data;
            channel_data += num_output_channels_;
        }
    }

    cv::Mat heatmap_vis, color_heatmap;
    cv::normalize(heatmap, heatmap_vis, 0, 255, cv::NORM_MINMAX, CV_8U);
    cv::applyColorMap(heatmap_vis, color_heatmap, cv::COLORMAP_JET);

    auto msg = cv_bridge::CvImage(header, "bgr8", color_heatmap).toImageMsg();
    debug_publishers_[topic_name]->publish(*msg);
}


} // namespace vision

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<vision::BotSegmentation>(rclcpp::NodeOptions());
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
