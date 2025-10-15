#include "vision/BallDetector.hpp"
#include <Eigen/Dense>
#include <fstream>
#include <sstream>
#define MULTIPLY_M_TO_MM 1000.0
#define MAX_PIXEL_DISTANCE_BETWEEN_BALLS 60.0
#define MIN_BOTTOM_BALL_CONFIDENCE_FOR_MERGE 200

namespace vision {

BallDetector::BallDetector() : Node("ball_detector") {

    vision_balls_publisher = this->create_publisher<runswift_interfaces::msg::VisionBalls>("/vision/VBalls", 1);
    cls_client = this->create_client<runswift_interfaces::srv::BallCls>("ball_cls_srv");

    cls_client->wait_for_service();
    tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, this, false, rclcpp::QoS(1).reliability(rclcpp::ReliabilityPolicy::BestEffort));
    this->declare_parameter("camera_fov_horizontal", 56.3); // NAO v6 camera FOV
    this->declare_parameter("camera_fov_vertical", 43.7);
    camera_fov_horizontal = this->get_parameter("camera_fov_horizontal").as_double();
    camera_fov_vertical = this->get_parameter("camera_fov_vertical").as_double();
    sub = this->create_subscription<runswift_interfaces::msg::VisionProcessedImageData>
    ("vision/vision_info_in", rclcpp::SensorDataQoS().keep_last(3), std::bind(&BallDetector::onCallback, this, std::placeholders::_1));

    //  subBot = this->create_subscription<sensor_msgs::msg::Image>("camera/bot/raw_image", 1, std::bind(&BallDetector::onCallbackBot, this, std::placeholders::_1));

    //  debug_publisher = this->create_publisher<sensor_msgs::msg::Image>("/vision/debug", rclcpp::SensorDataQoS().keep_last(3));
}

geometry_msgs::msg::Point BallDetector::projectRayToGround(
const vision_msgs::msg::Point2D& pixel_coords,
const uint img_width,
const uint img_height,
const std::string camera_frame,
const builtin_interfaces::msg::Time& header_stamp) {

    geometry_msgs::msg::Point world_point;
    try {
        geometry_msgs::msg::TransformStamped transform =
            tf_buffer->lookupTransform("base_footprint", camera_frame, tf2::TimePointZero);

        // horizontal angle relative to the camera frame at center y
        float horizontal_angle = - DEG2RAD(camera_fov_horizontal) * (pixel_coords.x - img_width / 2) / img_width;

        float vertical_angle;
        // vertical angle relative to the camera frame at center x
        // Commented out bot ball
        /*
        if (camera_frame == "CameraBottomMotion") {
            // for the bottom camera, the image is flipped
            vertical_angle = - DEG2RAD(camera_fov_vertical) * (pixel_coords.y - 0.5f *img_height) / img_height;
        } else{
            vertical_angle = - DEG2RAD(camera_fov_vertical) * (pixel_coords.y - 0.6104166f*img_height) / img_height;
        }
        */


        vertical_angle = - DEG2RAD(camera_fov_vertical) * (pixel_coords.y - 0.5*img_height) / img_height;


        // calculate x, which is forward
        float local_x = 1.0;
        float local_y = local_x*tan(horizontal_angle);
        float local_z = local_x*tan(vertical_angle);

        // create a point in camera frame
        geometry_msgs::msg::PointStamped camera_point;
        camera_point.point.x = local_x;
        camera_point.point.y = local_y;
        camera_point.point.z = local_z;

        geometry_msgs::msg::PointStamped camera_self;

        geometry_msgs::msg::PointStamped base_camera_self;
        // transform the point to base_footprint frame
        geometry_msgs::msg::PointStamped base_point;
        tf2::doTransform(camera_point, base_point, transform);
        tf2::doTransform(camera_self, base_camera_self, transform);

        // create the world vector and calc the intersection with the ground plane of a defined height
        Eigen::Vector3d world_vector(base_point.point.x, base_point.point.y, base_point.point.z);
        Eigen::Vector3d base_camera_vector(base_camera_self.point.x, base_camera_self.point.y, base_camera_self.point.z);
        // direction vector
        Eigen::Vector3d direction_vector = (world_vector - base_camera_vector).normalized();
        // calculate the intersection with the ground plane
        double t = (0.05 -base_camera_vector.z()) / direction_vector.z();

        if (t < 0.0) {
            world_point.x = -2.0;
            world_point.y = -2.0;
            world_point.z = -2.0;
            return world_point;
        }

        Eigen::Vector3d intersection = base_camera_vector + t * direction_vector;
        world_point.x = intersection.x();
        world_point.y = intersection.y();
        world_point.z = 0.05;

        return world_point;

    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Failed to transform point from %s to base_footprint: %s", camera_frame.c_str(), ex.what());
        world_point.x = -1.0;
        world_point.y = -1.0;
        world_point.z = -1.0;
        return world_point;
    }

}
void BallDetector::onCallbackBot(const sensor_msgs::msg::Image::SharedPtr msg) {
    uint raw_height = msg->height;
    uint raw_width = msg->width;
    uint square_size = raw_height * 0.5;
    cv::Mat rawImage = cv::Mat(raw_height, raw_width, CV_8UC2, const_cast<unsigned char *>(msg->data.data()));

    auto debugImage = std::make_shared<cv::Mat>();
    cv::cvtColor(rawImage, *debugImage, cv::COLOR_YUV2BGR_YUYV);

    auto request = std::make_shared<runswift_interfaces::srv::BallCls::Request>();
    // divided into 6 regions
    cv::Rect roiTL = cv::Rect(0, 0, square_size, square_size);
    cv::Rect roiTM = cv::Rect(raw_width/2 - square_size/2, 0, square_size, square_size);
    cv::Rect roiTR = cv::Rect(raw_width - square_size, 0, square_size, square_size);
    cv::Rect roiBL = cv::Rect(0, raw_height - square_size, square_size, square_size);
    cv::Rect roiBM = cv::Rect(raw_width/2 - square_size/2, raw_height - square_size, square_size, square_size);
    cv::Rect roiBR = cv::Rect(raw_width - square_size, raw_height - square_size, square_size, square_size);

    cv::Mat cropped_imgTL = rawImage(roiTL);
    cv::Mat cropped_imgTM = rawImage(roiTM);
    cv::Mat cropped_imgTR = rawImage(roiTR);
    cv::Mat cropped_imgBL = rawImage(roiBL);
    cv::Mat cropped_imgBM = rawImage(roiBM);
    cv::Mat cropped_imgBR = rawImage(roiBR);

    cv::cvtColor(cropped_imgTL, cropped_imgTL, cv::COLOR_YUV2GRAY_YUY2);
    cv::cvtColor(cropped_imgTM, cropped_imgTM, cv::COLOR_YUV2GRAY_YUY2);
    cv::cvtColor(cropped_imgTR, cropped_imgTR, cv::COLOR_YUV2GRAY_YUY2);
    cv::cvtColor(cropped_imgBL, cropped_imgBL, cv::COLOR_YUV2GRAY_YUY2);
    cv::cvtColor(cropped_imgBM, cropped_imgBM, cv::COLOR_YUV2GRAY_YUY2);
    cv::cvtColor(cropped_imgBR, cropped_imgBR, cv::COLOR_YUV2GRAY_YUY2);


    cv::resize(cropped_imgTL, cropped_imgTL, cv::Size(32, 32));
    cv::resize(cropped_imgTM, cropped_imgTM, cv::Size(32, 32));
    cv::resize(cropped_imgTR, cropped_imgTR, cv::Size(32, 32));
    cv::resize(cropped_imgBL, cropped_imgBL, cv::Size(32, 32));
    cv::resize(cropped_imgBM, cropped_imgBM, cv::Size(32, 32));
    cv::resize(cropped_imgBR, cropped_imgBR, cv::Size(32, 32));


    sensor_msgs::msg::Image cropped_img_msgTL;
    sensor_msgs::msg::Image cropped_img_msgTM;
    sensor_msgs::msg::Image cropped_img_msgTR;
    sensor_msgs::msg::Image cropped_img_msgBL;
    sensor_msgs::msg::Image cropped_img_msgBM;
    sensor_msgs::msg::Image cropped_img_msgBR;
    cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", cropped_imgTL.clone()).toImageMsg(cropped_img_msgTL);
    cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", cropped_imgTM.clone()).toImageMsg(cropped_img_msgTM);
    cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", cropped_imgTR.clone()).toImageMsg(cropped_img_msgTR);
    cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", cropped_imgBL.clone()).toImageMsg(cropped_img_msgBL);
    cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", cropped_imgBM.clone()).toImageMsg(cropped_img_msgBM);
    cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", cropped_imgBR.clone()).toImageMsg(cropped_img_msgBR);

    request->images.push_back(cropped_img_msgTL);
    request->images.push_back(cropped_img_msgTM);
    request->images.push_back(cropped_img_msgTR);
    request->images.push_back(cropped_img_msgBL);
    request->images.push_back(cropped_img_msgBM);
    request->images.push_back(cropped_img_msgBR);

    auto header = std::make_shared<std_msgs::msg::Header>(msg->header);
    auto result = cls_client->async_send_request(
        request,
        [this, raw_height, raw_width, roiTL, roiTM, roiTR, roiBL, roiBM, roiBR, header, debugImage] (
            rclcpp::Client<runswift_interfaces::srv::BallCls>::SharedFuture future) {
            if (!rclcpp::ok()) {
                return; // Early exit if ROS is shutting down
            }
            auto response = future.get();
            runswift_interfaces::msg::VisionBalls output;
            for (size_t i = 0; i < response->cls1_conf.size(); i++) {
                float conf1 = response->cls1_conf[i];
                float conf2 = response->cls2_conf[i];
                float conf3 = response->cls3_conf[i];
                if (conf1 > conf2 && conf1 > conf3) {
                    float ball_x = response->xs[i];
                    float ball_y = response->ys[i];
                    if (i == 0){
                        ball_x = roiTL.x + ball_x / 32 * roiTL.width;
                        ball_y = roiTL.y + ball_y / 32 * roiTL.height;
                    } else if (i == 1){
                        ball_x = roiTM.x + ball_x / 32 * roiTM.width;
                        ball_y = roiTM.y + ball_y / 32 * roiTM.height;
                    } else if (i == 2) {
                        ball_x = roiTR.x + ball_x / 32 * roiTR.width;
                        ball_y = roiTR.y + ball_y / 32 * roiTR.height;
                    } else if (i == 3) {
                        ball_x = roiBL.x + ball_x / 32 * roiBL.width;
                        ball_y = roiBL.y + ball_y / 32 * roiBL.height;
                    } else if (i == 4) {
                        ball_x = roiBM.x + ball_x / 32 * roiBM.width;
                        ball_y = roiBM.y + ball_y / 32 * roiBM.height;
                    } else if (i == 5) {
                        ball_x = roiBR.x + ball_x / 32 * roiBR.width;
                        ball_y = roiBR.y + ball_y / 32 * roiBR.height;
                    }


                    if (ball_y < 0.0) ball_y = 0.0;
                    if (ball_x < 0.0) ball_x = 0.0;
                    if (ball_y > raw_height) ball_y = raw_height;
                    if (ball_x > raw_width) ball_x = raw_width;

                    cv::putText(*debugImage, std::to_string(conf1), cv::Point(ball_x, ball_y), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 2);

                    vision_msgs::msg::Point2D ball_pixel_coordinates;
                    ball_pixel_coordinates.x = ball_x;
                    ball_pixel_coordinates.y = ball_y;

                    geometry_msgs::msg::Point ball_point;
                    ball_point = projectRayToGround(ball_pixel_coordinates, raw_width, raw_height, "CameraBottomMotion", header->stamp);
                    if (ball_point.z < 0.0) continue;
                    runswift_interfaces::msg::VisionBallFeature ballFeature;

                    ballFeature.ball_pixel_coordinates = ball_pixel_coordinates;

                    ballFeature.ball_coordinates = ball_point;
                    ballFeature.ball_coordinates.x *= MULTIPLY_M_TO_MM;
                    ballFeature.ball_coordinates.y *= MULTIPLY_M_TO_MM;
                    ballFeature.ball_coordinates.z *= MULTIPLY_M_TO_MM;
                    ballFeature.pixel_radius = response->ball_pixel_radius[i];
                    ballFeature.confidence_score = static_cast<uint8_t>((255 * conf1));

                    output.ball_features.push_back(ballFeature);
                }

            }
            sensor_msgs::msg::Image debug_img_msg;
            cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", *debugImage).toImageMsg(debug_img_msg);
            debug_publisher->publish(debug_img_msg);
            if (output.ball_features.empty()) {
                return;
            }
            // zijie li tells me that at maximum, output has 2 elements -> this is when a single ball appears in the 'overlap' of both images
            // check pixel distance between the two balls and combine into one if they are close enough
            if (output.ball_features.size() == 2) {
                vision_msgs::msg::Point2D ball1 = output.ball_features[0].ball_pixel_coordinates;
                vision_msgs::msg::Point2D ball2 = output.ball_features[1].ball_pixel_coordinates;
                uint conf1 = output.ball_features[0].confidence_score;
                uint conf2 = output.ball_features[1].confidence_score;
                double distance = std::sqrt(std::pow(ball1.x - ball2.x, 2) + std::pow(ball1.y - ball2.y, 2));
                // if the two balls are close enough and both have sufficient confidence, combine into one
                if (distance < MAX_PIXEL_DISTANCE_BETWEEN_BALLS && conf1 > MIN_BOTTOM_BALL_CONFIDENCE_FOR_MERGE && conf2 > MIN_BOTTOM_BALL_CONFIDENCE_FOR_MERGE) {
                    // combine two balls into one
                    runswift_interfaces::msg::VisionBallFeature combined_ballFeature;

                    vision_msgs::msg::Point2D combined_ball;
                    combined_ball.x = (ball1.x + ball2.x) / 2;
                    combined_ball.y = (ball1.y + ball2.y) / 2;
                    combined_ballFeature.ball_pixel_coordinates = combined_ball;

                    // calculate the average of the two ballFeature.ball_coordinates
                    geometry_msgs::msg::Point combined_ball_point;
                    combined_ball_point.x = (output.ball_features[0].ball_coordinates.x + output.ball_features[1].ball_coordinates.x) / 2.0f;
                    combined_ball_point.y = (output.ball_features[0].ball_coordinates.y + output.ball_features[1].ball_coordinates.y) / 2.0f;

                    combined_ballFeature.ball_coordinates = combined_ball_point;
                    combined_ballFeature.pixel_radius = (output.ball_features[0].pixel_radius + output.ball_features[1].pixel_radius) / 2;
                    combined_ballFeature.confidence_score = (conf1 + conf2) / 2;
                    output.ball_features.clear();
                    output.ball_features.push_back(combined_ballFeature);
                }
            }
            output.header.stamp = header->stamp;
            output.header.frame_id = "base_footprint_from_bottom";
            vision_balls_publisher->publish(output);

            return;
        }
    );


}
void BallDetector::onCallback(const std::shared_ptr<runswift_interfaces::msg::VisionProcessedImageData> msg) {
    cv::Mat rawImage = cv::Mat(msg->image_raw.height,
                          msg->image_raw.width,
                          CV_8UC2,
                          const_cast<unsigned char *>(msg->image_raw.data.data()));

    uint raw_height = msg->image_raw.height;
    uint raw_width = msg->image_raw.width;

    // auto debugImage = std::make_shared<cv::Mat>();
    // cv::cvtColor(rawImage, *debugImage, cv::COLOR_YUV2BGR_YUYV);

    float downsize_factor = msg->ratio;
    uint binarised_height = raw_height/downsize_factor;
    uint binarised_width = raw_width/downsize_factor;

    auto request = std::make_shared<runswift_interfaces::srv::BallCls::Request>();
    std::vector<sensor_msgs::msg::Image> cropped_imgs;
    auto rois = std::make_shared<std::vector<cv::Rect>>();

    for (vision_msgs::msg::BoundingBox2D roi_bbox : msg->roi_regions) {
        // read the sensor_
        cv::Rect roi(
            (roi_bbox.center.position.x - roi_bbox.size_x / 2) * downsize_factor,
            (roi_bbox.center.position.y - roi_bbox.size_y / 2) * downsize_factor,
            roi_bbox.size_x * downsize_factor,
            roi_bbox.size_y * downsize_factor
        );

        // make the roi square by expanding the height to the width, check if it goes out of bounds against raw_height
        roi.height = std::min(static_cast<uint>(roi.width), raw_height - roi.y);

        // expand the roi by 20% and check if it goes out of bounds against raw_width
        int expand_pixels = static_cast<int>(roi.width * 0.2);
        // Expand width while checking bounds
        int new_width = std::min(static_cast<uint>(roi.width + expand_pixels * 2), raw_width - roi.x);
        int x_adjustment = (new_width - roi.width) / 2;
        roi.x = std::max(0, roi.x - x_adjustment);
        roi.width = new_width;
        // Do the same for height
        int new_height = std::min(static_cast<uint>(roi.height + expand_pixels * 2), raw_height - roi.y);
        int y_adjustment = (new_height - roi.height) / 2;
        roi.y = std::max(0, roi.y - y_adjustment);
        roi.height = new_height;

        if (roi.width <= 0 || roi.height <= 0) {
            continue;
        }

        // check if the roi goes out of bounds against rawImage if so, clip it
        if (roi.x < 0) {
            roi.width += roi.x; // Adjust width to account for negative x
            roi.x = 0;
        } else if (roi.x + roi.width > raw_width) {
            roi.width = raw_width - roi.x; // Clip width to fit within rawImage
        }
        if (roi.y < 0) {
            roi.height += roi.y; // Adjust height to account for negative y
            roi.y = 0;
        } else if (roi.y + roi.height > raw_height) {
            roi.height = raw_height - roi.y; // Clip height to fit within rawImage
        }
        cv::Mat cropped_img = rawImage(roi);
        // grey scale the image
        cv::cvtColor(cropped_img, cropped_img, cv::COLOR_YUV2GRAY_YUY2);
        cv::resize(cropped_img, cropped_img, cv::Size(32, 32));
        // check if is ball candidates, if it is not eaerly exit
        std::vector<cv::Vec3f> circles;
        cv::HoughCircles(cropped_img, circles, cv::HOUGH_GRADIENT, 1,
                        cropped_img.rows/2,  // minimum distance between centers (can be larger since we expect one ball)
                        100,
                        15,           // threshold for center detection
                        std::max(cropped_img.rows/5, cropped_img.cols/5),  // min radius based on ROI size
                        std::max(cropped_img.rows/2, cropped_img.cols/2)); // max radius based on ROI size
        if(circles.empty()){
            continue;
        }
        sensor_msgs::msg::Image cropped_img_msg;
        cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", cropped_img.clone()).toImageMsg(cropped_img_msg);
        // add the cv2 rect
        cropped_imgs.push_back(cropped_img_msg);
        rois->push_back(roi);
    }
    if (cropped_imgs.empty()) {
        return;
    }
    request->images = cropped_imgs;
    // make the copy of the header, assign to var
    auto header = std::make_shared<std_msgs::msg::Header>(msg->header);
    auto result = cls_client->async_send_request(
        request,
        [this, rois, binarised_height, binarised_width, downsize_factor, header](
            rclcpp::Client<runswift_interfaces::srv::BallCls>::SharedFuture future) {
            if (!rclcpp::ok()) {
                return; // Early exit if ROS is shutting down
            }
            auto response = future.get();
            runswift_interfaces::msg::VisionBalls output;
            for (size_t i = 0; i < response->cls1_conf.size(); i++) {
                float conf1 = response->cls1_conf[i];
                float conf2 = response->cls2_conf[i];
                float conf3 = response->cls3_conf[i];

                if (conf1 > conf2 && conf1 > conf3) {
                    // detected ball, push to VisionBalls. FIELDS ARE NOT FINALISED YET

                    // Coordinates are in 32x32 image space, need to convert to raw image space.
                    float ball_x = response->xs[i];
                    float ball_y = response->ys[i];
                    // Convert to raw image space
                    ball_x = rois->at(i).x/downsize_factor + ball_x / 32 * rois->at(i).width/downsize_factor;
                    ball_y = rois->at(i).y/downsize_factor + ball_y / 32 * rois->at(i).height/downsize_factor;
                    vision_msgs::msg::Point2D ball_pixel_coordinates;
                    ball_pixel_coordinates.x = ball_x;
                    ball_pixel_coordinates.y = ball_y;
                    geometry_msgs::msg::Point ball_point;

                    ball_point = projectRayToGround(ball_pixel_coordinates, binarised_width, binarised_height, "CameraTopMotion", header->stamp);
                    if (ball_point.z < 0.0) continue;
                    runswift_interfaces::msg::VisionBallFeature ballFeature;

                    ballFeature.ball_pixel_coordinates = ball_pixel_coordinates;

                    ballFeature.ball_coordinates = ball_point;
                    ballFeature.ball_coordinates.x *= MULTIPLY_M_TO_MM;
                    ballFeature.ball_coordinates.y *= MULTIPLY_M_TO_MM;
                    ballFeature.ball_coordinates.z *= MULTIPLY_M_TO_MM;
                    ballFeature.pixel_radius = response->ball_pixel_radius[i];
                    ballFeature.confidence_score = static_cast<uint8_t>((255 * conf1));

                    output.ball_features.push_back(ballFeature);
                    // cv::putText(*debugImage, std::to_string(conf1), cv::Point(rois->at(i).x, rois->at(i).y), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 2);

                }
            }
            // sensor_msgs::msg::Image debug_img_msg;
            // cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", *debugImage).toImageMsg(debug_img_msg);
            // debug_publisher->publish(debug_img_msg);
            if (output.ball_features.empty()) {
                return;
            }
            output.header.stamp = header->stamp;
            output.header.frame_id = "base_footprint_from_top";
            vision_balls_publisher->publish(output);
            return;
        }
    );
}
}

int main(int argc, char * argv[])
{
    // customize main function below
    // Note: editing any of the generated code here could break the node.
    // take precaution when editing
    rclcpp::init(argc, argv);

    // Create a multi-threaded executor
    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(),
        2  // Number of threads - adjust based on your needs
    );

    // Create the node
    auto node = std::make_shared<vision::BallDetector>();

    // Add the node to the executor
    executor.add_node(node);

    // Spin the executor instead of the single node
    rclcpp::on_shutdown([&executor, &node]() {
        rclcpp::shutdown();
        executor.cancel();
        node.reset();  // Reset the node smart pointer
    });


    executor.spin();
    rclcpp::shutdown();
    return 0;
    // end main customization
}

