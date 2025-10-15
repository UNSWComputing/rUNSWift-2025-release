#include "vision/unwrap/FieldTraceHelpers.hpp"
#include <cmath>
#include <opencv2/core/types.hpp>
#include <rclcpp/subscription.hpp>
#include <utility>
#include <vector>

// #include <vision/unwrap/FieldTraceHelpers.hpp>

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <runswift_interfaces/msg/global_pose_observation.hpp>
#include <nao_lola_command_msgs/msg/joint_positions.hpp>
#include <nao_lola_sensor_msgs/msg/joint_indexes.hpp>

using std::vector;

using rclcpp::Node;
using rclcpp::Publisher;
using rclcpp::Subscription;
using rclcpp::QoS;
using rclcpp::ReliabilityPolicy;

using sensor_msgs::msg::Image;
using runswift_interfaces::msg::GlobalPoseObservation;
using nao_lola_command_msgs::msg::JointPositions;
using nao_lola_sensor_msgs::msg::JointIndexes;

const QoS QOS_BESTEFFORT = QoS(1).reliability(ReliabilityPolicy::BestEffort);

const float UNWRAP_WIDTH_M = 10.0;
const float UNWRAP_IM_WIDTH = 640.0;
const float UNWRAP_IM_HEIGHT = 480.0;
const float LEFT_MIDDLE_Y = UNWRAP_IM_HEIGHT * 0.5;

inline float pxToMeters(float pixelPos) {
    return pixelPos / UNWRAP_IM_WIDTH * UNWRAP_WIDTH_M;
}

bool doesLineIntersectCamera(std::pair<cv::Point, cv::Point> L) {
    auto ext = extendLine(L, (int) UNWRAP_IM_WIDTH, (int) UNWRAP_IM_HEIGHT);

    // how close the line has to hit to be considered intersecting the camera
    const int thresholdPx = 45.0;

    // skip if intersects left middle
    if (ext.first.x == 0 || ext.second.x == 0) {
        int y_int = (ext.first.x==0) ? ext.first.y : ext.second.y;
        return (abs(y_int - LEFT_MIDDLE_Y) < thresholdPx);
    } else if (abs(L.second.x - L.first.x) > 1e-6f) {
        float t = -float(L.first.x) / (L.second.x - L.first.x);
        
        if (t >= 0.0 && t <= 1.0) {
            int y_int = int(std::round(L.first.y + t*(L.second.y - L.first.y)));
            return (abs(y_int - LEFT_MIDDLE_Y) < thresholdPx);
        }
    }

    return false;
}

class CircleOnlyDetector: public Node {
    Subscription<Image>::SharedPtr imageSub;
    Subscription<JointPositions>::SharedPtr jointPositionsSub;
    Publisher<GlobalPoseObservation>::SharedPtr posePub;
    Publisher<Image>::SharedPtr debugImagePub;

    // incoming cv image
    cv::Mat img;
    cv::Mat gray;

    float headYaw = 0;

    void onCircle(cv::Point center, int radius) {
        cv::circle(img, center, radius, cv::Scalar(255,0,0), 2);
        cv::circle(img, center, 3,      cv::Scalar(255,0,0), -1);

        // shrink bbox so its fully inscribed in the circle
        int roiRadius = radius - 15;

        if (roiRadius <= 0) {
            return;
        }

        int x = center.x - roiRadius;
        int y = center.y - roiRadius;
        int w = roiRadius * 2;
        int h = roiRadius * 2;
        cv::Rect boundingBox(x, y, w, h);

        cv::Mat roi = gray(boundingBox);

        for (int row = 0; row < roi.rows; ++row) {
            for (int col = 0; col < roi.cols; ++col) {
                cv::Vec3b pixel = img.at<cv::Vec3b>(y + row, x + col);
                if (pixel[0] == 0 && pixel[1] == 154 && pixel[2] == 0) {
                    cv::rectangle(img, boundingBox, cv::Scalar(0, 0, 255), 2);
                    return;
                }
            }
        }

        cv::rectangle(img, boundingBox, cv::Scalar(0, 255, 0), 2);

        // Crop the circle region for line detection and detect in it
        cv::Mat edges;
        cv::Canny(gray(boundingBox), edges, 30, 90, 3);

        // Use HoughLinesP to detect lines in the circle region
        std::vector<cv::Vec4i> lines;
        cv::HoughLinesP(edges, lines, 1, CV_PI/180, 30, radius * 0.8, 10);

        double maxLen = 0;
        cv::Vec4i longestLine;
        for (const auto& l : lines) {
            double len = std::hypot(l[2] - l[0], l[3] - l[1]);
            if (len > maxLen) {
                maxLen = len;
                longestLine = l;
            }
        }

        if (maxLen <= 30.0) {
            return;
        }

        cv::Point pt1(longestLine[0] + x, longestLine[1] + y);
        cv::Point pt2(longestLine[2] + x, longestLine[3] + y);
        
        if (doesLineIntersectCamera(std::make_pair(pt1, pt2))) {
            cv::line(img, pt1, pt2, cv::Scalar(0, 0, 255), 3);
            return;
        }

        cv::line(img, pt1, pt2, cv::Scalar(255, 0, 255), 3);

        float angleRad = atan2f(pt2.y - pt1.y, pt2.x - pt1.x);
        float angleDeg = angleRad * 180.0f / M_PIf;
        RCLCPP_INFO(this->get_logger(), "Line angle: %.2f degrees", angleDeg);

        // since the circle is at 0, 0 then we can just offset ourselves from the center
        Eigen::Vector2f pos(
            pxToMeters(-center.y + (UNWRAP_IM_HEIGHT * 0.5)),  // not -center.y because flip_h = true
            pxToMeters(center.x)
        );
        pos = Eigen::Rotation2Df(-angleRad) * pos;

        //float heading = -angleRad - M_PI_2f;
        float heading = -angleRad - M_PI_2f - headYaw;

        // RCLCPP_WARN(this->get_logger(), "Head yaw angle: %.2f degrees", headYaw * 180.0f / M_PIf);

        GlobalPoseObservation poseMsg;
        poseMsg.header.stamp = this->now();
        poseMsg.header.frame_id = "world";

        poseMsg.position.x = pos.x() * 1000.0f;
        poseMsg.position.y = pos.y() * 1000.0f;
        poseMsg.position.z = 0.0;
        poseMsg.heading = heading;

        posePub->publish(poseMsg);
    }

    void onImage(const Image::SharedPtr msg) {
        // get image and convert to gray 8-bit
        img = cv_bridge::toCvCopy(msg, "bgr8")->image;
        cvtColor(img, gray, cv::COLOR_BGR2GRAY); // todo: simply specify "gray8" encoding to toCvCopy

        std::vector<cv::Vec3f> circles;

        GaussianBlur(gray, gray, cv::Size(9,9), 2, 2);

        HoughCircles(
            gray, circles,
            cv::HOUGH_GRADIENT,
            1.2,
            ((double) gray.rows) / 8,
            50, 55,
            40, 60
        );


        if (!circles.empty()) {
            cv::Point center(
                roundf(circles[0][0]),
                roundf(circles[0][1])
            );

            int radius = roundf(circles[0][2]);
            onCircle(center, radius);
        }

        debugImagePub->publish(*cv_bridge::CvImage(msg->header, "bgr8", img).toImageMsg());
    }

    void onJointPositions(JointPositions::SharedPtr msg) {
        headYaw = msg->positions[0];
        // RCLCPP_WARN(this->get_logger(), "JUST RECEIVED A Head yaw angle: %.2f degrees", headYaw * 180.0f / M_PIf);
    }

public:
    CircleOnlyDetector() : Node("CircleOnlyDetector") {
        imageSub = create_subscription<Image>(
            "/camera/top/unwrapped", QOS_BESTEFFORT,
            bind(&CircleOnlyDetector::onImage, this, std::placeholders::_1)
        );

        jointPositionsSub = create_subscription<JointPositions>(
            "/effectors/joint_positions", QOS_BESTEFFORT,
            bind(&CircleOnlyDetector::onJointPositions, this, std::placeholders::_1)
        );

        posePub = create_publisher<GlobalPoseObservation>("/global_pose_observation", QOS_BESTEFFORT);

        debugImagePub = create_publisher<Image>("/debug/circle", QOS_BESTEFFORT);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CircleOnlyDetector>());
    rclcpp::shutdown();
    return 0;
}
