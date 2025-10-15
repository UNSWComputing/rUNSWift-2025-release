#include <cv_bridge/cv_bridge.h>
#include <tf2/LinearMath/Quaternion.h>

#include <cmath>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <iomanip>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video.hpp>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sstream>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/point.hpp>
#include "runswift_interfaces/msg/unwrap_field_features.hpp"
#include "runswift_interfaces/msg/unwrap_field_feature.hpp"
#include <vector>

#include "vision/unwrap/FieldTraceHelpers.hpp"

using namespace cv;
using namespace std;
using sensor_msgs::msg::Image;

static constexpr float UNWRAP_RANGE_MM = 10000.0f; // 10 m view
static constexpr int UNWRAP_W = 640;
static constexpr int UNWRAP_H = 480;

enum class IntersectionType {
    CORNER,
    T_JUNCTION,
    UNKNOWN
};

struct ClassifiedIntersection {
    Point2f point;
    IntersectionType type;
    float orentation; // angle in radians
};


class FeatureDetector : public rclcpp::Node {
  public:
    FeatureDetector() : Node("where_am_i") {
        declare_parameter<bool>("debug", true);
        get_parameter("debug", debug_);

        sub_ = create_subscription<Image>(
            "/camera/top/unwrapped", 1, bind(&FeatureDetector::imageCallback, this, placeholders::_1));

        pub_lines_ = create_publisher<Image>("/WhatFeatures/debug/lines_detected", 1);
        pub_features_ = this->create_publisher<runswift_interfaces::msg::UnwrapFieldFeatures>("/WhatFeatures/field_features", 3);
    }

  private:
    void imageCallback(const Image::SharedPtr msg) {
        Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        Mat linesOv;
        vector<ClassifiedIntersection> intersections;        
        stepFieldTrace(frame, linesOv, intersections);
        pub_lines_->publish(*cv_bridge::CvImage(msg->header, "bgr8", linesOv).toImageMsg());
    }

    void stepFieldTrace(const Mat &in, Mat &out, vector<ClassifiedIntersection> &classified_intersections) {
        Mat gray, bin, filt;
        stepPreprocess(in, gray, bin, filt);
        Mat edge;
        vector<Vec4i> raw;
        stepEdgesAndHough(filt, edge, raw);

        auto merged = tooManyLines(raw);
        Mat draw = in.clone();
        auto original_lines = stepFilterRedLines(merged, in.cols, in.rows, draw);
        out = draw;
        
        vector<pair<Point, Point>> extended_lines;
        const float EXTENSION_PIXELS = 1.0f;
        
        for (const auto& line : original_lines) {
            extended_lines.push_back(extendLineSegment(line, EXTENSION_PIXELS));
        }
        
        // Use EXTENDED lines for intersection detection
        for (size_t i = 0; i < extended_lines.size(); ++i) {
            for (size_t j = i + 1; j < extended_lines.size(); ++j) {
                Point2f intersection_pt;
                RCLCPP_DEBUG(get_logger(), "Checking intersection between extended line %zu and %zu", i, j);
                
                if (getLineIntersection(extended_lines[i], extended_lines[j], intersection_pt)) {
                    if (isPointOnExtendedSegment(intersection_pt, extended_lines[i]) &&
                        isPointOnExtendedSegment(intersection_pt, extended_lines[j])) {
                        if (intersection_pt.x >= 0 && intersection_pt.y >= 0 &&
                            intersection_pt.x < draw.cols && intersection_pt.y < draw.rows) {
                            ClassifiedIntersection classified = classifyByEndpoints(intersection_pt, i, j, original_lines);
                            classified_intersections.push_back(classified);
                            
                            // Draw with different colors based on type
                            Scalar color;
                            string label;
                            switch (classified.type) {
                                case IntersectionType::CORNER:
                                    color = Scalar(0, 255, 0);  // Green for corners
                                    label = "C";
                                    break;
                                case IntersectionType::T_JUNCTION:
                                    color = Scalar(0, 255, 255);  // Yellow for T-junctions
                                    label = "T";
                                    break;
                                default:
                                    color = Scalar(128, 128, 128);  // Gray for unknown
                                    label = "?";
                                    break;
                            }
                            
                            circle(draw, intersection_pt, 6, color, -1);
                            circle(draw, intersection_pt, 8, Scalar(255, 255, 255), 1);
                            
                            // Add text label
                            putText(draw, label, 
                                   Point(intersection_pt.x + 10, intersection_pt.y - 10),
                                   FONT_HERSHEY_SIMPLEX, 0.5, color, 2);
                        }
                    }
                }
            }
        }

        runswift_interfaces::msg::UnwrapFieldFeatures final_output;
        for (size_t i = 0; i < classified_intersections.size(); i++) {
            const auto& classified = classified_intersections[i];
            // runswift_interfaces::msg::UnwrapFieldFeature feature;
            auto feature = runswift_interfaces::msg::UnwrapFieldFeature();
            feature.type = static_cast<uint>(classified.type);
            feature.point.x = classified.point.x;
            feature.point.y = classified.point.y;
            feature.real_world_point.x = feature.point.x * UNWRAP_RANGE_MM / UNWRAP_W;
            feature.real_world_point.y = feature.point.y * UNWRAP_RANGE_MM / UNWRAP_H;
            feature.orientation = classified.orentation; // Store the angle in radians
            final_output.field_features.push_back(feature);
        }
        pub_features_->publish(final_output);
    }

    // Check if a point is close to the endpoints of a line segment
    bool isNearEndpointorExtention(Point2f point, const pair<Point,Point>& line, float threshold = 8.0f) {
        Point2f p1(line.first.x, line.first.y);
        Point2f p2(line.second.x, line.second.y);
        float dist_to_p1 = norm(point - p1);
        float dist_to_p2 = norm(point - p2);
        return (dist_to_p1 <= threshold) || (dist_to_p2 <= threshold);
    }

    // Classify intersection based on whether it's near endpoints of the intersecting lines
    ClassifiedIntersection classifyByEndpoints(Point2f intersection, 
                                    int line1_idx, int line2_idx,
                                    const vector<pair<Point,Point>>& original_lines,
                                    float endpoint_threshold = 8.0f) {
        
        const auto& line1 = original_lines[line1_idx];
        const auto& line2 = original_lines[line2_idx];
        
        bool line1_near_endpoint = isNearEndpointorExtention(intersection, line1, endpoint_threshold);
        bool line2_near_endpoint = isNearEndpointorExtention(intersection, line2, endpoint_threshold);

        ClassifiedIntersection classified;
        classified.point = intersection;

        if (line1_near_endpoint && line2_near_endpoint) {
            // closer endpoint to the intersection point for line1
            float endpoint_1_dist = norm(intersection - Point2f(line1.first.x, line1.first.y));
            float endpoint_2_dist = norm(intersection - Point2f(line1.second.x, line1.second.y));
            Point2f further_endpoint_line1 = endpoint_1_dist > endpoint_2_dist ? line1.first : line1.second;

            // closer endpoint to the intersection point for line2
            endpoint_1_dist = norm(intersection - Point2f(line2.first.x, line2.first.y));
            endpoint_2_dist = norm(intersection - Point2f(line2.second.x, line2.second.y));
            Point2f further_endpoint_line2 = endpoint_1_dist > endpoint_2_dist ? line2.first : line2.second;

            Point2f corner_edge1 = further_endpoint_line1 - intersection;
            Point2f corner_edge2 = further_endpoint_line2 - intersection;
            
            Point2f direction_vector = corner_edge1/endpoint_1_dist + corner_edge2/endpoint_2_dist;
            Point2f origin_vector = Point2f(0, 0) - intersection;
            float angle = atan2(direction_vector.y, direction_vector.x) - atan2(origin_vector.y, origin_vector.x);
            while (angle <= -CV_PI) angle += 2 * CV_PI;
            while (angle > CV_PI) angle -= 2 * CV_PI;
            RCLCPP_INFO(get_logger(), "CORNER Intersection angle: %f radians", angle);
            RCLCPP_INFO(get_logger(), "CORNER Intersection angle in degrees: %f", angle * 180.0 / CV_PI);            
            classified.type = IntersectionType::CORNER;
            classified.orentation = angle; // Store the angle in radians
            return classified;
        } 
        else if (line1_near_endpoint || line2_near_endpoint) {
            const auto &ori_line = line1_near_endpoint ? line1 : line2;
            // from the intesection point, draw a vector to the further endpoint
            float endpoint_1_dist = norm(intersection - Point2f(ori_line.first.x, ori_line.first.y));
            float endpoint_2_dist = norm(intersection - Point2f(ori_line.second.x, ori_line.second.y));
            Point2f further_endpoint = endpoint_1_dist > endpoint_2_dist ? ori_line.first : ori_line.second;

            Point2f direction_vector = further_endpoint - intersection;
            Point2f origin_vector = Point2f(0, 0) - intersection;
            float angle = atan2(direction_vector.y, direction_vector.x) - atan2(origin_vector.y, origin_vector.x);
            while (angle <= -CV_PI) angle += 2 * CV_PI;
            while (angle > CV_PI) angle -= 2 * CV_PI;
            RCLCPP_INFO(get_logger(), "T_SECTION Intersection angle: %f radians", angle);
            RCLCPP_INFO(get_logger(), "T_SECTION Intersection angle in degrees: %f", angle * 180.0 / CV_PI);
            classified.type = IntersectionType::T_JUNCTION;
            classified.orentation = angle; // Store the angle in radians
            return classified;
        }
        else {
            classified.type = IntersectionType::UNKNOWN;
            return classified;
        }
    }

        // Extend a line segment by specified pixels in both directions
    pair<Point, Point> extendLineSegment(const pair<Point, Point>& line, float extension_pixels) {
        Point2f p1(line.first.x, line.first.y);
        Point2f p2(line.second.x, line.second.y);
        
        // Calculate direction vector
        Point2f dir = p2 - p1;
        float length = norm(dir);
        
        if (length < 1e-6f) {
            return line; // Can't extend a point
        }
        
        // Normalize direction
        dir /= length;
        
        // Extend in both directions
        Point2f extended_p1 = p1 - dir * extension_pixels;
        Point2f extended_p2 = p2 + dir * extension_pixels;
        
        return {
            Point(cvRound(extended_p1.x), cvRound(extended_p1.y)),
            Point(cvRound(extended_p2.x), cvRound(extended_p2.y))
        };
    }
    bool debug_{false};
    rclcpp::Subscription<Image>::SharedPtr sub_;
    rclcpp::Publisher<Image>::SharedPtr pub_lines_, pub_ideal_, pub_map_;
    rclcpp::Publisher<runswift_interfaces::msg::UnwrapFieldFeatures>::SharedPtr pub_features_;

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(make_shared<FeatureDetector>());
    rclcpp::shutdown();
    return 0;
}
