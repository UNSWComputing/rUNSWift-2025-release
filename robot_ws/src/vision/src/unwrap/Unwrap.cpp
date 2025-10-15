#include <cassert>
#include <cmath>
#include <memory>

#include <Eigen/Dense>

#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>

#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

using Eigen::Vector2f;
using Eigen::Vector3f;
using geometry_msgs::msg::TransformStamped;
using sensor_msgs::msg::Image;

class Unwrap : public rclcpp::Node {
  private:
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_subscriber;
    rclcpp::Subscription<Image>::SharedPtr raw_image_subscriber;
    rclcpp::Publisher<Image>::SharedPtr unwrapped_image_publisher;

    // pre-allocate the image to avoid reallocating
    cv::Mat unwrapped_image;

    TransformStamped tf2_world2cam;
    Eigen::Affine3f cam2world;

    static constexpr const char* footprint_frame_id = "base_footprint";
    static constexpr const char* camera_frame_id = "CameraTopMotion";

    // frame width and height
    int cam_w = 640;
    int cam_h = 480;

    const std::vector<Vector2f> screenPoints2 = {
        {0.0,   cam_h},     // so we always hit the ground plane
        {cam_w, cam_h},     // these are close to the bottom
        {0.0,   cam_h * 0.25},
        {cam_w, cam_h * 0.25},
    };

    const std::vector<cv::Point2f> screenPoints = [this] {
        std::vector<cv::Point2f> out;
        out.reserve(screenPoints2.size());

        for (Vector2f const &point : screenPoints2) {
            out.push_back(cv::Point2f(
                point.x() + cam_w * 0.5,
                point.y() + cam_h * 0.5
            ));
        }

        return out;
    }();

    // Modified view_ray using both horizontal and vertical focal lengths
    Vector3f view_ray(Vector2f const &filmback, const float cam_f, float aspect) {
        return {
            1.0f,
            filmback.x() / (cam_f * aspect),
            filmback.y() / cam_f,
        };
    }

    // Intersect a worldspace ray with a z-plane
    Vector3f ground_plane_intersect(Vector3f const &ray, float cam_z) {
        float k = -cam_z / ray.z();
        return {
            k * ray.x(),
            k * ray.y(),
            0.0f,
        };
    }

  public:
    Unwrap() : Node("Unwrap") {
        this->declare_parameter("camera_horizontal_fov_deg", 57.2);

        this->declare_parameter("unwrap_range_m", 10.0);

        this->declare_parameter("unwrap_output_width_px", 640);
        this->declare_parameter("unwrap_output_height_px", 480);

        this->declare_parameter("center_offset_x_px", 0.0);
        this->declare_parameter("center_offset_y_px", 0.0);

        this->declare_parameter("flip_h", true);
        this->declare_parameter("flip_v", false);

        unwrapped_image_publisher = this->create_publisher<Image>("/camera/top/unwrapped", 1);

        raw_image_subscriber = this->create_subscription<Image>(
            "/camera/top/raw_image",
            rclcpp::QoS(1).reliability(rclcpp::ReliabilityPolicy::BestEffort),
            [this](const Image::SharedPtr msg) {
                // RCLCPP_INFO(this->get_logger(), "Received image...");
                cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::YUV422_YUY2);

                bool flip_h, flip_v;
                this->get_parameter("flip_h", flip_h);
                this->get_parameter("flip_v", flip_v);

                if (flip_v) {
                    cv::flip(cv_ptr->image, cv_ptr->image, 0);
                }

                if (flip_h) {
                    cv::flip(cv_ptr->image, cv_ptr->image, 1);
                }

                unwrap_image(cv_ptr->image);
            }
        );

        // subscribe to /tf manually, tf buffer is too slow
        tf_subscriber = this->create_subscription<tf2_msgs::msg::TFMessage>(
            "/tf",
            rclcpp::QoS(1).reliability(rclcpp::ReliabilityPolicy::BestEffort),
            [this](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
                for (const auto &transform : msg->transforms) {
                    if (transform.child_frame_id == camera_frame_id) {
                        tf2_world2cam = transform;

                        cam2world.translation() << -tf2_world2cam.transform.translation.x,
                                                   -tf2_world2cam.transform.translation.y,
                                                   -tf2_world2cam.transform.translation.z;

                        cam2world.linear() = Eigen::Quaternionf(
                            tf2_world2cam.transform.rotation.w,
                            tf2_world2cam.transform.rotation.x,
                            tf2_world2cam.transform.rotation.y,
                            tf2_world2cam.transform.rotation.z
                        ).inverse().toRotationMatrix();

                        return;
                    }
                }
            }
        );

        RCLCPP_INFO(this->get_logger(), "Waiting for images & TF messages...");
    }

    void unwrap_image(cv::Mat &cv_image) {
        rclcpp::Time now = this->now();

        double tf2_stamp = rclcpp::Time(tf2_world2cam.header.stamp.sec, tf2_world2cam.header.stamp.nanosec).seconds();
        // RCLCPP_INFO(this->get_logger(), " TF Age: %lf s", now.seconds() - tf2_stamp);

        std_msgs::msg::Header header;
        header.stamp = now;
        header.frame_id = footprint_frame_id;

        Vector3f cam = cam2world * Vector3f(0, 0, 0);
        Vector3f cam_dir = cam2world * Vector3f(1, 0, 0);

        cam_dir.head<2>() -= cam.head<2>();
        cam_dir.z() = 0.0f;
        cam_dir.normalize();

        Eigen::Rotation2Df R(-atan2(cam_dir.y(), cam_dir.x()));

        float range_m;
        this->get_parameter("unwrap_range_m", range_m);

        int unwrap_w, unwrap_h;
        this->get_parameter("unwrap_output_width_px", unwrap_w);
        this->get_parameter("unwrap_output_height_px", unwrap_h);

        float cam_hfov;
        this->get_parameter("camera_horizontal_fov_deg", cam_hfov);
        cam_hfov *= M_PIf / 180.0f;

        const float cam_f = cam_w / (2.0f * tanf(cam_hfov * 0.5f));
        const float px_per_m = unwrap_w / range_m;
        const Vector2f cam_destination = {0.0f, unwrap_h * 0.5f};

        Vector2f optical_center_offset = Vector2f(0.0f, 0.0f);
        this->get_parameter("center_offset_x_px", optical_center_offset.x());
        this->get_parameter("center_offset_y_px", optical_center_offset.y());

        std::vector<cv::Point2f> ground_points;
        ground_points.reserve(screenPoints2.size());

        for (const Vector2f &point : screenPoints2) {
            Vector3f world_ray = cam2world * view_ray(point + optical_center_offset, cam_f, 1.0).normalized();

            // this line of code took a month of debugging and pain and suffering to discover
            // it is the most important line of code in this entire file, and must be protected
            // at all costs. -martin
            world_ray -= cam;

            Vector2f ground_point = ground_plane_intersect(world_ray, cam.z()).topRows(2);

            ground_point = R * ground_point * px_per_m + cam_destination;
            ground_points.push_back(cv::Point2f(ground_point.x(), ground_point.y()));
        }

        cv::Mat H = cv::findHomography(screenPoints, ground_points);

        cv::warpPerspective(cv_image, unwrapped_image, H, cv::Size(unwrap_w, unwrap_h), cv::INTER_LINEAR);
        
        unwrapped_image_publisher->publish(*cv_bridge::CvImage(header, sensor_msgs::image_encodings::YUV422_YUY2, unwrapped_image).toImageMsg());
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Unwrap>();
    try {
        rclcpp::spin(node);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(node->get_logger(), "Caught exception: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}
