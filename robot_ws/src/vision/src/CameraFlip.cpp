#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/usb/video.h>
#include <linux/uvcvideo.h>
#include <cerrno>       // Added for error handling
#include <cstring>      // Added for strerror

class CameraFlipNode : public rclcpp::Node {
public:
    CameraFlipNode() : Node("camera_flip") {
        // Declare parameters
        declare_parameter("video_device", "/dev/video-top");
        declare_parameter("hflip", true);
        declare_parameter("vflip", true);

        // Get parameters
        std::string video_device = get_parameter("video_device").as_string();
        bool hflip = get_parameter("hflip").as_bool();
        bool vflip = get_parameter("vflip").as_bool();

        RCLCPP_INFO(get_logger(), "Setting camera flips on device: %s", video_device.c_str());

        // Open device
        int fd = open(video_device.c_str(), O_RDWR);
        if (fd < 0) {
            RCLCPP_ERROR(get_logger(), "Failed to open device: %s", video_device.c_str());
            rclcpp::shutdown();
            return;
        }

        // Set flips
        if (hflip) {
            uint16_t val = 1;
            if (set_uvc_xu(fd, 3, 12, 2, &val) < 0) {
                RCLCPP_ERROR(get_logger(), "Failed to set horizontal flip");
            } else {
                RCLCPP_INFO(get_logger(), "Horizontal flip set");
            }
        }

        if (vflip) {
            uint16_t val = 1;
            if (set_uvc_xu(fd, 3, 13, 2, &val) < 0) {
                RCLCPP_ERROR(get_logger(), "Failed to set vertical flip");
            } else {
                RCLCPP_INFO(get_logger(), "Vertical flip set");
            }
        }

        close(fd);
        RCLCPP_INFO(get_logger(), "Camera flips configured, shutting down node");
        rclcpp::shutdown();
    }

private:
    int set_uvc_xu(int fd, uint8_t unit, uint8_t selector, uint16_t size, void *data) {
        // https://www.waveshare.com/w/upload/c/cb/OV5640_camera_module_software_application_notes_1.3_Sonix.pdf
        struct uvc_xu_control_query query = {
            .unit = unit,
            .selector = selector,
            .query = UVC_SET_CUR,
            .size = size,
            .data = (__u8 *)data
        };
        return ioctl(fd, UVCIOC_CTRL_QUERY, &query);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraFlipNode>());
    rclcpp::shutdown();
    return 0;
}
