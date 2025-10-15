#include "vision/Preprocessing.hpp"

namespace vision
{

    Preprocessing::Preprocessing() : Node("preprocessing")
    {   
        camera_raw_sub = this->create_subscription<runswift_interfaces::msg::VisionImageWithFieldBoundry>(
            "vision/image_with_boundry",
            rclcpp::SensorDataQoS().keep_last(1), // QoS history depth
            std::bind(&Preprocessing::imageCallback, this, std::placeholders::_1));

        publisher = this->create_publisher<runswift_interfaces::msg::VisionProcessedImageData>("vision/vision_info_in", rclcpp::SensorDataQoS().keep_last(3));
    }

    cv::Mat Preprocessing::makeBinary(
        cv::Mat grayImage,
        int windowSize,
        int percentage)
    {
        cv::Mat binaryImage;
        cv::adaptiveThreshold(
            grayImage,
            binaryImage,
            255,
            cv::ADAPTIVE_THRESH_MEAN_C,
            cv::THRESH_BINARY,
            windowSize,
            percentage);
        return binaryImage;
    }


    void Preprocessing::getROIs(cv::Mat image, runswift_interfaces::msg::VisionProcessedImageData* processed_image_data, std::vector<float> field_boundary) {
        constexpr int MIN_AREA = 35;
        constexpr float MAX_ASPECT_RATIO = 1.5f;
        cv::Mat labels, stats, centroids;
        int num_labels = cv::connectedComponentsWithStats(image, labels, stats, centroids, 8, CV_32S);
        std::vector<vision_msgs::msg::BoundingBox2D> bboxes;
        cv::Mat component_mask;
        auto createSafeCropROI = [&](double center_x, double center_y, int width, int height) -> cv::Rect {
            int x = std::max(0, static_cast<int>(std::round(center_x - width/2.0)));
            int y = std::max(0, static_cast<int>(std::round(center_y - height/2.0)));
            
            // Ensure the ROI stays within image bounds
            width = std::min(width, image.cols - x);
            height = std::min(height, image.rows - y);
            
            return cv::Rect(x, y, width, height);
        };
        for (int i = 1; i < num_labels; i++) {
            int left = stats.at<int>(i, cv::CC_STAT_LEFT);
            int top = stats.at<int>(i, cv::CC_STAT_TOP);
            int width = stats.at<int>(i, cv::CC_STAT_WIDTH);
            int height = stats.at<int>(i, cv::CC_STAT_HEIGHT);
            int area = stats.at<int>(i, cv::CC_STAT_AREA);
            if (area < MIN_AREA) {
                continue;
            }
            component_mask = (labels == i);
            float aspect_ratio = static_cast<float>(width) / static_cast<float>(height);
            if (aspect_ratio <= MAX_ASPECT_RATIO && aspect_ratio >= (1.0f / MAX_ASPECT_RATIO)) {
                double center_x = centroids.at<double>(i, 0);
                double center_y = centroids.at<double>(i, 1);
                vision_msgs::msg::BoundingBox2D bbox;
                bbox.size_x = static_cast<float>(width);
                bbox.size_y = static_cast<float>(height);
                bbox.center.position.x = center_x;
                bbox.center.position.y = center_y;
                float horizon_y = field_boundary[static_cast<int>(center_x)] * image.rows;
                if (bbox.center.position.y - bbox.size_y/2.0 < horizon_y) {
                    if (bbox.center.position.y + bbox.size_y/2.0 < horizon_y) {
                        continue;
                    }
                    float overlap = horizon_y - (bbox.center.position.y - bbox.size_y/2.0);
                    bbox.size_y -= overlap;
                    bbox.center.position.y = horizon_y + bbox.size_y/2.0;
                }
                cv::Rect crop_roi = createSafeCropROI(
                    bbox.center.position.x, 
                    bbox.center.position.y, 
                    static_cast<int>(bbox.size_x), 
                    static_cast<int>(bbox.size_y)
                );
                if (crop_roi.width <= 0 || crop_roi.height <= 0) {
                    continue;
                }
                cv::Mat cropped_image = component_mask(crop_roi);
                sensor_msgs::msg::Image::SharedPtr cropped_image_msg = 
                    cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", cropped_image).toImageMsg();
                bboxes.push_back(bbox);
            }
            else {
                int square_size = std::min(width, height);
                int num_squares;
                bool is_horizontal = (width > height);
                if (is_horizontal) {
                    num_squares = std::ceil(static_cast<float>(width) / square_size);
                } else {
                    num_squares = std::ceil(static_cast<float>(height) / square_size);
                }
                for (int j = 0; j < num_squares; j++) {
                    int square_left, square_top;
                    if (is_horizontal) {
                        square_left = left + j * square_size;
                        square_top = top;
                        if (square_left + square_size > left + width) {
                            square_left = left + width - square_size;
                        }
                    } else {
                        square_left = left;
                        square_top = top + j * square_size;
                        if (square_top + square_size > top + height) {
                            square_top = top + height - square_size;
                        }
                    }
                    
                    double center_x = square_left + square_size / 2.0;
                    double center_y = square_top + square_size / 2.0;
                    
                    float horizon_y = field_boundary[static_cast<int>(center_x)] * image.rows;
                    if (center_y - square_size/2.0 < horizon_y) {
                        if (center_y + square_size/2.0 < horizon_y) {
                            continue;
                        }
                        
                        // Adjust for field boundary
                        int new_square_size = static_cast<int>((center_y + square_size/2.0) - horizon_y);
                        center_y = horizon_y + new_square_size/2.0;
                        
                        // Use your createSafeCropROI function
                        cv::Rect square_roi = createSafeCropROI(center_x, center_y, square_size, new_square_size);
                        
                        if (square_roi.width <= 0 || square_roi.height <= 0 ||
                            square_roi.x + square_roi.width > component_mask.cols ||
                            square_roi.y + square_roi.height > component_mask.rows) {
                            continue;
                        }
                        
                        cv::Mat square_mask = component_mask(square_roi);
                        int pixels_in_square = cv::countNonZero(square_mask);
                        if (pixels_in_square < MIN_AREA) {
                            continue;
                        }
                        
                        vision_msgs::msg::BoundingBox2D bbox;
                        bbox.size_x = static_cast<float>(square_roi.width);
                        bbox.size_y = static_cast<float>(square_roi.height);
                        bbox.center.position.x = center_x;
                        bbox.center.position.y = center_y;
                        
                        cv::Mat cropped_image = square_mask.clone();
                        sensor_msgs::msg::Image::SharedPtr cropped_image_msg = 
                            cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", cropped_image).toImageMsg();
                        bboxes.push_back(bbox);
                    }
                    else {
                        cv::Rect square_roi = createSafeCropROI(center_x, center_y, square_size, square_size);
                        
                        if (square_roi.width <= 0 || square_roi.height <= 0 ||
                            square_roi.x + square_roi.width > component_mask.cols ||
                            square_roi.y + square_roi.height > component_mask.rows) {
                            continue;
                        }
                        cv::Mat square_mask = component_mask(square_roi);
                        int pixels_in_square = cv::countNonZero(square_mask);
                        if (pixels_in_square < MIN_AREA) {
                            continue;
                        }   
                        vision_msgs::msg::BoundingBox2D bbox;
                        bbox.size_x = static_cast<float>(square_roi.width);
                        bbox.size_y = static_cast<float>(square_roi.height);
                        bbox.center.position.x = center_x;
                        bbox.center.position.y = center_y;
                        
                        cv::Mat cropped_image = square_mask.clone();
                        sensor_msgs::msg::Image::SharedPtr cropped_image_msg = 
                            cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", cropped_image).toImageMsg();
                        bboxes.push_back(bbox);
                    }
                }
            }
        }
        processed_image_data->roi_regions = bboxes;
    }


    void Preprocessing::imageCallback(const std::shared_ptr<runswift_interfaces::msg::VisionImageWithFieldBoundry> msg)
    {
        // Access image data and dimensions
        int width = msg->image.width;
        int height = msg->image.height;
    
        int ratio = 1;  // No Downsizing :)
        int new_width = width / ratio;
        int new_height = height / ratio;

        // convert msg data to cv::Mat
        cv::Mat downsizeImage;

        if (ratio != 1) {
            // For YUYV 4:2:2, each macropixel (4 bytes) represents 2 pixels
            // So the actual byte width is width * 2
            const uint8_t* input = msg->image.data.data();
            
            std::vector<uint8_t> output(new_width * new_height * 2);  // 2 bytes per pixel in YUYV
            
            // Each macropixel in YUYV is: [Y1 U Y2 V]
            for(int y = 0; y < new_height; y++) {
                for(int x = 0; x < new_width/2; x++) {  // Divide by 2 because each loop handles 2 pixels
                    int in_idx = (y*ratio)*width*2 + (x*ratio)*4;  // Input index (* 2 for height, * 4 for macropixel)
                    int out_idx = y*new_width*2 + x*4;     // Output index
                    
                    // Copy Y1 U Y2 V for the macropixel
                    output[out_idx]     = input[in_idx];     // Y1
                    output[out_idx + 1] = input[in_idx + 1]; // U
                    output[out_idx + 2] = input[in_idx + 2]; // Y2
                    output[out_idx + 3] = input[in_idx + 3]; // V
                }
            }
            // convert msg data to cv::Mat
            downsizeImage = cv::Mat(new_height, new_width, CV_8UC2, output.data());
        } else {
            downsizeImage = cv::Mat(new_height, new_width, CV_8UC2, msg->image.data.data());
        }

        // map the msg->field_boundry to a vector of uint points
        // in the msg it is using the width of 40, but the image can be different
        // the acutal value of the uint is percentage pixel value so can be kept as is
        // create the vector of uint which is the same size as our downsizeImage
        std::vector<float> field_boundry;
        for (int i = 0; i < new_width; i++) {
            field_boundry.push_back(msg->field_boundary[i/40]);
        }

        cv::Mat grayImage;
        cv::cvtColor(downsizeImage, grayImage, cv::COLOR_YUV2GRAY_YUYV);
        
        // Call makeBinary to perform adaptive thresholding
        cv::Mat binarised_cv2_image = makeBinary(grayImage, 101, -40);
        
        // construct the outgoing message
        runswift_interfaces::msg::VisionProcessedImageData processed_image_data;
        getROIs(binarised_cv2_image, &processed_image_data, field_boundry); //binarised_cv2_image
        processed_image_data.header = msg->image.header;
        processed_image_data.image_raw = msg->image;
        processed_image_data.ratio = ratio;
        publisher->publish(processed_image_data);
    }

}
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<vision::Preprocessing>());
    rclcpp::shutdown();
    return 0;
}
