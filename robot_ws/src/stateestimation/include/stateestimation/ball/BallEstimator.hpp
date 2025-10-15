#pragma once

#include <memory>

#include <Eigen/Dense>
//#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include "tf2/exceptions.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

//msg interfaces
#include <std_msgs/msg/header.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <runswift_interfaces/msg/vision_balls.hpp>
#include <runswift_interfaces/msg/motion_odometry.hpp>
#include <runswift_interfaces/msg/motion_command.hpp>  //comment out for testing when motion is not implemented yet
#include <runswift_interfaces/msg/se_balls_absolute.hpp> 
#include <nao_lola_sensor_msgs/msg/gyroscope.hpp>

//in ros2, all built msg hpp files are snake case instead of camel case
//eg. visionBalls -> vision_balls
#include "stateestimation/types.hpp"

#include "stateestimation/ball/BallKalmanFilter.hpp"


#define LOOP_FREQUENCY 10


class BallEstimator : public rclcpp::Node {
public:
    BallEstimator();
    ~BallEstimator();

private:
	/*
	 *		Kalman Filter Core Components
	 */
	
	std::vector<BallKalmanFilter> ballKalmanFilters = {BallKalmanFilter()};

	void merge_similar();
	void prune_invalid();
	void decay();


	/*
	 *		Class Fields
	 */
	
    double current_timestamp;
    float dtInSeconds;
    Odometry currentOdom;
    Odometry odometryDiff;

    //Eigen::Vector3f robotPosition;
    //float robotHeading;

    //Gyro gyro; //directly get the boolean is_stable in callback
    bool is_gyro_stable; //the robot is looking straight forward without looking downwards or elsewhere
    bool is_stable;
    
    Eigen::Vector3f robotPosition;
    float robotHeading;


	/*
	 *		ROS2 Subscriptions and Publishers
	 */

    // Tick every time we receive info
    //----------ROS2 loop timer----------------//
    // future work: shall we make the predict and update event-driven instead?
    rclcpp::TimerBase::SharedPtr _loopTimer;

    //----------ROS2 subscriber(s)-------------//
    rclcpp::Subscription<runswift_interfaces::msg::VisionBalls>::SharedPtr _visionBallSubscriber;
    rclcpp::Subscription<runswift_interfaces::msg::MotionOdometry>::SharedPtr _motionOdomSubscriber;
    rclcpp::Subscription<runswift_interfaces::msg::MotionCommand>::SharedPtr _bodyCommandSubscriber;
    rclcpp::Subscription<nao_lola_sensor_msgs::msg::Gyroscope>::SharedPtr _gyroSubscriber;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _selfPoseSubscriber; //check if this is needed or should fetch TF2 instead, PoseStamped would be quicker

    std::unique_ptr<tf2_ros::TransformBroadcaster> _tf2Broadcaster;
    std::unique_ptr<tf2_ros::Buffer> _tf2Buffer;
    std::shared_ptr<tf2_ros::TransformListener> _tf2Listener;
    rclcpp::Publisher<runswift_interfaces::msg::SeBallsAbsolute>::SharedPtr _ballsPublisher;



	/*
	 *		ROS2 Subscription Callback Functions
	 */

    //----------callback functions: msg processing----------------//
    void callbackBallInfo(const runswift_interfaces::msg::VisionBalls::SharedPtr msg);
    void callbackMotionOdomInfo(const runswift_interfaces::msg::MotionOdometry::SharedPtr msg);
    void callbackBodyCommandInfo(const runswift_interfaces::msg::MotionCommand::SharedPtr msg);
    void callbackGyroInfo(const nao_lola_sensor_msgs::msg::Gyroscope::SharedPtr msg);
    void callbackSelfPoseInfo(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    //feedback function of the _loopTimer
    //contains the operation of 1 cycle
    //future work, remove the _loopTimer and break down the loop() to operations to be called in subscriber feedback function
    void loop();
};


void merge(BallKalmanFilter& kf1, BallKalmanFilter& kf2);
