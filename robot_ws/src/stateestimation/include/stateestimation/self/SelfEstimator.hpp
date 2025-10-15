#pragma once
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/header.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include "stateestimation/types.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include <nao_lola_sensor_msgs/msg/gyroscope.hpp>
#include <nao_lola_sensor_msgs/msg/angle.hpp>

#include <runswift_interfaces/msg/motion_odometry.hpp>
#include <runswift_interfaces/msg/motion_command.hpp>
#include <runswift_interfaces/msg/vision_field_feature.hpp>
#include <runswift_interfaces/msg/vision_field_features.hpp>
#include <runswift_interfaces/msg/comms_rcgcd.hpp>
#include <runswift_interfaces/msg/comms_robot_player_info.hpp>
#include <runswift_interfaces/msg/global_pose_observation.hpp>

#include "Localiser.hpp"
// #include "FieldFeatureObservation.hpp"
#include "stateestimation/GameState.hpp"

using geometry_msgs::msg::PoseStamped;

#define QOS_BESTEFFORT rclcpp::QoS(1).reliability(rclcpp::ReliabilityPolicy::BestEffort)

// publish a geometry_msgs/msg/PoseStamped on "/stateestimation/self"
constexpr const char* selfPublisherTopic = "/stateestimation/self";
typedef rclcpp::Publisher<PoseStamped> SelfPublisher;

constexpr const char* selfPathPublisherTopic = "/stateestimation/self_path";
typedef rclcpp::Publisher<nav_msgs::msg::Path> SelfPathPublisher;

// odometry information from /motion/odometry
constexpr const char* odometrySubscriptionTopic = "/motion_odometry";
typedef rclcpp::Subscription<runswift_interfaces::msg::MotionOdometry> OdometrySubscription;

// field features from /vision/field_features
// constexpr const char* fieldFeaturesSubscriptionTopic = "/vision/field_features";
// typedef rclcpp::Subscription<runswift_interfaces::msg::VisionFieldFeatures> FieldFeaturesSubscription;

typedef rclcpp::Subscription<runswift_interfaces::msg::GlobalPoseObservation> GlobalPoseObservationSubscription;

// field feature based hypotheses published
constexpr const char* poseArrayPublisherTopic = "/stateestimation/hypotheses";
typedef rclcpp::Publisher<geometry_msgs::msg::PoseArray> PoseArrayPublisher;

constexpr const char* initialPoseSubscriptionTopic = "/initialpose";
typedef rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped> InitialPoseSubscription;

#define ODOM_FREQUENCY 30

#define MM2M (1/1000.0f)
#define M2MM 1000.0f
#define S2MS 1000


// A measurement of some type T with an attached timestamp.
template<typename T>
class WithTimestamp {
    public:
    T measurement;
    float receivedAt;
    
    WithTimestamp<T>(T t, std_msgs::msg::Header header) 
    : measurement(t), 
    receivedAt((float)(header.stamp.sec) + (float)(header.stamp.nanosec) / 1e9f) {}
};

/**
* Global class for state estimation.
 *
 * Contains any and a
 ll information needed to estimate the state of the robot
 * and balls.
 */
 class SelfEstimator : public rclcpp::Node {
    protected:
        Localiser localiser;
        
        GameState gameState;
        int playerNum = 0;
        
        // The timer for the main odom loop.
        rclcpp::TimerBase::SharedPtr estimatePublishTimer;
        
        // Topic I/O
        SelfPublisher::SharedPtr selfPublisher;
        SelfPathPublisher::SharedPtr selfPathPublisher;
        std::vector<geometry_msgs::msg::PoseStamped> poseHistory;

        OdometrySubscription::SharedPtr odometrySubscription;
        std::optional<Odometry> lastOdometryMeasurement;

        GlobalPoseObservationSubscription::SharedPtr globalPoseObservationSubscription;

        // FieldFeaturesSubscription::SharedPtr fieldFeaturesSubscription;
        // std::optional<WithTimestamp<std::vector<FieldFeatureObservation>>> fieldFeatureObservations;
        
        rclcpp::Subscription<runswift_interfaces::msg::CommsRobotPlayerInfo>::SharedPtr playerNumSubscription;
        rclcpp::Subscription<runswift_interfaces::msg::CommsRCGCD>::SharedPtr gcSubscription;
        
        PoseArrayPublisher::SharedPtr poseArrayPublisher;

        std::unique_ptr<tf2_ros::TransformBroadcaster> tf2Broadcaster;

        InitialPoseSubscription::SharedPtr initialPoseSubscription;

        void publishEstimate();

        void onOdometry(runswift_interfaces::msg::MotionOdometry::SharedPtr odometry);
        void onGlobalPoseObservation(GlobalPoseObservation::SharedPtr observation);
        void onInitialPose(geometry_msgs::msg::PoseWithCovarianceStamped pose);
        
        // void onFieldFeatures(FieldFeaturesSubscription::SubscribedType::SharedPtr fieldFeatures);
        void onPlayerNum(runswift_interfaces::msg::CommsRobotPlayerInfo::SharedPtr msg);
        void onGameControllerMessage(runswift_interfaces::msg::CommsRCGCD::SharedPtr msg);

    public:
        SelfEstimator();

};
