#include <chrono>
#include <cmath>
#include <memory>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/path.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <optional>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <math.h>
// #include "stateestimation/self/FieldFeatureObservation.hpp"
#include "stateestimation/GameState.hpp"
#include "stateestimation/self/SelfKalmanFilter.hpp"
#include "stateestimation/self/SelfEstimator.hpp"
#include "stateestimation/types.hpp"


SelfEstimator::SelfEstimator() : Node("self_estimator") {
    selfPublisher = create_publisher<PoseStamped>(selfPublisherTopic, QOS_BESTEFFORT);

    selfPathPublisher = create_publisher<SelfPathPublisher::PublishedType>(
        selfPathPublisherTopic, QOS_BESTEFFORT
    );

    // Used for publishing FF hypotheses
    poseArrayPublisher = create_publisher<PoseArrayPublisher::PublishedType>(poseArrayPublisherTopic, 1);

    tf2Broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    // timer to publish estimates
    estimatePublishTimer = create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&SelfEstimator::publishEstimate, this)
    );

    // fieldFeaturesSubscription = create_subscription<FieldFeaturesSubscription::SubscribedType>(
    //     fieldFeaturesSubscriptionTopic, QOS_BESTEFFORT,
    //     std::bind(&SelfEstimator::onFieldFeatures, this, std::placeholders::_1)
    // );

    globalPoseObservationSubscription = create_subscription<GlobalPoseObservationSubscription::SubscribedType>(
        "/global_pose_observation", QOS_BESTEFFORT,
        std::bind(&SelfEstimator::onGlobalPoseObservation, this, std::placeholders::_1)
    );

    // read player number
    playerNumSubscription = create_subscription<rclcpp::Subscription<runswift_interfaces::msg::CommsRobotPlayerInfo>::SubscribedType>(
        "/robot_info", QOS_BESTEFFORT,
        std::bind(&SelfEstimator::onPlayerNum, this, std::placeholders::_1)
    );

    // get GC state
    gcSubscription = create_subscription<rclcpp::Subscription<runswift_interfaces::msg::CommsRCGCD>::SubscribedType>(
        "/gc/data", QOS_BESTEFFORT,
        std::bind(&SelfEstimator::onGameControllerMessage, this, std::placeholders::_1)
    );

    odometrySubscription = create_subscription<runswift_interfaces::msg::MotionOdometry>(
        odometrySubscriptionTopic, QOS_BESTEFFORT,
        std::bind(&SelfEstimator::onOdometry, this, std::placeholders::_1)
    );

    initialPoseSubscription = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        initialPoseSubscriptionTopic, QOS_BESTEFFORT,
        std::bind(&SelfEstimator::onInitialPose, this, std::placeholders::_1)
    );

    // send an initial message for the start position
    localiser.transitionToInitial(0);
    publishEstimate();
}

void SelfEstimator::onPlayerNum(runswift_interfaces::msg::CommsRobotPlayerInfo::SharedPtr msg) {
    RCLCPP_INFO(rclcpp::get_logger("SelfEstimator"), "onPlayerNum: Received player number: %d", msg->player_number);
    playerNum = msg->player_number;
}

void SelfEstimator::onGameControllerMessage(runswift_interfaces::msg::CommsRCGCD::SharedPtr msg) {
    RCLCPP_INFO(rclcpp::get_logger("SelfEstimator"), "GC Game state: %d", msg->state);
    gameState = (GameState) msg->state;

    if (gameState == GameState::Initial) {
        localiser.transitionToInitial(playerNum);
    }
}

void SelfEstimator::onOdometry(runswift_interfaces::msg::MotionOdometry::SharedPtr msg) {
    Odometry newOdometryMeasurement(*msg);

    if (lastOdometryMeasurement == std::nullopt) {
        lastOdometryMeasurement = newOdometryMeasurement;
        return;
    }

    Odometry odometryDelta = newOdometryMeasurement - *lastOdometryMeasurement;
    lastOdometryMeasurement = newOdometryMeasurement;

    localiser.onOdometry(odometryDelta);
}

void SelfEstimator::onGlobalPoseObservation(GlobalPoseObservation::SharedPtr observation) {
    if (gameState == GameState::Initial) {
        // in initial state, ignore global pose
        // return;
    }

    localiser.onGlobalPoseObserved(*observation);
}

void SelfEstimator::publishEstimate() {
    // RCLCPP_INFO(rclcpp::get_logger("SelfEstimator"), "publishEstimate: started");
    // Query an estimate from the localiser
    PoseEstimate estimate = localiser.estimate();

    // construct geometry_msgs/Pose
    geometry_msgs::msg::Pose pose = geometry_msgs::msg::Pose();
    pose.position.x = estimate.pose.x()*MM2M;
    pose.position.y = estimate.pose.y()*MM2M;
    pose.position.z = 0;

    geometry_msgs::msg::Transform transform = geometry_msgs::msg::Transform();
    transform.translation.x = estimate.pose.x()*MM2M;
    transform.translation.y = estimate.pose.y()*MM2M;
    transform.translation.z = 0;

    // construct rotation quaternion from a rotation around the z-axis
    Eigen::Quaternionf quaternion(
        Eigen::AngleAxisf(estimate.pose[State::H], Vector3f::UnitZ()));

    // eigen quaternion to geometry_msgs quaternion
    geometry_msgs::msg::Quaternion q;
    q.x = quaternion.x();
    q.y = quaternion.y();
    q.z = quaternion.z();
    q.w = quaternion.w();

    pose.orientation = q;
    transform.rotation = q;

    std_msgs::msg::Header header = std_msgs::msg::Header();
    header.frame_id = "/world";
    header.stamp = get_clock()->now();

    // TODO: DEBUG INFO
    // for (auto& kf: localiser.kfs)
        // RCLCPP_INFO(rclcpp::get_logger("SelfEstimator"), "X: %f Y: %f, H: %f, Weight: %f", kf.state[State::X], kf.state[State::Y], kf.state[State::H], kf.weight);

    // RCLCPP_INFO(rclcpp::get_logger("SelfEstimator"), "Publishing pose estimate: X: %f Y: %f, H: %f", pose.position.x, pose.position.y, estimate.pose[State::H]);

    // Package the input into a message ready to send through a ros2 topic
    geometry_msgs::msg::PoseStamped poseStamped =
        geometry_msgs::msg::PoseStamped();
    poseStamped.pose = pose;
    poseStamped.header = header;

    selfPublisher->publish(poseStamped);

    geometry_msgs::msg::TransformStamped transformStamped =
        geometry_msgs::msg::TransformStamped();
    transformStamped.transform = transform;
    transformStamped.header = header;
    transformStamped.child_frame_id = "/base_footprint";

    tf2Broadcaster->sendTransform(transformStamped);

    poseHistory.push_back(poseStamped);
    nav_msgs::msg::Path path;
    path.header = header;
    path.poses = poseHistory;
    selfPathPublisher->publish(path);

    // Publish array of hypotheses
    header.frame_id = "/world";
    geometry_msgs::msg::PoseArray pose_array;
    pose_array.header = header;

    geometry_msgs::msg::Pose hypothesis;
    for (auto& kf: localiser.newKfs) {
        hypothesis.position.x = kf.state[State::X]*MM2M;
        hypothesis.position.y = kf.state[State::Y]*MM2M;
        hypothesis.position.z = 0;
        pose_array.poses.push_back(hypothesis);
    }
    poseArrayPublisher->publish(pose_array);
}

void SelfEstimator::onInitialPose(geometry_msgs::msg::PoseWithCovarianceStamped pose_with_covariance_stamped) {
    // RCLCPP_INFO(get_logger(), "Received initial pose message");
    if (pose_with_covariance_stamped.header.frame_id != "world") {
        RCLCPP_WARN(get_logger(), "Received initial pose message with frame of reference \"%s\", but expected one in \"world\". Ignoring.", pose_with_covariance_stamped.header.frame_id.c_str());
        return;
    }

    const geometry_msgs::msg::Quaternion &q = pose_with_covariance_stamped.pose.pose.orientation;

    // extract heading from
    const float heading = atan2f(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    );

    const float x = pose_with_covariance_stamped.pose.pose.position.x * M2MM;
    const float y = pose_with_covariance_stamped.pose.pose.position.y * M2MM;
    poseHistory.clear();

    localiser.setState(Vector3f(x, y, heading));
    publishEstimate();
}

int main(int argc, const char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SelfEstimator>());
    rclcpp::shutdown();
    return 0;
}

