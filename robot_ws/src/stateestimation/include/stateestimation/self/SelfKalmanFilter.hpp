#pragma once

#include <Eigen/Dense>

// #include "FieldFeatureLocations.hpp"
// #include "FieldFeatureObservation.hpp"
#include <runswift_interfaces/msg/global_pose_observation.hpp>
#include "stateestimation/types.hpp"

using runswift_interfaces::msg::GlobalPoseObservation;

struct PoseEstimate {
    // Position and heading of the robot.
    const Pose pose;

    // The area of the smallest possible rectangle that can be fit around the
    // position covariance ellipse. Higher values indicate more uncertainty in the position.
    float positionUncertainty;
    float headingUncertainty;
};

// This is a kalman filter that represents one "hypothesis" of where we could be.
// in a sense: SelfKalmanFilter <=> SelfHypothesis
class SelfKalmanFilter {
   public:
    // These should be defined here because different Kalman filters may have different state vectors.
    // eg. ball vs self
    typedef Pose StateVector;
    typedef Eigen::Matrix3f CovarianceMatrix;
    
    // The state estimate.
    StateVector state;

    // The state covariance.
    CovarianceMatrix covariance;

    // How much we trust this hypothesis.
    float weight;

    // Initialise with zero values for state, covariance and 1.0 for weight.
    SelfKalmanFilter() : state(StateVector::Zero()), covariance(CovarianceMatrix::Zero()), weight(1.0) {}

    // Initialise with explicit values for state, covariance and weight.
    SelfKalmanFilter(StateVector _state, CovarianceMatrix _covariance, float _weight) : state(_state), covariance(_covariance), weight(_weight) {};

    // Update the state and covariance of the Kalman filter with the given odometry delta.
    void integrateOdometry(Odometry &odometryDelta);

    // // Update the state and covariance of the Kalman filter with the given field feature.
    // void integrateFieldFeature(const FieldFeatureObservation &ff, const FieldFeatureLocation &onField);

    void integrateGlobalPoseObservation(const GlobalPoseObservation &observation);

    // // Update the state and covariance of the Kalman filter with the given field
    // // line, given its direction (is it a X-line or Y-line) and whether we are
    // // on the "positive" side of this line.
    // void integrateFieldLine(const FieldFeatureObservation &observed, const FieldLineLocation &expected, Sign lineSide);

    // Merge with another filter, leaving its weight equal to -1 in the process.
    void mergeWith(SelfKalmanFilter *other);

    // We define uncertainty as the area of the smallest possible rectangle that can be fit around the
    // position covariance ellipse.
    float getPositionUncertainty();

    // Variance of the heading, straight from the covariance matrix.
    // Martin: is this really accurate?
    float getHeadingUncertainty();

    // Return a pose estimate based on this filter's state and covariance.
    PoseEstimate toPoseEstimate();

    // Create a copy of the Kalman filter with a new weight.
    SelfKalmanFilter copyWithWeight(float weight);
};
