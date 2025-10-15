#include <algorithm>
#include <cmath>

#include <rclcpp/rclcpp.hpp>

#include "stateestimation/self/SelfKalmanFilter.hpp"
#include "stateestimation/self/LocaliserParams.hpp"
#include "stateestimation/mathutils.hpp"
#include "stateestimation/types.hpp"


// void SelfKalmanFilter::integrateFieldFeature(const FieldFeatureObservation &observed, const FieldFeatureLocation &expected) {
/*
    Localiser lore:

    * Localise based on field features (take a look at FieldFeatureLocation
    * for a list of all the types and ground truth locations).
    *
    * Given a field feature observed to be at some location relative to the
    * robot, we iterate through every expected field feature. For each of
    * these expected features, we calculate where the robot *should* be.
    *
    * This function is called from Localiser::onFieldFeaturesObserved( ... )
    * (referred to as OFFO). If the predicted location and heading do not
    * match up with where we expect the robot to be from previous estimates,
    * then the kalman filter is pruned in the caller function (OFFO).
    *
    * This function is run for every possible field feature and observation
    * combination. While the time complexity is bad, we're only ever dealing
    * with up to around 5 observed field features, so this is not very
    * computationally expensive.
*/


// warning: this function is unused, for salvador '25 we just lerp the observation and the current state
void SelfKalmanFilter::integrateGlobalPoseObservation(const GlobalPoseObservation &observation) {
    float predictedX = observation.position.x;
    float predictedY = observation.position.y;
    float predictedH = observation.heading;

    // difference between the old state and our new state
    Vector3f innovationVector = Vector3f(predictedX, predictedY, predictedH) - state;

    // calculate the observation covar matrix
    CovarianceMatrix observationCovariance = CovarianceMatrix::Zero();

    // NOTE: Martin: the old codebase has lots of comments about these uncertainties not exactly
    // being scientifically calculated...
    // TODO: actually calculate what they should be?

    float distance = innovationVector.head(2).norm();

    // variance is somewhat dependant on distance from object
    double distUncertainty = distance * 0.25;  // mm

    double angleUncertainty = deg2radf(localiserParams.angleUncertainty); // rad

    // This is the heading uncertainy AND the observation angle uncertainty
    double headingUncertainty = deg2radf(localiserParams.updateHeadingUncertainty);

    // Convert from polar to cartesian covariance matrix as per
    // http://resource.npl.co.uk/docs/networks/anamet/members_only/meetings/30/hall.pdf
    // Heading is treated as independant from observation distance and angle

    double Ut = distance * tanf(angleUncertainty);
    double Ur = distUncertainty;

    double a = Ur * Ur;
    double b = Ut * Ut;
    double c = a - b;

    // variances
    observationCovariance(State::X, State::X) = pow(cos(predictedH), 2) * a + pow(sinf(predictedH), 2) * b;
    observationCovariance(State::Y, State::Y) = pow(cos(predictedH), 2) * b + pow(sinf(predictedH), 2) * a;
    observationCovariance(State::H, State::H) = headingUncertainty * headingUncertainty; //heading uncertainty hard coded

    // covariances
    observationCovariance(State::X, State::Y) = 0.5 * sinf(2 * predictedH) * c;
    observationCovariance(State::Y, State::X) = observationCovariance(State::X, State::Y); // Martin: uhh this one might need to be cos, maybe mistake in the old codebase?

    CovarianceMatrix innovationCovariance = covariance + observationCovariance;
    CovarianceMatrix innovationCovarianceInv = innovationCovariance.inverse();

    // compute kalman gain
    Eigen::Matrix3f kalmanGain = covariance * innovationCovarianceInv;

    // update state to new state with respect to kalman gain
    state += kalmanGain * innovationVector;

    // Identity minus kalman gain (just for clarity)
    Eigen::Matrix3f ImK = Eigen::Matrix3f::Identity() - kalmanGain;

    // update covariance
    covariance = ImK * covariance * ImK.transpose() + kalmanGain * observationCovariance * kalmanGain.transpose();

    // adjust weight
    
    // float weightAdjustment = 1.0;
    // float weightUpdate = innovationVector.transpose() * innovationCovarianceInv * innovationVector;
    // weightAdjustment = exp(-0.5 * weightUpdate);
    // weightAdjustment = std::clamp(weightAdjustment, 0.01f, 1.0f);
    // weight *= weightAdjustment;

    // // RCLCPP_INFO(rclcpp::get_logger("localiser"), "integrateFieldFeature is unimplemented!");
}

void SelfKalmanFilter::integrateOdometry(Odometry &odometryDelta) {
    // RCLCPP_INFO(rclcpp::get_logger("SelfKalmanFilter::integrateOdometry"), "Called");
	/*
	 * Update pose esimate with odometry.
	 *
	 * The odometry subsystem supplies us with position deltas in the robot's
	 * reference frame (how far have we moved in x and y, and how much have we
	 * turned since the last poll?). We perform euler integration (AHHHHHH) to
	 * estimate the robot's new position.
	 */

    // ported from: MultiModalCMKF::predict

    // Odometry is given relative to the robot, so we transform it to worldspace
    // (i.e. make forward be in the same direction as the robot's heading).
    Eigen::Rotation2Df rotation(state(State::H));
    Eigen::Vector2f delta = rotation * odometryDelta.displacement;

    // Update the robot pose estimate.
    state(State::H) = normaliseTheta(state(State::H) + odometryDelta.turn);
    // Take the first 2 entries and add delta (left and forward)
    state.block<2,1>(0,0) += delta;

    //check_finite(kf.state, "MMCMKF predict state");

    // Update the robot pose covariance.
    Eigen::Matrix2f relativeCovariance = Eigen::Matrix2f::Zero();

    relativeCovariance(0, 0) = pow(odometryDelta.displacement.x(), 2.0) * localiserParams.odometryForwardMultiplyFactor;
    relativeCovariance(1, 1) = pow(odometryDelta.displacement.y(), 2.0) * localiserParams.odometryLeftMultiplyFactor;

    // Calculate uncertainty propagation thing (http://www.cs.cmu.edu/~rasc/Download/AMRobots5.pdf)
    CovarianceMatrix propagationJacobian = CovarianceMatrix::Identity();

    //float distanceTravelled = sqrt(pow(odometry.forward, 2) + pow(odometry.left, 2));
    float distanceTravelled = odometryDelta.displacement.norm();
    propagationJacobian(State::X, State::H) = -distanceTravelled * sinf(state(State::H));
    propagationJacobian(State::Y, State::H) =  distanceTravelled * cosf(state(State::H));

    //kf.covariance.block<3, 3>(ME_X_DIM, ME_X_DIM)  = propagationJacobian * kf.covariance.block<3, 3>(ME_X_DIM, ME_X_DIM) * propagationJacobian.transpose();
    covariance = propagationJacobian * covariance * propagationJacobian.transpose();

    // New rotation for updated heading
    rotation = Eigen::Rotation2Df(state(State::H));

    Eigen::Matrix2f odometryCovariance = rotation * relativeCovariance * rotation.inverse(); // Rotation matrix is orthogonal, and hence transpose=inverse?

    //check_finite(odometryCovariance, "MMCMKF predict odom covariance");

    covariance.block<2, 2>(0, 0) += odometryCovariance;

    covariance(State::H, State::H) += localiserParams.odometryHeadingMultiplyFactor * pow(odometryDelta.turn, 2.0);

    //check_finite(kf.covariance, "MMCMKF covariance");
}

void SelfKalmanFilter::mergeWith(SelfKalmanFilter *other) {
    //// RCLCPP_INFO(rclcpp::get_logger("SelfKalmanFilter::mergeWith"), "Called");
    /*
     * We take a weight average if the weights are relatively similar to each other.
     * We don't take the weighted average if one weight is significanlty greater than
     * the other, since it causes drift (the drift is explained in the following paper)
     * https://www.cs.utexas.edu/~pstone/Courses/393Rfall11/resources/RC09-Quinlan.pdf
     */
    float sumWeights = this->weight + other->weight;

    if (this->weight > 10.0 * other->weight) {
        // our weight is much greater than the other's weight
        // discard the other's state
    } else if (other->weight > 10.0 * this->weight) {
        // the other's weight is much greater than our weight
        // discard our state and copy the other's
        this->state = other->state;
        this->covariance = other->covariance;
    } else {
        // the two weights are similar
        float ratio = this->weight / sumWeights;

        // Weighted sum of angles is a bit special (https://stackoverflow.com/questions/1686994/weighted-average-of-angles)
        float alphaSinSum = ratio * sinf(this->state.z()) + (1.0 - ratio) * sinf(other->state.z());
        float alphaCosSum = ratio * cosf(this->state.z()) + (1.0 - ratio) * cosf(other->state.z());
        float weightedAvgH = atan2f(alphaSinSum, alphaCosSum);

        this->state = ratio * this->state + (1.0 - ratio) * other->state;
        this->state.z() = weightedAvgH;

        this->covariance = ratio * this->covariance + (1.0 - ratio) * other->covariance;
    }

    // in any case, discard the other by setting its weight to -1, and set our weight to the sum of the weights
    this->weight = sumWeights;
    other->weight = -1;
}

SelfKalmanFilter SelfKalmanFilter::copyWithWeight(float newWeight) {
    return SelfKalmanFilter(this->state, this->covariance, newWeight);
}

float SelfKalmanFilter::getPositionUncertainty() {
    Eigen::Matrix2f positionCovariance = covariance.block<2, 2>(0, 0);
    Eigen::EigenSolver<Eigen::Matrix2f> solver(positionCovariance);
    Eigen::Vector2cf ellipseAxes = solver.eigenvalues();

    float std_a = sqrt(abs(ellipseAxes[0])); // std of one of the ellipse axes
    float std_b = sqrt(abs(ellipseAxes[1])); // std of the other ellipse axis

    return std_a * std_b;
}

float SelfKalmanFilter::getHeadingUncertainty() {
    return sqrt(covariance(2, 2));
}

PoseEstimate SelfKalmanFilter::toPoseEstimate() {
    return PoseEstimate {
        Pose(state),
        getPositionUncertainty(),
        getHeadingUncertainty()
    };
}
