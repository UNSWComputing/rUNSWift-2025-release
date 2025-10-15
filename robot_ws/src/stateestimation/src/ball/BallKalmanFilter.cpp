//#pragma once

#include "stateestimation/ball/BallKalmanFilter.hpp"
#include "stateestimation/ball/BallEstimatorParams.hpp"
#include "stateestimation/types.hpp"
#include <Eigen/Dense>
#include <climits>
#include <cmath>
#include <rclcpp/rclcpp.hpp>


void BallKalmanFilter::predict(float timeDelta) {
    if (timeDelta >= 1) {
        RCLCPP_WARN(rclcpp::get_logger("BallKalmanFilter"), "predict() called when timeDelta = %fs! Prediction is likely very inaccurate!", timeDelta);
    }

    Eigen::Matrix4f stateTransitionMatrix = Eigen::Matrix4f::Identity();
    stateTransitionMatrix(State::X, State::U) = timeDelta;
    stateTransitionMatrix(State::Y, State::V) = timeDelta;

    //hayden: A & B should be different,
    //A is for the covariance contributed by acceleration X
    //B is for the covariance contributed by acceleration Y
    // processNoiseCovarianceA        processNoiseCovarianceB
    //[dt^4/4 0      dt^3/2 0 |      [0      0      0      0      |
    // 0      0      0      0 |       0      dt^4/4 0      dt^3/2 |
    // dt^3/2 0      dt^2   0 |       0      0      0      0      |
    // 0      0      0      0 ]       0      dt^3/2 0      dt^2   ]
    CovarianceMatrix processNoiseCovarianceA = CovarianceMatrix::Zero();
    processNoiseCovarianceA(State::X, State::X) = pow(timeDelta, 4) / 4;
    processNoiseCovarianceA(State::X, State::U) = pow(timeDelta, 3) / 2;
    processNoiseCovarianceA(State::U, State::X) = processNoiseCovarianceA(State::X, State::U);
    processNoiseCovarianceA(State::U, State::U) = pow(timeDelta, 2);


    CovarianceMatrix processNoiseCovarianceB = CovarianceMatrix::Zero();
    processNoiseCovarianceB(State::Y, State::Y) = pow(timeDelta, 4) / 4;
    processNoiseCovarianceB(State::Y, State::V) = pow(timeDelta, 3) / 2;
    processNoiseCovarianceB(State::V, State::Y) = processNoiseCovarianceA(State::Y, State::V);
    processNoiseCovarianceB(State::V, State::V) = pow(timeDelta, 2);

    float ballSpeed = hypotf(state(State::U), state(State::V)); // sqrt(u^2 + v^2)
    float ballRollDirection = atan2f(state(State::V), state(State::U));

    //capped ball acceleration, to do, either remove this or always cap to 0 when ball's stopped
    //this value is always assigned to a vector with same direction as velocity
    //making it negative will deccelerate the ball
    float acceleration = std::max(
        ballEstimatorParams.ballAcceleration, //-390.0f, upper limit of decceleration magnitude
        (-ballSpeed) / timeDelta
    );
    Eigen::Vector4f accelerationTimeIntegrator (
        pow(timeDelta, 2) / 2 * cosf(ballRollDirection),
        pow(timeDelta, 2) / 2 * sinf(ballRollDirection),
        timeDelta * cosf(ballRollDirection),
        timeDelta * sinf(ballRollDirection)
    );
    //current kinematic model is x = x + v*dt + 1/2*a*t^2
    state = stateTransitionMatrix * state + accelerationTimeIntegrator * acceleration;
    covariance =
        stateTransitionMatrix * covariance * stateTransitionMatrix.transpose() +
        processNoiseCovarianceA * pow(ballEstimatorParams.stdAccelerationX, 2) +
        processNoiseCovarianceB * pow(ballEstimatorParams.stdAccelerationY, 2);
    //std::cout<<"Predict X: "<<state(State::X)<< ", Y: " << state(State::Y)<< std::endl;
}


// Alter the state based on collision between the ball and the robot.
void BallKalmanFilter::resolveSelfCollision() {
    // ported from: BallCMKF::resolveSelfCollision

    float ballDistance = hypotf(state(State::X), state(State::Y));

    if (ballDistance > ballEstimatorParams.collisionRobotRadius) {
        // no need for collision resolution
        return;
    }

    float ballHeading = atan2f(state(State::Y), state(State::X));
    float ballRollDirection = atan2f(state(State::V), state(State::U));

    bool ballIsRollingTowardsRobot = fabs(
        normaliseTheta(ballHeading + M_PI - ballRollDirection)
    ) < ballEstimatorParams.collisionAcceptableHeadingDiff;

    if (ballIsRollingTowardsRobot) {
        // if the ball is rolling into the robot, it bounces off
        state.block<2, 1>(State::U, 0) *= -ballEstimatorParams.collisionBounciness;
    } else {
        // if the ball is inside the robot collision radius, we push it out.
        // making sure ball as far as at least 100mm margin
        float distanceIncrease = ballEstimatorParams.collisionRobotRadius - ballDistance;
        state(State::X) += distanceIncrease * cosf(ballHeading);
        state(State::Y) += distanceIncrease * sinf(ballHeading);
    }
    //std::cout<<"Collision X: "<<state(State::X)<< ", Y: " << state(State::Y)<< std::endl;
}


// Alter the state based on registered odometry.
// hayden: it doesn't account for the covariance caused by linear motion, jacobian only concludes the rotation operation
void BallKalmanFilter::integrateOdometry(Odometry &odometryDelta) {
    state.head(2) -= odometryDelta.displacement;
    Eigen::Matrix2f rotation = Eigen::Rotation2Df(-odometryDelta.turn).toRotationMatrix();
    state.head(2) = rotation * state.head(2); // Update the position estimate
    state.tail(2) = rotation * state.tail(2); // Update the velocity estimate

    // (TODO-kenji) I need an appropriate covariance transform here this matrix
    // rotation is derived by myself and seems to be the same as B-Human's. I
    // think though, that it's not right, but "good enough" for our use.  I
    // think the 0's in the covariance parts shouldn't be 0... Some genius in
    // the future rUNSWift can derivce the actual covariance.

    // martin, inuka:
    // as future geniuses looking at this codebase, we can't be bothered to
    // check: we might need EVEN SMARTER future geniuses to check this code.
    // hayden: this logic is actually simple,
    //          when updating the state, it rotates (x,y) pose => block<2, 2>(0, 0) = rotation;
    //                                 then rotates (u,v) velocity => block<2, 2>(2, 2) = rotation;
    //          so when updating the state, the jacobian (linearised operation matrix) is defined as follows
    //the UPDATED covariance in this prediction step is covarianceRotated = Jacobian * covariance * Jacobian'
    Eigen::Matrix4f jacobianRotation = Eigen::Matrix4f::Zero();
    jacobianRotation.block<2, 2>(0, 0) = rotation;
    jacobianRotation.block<2, 2>(2, 2) = rotation;

    // Rotation matrix is orthogonal, and hence transpose=inverse
    covariance = jacobianRotation * covariance * jacobianRotation.transpose();
    //std::cout<<"Odom X: "<<state(State::X)<< ", Y: " << state(State::Y)<< std::endl;
}


void BallKalmanFilter::integrateObservation(BallObservation &ballPos, bool stationary) {
    static const Eigen::Matrix<float, 2, 4> ballObservationModel = (Eigen::Matrix<float, 2, 4>() <<
                                                                    1.0f, 0.0f, 0.0f, 0.0f,
                                                                    0.0f, 1.0f, 0.0f, 0.0f).finished();


    static const Eigen::Matrix<float, 4, 2> ballObservationModelTranspose(ballObservationModel.transpose());

    // ported from: BallCMKF::update
    const float stdObservationDistBase = stationary
        ? ballEstimatorParams.stdObservationDistBaseStanding
        : ballEstimatorParams.stdObservationDistBaseWalking;

    const float stdObservationDistIncreaseRate = stationary
        ? ballEstimatorParams.stdObservationDistIncreaseRateStanding
        : ballEstimatorParams.stdObservationDistIncreaseRateWalking;

    const float stdObservationHead = stationary
        ? ballEstimatorParams.stdObservationHeadStanding
        : ballEstimatorParams.stdObservationHeadWalking;

    // take X and Y components of the ball's position
    // hayden: can try to raise to 3d calculation in the future
    Eigen::Vector2f observationVector = ballPos.cameraSpacePosition.head(2);
    Eigen::Vector2f innovationVector = observationVector - ballObservationModel * state;

    Polar ballPolarPos = ballPos.toPolar();

    // calculate the observation covariance matrix in relative space
    Eigen::Matrix2f observationCovariance = Eigen::Matrix2f::Zero();
    observationCovariance(0, 0) = pow(ballPolarPos.distance * stdObservationDistBase + stdObservationDistIncreaseRate,  2);
    observationCovariance(1, 1) = pow(ballPolarPos.distance * stdObservationHead, 2);

    // rotate the observation covariance matrix to match the heading of the ball observation
    Eigen::Matrix2f rotation = Eigen::Rotation2Df(ballPolarPos.heading).toRotationMatrix();
    observationCovariance = rotation * observationCovariance * rotation.transpose(); //invalid matrix product, check the dimension, matrix2f * matrix4f * matrix2f

    // Calculate covariance of innovation
    Eigen::Matrix2f innovationCovariance = observationCovariance + ballObservationModel * covariance * ballObservationModelTranspose;

    // Calculate optimal Kalman gain
    Eigen::Matrix<float, 4, 2> kalmanGain = covariance * ballObservationModelTranspose * innovationCovariance.inverse() * 10.0f;

    // Heuristic: High confidence ball overwrite heuristic
    if (ballPos.confidenceScore >= 0.75f) {
        state.head(2) = ballPos.cameraSpacePosition.head(2);
        RCLCPP_WARN(rclcpp::get_logger("BallKalmanFilter::integrateObservation"), "Overwriting state with high confidence observation!");
    }
    else {
    	state += kalmanGain * innovationVector;
    }

    // Update covariance matrix
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity() - kalmanGain * ballObservationModel;
    covariance = T * covariance *  T.transpose();

	double weightUpdate = innovationVector.transpose() * innovationCovariance.inverse() * innovationVector;
	weightUpdate = std::exp(-0.25  * weightUpdate);
	weightUpdate = std::clamp(0.01, 1.0, weightUpdate);
	weight += ballEstimatorParams.weightGrowth;
	weight *= weightUpdate;
	weight = std::min(200.0, weight);
}


float BallKalmanFilter::distance() {
	return state.head(2).norm();
}

