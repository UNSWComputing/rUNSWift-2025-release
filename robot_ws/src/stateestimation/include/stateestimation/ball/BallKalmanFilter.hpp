#pragma once

#include <Eigen/Dense>
#include <vector>
#include "stateestimation/ball/BallObservation.hpp"
#include "stateestimation/ball/BallEstimatorParams.hpp"
#include "stateestimation/types.hpp"

typedef Eigen::Vector4f StateVector;
typedef Eigen::Matrix4f CovarianceMatrix;


class BallKalmanFilter {
public:
	
	/*
	 *		Class Fields		
	 */

	StateVector state;
	CovarianceMatrix covariance;
	double weight = ballEstimatorParams.weightInitial;

	/*
	 *		Class Methods
	 */

    // Extrapolate the new position and velocity of the ball.
    // Predict step, ball motion
    void predict(float timeDelta);

    // Alter the state based on collision between the ball and the robot.
    // Predict step, when collision is expected to happen
    void resolveSelfCollision();

    // Alter the state based on registered odometry.
    // Predict step, robot motion (relative motion of robot frame)
    void integrateOdometry(Odometry &odometryDelta);

    // Alter the state based on a ball observation.
    // Update step
    void integrateObservation(BallObservation &ball, bool stationary);

	// Helper function to calculate distance from robot to ball
	float distance();


	/*
	 *		Class Constructors
	 */

	BallKalmanFilter() : state(StateVector::Zero()), covariance(CovarianceMatrix::Identity() * 1000) {};
	
	BallKalmanFilter(BallObservation& ball) : covariance(CovarianceMatrix::Identity() * 1000) {
		state.head(2) = ball.cameraSpacePosition.head(2);
		covariance *= ball.confidenceScore;
	}
};
