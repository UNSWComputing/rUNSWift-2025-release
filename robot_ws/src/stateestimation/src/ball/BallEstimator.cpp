#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>

#include <stateestimation/ball/BallEstimator.hpp>
#include <stateestimation/mathutils.hpp>

#include <stateestimation/self/FieldFeatureLocations.hpp> // FIELD_LENGTH and FIELD_WIDTH


// The logical flow of the program begins in the BallEstimator
int main(int argc, const char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BallEstimator>());
    rclcpp::shutdown();
    return 0;
}


BallEstimator::BallEstimator() : Node("ball_estimator") {
    RCLCPP_INFO(this->get_logger(), "Started initialisation");

	RCLCPP_INFO(rclcpp::get_logger("BallEstimator"), "predict() is not being called. This means balls will not be simulated. I.e. ball's velocity doesn't matter.");

    is_gyro_stable = false;
    is_stable = false;

    //-----------------------Subscriber--------------//
    //hear the message, take the message and do something in callback function
    //if there's inputs in callback function to bind,
    //the 3rd argument onwards of std::bind() are arguments to taken as callback function's inputs,
    //std::placeholders::_1 stands for the 1st argumetn provided to bound function created by bind (or _2 for 2 inputs, _3, _n)
    _visionBallSubscriber = this->create_subscription<runswift_interfaces::msg::VisionBalls>(
        "/vision/VBalls", //topic name
        10, //message queue size
        std::bind(&BallEstimator::callbackBallInfo, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "BallEstimator: _visionBallSubscriber has been started");

    //---------------------Publisher/TF2 Broadcaster-----------------//
    _tf2Broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    _tf2Buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    _tf2Listener = std::make_shared<tf2_ros::TransformListener>(*(this->_tf2Buffer));
    //everytime need to publish state, just _ballsPublisher->publish(msg)
    _ballsPublisher  = this->create_publisher<runswift_interfaces::msg::SeBallsAbsolute>("/stateestimation/SeBallsAbsolute",10);



    //-----------timer for time driven loop-----------//
    //tick() is now the callback function for timer at defined frequency
    _loopTimer = this->create_wall_timer(std::chrono::milliseconds(1000/LOOP_FREQUENCY), std::bind(&BallEstimator::loop, this));
    //count the current timestamp, used for dtInSeconds calculation
    current_timestamp = this->get_clock()->now().seconds();
    RCLCPP_INFO(this->get_logger(), "BallEstimator: _loopTimer started");
}

BallEstimator::~BallEstimator() {}


// Even when new data isn't coming in, we want to time step the simulation.
void BallEstimator::loop()
{
    // try {
    //     geometry_msgs::msg::TransformStamped t = this->_tf2Buffer->lookupTransform(
    //     "world", "base_footprint",
    //     tf2::TimePointZero);

    //     Eigen::Quaternion<float> q;
    //     q.x() = t.transform.rotation.x;
    //     q.y() = t.transform.rotation.y;
    //     q.z() = t.transform.rotation.z;
    //     q.w() = t.transform.rotation.w;
    //     this->robotHeading = q.toRotationMatrix().eulerAngles(0, 1, 2)[2];


    //     Eigen::Vector3f p;
    //     p.x() = t.transform.translation.x;
    //     p.y() = t.transform.translation.y;
    //     p.z() = t.transform.translation.z;
    //     this->robotPosition = p;
    // } catch (const tf2::TransformException & ex) {
    //     RCLCPP_INFO( this->get_logger(), "Could not transform /world to /base_footprint: %s", ex.what());
    // }

    //wrap the dtInSeconds and integrate the speed with dtInSeconds
    double previous_timestamp = current_timestamp;

    //hayden: note that you have to assign it to double, otherwise the float variable will not be updated and
    //return sth like ******.000000 (decimal place values are always 0)
    //it is not overflow but it truncates precision for large value
    //such as timestamp 1735743665.786521 to 1735743680.000000 and freeze there for hundreds seconds
    current_timestamp = this->get_clock()->now().seconds();

    //for smaller values (~0.1 here), float type can handle that
    dtInSeconds = static_cast<float>(current_timestamp - previous_timestamp);


	BallKalmanFilter& best = ballKalmanFilters[0];
	for (auto& kf : ballKalmanFilters) {
		kf.predict(dtInSeconds);
		if (kf.weight > best.weight)
			best = kf;
	}

    geometry_msgs::msg::TransformStamped transformStamped;

	// We publish only the best estimate to the TF tree.
    // Set the position, rotation doesn't matter here.
    transformStamped.transform.translation.x = best.state[State::X] / 1000.0f;
    transformStamped.transform.translation.y = best.state[State::Y] / 1000.0f;
    transformStamped.transform.translation.z = 0;
    transformStamped.header.stamp = this->now();
    transformStamped.header.frame_id = "/world";
    transformStamped.child_frame_id = "/ball";

	// Write all of the estimates into an array
	runswift_interfaces::msg::SeBallsAbsolute absBallMsg;
	std::vector<runswift_interfaces::msg::SeBall> absBalls;

	for (auto& kf : ballKalmanFilters) {
		runswift_interfaces::msg::SeBall ball;

		ball.pos_x = kf.state[State::X];
		ball.pos_y = kf.state[State::Y];
		ball.vel_x = kf.state[State::U];
		ball.vel_y = kf.state[State::V];
		ball.confidence = kf.weight;

		absBalls.push_back(ball);
	}

    absBallMsg.header = transformStamped.header;
	absBallMsg.balls_absolute = absBalls;

    // Broadcast the transform and publish to topic
    _tf2Broadcaster->sendTransform(transformStamped);
    _ballsPublisher->publish(absBallMsg);
    RCLCPP_INFO(get_logger(), "Published best ball estimate | position = (%f, %f) | velocity = (%f, %f) | weight = %f | kf count = %ld", best.state[State::X], best.state[State::Y], best.state[State::U], best.state[State::V], best.weight, ballKalmanFilters.size());


	/*
	 *		Post-processing steps
	 */

	prune_invalid();
	merge_similar();
	decay();


	for (auto& kf : ballKalmanFilters) // TODO: !!! DEBUG ONLY REMOVE IF YOU SEE THIS !!!
		RCLCPP_INFO(rclcpp::get_logger("BallEstimator::callbackBallInfo"), "(%f, %f) with weight (%f)", kf.state(0), kf.state(1), kf.weight);

	if (ballKalmanFilters.empty())
		RCLCPP_INFO(rclcpp::get_logger("BallEstimator::loop"), "No valid Kalman filters!");

}


//--------------------callback wrappers------------------//


/*
 *	callbackBallInfo
 *
 * This function is a callback which is called whenever information is received
 * on /vision/VBalls.
 *
 * Whenever we observe a ball, we want to integrate the observation into the
 * existing kalman filters. The only issue is, sometimes balls are so far off
 * that they should form new kalman filters instead. What we can then do is
 * greedily merge each ball with the first kalman filter we find that is within
 * some distance.
 *
 * Possible issues:
 * 	- If two KFs are equally close to a ball, then shouldn't the ball be
 * 	merged into both? Yes, but since close KFs themselves are merged
 * 	together, this should be a non-issue.
 *
 */
void BallEstimator::callbackBallInfo(const runswift_interfaces::msg::VisionBalls::SharedPtr msg) {
	// RCLCPP_INFO(rclcpp::get_logger("BallEstimator::callbackBallInfo"), "Called");
	std::vector<BallObservation> balls;

    // lookup the transform from world to base_footprint
    try {
        geometry_msgs::msg::TransformStamped t = this->_tf2Buffer->lookupTransform(
        "world", "base_footprint",
        tf2_ros::fromMsg(msg->header.stamp));

        Eigen::Quaternion<float> q;
        q.x() = t.transform.rotation.x;
        q.y() = t.transform.rotation.y;
        q.z() = t.transform.rotation.z;
        q.w() = t.transform.rotation.w;
        this->robotHeading = q.toRotationMatrix().eulerAngles(0, 1, 2)[2];


        Eigen::Vector3f p;
        p.x() = t.transform.translation.x;
        p.y() = t.transform.translation.y;
        p.z() = t.transform.translation.z;
        this->robotPosition = p;
    } catch (const tf2::TransformException & ex) {
        RCLCPP_INFO( this->get_logger(), "Could not transform /world to /base_footprint: %s", ex.what());
    }

	// Write each observed ball into the ball stack
    for (const auto &ball_feature : msg->ball_features) {
        BallObservation ball;

        if (ball_feature.confidence_score < 180.0f) continue;

        //wrap cameraSpacePosition
        // TODO: Transform to global first
        ball.cameraSpacePosition(0) = ball_feature.ball_coordinates.x;
        ball.cameraSpacePosition(1) = ball_feature.ball_coordinates.y;
        ball.cameraSpacePosition(2) = ball_feature.ball_coordinates.z;
        ball.cameraSpacePosition = Eigen::AngleAxisf(robotHeading, Vector3f::UnitZ()) * ball.cameraSpacePosition;
        ball.cameraSpacePosition += this->robotPosition * 1000.0f;

        //wrap screenSpaceRadius
        ball.screenSpaceRadius = ball_feature.pixel_radius; //assigning uint16 to int (uint32)

        //wrap screenSpacePosition
        ball.screenSpacePosition(0) = ball_feature.ball_pixel_coordinates.x; //assigning double (float64) to int
        ball.screenSpacePosition(1) = ball_feature.ball_pixel_coordinates.y;

		ball.confidenceScore = ball_feature.confidence_score / 255.0f;

        balls.push_back(ball);
    }



	// Now, we consume each ball, integrating it with the closest kalman filter
	// as we go.
	for (auto& ball = balls.back(); !balls.empty();) {
		Eigen::Vector2f ball_pos = ball.cameraSpacePosition.head(2);

        // hack(martin): ball distance from robot appears 2x what it should be?
        //               adding this here so we don't get cooked at robocup 2025
        // ball_pos += (robotPosition.head(2) - ball_pos) * 0.5;

		for (auto& kf : ballKalmanFilters) {
			// Threshold hardcoded to 50cm until this starts being an issue
			if ((ball_pos - kf.state.head(2)).norm() < 500.0f) {
				// Since the ball is close enough, we can merge it into this kf
				kf.integrateObservation(ball, is_stable);

				// Don't flame me guys...
				// https://stackoverflow.com/questions/1257744/can-i-use-break-to-exit-multiple-nested-for-loops
				goto endballsloop;
			}
		}

		// If we can't find a close enough kf, we can create a new hypothesis
		ballKalmanFilters.push_back(ball);
		RCLCPP_INFO(rclcpp::get_logger("BallEstimator::callbackBallInfo"), "There are currently %ld ball Kalman filters present.", ballKalmanFilters.size());

endballsloop:
		balls.pop_back();
	}

}

/*
 *		Kalman Filter Helper Functions
 */

/*
 *	Any kfs that are within a certain distance of each other are merged.
 */
void BallEstimator::merge_similar() {
	if(ballKalmanFilters.size()<2) {
		return;
	}

	for (auto kf1 = ballKalmanFilters.begin(); kf1 != ballKalmanFilters.end(); ++kf1) {
		for (auto kf2 = kf1 + 1; kf2 != ballKalmanFilters.end(); ) {
			if ((kf1->state.head(2) - kf2->state.head(2)).norm() < ballEstimatorParams.mergeThresh) {
				merge(*kf1, *kf2);
				kf2 = ballKalmanFilters.erase(kf2);
			}else {
				++kf2;
			}
		}
	}
}


void merge(BallKalmanFilter& kf1, BallKalmanFilter& kf2) {
	/*
	 * Extract from old codebase:
	 * carlinwilliamson:
	 *		We take a weight average if the weights are relatively
	 *		similar to each other.  We don't take the weighted
	 *		average if one weight is significanlty greater than the
	 *		other, since it causes drift (the drift is explained in
	 *		the following paper)
	 *
	 *		https://www.cs.utexas.edu/~pstone/Courses/393Rfall11/resources/RC09-Quinlan.pdf
	 *
	 */

	float sumWeights = kf1.weight + kf2.weight;

	if (kf1.weight > 10.0 * kf2.weight) {
        kf1.weight = sumWeights;
        kf2.weight = -1;
    } else if (kf2.weight > 10.0 * kf1.weight) {
        kf1.state = kf2.state;
        kf1.covariance = kf2.covariance;
        kf1.weight = sumWeights;
        kf2.weight = -1;
    } else {
        float kf1Ratio = kf1.weight / sumWeights;
        float kf2Ratio = kf2.weight / sumWeights;

        kf1.covariance = kf1Ratio * kf1.covariance + kf2Ratio * kf2.covariance;

        kf1.weight = sumWeights;
        kf2.weight = -1;
    }
}


void BallEstimator::prune_invalid() {
	ballKalmanFilters.erase(
		std::remove_if(
			ballKalmanFilters.begin(), ballKalmanFilters.end(),
			[] (auto& kf) {
				bool cond = false;

				// Remove any filter which satisfies one of these conditions:
				cond |=		kf.weight < 1.0f;	// Old and low confidence estimates
				cond |=		std::fabs(kf.state(0)) > FIELD_X_CLIP; // Outside the field (x axis)
				cond |=		std::fabs(kf.state(1)) > FIELD_Y_CLIP; // Outside the field (y axis)

				return cond;
			}
		),
		ballKalmanFilters.end()
	);
}


void BallEstimator::decay() {
	for (auto& kf : ballKalmanFilters)
		kf.weight *= (kf.distance() < 1500.0f) ?
			ballEstimatorParams.closeDecayRate : ballEstimatorParams.farDecayRate;
}
