#include "stateestimation/self/Localiser.hpp"
#include "stateestimation/self/FieldFeatureLocations.hpp"
#include "stateestimation/self/LocaliserParams.hpp"
#include "stateestimation/self/SelfKalmanFilter.hpp"
#include "stateestimation/mathutils.hpp"

#include <Eigen/src/Core/Matrix.h>
#include <cmath>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>

void Localiser::mergeAndPruneKfs() {
    // // RCLCPP_INFO(rclcpp::get_logger("Localiser::mergeAndPruneKfs"), "Called");
    // prune off-field KFs
    kfs.erase(
        std::remove_if(kfs.begin(), kfs.end(), [](SelfKalmanFilter kf) {
            const float fieldXClip = (float) FIELD_LENGTH/2 + 700;
            const float fieldYClip = (float) FIELD_WIDTH/2 + 700;

            bool is_off_field = (fabs(kf.state.x()) > fieldXClip) || (fabs(kf.state.y()) > fieldYClip);
            bool has_nan = kf.state.hasNaN();

            if (is_off_field || has_nan) {
                RCLCPP_INFO(rclcpp::get_logger("Localiser::mergeAndPruneKfs"), "Pruned off-field KF");
            }

            return is_off_field || has_nan;
        }),
        kfs.end()
    );

    // merge similar KFs
    for (std::vector<SelfKalmanFilter>::iterator kf1 = kfs.begin(); kf1 != kfs.end(); ++kf1) {
        if (kf1->weight <= 0) continue;

        for (std::vector<SelfKalmanFilter>::iterator kf2 = kf1 + 1; kf2 != kfs.end(); ++kf2) {
            if (kf2->weight <= 0) continue;

            float dx = fabs(kf1->state.x() - kf2->state.x());
            float dy = fabs(kf1->state.y() - kf2->state.y());
            float dh = minThetaDiff(kf1->state.z(), kf2->state.z());

            if (dx > localiserParams.similarXThresh) continue;
            if (dy > localiserParams.similarYThresh) continue;
            if (dh > deg2radf(localiserParams.similarHeadingThresh)) continue;

            // RCLCPP_INFO(rclcpp::get_logger("Localiser::mergeAndPruneKfs"), "Merged KF ");
            RCLCPP_INFO(
                rclcpp::get_logger("Localiser::mergeAndPruneKfs"),
                "Merging KF1: x=%.2f, y=%.2f, h=%.2f, w=%.3f | KF2: x=%.2f, y=%.2f, h=%.2f, w=%.3f",
                kf1->state.x(), kf1->state.y(), kf1->state.z(), kf1->weight,
                kf2->state.x(), kf2->state.y(), kf2->state.z(), kf2->weight
            );

            kf1->mergeWith(&*kf2);
            // Martin: pondering: after this^, kf2 will have weight = -1, which
            // means we kind of just want to remove it?

            // in the original code, normalisation will make those -1's pretty
            // low, which might mean that they get erased during low-weight
            // pruning, but also, if everything has a low weight, they might
            // not? Maybe an extra for loop before normalisation where we erase
            // all -1's might be good, so I've added it below
        }
    }

    // // prune KFs with -1 weight
    kfs.erase(
        std::remove_if(kfs.begin(), kfs.end(), [](SelfKalmanFilter kf) {
            return kf.weight == -1;
        }),
        kfs.end()
    );

    // normalise weights
    normaliseKFWeights();

    // prune KFs with low weights
    kfs.erase(
        std::remove_if(kfs.begin(), kfs.end(), [](SelfKalmanFilter kf) {
            if (kf.weight < localiserParams.minKFWeight) {
                RCLCPP_INFO(rclcpp::get_logger("Localiser::mergeAndPruneKfs"), "Pruned low weight KF with weight %f", kf.weight);
            }
            return kf.weight < localiserParams.minKFWeight;
        }),
        kfs.end()
    );
}

void Localiser::onOdometry(Odometry &odometryDelta) {
    // // RCLCPP_INFO(rclcpp::get_logger("Localiser::onOdometry"), "Called");
    for (SelfKalmanFilter &kf : kfs) {
        // Predict the state of the Kalman filter
        kf.integrateOdometry(odometryDelta);
    }

    /*
    // warning: normalisation might be bad here
    mergeAndPruneKfs();
    */
}

void Localiser::onGlobalPoseObserved(GlobalPoseObservation &observation) {
    newKfs.clear();

    for (SelfKalmanFilter &kf : kfs) {
        // float weight = 0.8;

        // flipped variant where this observation could be
        GlobalPoseObservation flipped = observation;
        flipped.heading += M_PI;
        flipped.position.x *= -1;
        flipped.position.y *= -1;

        SelfKalmanFilter::StateVector obVector(observation.position.x, observation.position.y, observation.heading);
        SelfKalmanFilter::StateVector flippedObVector(flipped.position.x, flipped.position.y, flipped.heading);

        float dist_observation = (kf.state.head<2>() - obVector.head<2>()).norm();
        float dist_flipped = (kf.state.head<2>() - flippedObVector.head<2>()).norm();

        const SelfKalmanFilter::StateVector& v = (dist_observation <= dist_flipped) ? obVector : flippedObVector;

        // most braindead code ive ever written
        const float K = 0.8f;
        kf.state = kf.state * K + v * (1.0f - K);
    }
}

void Localiser::transitionToInitial(int playerNum) {
    // RCLCPP_INFO(rclcpp::get_logger("Localiser::transitionToInitial"), "Called");
    SelfKalmanFilter::StateVector state;

    switch(playerNum)
    {
    case 1:
        state = {-3900.0, 3000.0, -M_PI_2};
        break;

    case 2:
        state = {-2850.0, -3000.0, M_PI_2};
        break;

    case 3:
        state = {-2850.0, 3000.0, -M_PI_2};
        break;

    case 4:
        state = { -750.0, -3000.0, M_PI_2 };
        break;

    case 5:
        state = { -750.0, 3000.0, -M_PI_2 };
        break;

    case 6:
        state = { -750.0, -3000.0, M_PI_2 };
        break;

    case 7:
        state = { -750.0, 3000.0, -M_PI_2 };
        break;

    default:
        state = { -PENALTY_CROSS_ABS_X, 3000.0, -M_PI_2 };
        break;
    }

    // RCLCPP_INFO(rclcpp::get_logger("localiser"), "Transitioning to initial position: x=%f, y=%f, h=%f", state.x(), state.y(), state.z());
    setState(state);
}

void Localiser::setState(SelfKalmanFilter::StateVector state) {
    // RCLCPP_INFO(rclcpp::get_logger("localiser"), "State reset to: x=%.2f, y=%.2f, h=%.2f (%.2f deg).", state.x(), state.y(), state.z(), state.z() * 180.0f / M_PI);

    kfs.clear();
    kfs.push_back(
        SelfKalmanFilter(
            state,
            Eigen::Matrix3f::Identity() * 10.0,
            0.05
        )
    );
}

PoseEstimate Localiser::estimate() {
    // RCLCPP_INFO(rclcpp::get_logger("Localiser::estimate"), "Called");
    // best hypothesis of where we are right now
    std::optional<SelfKalmanFilter*> bestKf = findHeaviestKf();

    if (!bestKf.has_value()) {
        // RCLCPP_INFO(rclcpp::get_logger("localiser"), "Could not find a heaviest KF! kfs.size() = %ld", kfs.size());

        return PoseEstimate {
            Pose(Vector3f::Zero()),
            INFINITY,
            INFINITY
        };
    }

    return bestKf.value()->toPoseEstimate();
}

void Localiser::normaliseKFWeights() {
    // RCLCPP_INFO(rclcpp::get_logger("Localiser::normaliseKFWeights"), "Called");
    float weightSum = 0.0;
    for (SelfKalmanFilter &kf : kfs) {
        weightSum += kf.weight;
    }

    for (SelfKalmanFilter &kf : kfs) {
        kf.weight /= weightSum;
    }
}

std::optional<SelfKalmanFilter*> Localiser::findHeaviestKf() {
    std::vector<SelfKalmanFilter>::iterator it = std::max_element(
        kfs.begin(), kfs.end(),
        [](const SelfKalmanFilter& a, const SelfKalmanFilter& b) {
            return a.weight < b.weight;
        }
    );

    if (it == kfs.end()) {
        return std::nullopt;
    }

    return (SelfKalmanFilter *) &*it;
}

Localiser::Localiser() {};

