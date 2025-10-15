#pragma once
#include "stateestimation/types.hpp"
#include <Eigen/Dense>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

struct BallObservation {
    // The position of the ball in the world.Z axis is ignored.
    // Hayden: z is better included for 3d transform, we can wipe out z in ball
    // output though
    Eigen::Vector3f cameraSpacePosition;
    
    // The position of the ball on the screen.
    Eigen::Vector2f screenSpacePosition;

    // The radius of the ball on the screen.
    int screenSpaceRadius;

    // Score between 0.0f and 1.0f. If above 0.9f, we overwrite existing kalman
    // filters with this observation.
    float confidenceScore;

    Polar toPolar() {
        // if (cameraSpacePosition.z() != 0.0) {
        //     RCLCPP_WARN(rclcpp::get_logger("BallObservation"), "cameraSpacePosition.z is not zero! z = %f", cameraSpacePosition.z());
        // }

        return Polar {
            cameraSpacePosition.norm(),
            atan2f(cameraSpacePosition.y(), cameraSpacePosition.x())
        };
    }
};
