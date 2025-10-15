#pragma once
/// Math type definitions.

#include <Eigen/Dense>
#include <cmath>
#include <runswift_interfaces/msg/motion_odometry.hpp>

// General purpose vector types.
// If using these, you are assumed to be saying that the
// coordinates contained are cartesian. Please use `Polar`
// or `Pose` if you are interacting with angles.
using Eigen::Vector2f;
using Eigen::Vector3f;

// Represents a distance and heading.
struct Polar {
    float distance;
    float heading;

    Eigen::Vector2f toCartesian() const {
        return Eigen::Vector2f(
            distance * cosf(heading), 
            distance * sinf(heading)
        );
    }

    Polar(Eigen::Vector2f cartesian) {
        distance = cartesian.norm();
        heading = atan2f(cartesian.x(), cartesian.y());
    }

    Polar(float distance, float heading) : distance(distance), heading(heading) {}
};

// Represents a position and heading.
// The values are in the form (x, y, theta).
// Note that a lot of the time, it is better to use a Vector2f and a float
// as separate variables, as this makes it easier to interact with the
// position and heading separately. (eg. position.norm())
//
// This type mainly exists when you need to pack the position and heading
// together, such as when you are passing it around as a single variable.
typedef Vector3f Pose;

// Indices into StateVector's of various Kalman filters.
enum State {
    X = 0,  // x-coordinate
    Y = 1,  // y-coorindate
    H = 2,  // heading
    U = 2,  // x-velocity
    V = 3,  // y-velocity
};

enum class Sign {
    Positive = 1,
    Negative = -1,
};

// warning: this struct can either be a delta (input to most functions defined here)
//          or a since-robot-started odometry measurement (received from ros topics)
//          for the latter, it's common to have displacement.x() be like 8000 since the robot
//          has probably been told to walk forward a lot
class Odometry {
  public:
    // x corresponds to forward 
    // y corresponds to left
    Eigen::Vector2f displacement;
    float turn;

    Odometry() : displacement(Vector2f::Zero()), turn(0.0) {}
    
    Odometry(Eigen::Vector2f displacement, float turn) 
        : displacement(displacement), turn(turn) {}
    
    Odometry(float forward, float left, float turn)    
        : displacement(forward, left), turn(turn) {}
    
    Odometry(runswift_interfaces::msg::MotionOdometry msg) 
        : Odometry(msg.forward, msg.left, msg.turn) {}

    Odometry operator-(Odometry other) {
        return Odometry(displacement - other.displacement, turn - other.turn);
    }
};
