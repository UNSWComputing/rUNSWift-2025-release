#pragma once

#include <stateestimation/types.hpp>

enum class FieldFeatureType {
    None = 0,
    Line = 1,                   // used
    Corner = 2,                 // used
    TJunction = 3,              // used
    PenaltySpot = 4,            // not used for localisation               
    CentreCircle = 5,           // used
    FieldLinePoint = 6,         // not used for localisation
    XJunction = 7,              // not used for localisation
    ParallelLine = 8,           // not used for localisation
    GoalBoxCorner = 9,          // not used for localisation
    PenaltyBoxCorner = 10,      // not used for localisation
};


// The robot sees a feature or line of some type somewhere in its viewfinder.
// It also knows how the feature it sees is oriented.
class FieldFeatureObservation {
   public:
    // What kind of feature / line this is.
    FieldFeatureType type;

    // Position of this field feature relative to the camera.
    // +Y means further away from you, -X means to the left, +X means to the right, etc.
    Eigen::Vector2f cameraSpacePosition;

    /// The way this field feature is oriented, relative to the ray connecting
    /// the camera to the feature. This means that the orientation is 0 for all
    /// features facing towards the camera, irrespective of their position. Note
    /// that 0 does NOT mean facing in the same direction as the camera
    /// (forward), it means looking AT the camera.
    ///
    /// Source: Martin: The innovation vector calculations of the old codebase
    /// (https://github.com/UNSWComputing/rUNSWift/blob/3e518661ab8cf6ecf220064b876c002eb3b02727/robot/perception/stateestimation/localiser/multimodalcmkf/MultiModalCMKF.cpp#L367-L369)
    /// use `ff.orientation + observation.rr.orientation()` for the world-space angle between the feature `ff`
    /// and the robot. In this case, `ff.orientation` is the orientation of the feature relative to the world,
    /// and `observation.rr.orientation()` is the orientation of the feature relative to the camera view ray.
    ///
    /// This field is hence analogous to `observation.rr.orientation()` in the old codebase.
    float orientation;
    float confidence;
    FieldFeatureObservation() : type(FieldFeatureType::None),
                                cameraSpacePosition(Eigen::Vector2f::Zero()),
                                orientation(0.0) {};

    FieldFeatureObservation(
        FieldFeatureType _type,
        Eigen::Vector2f _cameraSpacePosition,
        float _cameraSpaceOrientation, float confidence) : type(_type),
                                         cameraSpacePosition(_cameraSpacePosition),
                                         orientation(_cameraSpaceOrientation), confidence(confidence) {};

    // Import camera space position from polar. Martin: This shouldn't be used
    // in production, but the old codebase's EstimatorInfoIn only has RR and we
    // want to use its vision pipeline for simulation. Remove this code when
    // it's time.
    FieldFeatureObservation(
        FieldFeatureType _type,
        Polar _polarPosition,
        float _orientation) : type(_type),
                              cameraSpacePosition(
                                  Eigen::Vector2f(_polarPosition.distance * std::cos(_polarPosition.heading),
                                                  _polarPosition.distance * std::sin(_polarPosition.heading))),
                              orientation(_orientation) {};

    // Equivalent to "rr" of FieldFeatureInfo in the old codebase.
    // Note that orientation is not included in this vector, please find it in the
    // `orientation` field on this class.
    Polar polarPosition() const;
};
