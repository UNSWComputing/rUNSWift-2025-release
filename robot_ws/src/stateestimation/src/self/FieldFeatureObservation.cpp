#include "stateestimation/self/FieldFeatureObservation.hpp"

Polar FieldFeatureObservation::polarPosition() const {
    return Polar {
        cameraSpacePosition.norm(),
        atan2f(cameraSpacePosition.y(), cameraSpacePosition.x())
    };
};
