#include "stateestimation/self/FieldFeatureLocations.hpp"

FieldFeatureLocations::FieldFeatureLocations() {

    // !! NOTE !!
    // The following comments describe field features as if we're looking at Offnao, but
    // the orientation is defined in the field coordinate convenstions.
    /**
     * corner 0 orientation:
     * 
     * y 
     * ^
     * |    /
     * |  /
     * |/
     * |--------------> x
     * |\
     * |  \
     * |    \
     * 
     */

    corners.push_back(FieldFeatureLocation(-FIELD_LENGTH / 2.0, FIELD_WIDTH / 2.0, -M_PI_4));                                       // Left Top
    corners.push_back(FieldFeatureLocation(-FIELD_LENGTH / 2.0 + GOAL_BOX_LENGTH, GOAL_BOX_WIDTH / 2.0, -3.0 * M_PI_4));            // Left Goal Top
    corners.push_back(FieldFeatureLocation(-FIELD_LENGTH / 2.0 + GOAL_BOX_LENGTH, -GOAL_BOX_WIDTH / 2.0, 3.0 * M_PI_4));            // Left Goal Bottom
    corners.push_back(FieldFeatureLocation(-FIELD_LENGTH / 2.0 + PENALTY_AREA_LENGTH, PENALTY_AREA_WIDTH / 2.0, - 3.0 * M_PI_4));   // Left Penalty Area Top
    corners.push_back(FieldFeatureLocation(-FIELD_LENGTH / 2.0 + PENALTY_AREA_LENGTH, -PENALTY_AREA_WIDTH / 2.0, 3.0 * M_PI_4));    // Left Penalty Area Bottom
    corners.push_back(FieldFeatureLocation(-FIELD_LENGTH / 2.0, -FIELD_WIDTH / 2.0, M_PI_4));                                       // Left Bottom
    corners.push_back(FieldFeatureLocation(FIELD_LENGTH / 2.0, FIELD_WIDTH / 2.0, -3.0 * M_PI_4));                                  // Right Top
    corners.push_back(FieldFeatureLocation(FIELD_LENGTH / 2.0 - GOAL_BOX_LENGTH, GOAL_BOX_WIDTH / 2.0, -M_PI_4));                   // Right Goal Top
    corners.push_back(FieldFeatureLocation(FIELD_LENGTH / 2.0 - GOAL_BOX_LENGTH, -GOAL_BOX_WIDTH / 2.0, M_PI_4));                   // Right Goal Bottom
    corners.push_back(FieldFeatureLocation(FIELD_LENGTH / 2.0 - PENALTY_AREA_LENGTH, PENALTY_AREA_WIDTH / 2.0, -M_PI_4));           // Right Penalty Area Top
    corners.push_back(FieldFeatureLocation(FIELD_LENGTH / 2.0 - PENALTY_AREA_LENGTH, -PENALTY_AREA_WIDTH / 2.0, M_PI_4));           // Right Penalty Area Bottom
    corners.push_back(FieldFeatureLocation(FIELD_LENGTH / 2.0, -FIELD_WIDTH / 2.0, 3.0 * M_PI_4));                                  // Right Bottom

    tJunctions.push_back(FieldFeatureLocation(-FIELD_LENGTH / 2.0, GOAL_BOX_WIDTH / 2.0, 0));          // Left Goal top
    tJunctions.push_back(FieldFeatureLocation(-FIELD_LENGTH / 2.0, -GOAL_BOX_WIDTH / 2.0, 0));         // Left Goal bottom
    tJunctions.push_back(FieldFeatureLocation(-FIELD_LENGTH / 2.0, PENALTY_AREA_WIDTH / 2.0, 0));      // Left Penalty Area top
    tJunctions.push_back(FieldFeatureLocation(-FIELD_LENGTH / 2.0, -PENALTY_AREA_WIDTH / 2.0, 0));     // Left Penalty Area bottom
    tJunctions.push_back(FieldFeatureLocation(0, FIELD_WIDTH / 2.0, -M_PI_2));                         // Centre Top
    tJunctions.push_back(FieldFeatureLocation(0, -FIELD_WIDTH / 2.0, M_PI_2));                         // Centre Bottom
    tJunctions.push_back(FieldFeatureLocation(FIELD_LENGTH / 2.0, GOAL_BOX_WIDTH / 2.0, M_PI));        // Right Goal Top
    tJunctions.push_back(FieldFeatureLocation(FIELD_LENGTH / 2.0, -GOAL_BOX_WIDTH / 2.0, M_PI));       // Right Goal Bottom
    tJunctions.push_back(FieldFeatureLocation(FIELD_LENGTH / 2.0, PENALTY_AREA_WIDTH / 2.0, M_PI));    // Right Penalty Area Top
    tJunctions.push_back(FieldFeatureLocation(FIELD_LENGTH / 2.0, -PENALTY_AREA_WIDTH / 2.0, M_PI));   // Right Penalty Area Bottom

    centreCircles.push_back(FieldFeatureLocation(0, 0, M_PI_2));  // Orientation 1
    centreCircles.push_back(FieldFeatureLocation(0, 0, -M_PI_2)); // Orientation 2

    // We only use these three lines in the x-direction, because only large lines are passed from field features,
    // and having too many lines in close locations can cause the hypotheses from that observation
    // to merge and get a high weight (because many hypotheses are merged!)
    lines.push_back(FieldLineLocation(LineDirection::X, -FIELD_LENGTH / 2.0));                          // Left Goal BaseLine
    lines.push_back(FieldLineLocation(LineDirection::X, -FIELD_LENGTH / 2.0 + PENALTY_AREA_LENGTH));    // Left Penalty Area line
    lines.push_back(FieldLineLocation(LineDirection::X, 0));                                            // centre line
    lines.push_back(FieldLineLocation(LineDirection::X, FIELD_LENGTH / 2.0));                           // Right Goal BaseLine
    lines.push_back(FieldLineLocation(LineDirection::X, FIELD_LENGTH / 2.0 - PENALTY_AREA_LENGTH));     // Right Penalty Area Line
    lines.push_back(FieldLineLocation(LineDirection::Y, FIELD_WIDTH / 2.0));                            // Top Side Line
    lines.push_back(FieldLineLocation(LineDirection::Y, -FIELD_WIDTH / 2.0));                           // Bottom Side Line
}
