#pragma once

#define _USE_MATH_DEFINES 
#include <cmath>
#include <vector>

#include <Eigen/Dense>
#include <stateestimation/types.hpp>

#define FIELD_LINE_WIDTH 64
#define ROBOTS_PER_TEAM 7

/** Field line dimensions */
#define FIELD_LENGTH (int) (9.07 * 1000)
#define FIELD_WIDTH  (int) (6.056 * 1000)

#define FIELD_X_CLIP (int) (FIELD_LENGTH/2 + 700)
#define FIELD_Y_CLIP (int) (FIELD_WIDTH/2 + 700)

#define FIELD_LENGTH_OFFSET (int) (0.638 * 1000)
#define FIELD_WIDTH_OFFSET (int) (0.638 * 1000)

#define OFFNAO_FIELD_LENGTH_OFFSET (int) (0.638 * 1000) + 30
#define OFFNAO_FIELD_WIDTH_OFFSET (int) (0.638 * 1000) + 30

/** Goal box */
#define GOAL_BOX_LENGTH (int) (0.6 * 1000)
#define GOAL_BOX_WIDTH (int) (2.27 * 1000)

/** Penalty Cross */
#define PENALTY_CROSS_DIMENSIONS (int) (0.1 * 1000) /* i.e. dimensions of square fitted around it */
#define DIST_GOAL_LINE_TO_PENALTY_CROSS (int) (1.28 * 1000) /* to middle of closest penalty cross */
#define PENALTY_CROSS_ABS_X (FIELD_LENGTH / 2 - DIST_GOAL_LINE_TO_PENALTY_CROSS)

/** Center Circle */
#define CENTER_CIRCLE_DIAMETER (int) (1.55 * 1000)

/** Goal Posts */
#define GOAL_POST_DIAMETER (int) (0.1 * 1000)
#define GOAL_BAR_DIAMETER 100  // Double check this once field is built
#define GOAL_POST_HEIGHT (int) (0.8 * 1000) // Measured from the bottom of the crossbar to the ground

#define GOAL_SUPPORT_DIAMETER 46
#define GOAL_WIDTH (int) (1.5 * 1000) /* top view end-to-end from middle of goal posts */
#define GOAL_DEPTH (int) (0.435 * 1000) /* Measured from the front edge of the crossbar to the centre of the rear bar */

#define PENALTY_AREA_LENGTH (int) (1.708 * 1000) // Measured from inside of goal line to outside of penalty box
#define PENALTY_AREA_WIDTH (int) (4.09 * 1000)  // Measured from the outside of one side of the penalty box line to the inside of the line on the other side

// Field feature as it appears on the field
class FieldFeatureLocation {
    public:
    	Eigen::Vector2f position;
        float orientation;

        FieldFeatureLocation(float x, float y, float orientation) : position(x, y), orientation(orientation) {};
};


// +y towards enemy goal
// +x towards left
enum class LineDirection {
    // All of the short lines stretching across the short side of the field.
    // eg. the centre line.
    X,

    // The two long lines stretching across the long side of the field.
    // Shorter Y lines are ommitted for now for some reason.
    Y
};

class FieldLineLocation {
	public:
		LineDirection direction;

		// equal to the line's x coordinate if direction is X,
		// and equal to the line's y coordinate if direction is Y
		float alongAxis;
		FieldLineLocation(LineDirection _direction, float _alongAxis) : direction(_direction), alongAxis(_alongAxis) {};

		// Returns the angle to the line relative to the positive x-axis
		float globalAngle(Sign lineSide) const {
			float heading = 0;
			if (direction == LineDirection::Y) {
				heading -= M_PI_2;
			}

			if (lineSide == Sign::Negative) {
				heading += M_PI;
			}

			/* TODO: Debug comments. Remove before commit
				X SIDE: 
			if (onPositiveSideOfLine)
				myHFromObservation = normaliseTheta(DEG2RAD(180) - obs_heading);
			else
				myHFromObservatoion = -obs_heading;	

				Y SIDE:
			if (onPositiveSideOfLine)
				myHFromObservation = normaliseTheta(-DEG2RAD(90) - obs_heading);
			else
				myHFromObservation = normaliseTheta(DEG2RAD(90) - obs_heading);
			*/

			return heading;
		};
};

// All the field features on the field
class FieldFeatureLocations
{
	public:
		FieldFeatureLocations();

		std::vector<FieldFeatureLocation> corners;
		std::vector<FieldFeatureLocation> tJunctions;
		std::vector<FieldFeatureLocation> centreCircles;
		std::vector<FieldFeatureLocation> penaltySpots;
		std::vector<FieldLineLocation> lines;
};
