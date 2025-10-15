#pragma once

#include "stateestimation/mathutils.hpp"

static struct BallEstimatorParams {
  // martin: values sourced from image/home/nao/data/BallCMKFParams.cfg
  float stdAccelerationX = 4000;                        // 1000 - 200000
  float stdAccelerationY = 4000;                        // 1000 - 200000

  float stdObservationDistBaseWalking = 100;            // 5 - 100
  float stdObservationDistIncreaseRateWalking = 0.5;    // 0.1 - 1.0
  float stdObservationHeadWalking = 30;                 // 5 - 50
  float stdObservationDistBaseStanding = 40;            // 5 - 100
  float stdObservationDistIncreaseRateStanding = 0.1;   // 0.1 - 1.0
  float stdObservationHeadStanding = 8;                 // 5 - 50
                                                        // Make sure this value is the same as in behaviours
  float ballAcceleration = -390.0f;                     // mm/s^2
  float offFieldBallMargin = 400;                       // mm
  float collisionRobotRadius = 100;                     // mm
  float collisionBounciness = 0.4;						// no units (coefficient of restitution)
  float collisionAcceptableHeadingDiff = deg2radf(20);  // radians
	
  /*
   * At 10Hz, we decay each kf by multiplying it with this rate.
   * 
   * Maximum confidence ball observations that are close will (without extra observations) become invalid after 10 seconds.
   * 100*(rate)^100 = 1
   *
   * Far away ones will decay in 20 seconds
   * 200*(rate)^200 = 1
   */

  float closeDecayRate = 0.9550f; // Previously 0.5f (but this was a constant decrease)
  float farDecayRate = 0.9738f; // Previously 0.1f (but this was a constant decrease)
  float weightGrowth = 1.5f;
  float weightInitial = 20.0f;

  float mergeThresh = 500.0f;
} ballEstimatorParams;

