#pragma once

static struct LocaliserParams {
    // martin: values sourced from image/home/nao/data/MultiModalCMKFParams.cfg
    //         there seems to be a parameter optimiser that might have initially been used to determine these

    float modeSplitWeightMultiplyFactor = 0.5;
    float lineModeSplitWeightMultiplyFactor = 0.2;
    float odometryForwardMultiplyFactor = 10;
    float odometryLeftMultiplyFactor = 10;
    float odometryHeadingMultiplyFactor = 5;
    float angleUncertainty = 10;
    float updateHeadingUncertainty = 10;
    float similarHeadingThresh = 30;
    float similarXThresh = 500;
    float similarYThresh = 500;
    float minKFWeight = 0.03;

	// In practice, the robot's time integrated motion_status prediction is 0.7
	// times smaller than what is expected.
	float carpetFrictionAdjustmentForward = 0.7f;
    float carpetFrictionAdjustmentSideways = 1.0f;
    float turnFrictionAdjustment = 1.05f;

} localiserParams;