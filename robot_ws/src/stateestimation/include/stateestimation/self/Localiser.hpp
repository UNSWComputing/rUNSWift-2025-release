#pragma once

#include <optional>

#include <runswift_interfaces/msg/detail/global_pose_observation__struct.hpp>
#include <stateestimation/types.hpp>

#include "FieldFeatureLocations.hpp"
#include "SelfKalmanFilter.hpp"
// #include "FieldFeatureObservation.hpp"


/**
 * Localiser for robot pose which keeps track of several hypotheses
 * in the form of Kalman filters.
 *
 * As the localiser receives more data, these hypotheses are updated,
 * split and merged as necessary.
 *
 */

class Localiser {
    protected:
        void normaliseKFWeights();

        // low-weight pruning, merging, weight normalisation, etc.
        void mergeAndPruneKfs();

        std::optional<SelfKalmanFilter*> findHeaviestKf();
    
    public:
        // All field features on the field, hard coded.
        FieldFeatureLocations fieldFeatureLocations;

        // List of hypotheses for where we are right now.
        std::vector<SelfKalmanFilter> kfs;
        std::vector<SelfKalmanFilter> newKfs;


        // PoseEstimate localise(
        //     std::vector<FieldFeatureObservation> &fieldFeatures,
        //     Odometry &odometryDelta
        // );

        void onOdometry(Odometry &odometryDelta);

        // void onFieldFeaturesObserved(std::vector<FieldFeatureObservation> &fieldFeatures);

        void onGlobalPoseObserved(GlobalPoseObservation &observation);

        void transitionToInitial(int playerNum);
        
        void setState(SelfKalmanFilter::StateVector state);

        PoseEstimate estimate();

        Localiser();

    private:
/*

// Inuka:
// TODO: Here is a snippet of the old codebase. handleTransition() should be run
// at the beginning of every iteration to reset the pose, etc. if the game state
// changes. Take a look at the old codebase implementations. We need to pass the
// game state info into this class without using the old EstimatorInfoIn type
// god class. I've made it a nested class since these functions are only ever
// used here.

        class Transitioner {
            public:
                LocaliserTransitioner(// TODO );
                void handleTransition(// TODO );
                virtual ~LocaliserTransitioner(){};

            protected:
                void resetToInitialPose();

            private:

                uint8_t prevCompetitionType;
                uint8_t prevGameState;
                uint8_t prevGamePhase;
                uint8_t prevPenalty;
                bool prevPickedup;

                // TODO: Use new codebase equivalent of Timer, whatever that is
                Timer penalisedTimer;
                Timer refPickupTimer;
                Timer unpenalisedTimer;
                bool pickedUpDuringPenalised;

                virtual void resetToGameInitialPose() = 0;
                virtual void resetToOneVsOneInitialPose() = 0;
                virtual void resetToSpecifiedInitialPose() = 0;
                virtual void resetToUnpenalisedPose() = 0;
                virtual void motionInStandbyPenaltyPoses() = 0;
                virtual void resetToPenaltyshootPhasePoseOffense() = 0;
                virtual void resetToPenaltyshootPhasePoseDefense() = 0;
                virtual void resetToPenaltyshootPhasePlayerSelectedPose() = 0;
        };
*/
};
