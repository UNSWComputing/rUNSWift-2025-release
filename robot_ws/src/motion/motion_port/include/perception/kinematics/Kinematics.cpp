#include "Kinematics.hpp"

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/ublas/lu.hpp>
#include <cmath>
#include <climits>
#include <rclcpp/logging.hpp>
#include <vector>

#include <perception/vision/VisionDefinitions.hpp>
#include <utils/Timer.hpp>
#include <utils/body.hpp>

#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include <tf2/LinearMath/Transform.h>
#include "tf2/LinearMath/Vector3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include <rclcpp/rclcpp.hpp>

// used with filterZeros function
static const float VERY_SMALL = 0.0001;

// These are the offsets of each of the parts in the kinematics chain, see
// http://runswift.cse.unsw.edu.au/confluence/download/attachments/3047440/100215-NaoForwardKinematicsLegToCamera.pptx.pdf?version=2&modificationDate=1266227985534
// for a visual representation of how the Foot->Camera DH chain was calculated
// for the v3s.
// For a more up to date version (v4 H21 robots) with the DH chain for the limbs
// and centre of masses, see section 3.1 of
// http://cgi.cse.unsw.edu.au/~robocup/2012site/reports/Belinda_Teh_Thesis.pdf
// Latest data for these constants can be obtained from aldebaran documentation
// http://www.aldebaran-robotics.com/documentation/family/nao_h25/links_h25.html
// and should be updated in the utils/body.hpp file.
static const float trunk_length = Limbs::HipOffsetZ + Limbs::NeckOffsetZ;

// http://doc.aldebaran.com/2-8/_downloads/NAOH25V60.urdf
// xyz="0.01774 0.05071 0"
static const float camera_out_bottom = 50.71;
static const float camera_up_bottom = 17.74;

// xyz="0.06364 0.05871 0"
static const float camera_out_top = 58.71;
static const float camera_up_top = 63.64;

//rpy="1.5708 5.55112e-17 2.26369"
// =(2.26369-pi/2) * 180/pi
static const float camera_bottom_angle = DEG2RAD(39.7);
// rpy="1.5708 -0 1.59174"
// =(1.59174-pi/2) * 180/pi
static const float camera_top_angle = DEG2RAD(1.2);

static const float d2 = trunk_length - Limbs::HipOffsetY;
static const float d3 = Limbs::HipOffsetY * sqrt(2);

static const float d1Top = sqrt(pow(camera_out_top, 2) + pow(camera_up_top, 2));
static const float a1Top = atan(camera_up_top / camera_out_top) + camera_top_angle;
static const float l10Top = d1Top * sin(a1Top);
static const float d11Top = d1Top * cos(a1Top);
static const float a3Top = camera_top_angle - M_PI;

static const float d1Bot = sqrt(pow(camera_out_bottom, 2) + pow(camera_up_bottom, 2));
static const float a1Bot = atan(camera_up_bottom / camera_out_bottom) + camera_bottom_angle;
static const float l10Bot = d1Bot * sin(a1Bot);
static const float d11Bot = d1Bot * cos(a1Bot);
static const float a3Bot = camera_bottom_angle - M_PI;

Kinematics::Kinematics(const rclcpp::Node::SharedPtr & node)
: motion_node_(node)
{
  std::vector<boost::numeric::ublas::matrix<float>> chest;

  chest.push_back(
    vec4<float>(
      -44, -39.5,
      Limbs::NeckOffsetZ - 32 + Limbs::HipOffsetZ - Limbs::HipOffsetY, 1));
  chest.push_back(
    vec4<float>(
      -34, -54,
      Limbs::NeckOffsetZ - 32 + Limbs::HipOffsetZ - Limbs::HipOffsetY, 1));
  chest.push_back(
    vec4<float>(
      -38, -70,
      Limbs::NeckOffsetZ - 25 + Limbs::HipOffsetZ - Limbs::HipOffsetY, 1));
  chest.push_back(
    vec4<float>(
      -33, -90, Limbs::NeckOffsetZ + Limbs::HipOffsetZ - Limbs::HipOffsetY,
      1));
  chest.push_back(
    vec4<float>(
      -22, -95,
      Limbs::NeckOffsetZ + 9 + Limbs::HipOffsetZ - Limbs::HipOffsetY, 1));
  chest.push_back(
    vec4<float>(
      -8, -90,
      Limbs::NeckOffsetZ + 15.5 + Limbs::HipOffsetZ - Limbs::HipOffsetY, 1));
  chest.push_back(
    vec4<float>(
      10, -90,
      Limbs::NeckOffsetZ + 17.5 + Limbs::HipOffsetZ - Limbs::HipOffsetY, 1));
  chest.push_back(
    vec4<float>(
      25, -90,
      Limbs::NeckOffsetZ + 8.5 + Limbs::HipOffsetZ - Limbs::HipOffsetY, 1));
  chest.push_back(
    vec4<float>(
      34, -81,
      Limbs::NeckOffsetZ - 2.2 + Limbs::HipOffsetZ - Limbs::HipOffsetY, 1));
  chest.push_back(
    vec4<float>(
      43, -83,
      Limbs::NeckOffsetZ - 21 + Limbs::HipOffsetZ - Limbs::HipOffsetY, 1));
  chest.push_back(
    vec4<float>(
      31, -61,
      Limbs::NeckOffsetZ - 35 + Limbs::HipOffsetZ - Limbs::HipOffsetY, 1));
  chest.push_back(
    vec4<float>(
      50, -39,
      Limbs::NeckOffsetZ - 48 + Limbs::HipOffsetZ - Limbs::HipOffsetY, 1));
  chest.push_back(
    vec4<float>(
      55, -20,
      Limbs::NeckOffsetZ - 48 + Limbs::HipOffsetZ - Limbs::HipOffsetY, 1));
  chest.push_back(
    vec4<float>(
      55, 0,
      Limbs::NeckOffsetZ - 45 + Limbs::HipOffsetZ - Limbs::HipOffsetY, 1));
  chest.push_back(
    vec4<float>(
      55, 20,
      Limbs::NeckOffsetZ - 48 + Limbs::HipOffsetZ - Limbs::HipOffsetY, 1));
  chest.push_back(
    vec4<float>(
      50, 39,
      Limbs::NeckOffsetZ - 48 + Limbs::HipOffsetZ - Limbs::HipOffsetY, 1));
  chest.push_back(
    vec4<float>(
      31, 61,
      Limbs::NeckOffsetZ - 35 + Limbs::HipOffsetZ - Limbs::HipOffsetY, 1));
  chest.push_back(
    vec4<float>(
      43, 83,
      Limbs::NeckOffsetZ - 21 + Limbs::HipOffsetZ - Limbs::HipOffsetY, 1));
  chest.push_back(
    vec4<float>(
      34, 81,
      Limbs::NeckOffsetZ - 2.2 + Limbs::HipOffsetZ - Limbs::HipOffsetY, 1));
  chest.push_back(
    vec4<float>(
      25, 90,
      Limbs::NeckOffsetZ + 8.5 + Limbs::HipOffsetZ - Limbs::HipOffsetY, 1));
  chest.push_back(
    vec4<float>(
      10, 90,
      Limbs::NeckOffsetZ + 17.5 + Limbs::HipOffsetZ - Limbs::HipOffsetY, 1));
  chest.push_back(
    vec4<float>(
      -8, 90,
      Limbs::NeckOffsetZ + 15.5 + Limbs::HipOffsetZ - Limbs::HipOffsetY, 1));
  chest.push_back(
    vec4<float>(
      -22, 95,
      Limbs::NeckOffsetZ + 9 + Limbs::HipOffsetZ - Limbs::HipOffsetY, 1));
  chest.push_back(
    vec4<float>(
      -33, 90, Limbs::NeckOffsetZ + Limbs::HipOffsetZ - Limbs::HipOffsetY,
      1));
  chest.push_back(
    vec4<float>(
      -38, 70,
      Limbs::NeckOffsetZ - 25 + Limbs::HipOffsetZ - Limbs::HipOffsetY, 1));
  chest.push_back(
    vec4<float>(
      -34, 54,
      Limbs::NeckOffsetZ - 32 + Limbs::HipOffsetZ - Limbs::HipOffsetY, 1));
  chest.push_back(
    vec4<float>(
      -44, 39.5,
      Limbs::NeckOffsetZ - 32 + Limbs::HipOffsetZ - Limbs::HipOffsetY, 1));
  bodyParts.push_back(chest);
  chest.clear();

  // Setting up the masses vector to store the mass of each joint in kgs.
  // These are particularly ordered in the order that they are multiplied up
  // through the kinematics chain for use in Centre of Mass calculations.
  masses.clear();
  masses.push_back(Limbs::TorsoMass);  // 0 - Torso
  masses.push_back(Limbs::NeckMass);  // 1 - Neck (HeadYaw)
  masses.push_back(Limbs::HeadMass);  // 2 - Head (HeadPitch)

  masses.push_back(Limbs::RightShoulderMass);  // 3 - Right Shoulder (RShoulderPitch)
  masses.push_back(Limbs::RightBicepMass);  // 4 - Right Bicep (RShoulderRoll)
  masses.push_back(Limbs::RightElbowMass);  // 5 - Right Elbow (RElbowYaw)
  masses.push_back(Limbs::RightForearmMass);  // 6 - Right Motorized Forearms (RElbowRoll)
  masses.push_back(Limbs::RightHandMass);  // 7 - Right Motorized Hand (RWristYaw)

  masses.push_back(Limbs::LeftShoulderMass);  // 8 - Left Shoulder (LShoulderPitch)
  masses.push_back(Limbs::LeftBicepMass);  // 9 - Left Bicep (LShoulderRoll)
  masses.push_back(Limbs::LeftElbowMass);  // 10 - Left Elbow (LElbowYaw)
  masses.push_back(Limbs::LeftForearmMass);  // 11 - Left Motorized Forearm (LElboRoll)
  masses.push_back(Limbs::LeftHandMass);  // 12 - Left Motorized Hand (LWristYaw)

  masses.push_back(Limbs::RightPelvisMass);  // 13 - Right Pelvis (RHipYawPitch)
  masses.push_back(Limbs::RightHipMass);  // 14 - Right Hip (RHipRoll)
  masses.push_back(Limbs::RightThighMass);  // 15 - Right Thigh (RHipPitch)
  masses.push_back(Limbs::RightTibiaMass);  // 16 - Right Tibia (RKneePitch)
  masses.push_back(Limbs::RightAnkleMass);  // 17 - Right Ankle (RAnklePitch)
  masses.push_back(Limbs::RightFootMass);  // 18 - Right Foot (RAnkleRoll)

  masses.push_back(Limbs::LeftPelvisMass);  // 19 - Left Pelvis (LHipYawPitch)
  masses.push_back(Limbs::LeftHipMass);  // 20 - Left Hip (LHipRoll)
  masses.push_back(Limbs::LeftThighMass);  // 21 - Left Thigh (LHipPitch)
  masses.push_back(Limbs::LeftTibiaMass);  // 22 - Left Tibia (LKneePitch)
  masses.push_back(Limbs::LeftAnkleMass);  // 23 - Left Ankle (LAnklePitch)
  masses.push_back(Limbs::LeftFootMass);  // 24 - Left Foot (LAnkleRoll)


  // Same order as above, but this time for the position of the centre of mass for each joint.
  massesCom.clear();
  massesCom.push_back(vec4(Limbs::TorsoCoM));  // Torso
  massesCom.push_back(vec4(Limbs::NeckCoM));  // Head
  massesCom.push_back(vec4(Limbs::HeadCoM));
  massesCom.push_back(vec4(Limbs::RightShoulderCoM));  // R Arm
  massesCom.push_back(vec4(Limbs::RightBicepCoM));
  massesCom.push_back(vec4(Limbs::RightElbowCoM));
  massesCom.push_back(vec4(Limbs::RightForearmCoM));
  massesCom.push_back(vec4(Limbs::RightHandCoM));
  massesCom.push_back(vec4(Limbs::LeftShoulderCoM));  // L Arm
  massesCom.push_back(vec4(Limbs::LeftBicepCoM));
  massesCom.push_back(vec4(Limbs::LeftElbowCoM));
  massesCom.push_back(vec4(Limbs::LeftForearmCoM));
  massesCom.push_back(vec4(Limbs::LeftHandCoM));
  massesCom.push_back(vec4(Limbs::RightPelvisCoM));  // R Leg
  massesCom.push_back(vec4(Limbs::RightHipCoM));
  massesCom.push_back(vec4(Limbs::RightThighCoM));
  massesCom.push_back(vec4(Limbs::RightTibiaCoM));
  massesCom.push_back(vec4(Limbs::RightAnkleCoM));
  massesCom.push_back(vec4(Limbs::RightFootCoM));
  massesCom.push_back(vec4(Limbs::LeftPelvisCoM));  // L Leg
  massesCom.push_back(vec4(Limbs::LeftHipCoM));
  massesCom.push_back(vec4(Limbs::LeftThighCoM));
  massesCom.push_back(vec4(Limbs::LeftTibiaCoM));
  massesCom.push_back(vec4(Limbs::LeftAnkleCoM));
  massesCom.push_back(vec4(Limbs::LeftFootCoM));

  // Initialise transform matrices for DH chain
  for (int i = 0; i < CAMERA_DH_CHAIN_LEN; ++i) {
    transformLTop[i] = boost::numeric::ublas::identity_matrix<float>(4);
    transformRTop[i] = boost::numeric::ublas::identity_matrix<float>(4);
    transformLBot[i] = boost::numeric::ublas::identity_matrix<float>(4);
    transformRBot[i] = boost::numeric::ublas::identity_matrix<float>(4);
  }
  for (int i = 0; i < HEAD_DH_CHAIN_LEN; ++i) {
    transformHB[i] = boost::numeric::ublas::identity_matrix<float>(4);
  }
  for (int i = 0; i < ARM_DH_CHAIN_LEN; ++i) {
    transformLAB[i] = boost::numeric::ublas::identity_matrix<float>(4);
    transformRAB[i] = boost::numeric::ublas::identity_matrix<float>(4);
  }
  for (int i = 0; i < LEG_DH_CHAIN_LEN; ++i) {
    transformLFB[i] = boost::numeric::ublas::identity_matrix<float>(4);
    transformRFB[i] = boost::numeric::ublas::identity_matrix<float>(4);
  }

  // Set up constant DH transforms since they only need to be calculated once
  boost::numeric::ublas::matrix<float> pi2AboutX = createDHMatrix<float>(0, M_PI / 2, 0, 0);
  boost::numeric::ublas::matrix<float> negpi2AboutX = createDHMatrix<float>(0, -M_PI / 2, 0, 0);
  boost::numeric::ublas::matrix<float> pi4AboutX = createDHMatrix<float>(0, M_PI / 4, 0, 0);
  boost::numeric::ublas::matrix<float> negpi2AboutXnegpi2AboutZ = createDHMatrix<float>(
    0,
    -M_PI / 2,
    0,
    -M_PI / 2);
  boost::numeric::ublas::matrix<float> pi2AboutXpi2AboutZ = createDHMatrix<float>(
    0, M_PI / 2, 0,
    M_PI / 2);

  // Constants in camera transforms
  transformLTop[0] = createDHMatrix<float>(0, 0, Limbs::FootHeight, M_PI / 2);
  transformLTop[7] = createDHMatrix<float>(0, 3 * M_PI / 4, 0, 0);

  transformRTop[0] = transformLTop[0];
  transformRTop[7] = pi4AboutX;

  transformLBot[0] = transformLTop[0];
  transformLBot[7] = transformLTop[7];

  transformRBot[0] = transformRTop[0];
  transformRBot[7] = transformRTop[7];

  // Constants in mass transforms
  transformHB[2] = pi2AboutX;

  transformRAB[0] = createDHMatrix<float>(0, 0, Limbs::ShoulderOffsetZ, 0);
  transformRAB[1] = createDHMatrix<float>(0, M_PI / 2, Limbs::ShoulderOffsetY, 0);
  transformRAB[3] = pi2AboutX;
  transformRAB[5] = createDHMatrix<float>(
    Limbs::UpperArmLength, M_PI / 2,
    Limbs::ElbowOffsetY, M_PI / 2);
  transformRAB[7] = negpi2AboutXnegpi2AboutZ;
  transformRAB[8] = negpi2AboutX;
  transformRAB[10] = createDHMatrix<float>(Limbs::LowerArmLength, M_PI / 2, 0, M_PI / 2);
  transformRAB[12] = negpi2AboutXnegpi2AboutZ;
  transformRAB[13] = negpi2AboutX;

  transformLAB[0] = transformRAB[0];
  transformLAB[1] = createDHMatrix<float>(0, M_PI / 2, -Limbs::ShoulderOffsetY, 0);
  transformLAB[3] = transformRAB[3];
  transformLAB[5] = createDHMatrix<float>(
    Limbs::UpperArmLength, M_PI / 2,
    -Limbs::ElbowOffsetY, M_PI / 2);
  transformLAB[7] = transformRAB[7];
  transformLAB[8] = transformRAB[8];
  transformLAB[10] = transformRAB[10];
  transformLAB[12] = transformRAB[12];
  transformLAB[13] = transformRAB[13];

  transformRFB[0] = createDHMatrix<float>(0, 0, -Limbs::HipOffsetZ, 0);
  transformRFB[1] = createDHMatrix<float>(0, M_PI / 2, Limbs::HipOffsetY, 0);
  transformRFB[3] = pi4AboutX;
  transformRFB[4] = createDHMatrix<float>(0, M_PI / 2, 0, -M_PI / 2);
  transformRFB[6] = pi2AboutXpi2AboutZ;
  transformRFB[7] = negpi2AboutX;
  transformRFB[9] = pi2AboutX;
  transformRFB[10] = createDHMatrix<float>(0, 0, -Limbs::ThighLength, 0);
  transformRFB[12] = pi2AboutX;
  transformRFB[13] = createDHMatrix<float>(0, 0, -Limbs::TibiaLength, 0);
  transformRFB[15] = pi2AboutX;
  transformRFB[16] = createDHMatrix<float>(0, 0, 0, -M_PI / 2);
  transformRFB[18] = pi2AboutXpi2AboutZ;

  transformLFB[0] = transformRFB[0];
  transformLFB[1] = createDHMatrix<float>(0, M_PI / 2, -Limbs::HipOffsetY, 0);
  transformLFB[3] = createDHMatrix<float>(0, -M_PI / 4, 0, 0);
  transformLFB[4] = transformRFB[4];
  transformLFB[6] = transformRFB[6];
  transformLFB[7] = transformRFB[7];
  transformLFB[9] = transformRFB[9];
  transformLFB[10] = transformRFB[10];
  transformLFB[12] = transformRFB[12];
  transformLFB[13] = transformRFB[13];
  transformLFB[15] = transformRFB[15];
  transformLFB[16] = transformRFB[16];
  transformLFB[18] = transformRFB[18];

}

Kinematics::Chain
Kinematics::determineSupportChain()
{
  float lsum = sensorValues.sensors[Sensors::LFoot_FSR_FrontLeft] +
    sensorValues.sensors[Sensors::LFoot_FSR_FrontRight] +
    sensorValues.sensors[Sensors::LFoot_FSR_RearLeft] +
    sensorValues.sensors[Sensors::LFoot_FSR_RearRight];
  float rsum = sensorValues.sensors[Sensors::RFoot_FSR_FrontLeft] +
    sensorValues.sensors[Sensors::RFoot_FSR_FrontRight] +
    sensorValues.sensors[Sensors::RFoot_FSR_RearLeft] +
    sensorValues.sensors[Sensors::RFoot_FSR_RearRight];
  if (lsum > rsum) {return LEFT_CHAIN;}
  return RIGHT_CHAIN;
}

void Kinematics::updateDHChain()
{
  // These are the camera offsets from kinematics calibrations
  float coffsetY, coffsetX, coffsetZ;

  JointValues jointValues = sensorValues.joints;

  float Cp = jointValues.angles[Joints::HeadPitch];
  float Cy = jointValues.angles[Joints::HeadYaw];

  // Calculate the top left camera transform
  float coffsetYL = DEG2RAD(parameters.cameraPitchTopWhenLookingLeft);
  float coffsetXL = DEG2RAD(parameters.cameraYawTopWhenLookingLeft);
  float coffsetZL = DEG2RAD(parameters.cameraRollTopWhenLookingLeft);

  float coffsetYM = DEG2RAD(parameters.cameraPitchTopWhenLookingStraight);
  float coffsetXM = DEG2RAD(parameters.cameraYawTopWhenLookingStraight);
  float coffsetZM = DEG2RAD(parameters.cameraRollTopWhenLookingStraight);

  float coffsetYR = DEG2RAD(parameters.cameraPitchTopWhenLookingRight);
  float coffsetXR = DEG2RAD(parameters.cameraYawTopWhenLookingRight);
  float coffsetZR = DEG2RAD(parameters.cameraRollTopWhenLookingRight);

  // RCLCPP_WARN(motion_node_->get_logger(), "coffsetYL: %.2f\tcoffsetXL: %.2f\tcoffsetZL: %.2f", coffsetYL, coffsetXL, coffsetZL);
  // RCLCPP_WARN(motion_node_->get_logger(), "coffsetYM: %.2f\tcoffsetXM: %.2f\tcoffsetZM: %.2f", coffsetYM, coffsetXM, coffsetZM);
  // RCLCPP_WARN(motion_node_->get_logger(), "coffsetYR: %.2f\tcoffsetXR: %.2f\tcoffsetZR: %.2f", coffsetYR, coffsetXR, coffsetZR);

  // Interpolate camera offsets between Left (L), Middle (M), and Right (R) based on Cy (HeadYaw)
  float Cy_deg = RAD2DEG(Cy);
  
  // RCLCPP_WARN(motion_node_->get_logger(), "HeadYaw (Cy_deg): %.2f", Cy_deg);

  // robot looking right: Cy_deg = -70
  // robot looking left:  Cy_deg = 70
  const float YAW_LIMIT = 70.0;

  if (Cy_deg >= YAW_LIMIT) {
    // Left
    coffsetY = coffsetYL;
    coffsetX = coffsetXL;
    coffsetZ = coffsetZL;
  } else if (Cy_deg <= -YAW_LIMIT) {
    // Right
    coffsetY = coffsetYR;
    coffsetX = coffsetXR;
    coffsetZ = coffsetZR;
  } else if (Cy_deg > 0.0f) {
    // Interpolate between L and M
    float t = Cy_deg / YAW_LIMIT;
    // RCLCPP_WARN(motion_node_->get_logger(), "Interpolating between M (%.2f) L", t);
    coffsetY = coffsetYM * (1 - t) + coffsetYL * t;
    coffsetX = coffsetXM * (1 - t) + coffsetXL * t;
    coffsetZ = coffsetZM * (1 - t) + coffsetZL * t;
  } else {
    // Interpolate between M and R
    float t = -Cy_deg / YAW_LIMIT;
    // RCLCPP_WARN(motion_node_->get_logger(), "Interpolating between M (%.2f) R", t);
    coffsetY = coffsetYM * (1 - t) + coffsetYR * t;
    coffsetX = coffsetXM * (1 - t) + coffsetXR * t;
    coffsetZ = coffsetZM * (1 - t) + coffsetZR * t;
  }

  float Hyp = jointValues.angles[Joints::LHipYawPitch];
  float HpL = jointValues.angles[Joints::LHipPitch];
  float HrL = jointValues.angles[Joints::LHipRoll];
  float KpL = jointValues.angles[Joints::LKneePitch];
  float ApL = jointValues.angles[Joints::LAnklePitch];
  float ArL = jointValues.angles[Joints::LAnkleRoll];

  cameraPanInverseHack = createDHMatrix<float>(0, 0, d2, 0.0);
  // DH parameters
  // Some of these are commented out since they're constant and can be calculated
  // once at the start
  //transformLTop[0] = createDHMatrix<float>(0, 0, Limbs::FootHeight, M_PI / 2);
  transformLTop[1] = createDHMatrix<float>(0, M_PI / 2, 0, M_PI / 2 - ArL);
  transformLTop[2] = createDHMatrix<float>(0, M_PI / 2, 0, -ApL);
  transformLTop[3] = createDHMatrix<float>(Limbs::TibiaLength, 0, 0, -KpL);
  transformLTop[4] = createDHMatrix<float>(Limbs::ThighLength, 0, 0, -HpL);
  transformLTop[5] = createDHMatrix<float>(0, -M_PI / 2, 0, -M_PI / 4 - HrL);
  transformLTop[6] = createDHMatrix<float>(0, M_PI / 2, -d3, M_PI / 2 - Hyp);
  // transformLTop[7] = createDHMatrix<float>(0, 3 * M_PI / 4, 0, 0);
  transformLTop[8] = createDHMatrix<float>(0, 0, d2, Cy);
  transformLTop[9] = createDHMatrix<float>(0, -M_PI / 2, 0, a3Top + Cp);
  transformLTop[10] = createDHMatrix<float>(0, -M_PI / 2, l10Top, M_PI / 2 + coffsetX);
  transformLTop[11] = createDHMatrix<float>(0, -M_PI / 2 + coffsetY, d11Top, coffsetZ);

  // Calculate the top right camera transform
  float HpR = jointValues.angles[Joints::RHipPitch];
  float HrR = jointValues.angles[Joints::RHipRoll];
  float KpR = jointValues.angles[Joints::RKneePitch];
  float ApR = jointValues.angles[Joints::RAnklePitch];
  float ArR = jointValues.angles[Joints::RAnkleRoll];

  // DH parameters
  // Some of these are commented out since they're constant and can be calculated once
  // Just leaving them here so they can be seen in order
  //transformRTop[0] = transformLTop[0];
  transformRTop[1] = createDHMatrix<float>(0, M_PI / 2, 0, M_PI / 2 - ArR);
  transformRTop[2] = createDHMatrix<float>(0, M_PI / 2, 0, -ApR);
  transformRTop[3] = createDHMatrix<float>(Limbs::TibiaLength, 0, 0, -KpR);
  transformRTop[4] = createDHMatrix<float>(Limbs::ThighLength, 0, 0, -HpR);
  transformRTop[5] = createDHMatrix<float>(0, -M_PI / 2, 0, M_PI / 4 - HrR);
  transformRTop[6] = createDHMatrix<float>(0, M_PI / 2, d3, M_PI / 2 - Hyp);
  //transformRTop[7] = createDHMatrix<float>(0, M_PI / 4, 0, 0);
  transformRTop[8] = transformLTop[8];
  transformRTop[9] = transformLTop[9];
  transformRTop[10] = transformLTop[10];
  transformRTop[11] = transformLTop[11];

  // Calculate the bottom left camera transform
  coffsetY = DEG2RAD(parameters.cameraPitchBottom);
  coffsetX = DEG2RAD(parameters.cameraYawBottom);
  coffsetZ = DEG2RAD(parameters.cameraRollBottom);

  //transformLBot[0] = transformLTop[0];
  transformLBot[1] = transformLTop[1];
  transformLBot[2] = transformLTop[2];
  transformLBot[3] = transformLTop[3];
  transformLBot[4] = transformLTop[4];
  transformLBot[5] = transformLTop[5];
  transformLBot[6] = transformLTop[6];
  //transformLBot[7] = transformLTop[7];
  transformLBot[8] = transformLTop[8];
  transformLBot[9] = createDHMatrix<float>(0, -M_PI / 2, 0, a3Bot + Cp);
  transformLBot[10] = createDHMatrix<float>(0, -M_PI / 2, l10Bot, M_PI / 2 + coffsetX);
  transformLBot[11] = createDHMatrix<float>(0, -M_PI / 2 + coffsetY, d11Bot, coffsetZ);

  // Calculate the bottom right transform
  //transformRBot[0] = transformRTop[0];
  transformRBot[1] = transformRTop[1];
  transformRBot[2] = transformRTop[2];
  transformRBot[3] = transformRTop[3];
  transformRBot[4] = transformRTop[4];
  transformRBot[5] = transformRTop[5];
  transformRBot[6] = transformRTop[6];
  //transformRBot[7] = transformRTop[7];
  transformRBot[8] = transformRTop[8];
  transformRBot[9] = transformLBot[9];
  transformRBot[10] = transformLBot[10];
  transformRBot[11] = transformLBot[11];

  // Transform parameters for centre of mass
  // Head to Body
  transformHB[0] = createDHMatrix<float>(0, 0, Limbs::NeckOffsetZ, Cy);
  transformHB[1] = createDHMatrix<float>(0, -M_PI / 2, 0, Cp);
  //transformHB[2] = createDHMatrix<float>(0, M_PI / 2, 0, 0);

  // Right Arm to Body
  // Some of these are commented out since they're constant and can be calculated once
  // Just leaving them here so they can be seen in order
  float Sp = jointValues.angles[Joints::RShoulderPitch];
  float Sr = jointValues.angles[Joints::RShoulderRoll];
  float Ey = jointValues.angles[Joints::RElbowYaw];
  float Er = jointValues.angles[Joints::RElbowRoll];
  float Wy = jointValues.angles[Joints::RWristYaw];
  //transformRAB[0] = createDHMatrix<float>(0, 0, Limbs::ShoulderOffsetZ, 0);
  //transformRAB[1] = createDHMatrix<float>(0, M_PI / 2, Limbs::ShoulderOffsetY, 0);
  transformRAB[2] = createDHMatrix<float>(0, -M_PI, 0, Sp);
  //transformRAB[3] = createDHMatrix<float>(0, M_PI / 2, 0, 0);
  transformRAB[4] = createDHMatrix<float>(0, 0, 0, Sr);
  //transformRAB[5] = createDHMatrix<float>(Limbs::UpperArmLength, M_PI / 2,
  //                                        Limbs::ElbowOffsetY, M_PI / 2);
  transformRAB[6] = createDHMatrix<float>(0, M_PI / 2, 0, Ey);
  //transformRAB[7] = createDHMatrix<float>(0, -M_PI / 2, 0, -M_PI / 2);
  //transformRAB[8] = createDHMatrix<float>(0, -M_PI / 2, 0, 0);
  transformRAB[9] = createDHMatrix<float>(0, 0, 0, Er);
  //transformRAB[10] = createDHMatrix<float>(Limbs::LowerArmLength, M_PI / 2, 0, M_PI / 2);
  transformRAB[11] = createDHMatrix<float>(0, M_PI / 2, 0, Wy);
  //transformRAB[12] = createDHMatrix<float>(0, -M_PI / 2, 0, -M_PI / 2);
  //transformRAB[13] = createDHMatrix<float>(0, -M_PI / 2, 0, 0);

  // Left Arm to Body
  Sp = jointValues.angles[Joints::LShoulderPitch];
  Sr = jointValues.angles[Joints::LShoulderRoll];
  Ey = jointValues.angles[Joints::LElbowYaw];
  Er = jointValues.angles[Joints::LElbowRoll];
  Wy = jointValues.angles[Joints::LWristYaw];
  //transformLAB[0] = transformRAB[0];
  //transformLAB[1] = createDHMatrix<float>(0, M_PI / 2, -Limbs::ShoulderOffsetY, 0);
  transformLAB[2] = createDHMatrix<float>(0, -M_PI, 0, Sp);
  //transformLAB[3] = transformRAB[3];
  transformLAB[4] = createDHMatrix<float>(0, 0, 0, Sr);
  //transformRAB[5] = createDHMatrix<float>(Limbs::UpperArmLength, M_PI / 2,
  //                                       -Limbs::ElbowOffsetY, M_PI / 2);
  transformLAB[6] = createDHMatrix<float>(0, M_PI / 2, 0, Ey);
  //transformLAB[7] = transformRAB[7];
  //transformLAB[8] = transformRAB[8];
  transformLAB[9] = createDHMatrix<float>(0, 0, 0, Er);
  //transformLAB[10] = transformRAB[10];
  transformLAB[11] = createDHMatrix<float>(0, M_PI / 2, 0, Wy);
  //transformLAB[12] = transformRAB[12];
  //transformLAB[13] = transformRAB[13];

  // Right Foot to Body
  HpR = jointValues.angles[Joints::RHipPitch];
  HrR = jointValues.angles[Joints::RHipRoll];
  //transformRFB[0] = createDHMatrix<float>(0, 0, -Limbs::HipOffsetZ, 0);
  //transformRFB[1] = createDHMatrix<float>(0, M_PI / 2, Limbs::HipOffsetY, 0);
  transformRFB[2] = createDHMatrix<float>(0, -3 * M_PI / 4, 0, Hyp);
  //transformRFB[3] = createDHMatrix<float>(0, M_PI / 4, 0, 0);
  //transformRFB[4] = createDHMatrix<float>(0, M_PI / 2, 0, -M_PI / 2);
  transformRFB[5] = createDHMatrix<float>(0, -M_PI / 2, 0, HrR);
  //transformRFB[6] = createDHMatrix<float>(0, M_PI / 2, 0, M_PI / 2);
  //transformRFB[7] = createDHMatrix<float>(0, -M_PI / 2, 0, 0);
  transformRFB[8] = createDHMatrix<float>(0, -M_PI / 2, 0, HpR);
  //transformRFB[9] = createDHMatrix<float>(0, M_PI / 2, 0, 0);
  //transformRFB[10] = createDHMatrix<float>(0, 0, -Limbs::ThighLength, 0);
  transformRFB[11] = createDHMatrix<float>(0, -M_PI / 2, 0, KpR);
  //transformRFB[12] = createDHMatrix<float>(0, M_PI / 2, 0, 0);
  //transformRFB[13] = createDHMatrix<float>(0, 0, -Limbs::TibiaLength, 0);
  transformRFB[14] = createDHMatrix<float>(0, -M_PI / 2, 0, ApR);
  //transformRFB[15] = createDHMatrix<float>(0, M_PI / 2, 0, 0);
  //transformRFB[16] = createDHMatrix<float>(0, 0, 0, -M_PI / 2);
  transformRFB[17] = createDHMatrix<float>(0, -M_PI / 2, 0, ArR);
  //transformRFB[18] = createDHMatrix<float>(0, M_PI / 2, 0, M_PI / 2);

  // Left Foot to Body
  HpL = jointValues.angles[Joints::LHipPitch];
  HrL = jointValues.angles[Joints::LHipRoll];
  //transformLFB[0] = transformRFB[0];
  //transformLFB[1] = createDHMatrix<float>(0, M_PI / 2, -Limbs::HipOffsetY, 0);
  transformLFB[2] = createDHMatrix<float>(0, -M_PI / 4, 0, -Hyp);
  //transformLFB[3] = createDHMatrix<float>(0, -M_PI / 4, 0, 0);
  //transformLFB[4] = transformRFB[4];
  transformLFB[5] = createDHMatrix<float>(0, -M_PI / 2, 0, HrL);
  //transformLFB[6] = transformRFB[6];
  //transformLFB[7] = transformRFB[7];
  transformLFB[8] = createDHMatrix<float>(0, -M_PI / 2, 0, HpL);
  //transformLFB[9] = transformRFB[9];
  //transformLFB[10] = transformRFB[10];
  transformLFB[11] = createDHMatrix<float>(0, -M_PI / 2, 0, KpL);
  //transformLFB[12] = transformRFB[12];
  //transformLFB[13] = transformRFB[13];
  transformLFB[14] = createDHMatrix<float>(0, -M_PI / 2, 0, ApL);
  //transformLFB[15] = transformRFB[15];
  //transformLFB[16] = transformRFB[16];
  transformLFB[17] = createDHMatrix<float>(0, -M_PI / 2, 0, ArL);
  //transformLFB[18] = transformRFB[18];
}

Pose Kinematics::getPose()
{
  Chain foot = determineSupportChain();

  boost::numeric::ublas::matrix<float> c2wTop = createCameraToWorldTransform(foot, true);
  boost::numeric::ublas::matrix<float> c2wBot = createCameraToWorldTransform(foot, false);
  boost::numeric::ublas::matrix<float> n2w = createNeckToWorldTransform(foot);
  Pose pose(c2wTop, c2wBot, n2w);

  boost::numeric::ublas::matrix<float> b2cTop =
    evaluateDHChain(BODY, CAMERA, foot, true);
  determineBodyExclusionArray(b2cTop, pose.getTopExclusionArray(), true);

  boost::numeric::ublas::matrix<float> b2cBot =
    evaluateDHChain(BODY, CAMERA, foot, false);
  determineBodyExclusionArray(b2cBot, pose.getBotExclusionArray(), false);

  return pose;
}

boost::numeric::ublas::matrix<float>
Kinematics::createCameraToWorldTransform(Chain foot, bool top)
{
  boost::numeric::ublas::matrix<float> c2f = createCameraToFootTransform(foot, top);
  boost::numeric::ublas::matrix<float> f2w = createFootToWorldTransform(foot, top);
  return prod(f2w, c2f);
}

boost::numeric::ublas::matrix<float>
Kinematics::createNeckToWorldTransform(Chain foot)
{
  boost::numeric::ublas::matrix<float> n2f = createNeckToFootTransform(foot);
  boost::numeric::ublas::matrix<float> f2w = createFootToWorldTransform(foot);
  return prod(f2w, n2f);
}

boost::numeric::ublas::matrix<float>
Kinematics::evaluateDHChain(Link from, Link to, Chain foot, bool top)
{
  boost::numeric::ublas::matrix<float> finalTransform =
    boost::numeric::ublas::identity_matrix<float>(4);
  if (foot == RIGHT_CHAIN) {
    if (top) {
      for (int i = from; i < to; i++) {
        finalTransform = boost::numeric::ublas::prod(
          finalTransform,
          transformRTop[i]);
      }
    } else {
      for (int i = from; i < to; i++) {
        finalTransform = boost::numeric::ublas::prod(
          finalTransform,
          transformRBot[i]);
      }
    }
  } else {
    if (top) {
      for (int i = from; i < to; i++) {
        finalTransform =
          boost::numeric::ublas::prod(finalTransform, transformLTop[i]);
      }
    } else {
      for (int i = from; i < to; i++) {
        finalTransform =
          boost::numeric::ublas::prod(finalTransform, transformLBot[i]);
      }
    }
  }
  return finalTransform;
}

// Evaluate kinematics chain from all limbs back to the IMU, taking into account the COM at each part.
boost::numeric::ublas::matrix<float>
Kinematics::evaluateMassChain()
{
  int i, joint = 0;
  float totalMass = 0;
  boost::numeric::ublas::matrix<float> finalTransform(4, 1);
  finalTransform = boost::numeric::ublas::zero_matrix<float>(4, 1);

  // Mass of torso
  finalTransform += massesCom[joint] * masses[joint];
  totalMass += masses[joint];
  ++joint;

  // Mass of head
  boost::numeric::ublas::matrix<float> headTransform =
    boost::numeric::ublas::identity_matrix<float>(4);
  for (i = 0; i < HEAD_DH_CHAIN_LEN; ++i) {
    headTransform = boost::numeric::ublas::prod(headTransform, transformHB[i]);
    // Up to head yaw, head pitch
    if (i == 0 || i == 2) {
      finalTransform +=
        boost::numeric::ublas::prod(headTransform, massesCom[joint]) * masses[joint];
      totalMass += masses[joint];
      ++joint;
    }
  }

  // Mass of right arm
  boost::numeric::ublas::matrix<float> rArmTransform =
    boost::numeric::ublas::identity_matrix<float>(4);
  for (i = 0; i < ARM_DH_CHAIN_LEN; ++i) {
    rArmTransform = boost::numeric::ublas::prod(rArmTransform, transformRAB[i]);
    // Up to shoulder pitch, shoulder roll, elbow yaw, elbow roll, wrist yaw
    if (i == 3 || i == 4 || i == 8 || i == 9 || i == 13) {
      finalTransform +=
        boost::numeric::ublas::prod(rArmTransform, massesCom[joint]) * masses[joint];
      totalMass += masses[joint];
      ++joint;
    }
  }

  // Mass of left arm
  boost::numeric::ublas::matrix<float> lArmTransform =
    boost::numeric::ublas::identity_matrix<float>(4);
  for (i = 0; i < ARM_DH_CHAIN_LEN; ++i) {
    // Up to shoulder pitch, shoulder roll, elbow yaw, elbow roll, wrist yaw
    lArmTransform = boost::numeric::ublas::prod(lArmTransform, transformLAB[i]);
    if (i == 3 || i == 4 || i == 8 || i == 9 || i == 13) {
      finalTransform +=
        boost::numeric::ublas::prod(lArmTransform, massesCom[joint]) * masses[joint];
      totalMass += masses[joint];
      ++joint;
    }
  }

  // Mass of right leg
  boost::numeric::ublas::matrix<float> rLegTransform =
    boost::numeric::ublas::identity_matrix<float>(4);
  for (i = 0; i < LEG_DH_CHAIN_LEN; ++i) {
    rLegTransform = boost::numeric::ublas::prod(rLegTransform, transformRFB[i]);
    // Up to hip yaw pitch, hip roll, hip pitch, knee pitch, ankle pitch, ankle roll
    if (i == 3 || i == 7 || i == 9 || i == 12 || i == 15 || i == 18) {
      finalTransform +=
        boost::numeric::ublas::prod(rLegTransform, massesCom[joint]) * masses[joint];
      totalMass += masses[joint];
      ++joint;
    }
  }

  // Mass of left leg
  boost::numeric::ublas::matrix<float> lLegTransform =
    boost::numeric::ublas::identity_matrix<float>(4);
  for (i = 0; i < LEG_DH_CHAIN_LEN; ++i) {
    lLegTransform = boost::numeric::ublas::prod(lLegTransform, transformLFB[i]);
    // Up to hip yaw pitch, hip roll, hip pitch, knee pitch, ankle pitch, ankle roll
    if (i == 3 || i == 7 || i == 9 || i == 12 || i == 15 || i == 18) {
      finalTransform +=
        boost::numeric::ublas::prod(lLegTransform, massesCom[joint]) * masses[joint];
      totalMass += masses[joint];
      ++joint;
    }
  }

  return finalTransform / totalMass;
}

boost::numeric::ublas::matrix<float>
Kinematics::createCameraToFootTransform(Chain foot, bool top)
{
  boost::numeric::ublas::matrix<float> b2f = evaluateDHChain(FOOT, BODY, foot, top);
  // When we get to the torso, we need to adjust the transform by the forward and side lean to account
  // for when the robot's feet are not flat on the ground. We apply the the rotation of the lean to the
  // hip vector to account for the lean already introduced by the leg joints.
  // We also adjust by the body pitch offset from kinematics calibration.
  boost::numeric::ublas::matrix<float> z(4, 1);
  z(0, 0) = 0;
  z(1, 0) = 0;
  z(2, 0) = 0;
  z(3, 0) = 1;
  boost::numeric::ublas::matrix<float> hipPt = prod(b2f, z);
  float bodyPitchOffset = DEG2RAD(parameters.bodyPitch);
  float forwardLean = sensorValues.sensors[Sensors::InertialSensor_IntegratedAngleY];
  float sideLean = sensorValues.sensors[Sensors::InertialSensor_IntegratedAngleX];

  boost::numeric::ublas::matrix<float> transform = createDHMatrix<float>(hipPt(0, 0), 0, 0, 0);
  transform = prod(transform, createDHMatrix<float>(0, 0, hipPt(2, 0), M_PI / 2));  // move up by hip height
  transform = prod(transform, createDHMatrix<float>(hipPt(1, 0), 0, 0, 0));  // move sideways
  transform =
    prod(transform, createDHMatrix<float>(0, forwardLean + bodyPitchOffset, 0, -M_PI / 2));
  transform = prod(transform, createDHMatrix<float>(0, sideLean, 0, 0));

  return prod(transform, evaluateDHChain(BODY, CAMERA, foot, top));
}

boost::numeric::ublas::matrix<float>
Kinematics::createNeckToFootTransform(Chain foot)
{
  boost::numeric::ublas::matrix<float> b2f = evaluateDHChain(FOOT, BODY, foot);
  // When we get to the torso, we need to adjust the transform by the forward and side lean to account
  // for when the robot's feet are not flat on the ground. We apply the the rotation of the lean to the
  // hip vector to account for the lean already introduced by the leg joints.
  // We also adjust by the body pitch offset from kinematics calibration.
  boost::numeric::ublas::matrix<float> z(4, 1);
  z(0, 0) = 0;
  z(1, 0) = 0;
  z(2, 0) = 0;
  z(3, 0) = 1;
  boost::numeric::ublas::matrix<float> hipPt = prod(b2f, z);
  float bodyPitchOffset = DEG2RAD(parameters.bodyPitch);
  float forwardLean = sensorValues.sensors[Sensors::InertialSensor_IntegratedAngleY];
  float sideLean = sensorValues.sensors[Sensors::InertialSensor_IntegratedAngleX];

  boost::numeric::ublas::matrix<float> transform = createDHMatrix<float>(hipPt(0, 0), 0, 0, 0);
  transform = prod(transform, createDHMatrix<float>(0, 0, hipPt(2, 0), M_PI / 2));  // move up by hip height
  transform = prod(transform, createDHMatrix<float>(hipPt(1, 0), 0, 0, 0));  // move sideways
  transform =
    prod(transform, createDHMatrix<float>(0, forwardLean + bodyPitchOffset, 0, -M_PI / 2));
  transform = prod(transform, createDHMatrix<float>(0, sideLean, 0, 0));

  return prod(transform, cameraPanInverseHack);
}

// World is defined as the centre of the two feet on the ground plane,
// with a heading equal to the average of the two feet directions
boost::numeric::ublas::matrix<float>
Kinematics::createFootToWorldTransform(Chain foot, bool top)
{
  boost::numeric::ublas::matrix<float> b2lf =
    evaluateDHChain(FOOT, BODY, foot, top);
  boost::numeric::ublas::matrix<float> b2rf =
    evaluateDHChain(FOOT, BODY, (Chain) !foot, top);

  boost::numeric::ublas::matrix<float> rf2b(4, 4);
  invertMatrix(b2rf, rf2b);

  boost::numeric::ublas::matrix<float> rf2lf(4, 4);
  rf2lf = prod(rf2b, b2lf);

  boost::numeric::ublas::matrix<float> z(4, 1);
  z(0, 0) = 0;
  z(1, 0) = 0;
  z(2, 0) = 0;
  z(3, 0) = 1;
  boost::numeric::ublas::matrix<float> forward(4, 1);
  forward(0, 0) = 1;
  forward(1, 0) = 0;
  forward(2, 0) = 0;
  forward(3, 0) = 1;

  // first find position of centre of two feet on the ground.
  z = prod(rf2lf, z);

  // find direction of second foot in first foot coords
  forward = prod(rf2lf, forward) - z;

  boost::numeric::ublas::matrix<float> position = -z / 2;
  position(3, 0) = 1;
  position(2, 0) = 0;   // on the ground

  boost::numeric::ublas::matrix<float> result =
    boost::numeric::ublas::identity_matrix<float>(4);
  result = prod(
    translateMatrix<float>(position(0, 0), position(1, 0), 0),
    result);
  result = prod(
    rotateZMatrix<float>(-atan2f(forward(1, 0), forward(0, 0)) / 2.0),
    result);
  return result;
}

void Kinematics::setSensorValues(SensorValues sensorValues)
{
  this->sensorValues = sensorValues;
}

boost::numeric::ublas::matrix<float>
Kinematics::createWorldToFOVTransform(
  const boost::numeric::ublas::matrix<float> & c2w)
{
  boost::numeric::ublas::matrix<float> w2c = c2w;

  invertMatrix(c2w, w2c);

  float ex = 0;
  float ey = 0;
  float ez = 1.0 / tan(IMAGE_HFOV / 2);

  boost::numeric::ublas::matrix<float> projection = projectionMatrix(ex, ey, ez);
  boost::numeric::ublas::matrix<float> transform =
    boost::numeric::ublas::prod(projection, w2c);

  return transform;
}

void Kinematics::determineBodyExclusionArray(
  const boost::numeric::ublas::matrix<float> & m,
  int16_t * points, bool top)
{

  const int COLS = (top) ? TOP_IMAGE_COLS : BOT_IMAGE_COLS;

  for (int i = 0; i < Pose::EXCLUSION_RESOLUTION; i++) {
    points[i] = TOP_IMAGE_ROWS;
    if (!top) {points[i] += BOT_IMAGE_ROWS;}
  }
  // pixel off screen really low
  boost::numeric::ublas::matrix<float> transform =
    createWorldToFOVTransform(m);

  for (unsigned int part = 0; part < bodyParts.size(); part++) {
    boost::numeric::ublas::matrix<float> last =
      fovToImageSpaceTransform(
      transform,
      bodyParts[part][0],
      top);
    for (unsigned int i = 0; i < bodyParts[part].size(); i++) {
      boost::numeric::ublas::matrix<float> m =
        fovToImageSpaceTransform(
        transform,
        bodyParts[part][i],
        top);
      if (m(2, 0) <= 0) {
        last = m;
        continue;
      }
      int lIndex = (int)(last(0, 0) / COLS *
        Pose::EXCLUSION_RESOLUTION);
      int cIndex = (int)(m(0, 0) / COLS *
        Pose::EXCLUSION_RESOLUTION);
      int lPixel = last(1, 0);
      int cPixel = m(1, 0);
      float gradient = 0;
      if (cIndex - lIndex != 0) {
        float denom = cIndex - lIndex;
        gradient = (cPixel - lPixel) / (denom);
      }
      cIndex = MIN(MAX(cIndex, 0), (int) Pose::EXCLUSION_RESOLUTION);
      lIndex = MIN(MAX(lIndex, 0), (int) Pose::EXCLUSION_RESOLUTION);
      int index = lIndex;
      while (index != cIndex && last(2, 0) > 0) {
        if (index >= 0 && index < Pose::EXCLUSION_RESOLUTION &&
          index != cIndex)
        {
          int nPixel = last(1, 0) + gradient * (index - lIndex);
          if (nPixel < points[index]) {points[index] = nPixel;}
        }
        index += (cIndex - lIndex) > 0 ? 1 : -1;
      }

      // get Index of last.
      // keep adding one and linearly interpolate
      index = (int)(m(0, 0) / COLS *
        Pose::EXCLUSION_RESOLUTION);
      if (index >= 0 && index < Pose::EXCLUSION_RESOLUTION) {
        if (m(1, 0) < points[index]) {
          points[index] = m(1, 0);
        }
      }
      last = m;
    }
  }
}

boost::numeric::ublas::matrix<float>
Kinematics::fovToImageSpaceTransform(
  const boost::numeric::ublas::matrix<float> & transform,
  const boost::numeric::ublas::matrix<float> & point, bool top)
{

  // Constants
  const int COLS = (top) ? TOP_IMAGE_COLS : BOT_IMAGE_COLS;
  const int ROWS = (top) ? TOP_IMAGE_ROWS : BOT_IMAGE_ROWS;

  // use image space transform to find the perspective scaling factor
  boost::numeric::ublas::matrix<float> pixel =
    boost::numeric::ublas::prod(transform, point);

  // divide x and y by the perspective scaling factor
  pixel(0, 0) /= pixel(3, 0);
  pixel(1, 0) /= pixel(3, 0);
  pixel(2, 0) = pixel(3, 0);
  pixel(3, 0) = 1;

  // now we have the pixel in a space that spans from (-1, 1) in the x
  // direction and (-1, 1) in the y.

  // therefore we need to scale this up to our image size which is what
  // the code below does
  float xscale = COLS / 2;
  float yscale = ROWS / 2;
  pixel(0, 0) = (pixel(0, 0)) * xscale + xscale;
  pixel(1, 0) = (pixel(1, 0)) * xscale + yscale;
  return pixel;
}

void Kinematics::readParameters()
{
  parameters.cameraPitchTopWhenLookingLeft     = motion_node_->get_parameter("camera_pitch_top_when_looking_left"    ).as_double();
  parameters.cameraPitchTopWhenLookingStraight = motion_node_->get_parameter("camera_pitch_top_when_looking_straight").as_double();
  parameters.cameraPitchTopWhenLookingRight    = motion_node_->get_parameter("camera_pitch_top_when_looking_right"   ).as_double();
  parameters.cameraRollTopWhenLookingLeft      = motion_node_->get_parameter("camera_roll_top_when_looking_left"     ).as_double();
  parameters.cameraRollTopWhenLookingStraight  = motion_node_->get_parameter("camera_roll_top_when_looking_straight" ).as_double();
  parameters.cameraRollTopWhenLookingRight     = motion_node_->get_parameter("camera_roll_top_when_looking_right"    ).as_double();
  parameters.cameraYawTopWhenLookingLeft       = motion_node_->get_parameter("camera_yaw_top_when_looking_left"      ).as_double();
  parameters.cameraYawTopWhenLookingStraight   = motion_node_->get_parameter("camera_yaw_top_when_looking_straight"  ).as_double();
  parameters.cameraYawTopWhenLookingRight      = motion_node_->get_parameter("camera_yaw_top_when_looking_right"     ).as_double();

  parameters.bodyPitch = motion_node_->get_parameter("body_pitch").as_double();
}

void Kinematics::publishPoseToTF2(
  tf2_ros::TransformBroadcaster & broadcaster,
  const rclcpp::Time & timestamp,
  const std::string & parent_frame)
{

  Pose pose = getPose();

  std::vector<geometry_msgs::msg::TransformStamped> transforms;

  tf2::Quaternion camera_orientation_correction;
  camera_orientation_correction.setRPY(M_PI / 2, -M_PI / 2, 0);

// Top camera transform
  geometry_msgs::msg::TransformStamped top_camera_tf;
  top_camera_tf.header.stamp = timestamp;
  top_camera_tf.header.frame_id = parent_frame; // base_footprint by default if not given.
  top_camera_tf.child_frame_id = "CameraTopMotion";

  tf2::Matrix3x3 top_rot_matrix(
    pose.topWorldToCameraTransform(0, 0), pose.topWorldToCameraTransform(0, 1),
    pose.topWorldToCameraTransform(0, 2),
    pose.topWorldToCameraTransform(1, 0), pose.topWorldToCameraTransform(1, 1),
    pose.topWorldToCameraTransform(1, 2),
    pose.topWorldToCameraTransform(2, 0), pose.topWorldToCameraTransform(2, 1),
    pose.topWorldToCameraTransform(2, 2)
  );

  tf2::Vector3 top_translation(
    pose.topWorldToCameraTransform(0, 3),
    pose.topWorldToCameraTransform(1, 3),
    pose.topWorldToCameraTransform(2, 3)
  );

  tf2::Transform top_to_world_tf(top_rot_matrix, top_translation);
  tf2::Transform world_to_top_tf = top_to_world_tf.inverse();
  tf2::Vector3 translation = world_to_top_tf.getOrigin();

  float x = translation.x() / 1000.0;
  float y = translation.y() / 1000.0;
  float z = translation.z() / 1000.0;

  top_camera_tf.transform.translation.x = x;
  top_camera_tf.transform.translation.y = y;
  top_camera_tf.transform.translation.z = z;

  tf2::Quaternion top_rotation;
  world_to_top_tf.getBasis().getRotation(top_rotation);

  tf2::Quaternion adjusted_top_rotation = top_rotation * camera_orientation_correction;

  top_camera_tf.transform.rotation.x = adjusted_top_rotation.x();
  top_camera_tf.transform.rotation.y = adjusted_top_rotation.y();
  top_camera_tf.transform.rotation.z = adjusted_top_rotation.z();
  top_camera_tf.transform.rotation.w = adjusted_top_rotation.w();

// Bottom camera transform
  geometry_msgs::msg::TransformStamped bottom_camera_tf;
  bottom_camera_tf.header.stamp = timestamp;
  bottom_camera_tf.header.frame_id = parent_frame;
  bottom_camera_tf.child_frame_id = "CameraBottomMotion";

  tf2::Matrix3x3 bottom_rot_matrix(
    pose.botWorldToCameraTransform(0, 0), pose.botWorldToCameraTransform(0, 1),
    pose.botWorldToCameraTransform(0, 2),
    pose.botWorldToCameraTransform(1, 0), pose.botWorldToCameraTransform(1, 1),
    pose.botWorldToCameraTransform(1, 2),
    pose.botWorldToCameraTransform(2, 0), pose.botWorldToCameraTransform(2, 1),
    pose.botWorldToCameraTransform(2, 2)
  );

  tf2::Vector3 bottom_translation(
    pose.botWorldToCameraTransform(0, 3),
    pose.botWorldToCameraTransform(1, 3),
    pose.botWorldToCameraTransform(2, 3)
  );

  tf2::Transform bottom_to_world_tf(bottom_rot_matrix, bottom_translation);
  tf2::Transform world_to_bottom_tf = bottom_to_world_tf.inverse();
  translation = world_to_bottom_tf.getOrigin();
  x = translation.x() / 1000.0;
  y = translation.y() / 1000.0;
  z = translation.z() / 1000.0;
  bottom_camera_tf.transform.translation.x = x;
  bottom_camera_tf.transform.translation.y = y;
  bottom_camera_tf.transform.translation.z = z;
  tf2::Quaternion bottom_rotation;
  world_to_bottom_tf.getBasis().getRotation(bottom_rotation);
  tf2::Quaternion adjusted_bottom_rotation = bottom_rotation * camera_orientation_correction;

  bottom_camera_tf.transform.rotation.x = adjusted_bottom_rotation.x();
  bottom_camera_tf.transform.rotation.y = adjusted_bottom_rotation.y();
  bottom_camera_tf.transform.rotation.z = adjusted_bottom_rotation.z();
  bottom_camera_tf.transform.rotation.w = adjusted_bottom_rotation.w();

  transforms.push_back(top_camera_tf);
  transforms.push_back(bottom_camera_tf);
  broadcaster.sendTransform(transforms);
}
