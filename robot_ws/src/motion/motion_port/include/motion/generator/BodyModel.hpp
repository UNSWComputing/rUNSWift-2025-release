#pragma once
#include "types/XYZ_Coord.hpp"
#include "perception/kinematics/Kinematics.hpp"

class BodyModel
{
public:
  BodyModel();
  void update(const SensorValues & sensors);

  inline XYZ_Coord getCoM()
  {
    return centreOfMass;
  }

  void setIsLeftPhase(bool b)
  {
    isLeftPhase = b;
  }

  void processUpdate();

  Kinematics * kinematics;

  // Used by Walk2014Generator.cpp
  bool isLeftPhase;
  float lastZMPL;
  float ZMPL;

  // For disturbance rejection
  bool isFootOnGround(const SensorValues & sensors);
  bool isOnFrontOfFoot(const SensorValues & sensor);
  float getFootZMP(bool isLeft, const SensorValues & sensors);
  float getHorizontalFootZMP(bool isLeft, const SensorValues & sensors);

private:
  float forwardL, forwardR, turnL, turnR;
  float pressureL, pressureR;
  float fsLfr, fsLfl, fsLrr, fsLrl;                           // Maximum foot sensor reading during latest run
  float fsRfr, fsRfl, fsRrr, fsRrl;                           // ... used to scale each foot sensor to read in similar range

  XYZ_Coord centreOfMass;
  XYZ_Coord centreOfMassOther;     // other foot
};
