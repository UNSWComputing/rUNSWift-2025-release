#include "Parameters.hpp"

Parameters::Parameters()
{
  // cameraPitchTop = 0.f;
  // cameraYawTop = 0.f;
  // cameraRollTop = 0.f;
  cameraYawBottom = 0.0f;
  cameraPitchBottom = 0.0f;
  cameraRollBottom = 0.0f;

  cameraPitchTopWhenLookingLeft = 0.0f;
  cameraPitchTopWhenLookingStraight = 0.0f;
  cameraPitchTopWhenLookingRight = 0.0f;
  cameraRollTopWhenLookingLeft = 0.0f;
  cameraRollTopWhenLookingStraight = 0.0f;
  cameraRollTopWhenLookingRight = 0.0f;
  cameraYawTopWhenLookingLeft = 0.0f;
  cameraYawTopWhenLookingStraight = 0.0f;
  cameraYawTopWhenLookingRight = 0.0f;
  bodyPitch = 0.0f;
}
