#ifndef PARAMETERS_HPP
#define PARAMETERS_HPP

class Parameters
{
public:
  Parameters();

  // float cameraPitchTop;
  // float cameraYawTop;
  // float cameraRollTop;
  float cameraYawBottom;
  float cameraPitchBottom;
  float cameraRollBottom;

  float cameraPitchTopWhenLookingLeft;
  float cameraPitchTopWhenLookingStraight;
  float cameraPitchTopWhenLookingRight;
  float cameraRollTopWhenLookingLeft;
  float cameraRollTopWhenLookingStraight;
  float cameraRollTopWhenLookingRight;
  float cameraYawTopWhenLookingLeft;
  float cameraYawTopWhenLookingStraight;
  float cameraYawTopWhenLookingRight;

  float bodyPitch;
};

#endif // PARAMETERS_HPP
