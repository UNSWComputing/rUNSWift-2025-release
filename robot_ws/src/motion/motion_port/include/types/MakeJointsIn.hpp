#pragma once
#include "motion/generator/BodyModel.hpp"
#include "runswift_interfaces/msg/motion_command.hpp"
#include "types/Odometry.hpp"
#include "types/SensorValues.hpp"

using MotionCommand = runswift_interfaces::msg::MotionCommand;

/**
 * A container for make joint inputs
 * @param request New command for body movement. Generator should modify
 *           the request with the command it is currently doing if it is not the same
 * @param odometry Running tally of walk distance in f, l, t
 * @param sensors Last-read values of sensors
 * @param model of the robot
 * @param ballX relative x (forward) position of the ball
 * @param ballY relative y (side -> left) position of the ball
 */
struct MakeJointsIn
{
  MakeJointsIn(
    MotionCommand * request,
    Odometry * odometry,
    const SensorValues & sensors,
    BodyModel & bodyModel,
    float ballX,
    float ballY
  )
  : request(request),
    odometry(odometry),
    sensors(sensors),
    bodyModel(bodyModel),
    ballX(ballX),
    ballY(ballY)
  {
  }

  MotionCommand * request;
  Odometry * odometry;
  const SensorValues & sensors;
  BodyModel & bodyModel;
  float ballX;
  float ballY;
};
