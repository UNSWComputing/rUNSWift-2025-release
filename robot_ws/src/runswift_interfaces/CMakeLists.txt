
cmake_minimum_required(VERSION 3.8)
project(runswift_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs)
find_package(sensor_msgs)
find_package(vision_msgs)
find_package(geometry_msgs)


if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()


# Generating custom msgs
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BehavioursRobotInfo.msg"
  "msg/BehavioursGamestate.msg"
  "msg/RGB.msg"
  "msg/BodyCommand.msg"
  "msg/ButtonEvent.msg"
  "msg/CommsGamestate.msg"
  "msg/CommsRCGCD.msg"
  "msg/CommsRCGCRD.msg"
  "msg/CommsRCGCRRD.msg"
  "msg/CommsRobotInfo.msg"
  "msg/CommsRobotPlayerInfo.msg"
  "msg/CommsTeamInfo.msg"
  "msg/CommsTeamstate.msg"
  "msg/CommsWhistle.msg"
  "msg/CommsRemainingPackets.msg"
  "msg/HeadCommand.msg"
  "msg/MotionCommand.msg"
  "msg/MotionOdometry.msg"
  "msg/SeBall.msg"
  "msg/SeBallsAbsolute.msg"
  "msg/SeBallsRelative.msg"
  "msg/SimulationRobotState.msg"
  "msg/SimulationState.msg"
  "msg/VisionBallFeature.msg"
  "msg/VisionBalls.msg"
  "msg/VisionObstacles.msg"
  "msg/VisionObstacleFeature.msg"
  "msg/VisionFieldFeature.msg"
  "msg/VisionFieldFeatures.msg"
  "msg/VisionImageWithFieldBoundry.msg"
  "msg/VisionProcessedImageData.msg"
  "msg/VisionPipelineSwitchAsk.msg"
  "msg/VisionPipelineSwitchResult.msg"
  "msg/RobotDetection.msg"
  "srv/DetectRobots.srv"
  "srv/BallCls.srv"
  "srv/BotModel.srv"
  "srv/FieldFeatureCls.srv"
  "msg/UnwrapFieldFeature.msg"
  "msg/UnwrapFieldFeatures.msg"
  "msg/GlobalPoseObservation.msg"
  DEPENDENCIES sensor_msgs vision_msgs std_msgs geometry_msgs
 )

ament_package()
