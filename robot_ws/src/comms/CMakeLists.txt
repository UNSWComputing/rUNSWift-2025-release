
cmake_minimum_required(VERSION 3.8)
project(comms)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(runswift_interfaces REQUIRED)


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

if(BUILD_TESTING)
find_package(ament_cmake_pytest REQUIRED)
set(_pytest_tests
  # Add test files here
)
foreach(_test_path ${_pytest_tests})
  get_filename_component(_test_name ${_test_path} NAME_WE)
  ament_add_pytest_test(${_test_name} ${_test_path}
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
    TIMEOUT 60
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  )
endforeach()
endif()


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Install Python modules
# ament_python_install_package(${PROJECT_NAME})
ament_export_dependencies(rosidl_default_runtime)
# Install Python executables
install(PROGRAMS
  src/game_controller_spl.py
  src/game_controller_publisher.py
  src/player_number.py
  src/whistle_detector.py
  src/whistle_detector_functions.py
  src/robot_communication_publisher.py
  src/ggwaveReceiver.py
  src/ggwaveSpeaker.py
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  src/rcgcd/conversion.py
  src/rcgcd/robocup_game_control_data.py
  DESTINATION lib/${PROJECT_NAME}/rcgcd
)
install(PROGRAMS
  src/rcgcrd/conversion.py
  src/rcgcrd/robocup_game_control_return_data.py
  DESTINATION lib/${PROJECT_NAME}/rcgcrd
)
install(PROGRAMS
  src/rcgcrrd/conversion.py
  src/rcgcrrd/robocup_game_control_robot_return_data.py
  DESTINATION lib/${PROJECT_NAME}/rcgcrrd
)
install(PROGRAMS
  src/rcgctd/conversion.py
  src/rcgctd/robocup_game_control_team_data.py
  DESTINATION lib/${PROJECT_NAME}/rcgctd
)
add_executable(GameInfo src/GameInfo.cpp)
ament_target_dependencies(GameInfo std_msgs rclcpp runswift_interfaces)

add_executable(RobotCommunication src/RobotCommunication.cpp)
ament_target_dependencies(RobotCommunication std_msgs rclcpp runswift_interfaces)



# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install cpp nodes
install(
  TARGETS  GameInfo RobotCommunication
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python executables
install(PROGRAMS
# Commented to keep 'colcon build' working, if this is a good place please fix this, if not, remove it
#  comms/whistleDetector.py

  DESTINATION lib/${PROJECT_NAME}
)


# customize CMakeLists.txt below

# end CMakeLists.txt customization

ament_package()
