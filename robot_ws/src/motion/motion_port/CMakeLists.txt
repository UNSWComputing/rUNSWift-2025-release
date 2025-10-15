cmake_minimum_required(VERSION 3.8)
project(motion_port)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(runswift_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nao_lola_command_msgs REQUIRED)
find_package(nao_lola_sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIRS})

set(
  THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclcpp_components
  runswift_interfaces
  nao_lola_command_msgs
  nao_lola_sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  sensor_msgs
  )

add_executable(motion_adapter src/motion_adapter.cpp)
add_executable(kick_calibration scripts/kick_calibration.cpp)

file(GLOB_RECURSE MOTION_SOURCES "${CMAKE_SOURCE_DIR}/include/*.cpp")
target_sources(motion_adapter PRIVATE ${MOTION_SOURCES})

# Component library
add_library(motion_adapter_component SHARED
  src/motion_adapter.cpp
)
target_include_directories(motion_adapter_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(motion_adapter_component ${THIS_PACKAGE_INCLUDE_DEPENDS})
rclcpp_components_register_node(motion_adapter_component
  PLUGIN "MotionAdapter"
  EXECUTABLE motion_adapter_component_node
)

ament_target_dependencies(motion_adapter ${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_target_dependencies(kick_calibration ${THIS_PACKAGE_INCLUDE_DEPENDS})

target_link_libraries(
    motion_adapter
    ${ament_TARGET_DEPENDENCIES}
    ${Boost_LIBRARIES} 
)

# Install cpp nodes
install(
  TARGETS
  motion_adapter
  kick_calibration
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

# Install Python scripts
install(
  PROGRAMS scripts/test_joints_movement_node.py
  DESTINATION lib/${PROJECT_NAME}
)

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

ament_package()
