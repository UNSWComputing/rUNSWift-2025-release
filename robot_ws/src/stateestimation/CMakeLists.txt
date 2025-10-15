cmake_minimum_required(VERSION 3.8)
project(stateestimation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(runswift_interfaces REQUIRED)
find_package(nao_lola_sensor_msgs REQUIRED)

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

add_executable(SelfEstimator src/self/SelfEstimator.cpp)
add_executable(BallEstimator src/ball/BallEstimator.cpp)
include_directories(include)

file(GLOB statestimation_sources "src/self/*")
file(GLOB ballestimation_sources "src/ball/*")

target_sources(SelfEstimator PUBLIC ${statestimation_sources})
target_sources(BallEstimator PUBLIC ${ballestimation_sources})

target_link_libraries(SelfEstimator Eigen3::Eigen)
target_link_libraries(BallEstimator Eigen3::Eigen)

ament_target_dependencies(SelfEstimator runswift_interfaces nao_lola_sensor_msgs rclcpp geometry_msgs nav_msgs tf2 tf2_ros)
ament_target_dependencies(BallEstimator runswift_interfaces nao_lola_sensor_msgs rclcpp geometry_msgs nav_msgs tf2 tf2_ros)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install cpp nodes
install(
  TARGETS  SelfEstimator BallEstimator
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
