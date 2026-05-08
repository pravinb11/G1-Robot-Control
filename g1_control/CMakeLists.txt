cmake_minimum_required(VERSION 3.8)
project(g1_control)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_executable(standing_controller
  src/standing_controller.cpp
  src/standing_controller_node.cpp
)

ament_target_dependencies(
  standing_controller
  rclcpp
  sensor_msgs
  std_msgs
)

install(TARGETS
  standing_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()