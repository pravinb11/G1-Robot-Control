cmake_minimum_required(VERSION 3.14)
project(g1_zmp_walking)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic -O2)

find_package(ament_cmake               REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp                    REQUIRED)
find_package(rclcpp_action             REQUIRED)
find_package(rclcpp_components         REQUIRED)
find_package(geometry_msgs             REQUIRED)
find_package(visualization_msgs        REQUIRED)
find_package(std_msgs                  REQUIRED)
find_package(builtin_interfaces        REQUIRED)
find_package(action_msgs               REQUIRED)
find_package(tf2_ros                   REQUIRED)
find_package(tf2                       REQUIRED)
find_package(nav_msgs                  REQUIRED)
find_package(Eigen3                    REQUIRED)
find_package(trajectory_msgs           REQUIRED)
find_package(sensor_msgs               REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/PlanFootsteps.action"
  DEPENDENCIES geometry_msgs builtin_interfaces
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Header-only library (footstep_planner.hpp + zmp_reference_generator.hpp
#                      + dare_solver.hpp + pattern_generator.hpp)
add_library(footstep_planner_lib INTERFACE)
target_include_directories(footstep_planner_lib INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(footstep_planner_lib INTERFACE Eigen3::Eigen)

# ── footstep_planner_node ──────────────────────────────────────────────────────
add_executable(footstep_planner_node src/footstep_planner_node.cpp)
target_include_directories(footstep_planner_node PRIVATE include)
ament_target_dependencies(footstep_planner_node
  rclcpp rclcpp_action geometry_msgs visualization_msgs
  std_msgs builtin_interfaces action_msgs tf2_ros tf2
)
target_link_libraries(footstep_planner_node
  footstep_planner_lib
  "${cpp_typesupport_target}"
)

# ── zmp_reference_node ─────────────────────────────────────────────────────────
add_executable(zmp_reference_node src/zmp_reference_node.cpp)
target_include_directories(zmp_reference_node PRIVATE include)
ament_target_dependencies(zmp_reference_node
  rclcpp geometry_msgs nav_msgs visualization_msgs
  std_msgs builtin_interfaces
)
target_link_libraries(zmp_reference_node
  footstep_planner_lib
  ${rclcpp_LIBRARIES}
)

# ── pattern_generator_node ─────────────────────────────────────────────────────
add_executable(pattern_generator_node src/pattern_generator_node.cpp)
target_include_directories(pattern_generator_node PRIVATE include)
ament_target_dependencies(pattern_generator_node
  rclcpp geometry_msgs nav_msgs visualization_msgs
  std_msgs builtin_interfaces
)
target_link_libraries(pattern_generator_node
  footstep_planner_lib
  Eigen3::Eigen
)


# ── ik_solver_node ─────────────────────────────────────────────────────────────
add_executable(ik_solver_node src/ik_solver_node.cpp)
target_include_directories(ik_solver_node PRIVATE include)
ament_target_dependencies(ik_solver_node
  rclcpp geometry_msgs nav_msgs visualization_msgs
  std_msgs sensor_msgs trajectory_msgs builtin_interfaces
)
target_link_libraries(ik_solver_node
  footstep_planner_lib
  Eigen3::Eigen
)


# ── swing_foot_node ────────────────────────────────────────────────────────────
add_executable(swing_foot_node src/swing_foot_node.cpp)
target_include_directories(swing_foot_node PRIVATE include)
ament_target_dependencies(swing_foot_node
  rclcpp geometry_msgs nav_msgs visualization_msgs
  std_msgs builtin_interfaces
)
target_link_libraries(swing_foot_node
  footstep_planner_lib
  Eigen3::Eigen
)


# ── walking_fsm_node ───────────────────────────────────────────────────────────
add_executable(walking_fsm_node src/walking_fsm_node.cpp)
target_include_directories(walking_fsm_node PRIVATE include)
ament_target_dependencies(walking_fsm_node
  rclcpp geometry_msgs visualization_msgs
  std_msgs builtin_interfaces
)
target_link_libraries(walking_fsm_node
  footstep_planner_lib
  Eigen3::Eigen
)


# ── walking_controller_node (main integrating controller) ──────────────────────
add_executable(walking_controller_node src/walking_controller_node.cpp)
target_include_directories(walking_controller_node PRIVATE include)
ament_target_dependencies(walking_controller_node
  rclcpp rclcpp_action geometry_msgs nav_msgs visualization_msgs
  std_msgs sensor_msgs trajectory_msgs builtin_interfaces
  action_msgs
)
target_link_libraries(walking_controller_node
  footstep_planner_lib
  "${cpp_typesupport_target}"
  Eigen3::Eigen
)

# ── Install ────────────────────────────────────────────────────────────────────
install(TARGETS
  footstep_planner_node
  zmp_reference_node
  pattern_generator_node
  ik_solver_node
  swing_foot_node
  walking_fsm_node
  walking_controller_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

# ── Tests ──────────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_footstep_planner test/test_footstep_planner.cpp)
  target_include_directories(test_footstep_planner PRIVATE include)
  ament_add_gtest(test_zmp_reference test/test_zmp_reference.cpp)
  target_include_directories(test_zmp_reference PRIVATE include)

  ament_add_gtest(test_walking_fsm test/test_walking_fsm.cpp)
  target_include_directories(test_walking_fsm PRIVATE include)
  target_link_libraries(test_walking_fsm Eigen3::Eigen)
endif()

ament_package()