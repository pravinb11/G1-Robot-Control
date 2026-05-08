cmake_minimum_required(VERSION 3.8)
project(g1_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Install URDF
install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install meshes
install(
  DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/g1_standing_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
