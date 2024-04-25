cmake_minimum_required(VERSION 3.5)
project(tricycle_test)  # Ensure this matches exactly with your package.xml name

# Set the C99 standard for C and C++14 for C++
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)  # Enforces C++14 compatibility
endif()

# Compiler options for better code quality
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# # Finding required ROS 2 packages
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(geometry_msgs REQUIRED)

# # Define the executable for the tricycle drive example
# add_executable(example_tricycle_drive src/examples/example_tricycle_drive.cpp)

# # Link ROS 2 dependencies
# ament_target_dependencies(example_tricycle_drive
#   rclcpp
#   geometry_msgs
# )

# # Installation of the executable
# install(TARGETS
#   example_tricycle_drive
#   DESTINATION lib/${PROJECT_NAME}
# )

# Install other directories if they contain necessary resources
install(DIRECTORY
  launch
  config
  urdf
  meshes
  DESTINATION share/${PROJECT_NAME}/
)

# Setup testing configurations if BUILD_TESTING is enabled
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Final packaging
ament_package()
