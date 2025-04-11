cmake_minimum_required(VERSION 3.8)
project(rock_stacking)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Add executable
add_executable(rock_stacking src/rock_stacking.cpp)
target_include_directories(rock_stacking PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  rock_stacking
  "rclcpp"
  "geometry_msgs"
  "std_msgs"
)

# Install executable
install(TARGETS rock_stacking
  DESTINATION lib/${PROJECT_NAME})

# Install launch files if any
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
  PATTERN ".svn" EXCLUDE
)

# Export package dependencies
ament_export_dependencies(rclcpp geometry_msgs std_msgs)
ament_package()