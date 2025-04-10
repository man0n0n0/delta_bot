cmake_minimum_required(VERSION 3.8)
project(rock_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL 1.10 REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(rock_detector_node src/rock_detector_node.cpp)
ament_target_dependencies(rock_detector_node
  rclcpp
  sensor_msgs
  pcl_conversions
  pcl_ros
)
target_link_libraries(rock_detector_node ${PCL_LIBRARIES})

install(TARGETS
  rock_detector_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()