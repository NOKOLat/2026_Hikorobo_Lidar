cmake_minimum_required(VERSION 3.8)
project(lidar_get_points)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(yaml-cpp REQUIRED)

# Livox SDK
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  /usr/local/include
)

link_directories(
  /usr/local/lib
)

# Livox data acquisition node
add_executable(livox_node src/livox_node.cpp)
ament_target_dependencies(livox_node
  rclcpp
  sensor_msgs
  std_msgs
  pcl_conversions
  pcl_ros
  yaml-cpp
)
target_link_libraries(livox_node
  ${PCL_LIBRARIES}
  livox_sdk_static
  yaml-cpp
)

install(TARGETS
  livox_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
