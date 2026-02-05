cmake_minimum_required(VERSION 3.8)
project(lidar_background_diff)

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

# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# Background subtraction node
add_executable(background_diff_node
  src/background_diff_node.cpp
  src/main.cpp
)

ament_target_dependencies(background_diff_node
  rclcpp
  sensor_msgs
  std_msgs
  pcl_conversions
  pcl_ros
  yaml-cpp
)

target_link_libraries(background_diff_node
  ${PCL_LIBRARIES}
  yaml-cpp
)

install(TARGETS
  background_diff_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
