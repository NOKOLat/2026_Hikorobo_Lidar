cmake_minimum_required(VERSION 3.8)
project(lidar_clustering)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(yaml-cpp REQUIRED)

# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# Clustering node
add_executable(clustering_node
  src/clustering_node.cpp
  src/main.cpp
)

ament_target_dependencies(clustering_node
  rclcpp
  sensor_msgs
  std_msgs
  visualization_msgs
  pcl_conversions
  pcl_ros
  yaml-cpp
)

target_link_libraries(clustering_node
  ${PCL_LIBRARIES}
  yaml-cpp
)

install(TARGETS
  clustering_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
