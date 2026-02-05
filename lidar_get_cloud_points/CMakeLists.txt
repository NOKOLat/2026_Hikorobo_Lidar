cmake_minimum_required(VERSION 3.8)
project(lidar_get_cloud_points)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Livox SDK path
set(LIVOX_SDK_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../../../Livox-SDK)

# Include directories
include_directories(
  ${LIVOX_SDK_PATH}/sdk_core/include
)

# Link directories
link_directories(
  ${LIVOX_SDK_PATH}/build/sdk_core
)

# Add component library
add_library(lidar_get_cloud_points_component SHARED
  src/lidar_get_cloud_points_component.cpp)
target_compile_definitions(lidar_get_cloud_points_component PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(lidar_get_cloud_points_component
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs)
target_link_libraries(lidar_get_cloud_points_component
  livox_sdk_static
  pthread)
rclcpp_components_register_nodes(lidar_get_cloud_points_component "lidar_get_cloud_points::LidarGetCloudPointsComponent")

# Install
install(TARGETS
  lidar_get_cloud_points_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
