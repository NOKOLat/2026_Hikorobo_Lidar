cmake_minimum_required(VERSION 3.8)
project(lidar_preprocess)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Include directories
include_directories(include)

# Add filter libraries
add_library(passthrough_filter SHARED
  src/passthrough_filter.cpp)
target_include_directories(passthrough_filter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(passthrough_filter ${PCL_LIBRARIES})

add_library(voxel_grid_filter SHARED
  src/voxel_grid_filter.cpp)
target_include_directories(voxel_grid_filter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(voxel_grid_filter ${PCL_LIBRARIES})

add_library(statistical_outlier_removal_filter SHARED
  src/statistical_outlier_removal_filter.cpp)
target_include_directories(statistical_outlier_removal_filter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(statistical_outlier_removal_filter ${PCL_LIBRARIES})

# Add component library
add_library(lidar_preprocess_component SHARED
  src/lidar_preprocess_component.cpp)
target_compile_definitions(lidar_preprocess_component PRIVATE "COMPOSITION_BUILDING_DLL")
target_include_directories(lidar_preprocess_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(lidar_preprocess_component
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  pcl_conversions
  pcl_ros
  ament_index_cpp)
target_link_libraries(lidar_preprocess_component
  ${PCL_LIBRARIES}
  yaml-cpp
  passthrough_filter
  voxel_grid_filter
  statistical_outlier_removal_filter)
rclcpp_components_register_nodes(lidar_preprocess_component "lidar_preprocess::LidarPreprocessComponent")

# Install
install(TARGETS
  passthrough_filter
  voxel_grid_filter
  statistical_outlier_removal_filter
  lidar_preprocess_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/
  DESTINATION include)

# Install configuration file
install(FILES lidar_preprocess_config.yaml
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
