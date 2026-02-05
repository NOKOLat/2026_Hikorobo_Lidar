# Livox MID-70 LiDAR Data Acquisition & Processing System - AI Coding Guidelines

## Project Architecture

This is a **ROS 2 (Robot Operating System) point cloud pipeline** for Livox MID-70 LiDAR data:

- **`lidar_get_points/`**: Data acquisition node that connects to hardware via Livox SDK and publishes raw point clouds to ROS topic `livox/pointcloud` at ~10Hz
- **`lidar_preprocess/`**: Real-time point cloud filtering pipeline that subscribes to raw data and publishes processed data to `preprocess/pointcloud`
- **Data Flow**: Hardware → `livox_node` → ROS Topic → `preprocess_node` → (ROI → Voxel → SOR filters) → Output Topic

## Build & Runtime

**Build System**: CMake with ament (ROS 2 build framework)

```bash
./build.sh              # Builds both packages using: colcon build --packages-select lidar_get_points lidar_preprocess
./start.sh              # Interactive launcher with 3 options: (1) Get points only, (2) Preprocess only, (3) Both
```

**Key Detail**: Both nodes must be sourced with `install/setup.bash` before running. Build artifacts are in `build/` and `install/` directories. Latest logs are in `log/latest_build/`.

## Code Patterns & Dependencies

### ROS 2 Node Architecture
- Both nodes inherit from `rclcpp::Node` and use parameter system: `declare_parameter()` → `get_parameter()`
- **Publishers**: Use `sensor_msgs::msg::PointCloud2` for point cloud data
- **Subscribers**: `CloudCallback()` pattern in preprocess_node subscribes to `livox/pointcloud`
- Critical dependency: `pcl_conversions` for `pcl::fromROSMsg()` / `pcl::toROSMsg()` conversions

### LiDAR Hardware Integration
- `livox_node.cpp` initializes Livox SDK via `InitLivoxSdk()` and device discovery callbacks
- SDK callbacks: `OnDeviceBroadcast()` for device discovery, `OnDeviceChange()` for connection changes
- LiDAR frame data is buffered (`buffer_frames_`, `integration_time_ms_`) before publishing to integrate multiple device frames
- Parameter `flip_yz: false` allows coordinate system flipping if needed

### Point Cloud Filtering Pipeline
Three filters applied sequentially in `preprocess_node.cpp`:

1. **ROI Filter** (`roi_filter.hpp/.cpp`): Crop to region of interest (x: 0.5-20m, y: -5-5m, z: -2-5m by default)
2. **Voxel Filter** (`voxel_filter.hpp/.cpp`): Downsampling via leaf size (default 0.01m/1cm)
3. **SOR Filter** (`sor_filter.hpp/.cpp`): Statistical outlier removal (mean_k=50 neighbors, std_dev_mul=1.0)

**Filter Parameters**: Configurable via ROS parameters in `preprocess_node.cpp` (lines 17-28). Each filter runs synchronously in callback thread.

## Development Conventions

- **C++ Standard**: CMake uses `-Wall -Wextra -Wpedantic` flags; modern C++14+ with PCL + rclcpp
- **PCL Usage**: Direct use of `pcl::PointCloud<pcl::PointXYZ>` and PCL algorithms (PassThrough, VoxelGrid, StatisticalOutlierRemoval)
- **Message Headers**: Preserve ROS message headers through pipeline to maintain timestamps and frame IDs
- **Logging**: Use `RCLCPP_INFO()`, `RCLCPP_DEBUG()`, `RCLCPP_ERROR()` macros (not std::cout)
- **Thread Safety**: Use `std::mutex` for shared buffers (e.g., frame deque in livox_node)

## External Dependencies

- **Livox SDK**: Statically linked (`livox_sdk_static`), headers in `/usr/local/include/`, library in `/usr/local/lib/`
- **PCL**: Full Point Cloud Library with standard filters
- **ROS 2**: rclcpp, sensor_msgs, std_msgs (system dependencies)

## Common Tasks

**Adding a new filter**: Create `new_filter.hpp` in `lidar_preprocess/include/lidar_preprocess/`, implement with `filter(input, output)` method, instantiate in `PreprocessNode` constructor, apply in `CloudCallback()`.

**Testing point cloud**: Use `ros2 topic echo /livox/pointcloud | head` to inspect raw data; use RViz with PointCloud2 display to visualize.

**Debugging hardware**: Check device discovery by adding debug logs in `OnDeviceBroadcast()` callback; verify Livox SDK version with `GetLivoxSdkVersion()`.

**Tuning parameters**: Edit defaults in source (lines referenced above) or override at runtime via ROS param server with `--ros-args -p param_name:=value`.
