#!/bin/bash
# LiDAR System Cleanup Script
# 残存しているノードやプロセスをクリーンアップします

echo "Cleaning up LiDAR system..."

# Kill existing component containers
pkill -9 -f "component_container.*lidar_container" 2>/dev/null
pkill -9 -f "lidar_get_cloud_points" 2>/dev/null
pkill -9 -f "lidar_preprocess" 2>/dev/null
pkill -9 -f "lidar_detection" 2>/dev/null

# Restart ROS2 daemon to clear topics
ros2 daemon stop
sleep 1
ros2 daemon start

echo "Cleanup complete. You can now launch the system."
