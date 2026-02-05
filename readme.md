# Livox MID-70 データ取得

## ビルド
```bash
colcon build --packages-select lidar_get_points
```

## 起動
```bash
source install/setup.bash
ros2 launch lidar_get_points livox.launch.py
```

## トピック
- `/livox/pointcloud` (sensor_msgs/PointCloud2)

## RViz2設定
- Fixed Frame: `map`
- Add → PointCloud2 → Topic: `/livox/pointcloud`