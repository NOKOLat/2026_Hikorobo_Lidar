これらは１つのコンポーネントとして動く

```
2026_hikorobo_mid70/
│ 
├── lidar_get_points/    <-- [1] データ出力用パッケージ
│   ├── package.xml
│   └── src/ ...
│
├── lidar_preprocess/    <-- [2] 前処理用パッケージ
│   ├── package.xml
│   ├── CMakeLists.txt
│   └── src/ ...
│
├── lidar_detection/           <-- [3] 検知用パッケージ //処理済みデータをrvizに提供
│   ├── package.xml
│   ├── CMakeLists.txt
│   └── src/ ...
│
└── lidar_bringup/             <-- [4] 統合・起動パッケージ
    └── launch/ ...
```