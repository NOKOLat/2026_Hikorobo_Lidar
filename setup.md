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
├──/           <-- [3] 背景差分用のノード
│   ├── package.xml
│   ├── CMakeLists.txt
│   └── src/ ...
├──/           <-- [4] クラスタリング用のノード
│   ├── package.xml
│   ├── CMakeLists.txt
│   └── src/ ...
│
└── lidar_bringup/             <-- [5] 統合・起動パッケージ
    └── launch/ ...
```