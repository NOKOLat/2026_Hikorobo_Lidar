## 環境構築手順

### 1. Ubuntu 22.04のインストール

Linux環境（Ubuntu 22.04）をセットアップします。

### 2. 基本ツールのインストール

```bash
sudo apt update
sudo apt install -y git build-essential cmake
```

### 3. ROS 2 Humbleのインストール

[ROS 2 Humble公式インストールガイド](https://docs.ros.org/en/humble/Installation.html)に従ってROS 2をインストールしてください。

```bash
# セットアップスクリプトを実行
source /opt/ros/humble/setup.bash
```

### 4. Livox SDKのインストール

[Livox公式GitHub](https://github.com/Livox-SDK/Livox-SDK)からSDKをダウンロードしてビルド・インストールします。

```bash
cd ~
git clone https://github.com/Livox-SDK/Livox-SDK.git
cd Livox-SDK
mkdir build
cd build
cmake ..
make -j$(nproc)
sudo make install
```

**インストール先**: `/usr/local/include` と `/usr/local/lib`（CMakeLists.txtで指定）

### 5. 依存ライブラリのインストール

PCL（Point Cloud Library）、yaml-cpp、およびその他の依存関係をインストールします。

```bash
sudo apt install -y libyaml-cpp-dev
sudo apt install -y ros-humble-pcl-ros
sudo apt install -y ros-humble-sensor-msgs
```

### 6. OpenCV（オプション）

必要に応じてOpenCVをインストールしてください。

```bash
sudo apt install -y libopencv-dev python3-opencv
```

