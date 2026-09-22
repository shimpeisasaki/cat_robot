# cat_robot

CAT差動二輪ロボットの共通基盤。自作パッケージは同じGitリポジトリ、
外部ドライバーは`dependencies.repos`でコミットを固定して管理します。

```
ros2_ws/src/
├── cat_robot/
│   ├── cat_bringup/          センサ・TF・odom・入力切替・スムーサー
│   ├── ddsm115_controller/   モーター通信・差動二輪制御
│   └── dependencies.repos
├── sllidar_ros2/
├── zed-ros2-wrapper/
└── ublox_dgnss/
```

## 別PCへの導入

Ubuntu 22.04 / ROS 2 Humbleが前提です。クローンだけではOS依存やSDKは入りません。
先にNVIDIAドライバー・CUDA・利用するwrapperと互換性のあるZED SDKを導入してください。
ZEDの導入手順: https://www.stereolabs.com/docs/ros2

新規ワークスペースで実行します。既存環境へのimportは、その環境のブランチや独自変更を確認してから行ってください。

```bash
source /opt/ros/humble/setup.bash
sudo apt install git python3-vcstool python3-rosdep python3-colcon-common-extensions
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
git clone https://github.com/shimpeisasaki/cat_robot.git src/cat_robot
vcs import src < src/cat_robot/dependencies.repos
# rosdepを初めて使うPCでのみ実行
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -y
colcon build --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=123
ros2 launch cat_bringup bringup.launch.py
```

aptで提供される`zed_msgs`・`zed_description`などはrosdepで取得します。
`.repos`はGitソースの版を固定しますが、apt/SDK/OSの版までは固定しません。
SDKの導入・GPUの動作確認・USBアクセス権は別PCごとに必要です。

## USBとロボット固有の設定

- `cat_bringup/config/robot_nav2.yaml`のモーターのUSB by-idと左右IDを確認。
- `cat_bringup/launch/zed_sensors.launch.py`の既定シリアル番号は現在のZED Mini用。
- LiDARの`/dev/rplidar`を作るudevルールをインストール。
- モーター・LiDARのシリアルアクセスには`dialout`、GNSSの付属ルールには`plugdev`所属を確認。

```bash
sudo install -m 644 src/cat_robot/cat_bringup/udev/99-experiment-rplidar.rules /etc/udev/rules.d/
sudo install -m 644 src/ublox_dgnss/ublox_dgnss/udev/99-ublox-gnss.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
# 必要に応じてユーザーをグループへ追加し、再ログイン後にUSBを再接続
sudo usermod -aG dialout,plugdev "$USER"
```

## GNSS

forkした`shimpeisasaki/ublox_dgnss`の互換トピック対応コミットを取得します。
通常bringupではGNSSを自動起動せず、必要な実験で別途起動します。

```bash
ros2 launch ublox_dgnss gnss_launch_compatible.launch.py
```

`/gnss/fix`と`/gnss/fix_velocity`が出力されます。

## 既存PCでの移行

パッケージのソース位置が変わったため、旧CMakeキャッシュを再利用せず別のbuild/install先で再ビルドします。
この手順は既存のセンサドライバーが`install/`にビルド済みのPC用です。
旧モーターリポジトリは`.repository_backups/ddsm115_controller_ros2`へ退避してあります。
退避先はCOLCON_IGNOREで探索対象外にしています。

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --build-base build_cat_robot --install-base install_cat_robot \
  --packages-select ddsm115_controller cat_bringup experiment_nav2 --symlink-install
source install_cat_robot/setup.bash
```

Nav2用の`experiment_nav2`は別リポジトリです。通常bringupとnavigationは同時起動しません。
操作方法は[cat_bringup/README.md](cat_bringup/README.md)を参照してください。
