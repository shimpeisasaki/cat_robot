# cat_robot

DDSM115差動二輪ロボットのセンサ・オドメトリ・手動／外部制御をまとめたROS 2パッケージ群です。

| パッケージ | 役割 |
|---|---|
| `cat_bringup` | センサ起動、TF、オドメトリ、入力切替、速度平滑化 |
| `ddsm115_controller` | モーター通信・差動二輪制御 |

外部ドライバー（RPLIDAR・ZED・u-blox）は[dependencies.repos](dependencies.repos)でバージョンを固定しています。

## インストール

Ubuntu 22.04 / ROS 2 Humbleを使用します。ZEDには対応するNVIDIAドライバー・CUDA・ZED SDKが必要です。

```bash
source /opt/ros/humble/setup.bash
sudo apt install git python3-vcstool python3-rosdep python3-colcon-common-extensions
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
git clone https://github.com/shimpeisasaki/cat_robot.git src/cat_robot
vcs import src < src/cat_robot/dependencies.repos
sudo rosdep init  # 初回のみ
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -y
colcon build --symlink-install
source install/setup.bash
```

モーターのUSBパス・左右IDは[robot_nav2.yaml](cat_bringup/config/robot_nav2.yaml)で設定します。
LiDAR・GNSSのUSBアクセス設定は以下です。グループ追加後は再ログインし、USBを再接続してください。

```bash
sudo install -m 644 src/cat_robot/cat_bringup/udev/99-experiment-rplidar.rules /etc/udev/rules.d/
sudo install -m 644 src/ublox_dgnss/ublox_dgnss/udev/99-ublox-gnss.rules /etc/udev/rules.d/
sudo usermod -aG dialout,plugdev "$USER"
sudo udevadm control --reload-rules
```

## 起動

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch cat_bringup bringup.launch.py
```

### 共通bringupの引数

`ros2 launch cat_bringup bringup.launch.py 引数:=値`で指定します。

| 引数 | 既定値 | 内容 |
|---|---|---|
| `use_base` | `true` | モーター・オドメトリ・入力選択・スムーサー |
| `use_zed` | `true` | ZED Mini |
| `use_lidar` | `true` | RPLIDAR S1 |
| `enable_joystick` | `true` | ジョイスティック。falseでは外部入力を自動選択 |
| `rviz` | `true` | RViz表示 |
| `odom_source` | `vio` | `vio`：ZED VIO、`wheel`：車輪＋IMUのEKF |
| `serial_number` | `10028118` | ZEDのシリアル番号。使用する個体に合わせて変更 |
| `zed_config` | odom_sourceに応じて選択 | ZED設定YAMLのパス（下表） |
| `robot_config` | `config/robot_nav2.yaml` | モーター・車輪設定 |
| `manual_config` | `config/manual_control.yaml` | 手動操作・スムーサー設定 |

### ZEDの画像・点群

| 設定ファイル／項目 | 内容 |
|---|---|
| `config/zed_vio_test.yaml` | vioの既定。RGB 672×376・30 fps、IMU、VIO。点群・深度画像の配信はOFF |
| `config/zed_sensors.yaml` | wheelの既定。RGB 672×376・15 fps、IMU。VIO・深度処理はOFF |
| `depth.publish_point_cloud: true` | 色付き点群を配信（YAML内） |
| `depth.publish_depth_map: true` | 深度画像を配信（YAML内） |
| `depth.depth_mode: PERFORMANCE` | 点群・深度画像に必要な深度処理を有効化（YAML内） |

点群専用のlaunch引数はありません。`zed_vio_test.yaml`をコピーし、
`depth`配下の`publish_point_cloud`を`true`へ変更して指定します。

```bash
ros2 launch cat_bringup bringup.launch.py zed_config:=/absolute/path/zed_points.yaml
# カメラを使用しない場合
ros2 launch cat_bringup bringup.launch.py use_zed:=false odom_source:=wheel
```

### センサ単体起動

共通bringupで起動済みのセンサを重複起動しないでください。

```bash
ros2 launch cat_bringup zed_sensors.launch.py
ros2 launch cat_bringup rplidar_s1.launch.py
ros2 launch ublox_dgnss gnss_launch_compatible.launch.py
```

| 起動対象 | 引数 | 既定値 | 内容 |
|---|---|---|---|
| ZED | `serial_number` | `10028118` | カメラのシリアル番号 |
| ZED | `zed_config` | `config/zed_sensors.yaml` | ZED設定YAML |
| LiDAR | `serial_port` | `/dev/rplidar` | シリアルデバイス |
| LiDAR | `frame_id` | `laser` | スキャンの座標系 |
| LiDAR | `lidar_rviz` | `true` | 単体起動時のRViz表示 |
| GNSS | `device_serial_string` | 空文字 | 接続する受信機のシリアル番号 |
| GNSS | `frame_id` | `gps` | GNSSの座標系 |
| GNSS | `log_level` | `INFO` | ログレベル |

GNSSは別起動です。共通bringupに`use_gnss`引数はありません。

## 操作

X：手動、A：外部入力、B：ブレーキ、Y：フリー。起動時はブレーキ状態です。
A/X切替時は速度指令をゼロまで平滑化してから入力元を切り替えます。

外部制御は`/cmd_vel`へ`geometry_msgs/msg/Twist`を継続配信します。
0.3秒入力が途絶えると減速します。非選択側の入力は使いません。

## 主なトピック

型名は`パッケージ/msg/型`の`/msg`を省略しています。
対応するセンサ・機能が有効なときに利用できます。

| トピック | 型 | 内容 |
|---|---|---|
| `/scan_raw` | sensor_msgs/LaserScan | LiDARの未フィルタ距離データ |
| `/scan` | sensor_msgs/LaserScan | 車体周辺の除外処理後の距離データ |
| `/zed/zed_node/rgb/color/rect/image` | sensor_msgs/Image | ZEDの補正済みRGB画像 |
| `/zed/zed_node/imu/data` | sensor_msgs/Imu | ZEDの姿勢・角速度・加速度 |
| `/zed/zed_node/odom` | nav_msgs/Odometry | カメラ座標系のVIO（VIO有効時） |
| `/zed/zed_node/point_cloud/cloud_registered` | sensor_msgs/PointCloud2 | 色付き点群（配信有効時） |
| `/zed/zed_node/depth/depth_registered` | sensor_msgs/Image | 深度画像（配信有効時） |
| `/wheel/odom` | nav_msgs/Odometry | 車輪のみのオドメトリ |
| `/wheel_odom` | nav_msgs/Odometry | 車輪＋IMUのEKF推定 |
| `/odom` | nav_msgs/Odometry | 選択したオドメトリを車体基準で出力 |
| `/imu/ekf` | sensor_msgs/Imu | EKFへ渡す検証済みIMU |
| `/gnss/fix` | sensor_msgs/NavSatFix | GNSSの緯度・経度・高度 |
| `/gnss/fix_velocity` | geometry_msgs/TwistWithCovarianceStamped | GNSS速度（東・北・上） |
| `/joy` | sensor_msgs/Joy | ゲームパッド入力 |
| `/cmd_vel` | geometry_msgs/Twist | 外部システムからの速度入力 |
| `/cmd_vel_teleop` | geometry_msgs/Twist | ジョイスティックの速度指令 |
| `/cmd_vel_selected` | geometry_msgs/Twist | 選択された入力・減速指令 |
| `/cmd_vel_smoothed` | geometry_msgs/Twist | 平滑化後の速度 |
| `/cmd_vel_safe` | geometry_msgs/Twist | 車輪へ渡す最終指令 |
| `/ddsm115/rpm_fb` | std_msgs/Int16MultiArray | モーター回転数 |
| `/ddsm115/cur_fb` | std_msgs/Float32MultiArray | モーター電流 |
| `/ddsm115/temp_fb` | std_msgs/Int8MultiArray | モーター温度 |
| `/ddsm115/error` | std_msgs/Int8MultiArray | モーターエラー |
| `/ddsm115/online_id` | std_msgs/UInt8MultiArray | 応答中のモーターID |
| `/command_mux/status` | std_msgs/String | 入力選択・切替状態 |
| `/localization/imu_status` | std_msgs/String | IMU入力の状態 |
| `/tf`, `/tf_static` | tf2_msgs/TFMessage | 座標変換 |

## ライセンス

[Apache License 2.0](LICENSE)。外部ドライバーには各リポジトリのライセンスが適用されます。
