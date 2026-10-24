# CATロボット 機体運用マニュアル

`cat_robot` はモーター、LiDAR、ZED、任意のGNSS、機体モデル、オドメトリ、手動操作、安全停止、任意の画像撮影を管理する。`cat_navigation` は同じリポジトリ内で2D LiDAR SLAM、保存地図上の自己位置推定、Nav2、waypoint巡回を管理する。`cat_navigation` の起動時には機体起動も含まれるため、`cat_bringup` を別途起動しない。

## 新規PCへの導入

ROS 2 Humble、`colcon`、`vcstool`、`rosdep` を用意する。以下は2D LiDARだけで地図作成とナビゲーションを行う構成で、ワークスペースを `~/ros2_ws` とする例である。

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/shimpeisasaki/cat_robot.git
cd ~/ros2_ws
vcs import src < src/cat_robot/dependencies_lidar.repos
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to cat_navigation
source install/setup.bash
```

`dependencies_lidar.repos` はLiDARドライバとemcl2だけを取得する。ZEDやGNSSも使用する場合は、これに代えて `dependencies.repos` を使い、ZED SDKも用意する。その場合は `colcon build --symlink-install --packages-up-to cat_navigation zed_wrapper ublox_dgnss` で任意のセンサ用パッケージもビルドする。Nav2とslam_toolboxはROS 2 Humbleの依存パッケージとして導入する。各端末でROSコマンドを使用する前に、ROS 2とワークスペースの `setup.bash` を読み込む。

## 機器と接続の確認

- モーターの電源を入れ、左右モーターのUSBシリアルが `cat_bringup/config/robot.yaml` の `left_usb_dev`、`right_usb_dev` と一致することを確認する。初期設定では左がモーターID 2、右がID 1である。
- LiDARのudevルールは `cat_bringup/udev/99-experiment-rplidar.rules` に置く。適用後、`/dev/rplidar` が現れることと、使用者に `dialout` グループの権限があることを確認する。
- ZEDのシリアル番号は起動引数 `serial_number` の既定値 `10028118` である。別の機体・カメラを使用する場合は実機に合わせる。
- ジョイスティックによる操作を使用する場合は、ROSの `joy_node` がデバイスを読み取れることを確認する。

LiDARのudevルールを新しいPCへ適用する場合は、次を実行する。グループ権限の変更後は再ログインする。

```bash
sudo install -m 644 ~/ros2_ws/src/cat_robot/cat_bringup/udev/99-experiment-rplidar.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo usermod -aG dialout "$USER"
ls -l /dev/rplidar /dev/serial/by-id/
```

機器固有の値は `cat_bringup/config/robot.yaml` と起動引数で管理する。シリアルポートの番号だけを根拠に設定せず、`/dev/serial/by-id/` の識別子を使用する。

## 機体のみの起動

Nav2やSLAMを使わずに、センサと手動操作を起動する場合に使用する。

```bash
ros2 launch cat_bringup bringup.launch.py
```

旋回撮影は既定で有効である。RGB画像を24方向分保存し、画像合成は行わない。操作と保存形式は [cat_capture/README.md](cat_capture/README.md) を参照する。

主な起動引数は次のとおりである。

| 引数 | 既定値 | 用途 |
|---|---|---|
| `use_base`, `use_lidar`, `use_zed` | `true` | 駆動部、LiDAR、ZEDの有効化 |
| `lidar_source` | `rplidar` | `rplidar`、`vlp16`、`both` から起動するLiDARを選択 |
| `odom_source` | `vio` | `vio` または `wheel`。`wheel` は車輪・IMUの推定値を使用 |
| `use_gnss` | `false` | GNSSの有効化 |
| `serial_port` | `/dev/rplidar` | RPLIDARのシリアルポート |
| `rplidar_mount` | `back` | 移設後 `RPLIDAR_B`。`front` は旧位置 `RPLIDAR_F`。`legacy` は `front` の別名。`custom` は `rplidar_x/y/z/roll/yaw`（m、rad） |
| `serial_number` | `10028118` | ZEDのシリアル番号 |
| `enable_joystick`, `rviz` | `true` | 操作入力と表示 |
| `enable_panorama` | `true` | 24方向のRGB画像を保存する撮影アクション。画像合成は行わない |
| `robot_config` | パッケージ内の `robot.yaml` | 機体設定ファイル |

ZEDを使わない構成では、`use_zed:=false odom_source:=wheel` を指定する。ZEDとGNSSを使う構成には、それぞれのROSパッケージが必要である。GNSSを使用する場合は `use_gnss:=true` と必要に応じて `gnss_serial:=...` を指定する。GNSSデータのNav2への融合は、この起動には含まれない。

### VLP-16

ROS 2 HumbleのVelodyne上流パッケージは `src/velodyne` に独立して置く。`dependencies.repos` と `dependencies_lidar.repos` はリリース2.5.1を固定している。ビルド前に `libpcap0.8-dev`（または `libpcap-dev`）を導入し、`colcon build --symlink-install --packages-up-to cat_bringup velodyne_driver velodyne_pointcloud` を実行する。

この機体のVLP-16はセンサIP `192.168.0.201/24`、送信先 `192.168.0.100`、データポート `2368`、回転数600 rpmである。PCのUSB LANアダプタに `192.168.0.100/24` を設定し、ゲートウェイは設定しない。このPCではインターフェース `enx6c1ff7cbb97f` のNetworkManager接続 `VLP16 USB Ethernet` に保存済みで、Wi-Fiがデフォルト経路を維持する。センサ側の値が異なる場合は `cat_bringup/config/vlp16.yaml` を合わせる。

モータを起動せずに点群を確認するには `ros2 launch cat_bringup vlp16.launch.py show_rviz:=true` を使う。`/velodyne_packets` と `/velodyne_points` を配信し、RVizのFixed Frameは `vlp16` に設定される。通常の機体起動では `lidar_source:=rplidar`、`lidar_source:=vlp16`、`lidar_source:=both` を指定する。`both` でもRPLIDARの `/scan` とVLP-16の `/velodyne_points` は別トピックである。`use_lidar:=false` は両方を起動しない。

`base_link` からVLP-16計測原点へのTFは前方0.0186 m、中央、高さ0.207 m、前傾7°である。RPLIDARの旧位置 `RPLIDAR_F` は x=0.03319 m、z=0.24883 m、yaw=180°。移設後 `RPLIDAR_B` は x=-0.3095 m、y=0、z=0.49735 m、roll=180°（逆さま）、yaw=+90°である。yawはRVizで確認した前後の向きに合わせて180°補正した。両位置をTFに表示し、実際の `/scan` のフレームを `rplidar_mount:=front|back` で選ぶ。既定は `back`。`custom` は `RPLIDAR_custom` フレームを使う。ドライバの `inverted` は `false` のままにし、逆さまによる角度の反転はTFと車体セクタフィルタで扱う。

現時点でVLP-16の点群をNav2の `/scan` に変換していない。地図作成、自己位置推定、ナビゲーションは従来どおりRPLIDARの `/scan` を使う。

## 操作と停止

起動時は停止モードである。ジョイスティックの割当ては、Xが手動、Aが外部指令、Bが停止、Yが停止確認後のフリーである。手動モード中のRBは24方向のRGB画像の撮影を開始する。保存先は `~/ros2_ws/data/d1_vision` で、画像合成は行わない。手動走行には右スティックを使用する。モードの切替時と通信異常時は停止モードに戻る。復旧後に自動で走行を再開しない。撮影後も停止モードになるため、再び手動で走るにはXを押す。

外部指令による走行を許可する場合は、停止状態とセンサ・モーター状態を確認してから次のサービスを使用する。`arm` は走行許可のみを行い、ゴールや速度指令は生成しない。

```bash
ros2 service call /base_safety/arm std_srvs/srv/Trigger '{}'
ros2 service call /base_safety/brake std_srvs/srv/Trigger '{}'
ros2 service call /base_safety/free std_srvs/srv/Trigger '{}'
```

状態の確認には `/base_safety/status` と `/base/state` を使用する。`/base/state` のbit 0は両輪の通信正常、bit 1は両輪の停止を表す。`base_driver` は通信断、速度指令の途絶、動作許可の途絶を検出して停止する。`freewheel_on_shutdown` の初期設定は `true` であり、プロセス終了時はモーターのトルクが解除される。

## 主なトピックと設定

| トピック | 内容 |
|---|---|
| `/wheel/odom` | 車輪RPMから生成したオドメトリ |
| `/wheel_odom` | 車輪・IMU EKFによる推定値 |
| `/odom` | 選択した推定値。`odom → base_link` のTFも配信 |
| `/scan`、`/scan_raw` | フィルタ後のLiDAR、フィルタ前のLiDAR |
| `/zed/zed_node/rgb/color/raw/image` | ZEDの未補正RGB画像。内部取得は30 fps、配信目標上限は24 Hz。実際の受信周期は処理負荷で変動する。深度画像・点群の配信は無効 |
| `/base/state`、`/diagnostics` | モーター状態、電流・温度・異常の診断 |
| `/cmd_vel_external`、`/cmd_vel_teleop`、`/cmd_vel_panorama` | 外部走行指令、平滑化済み手動指令、撮影旋回指令 |
| `/cmd_vel_safe`、`/base_safety/mode` | 安全制御後の指令、ドライバへの動作許可 |

機体寸法、モーターID、方向、シリアル設定は `cat_bringup/config/robot.yaml` に集約する。手動操作は `manual_control.yaml`、LiDARの近距離除外は `scan_filter.yaml`、車輪・IMU推定は `ekf.yaml` で設定する。ZEDの設定は `zed_vio.yaml` と `zed_sensors.yaml` を使い分ける。通常運転では生RPMトピックを公開しない。

## モーター保守

通常の機体起動を終了し、モーターのシリアルポートを他のプロセスが使用していない状態で起動する。

```bash
ros2 launch ddsm115_controller motor_test_gui.launch.py
```

このGUIでは生RPMの操作と個別テレメトリを有効にする。ID確認・変更、直接ブレーキの手順は [ddsm115_controller/tools/README.md](ddsm115_controller/tools/README.md) を参照する。

## 障害時の確認

| 状態 | 確認箇所 |
|---|---|
| `No reply from ... motor` | モーター電源、左右のUSB接続、`robot.yaml` のポートとID |
| `/dev/rplidar` がない | USB接続、udevルール、`dialout` 権限 |
| `/scan` がない | LiDARドライバ、`/scan_raw`、`sector_scan_filter` |
| 走行が許可されない | `/base_safety/status`、`/base/state`、ジョイスティックのB/Y入力 |

Nav2・地図・waypointに関する確認は [cat_navigation/README.md](cat_navigation/README.md) を参照する。ZEDを使わない地図作成は `ros2 launch cat_navigation mapping.launch.py`、保存地図でのナビゲーションは `ros2 launch cat_navigation navigation.launch.py` で起動する。両方とも既定で車輪オドメトリと2D LiDARを使用する。

ノード間の制御状態は `/base_safety/control_state`（`cat_control_interfaces/msg/ControlState`）を使用する。`mode` はブレーキ0・フリー1・走行2、`source` は `navigation`・`manual`・`panorama` で、停止時は空になる。`/base_safety/status` は従来どおり表示用文字列を配信する。

回帰テストは `colcon test --packages-select cat_bringup cat_capture cat_navigation` で実行できる。

任意の操作画面として [cat_operator_gui](cat_operator_gui/README.md) を利用できます。従来のCLI操作でも運用できます。

## ライセンス

本リポジトリは [Apache License, Version 2.0](LICENSE)（SPDX識別子: `Apache-2.0`）で提供する。外部依存パッケージには、それぞれのライセンスが適用される。
