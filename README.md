# CATロボット 機体運用マニュアル

`cat_robot` はモーター、LiDAR、ZED、任意のGNSS、機体モデル、オドメトリ、手動操作、安全停止を管理する。Nav2、SLAM、自己位置推定、waypoint巡回は [experiment_cat](https://github.com/shimpeisasaki/experiment_cat) が管理する。`experiment_cat` の起動時には本パッケージの機体起動も含まれるため、`cat_bringup` を別途起動しない。

## 新規PCへの導入

ROS 2 Humble、`colcon`、`vcstool`、`rosdep` を用意する。ZEDを使用するPCには、対応するZED SDKと実行環境も必要である。以下はワークスペースを `~/ros2_ws` とする例である。

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/shimpeisasaki/cat_robot.git
git clone https://github.com/shimpeisasaki/experiment_cat.git
cd ~/ros2_ws
vcs import src < src/cat_robot/dependencies.repos
vcs import src < src/experiment_cat/dependencies.repos
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to experiment_cat
source install/setup.bash
```

`dependencies.repos` は、LiDAR、ZED、GNSS、emcl2のソースをワークスペースの `src/` に取得するための設定である。各端末でROSコマンドを使用する前に、ROS 2とワークスペースの `setup.bash` を読み込む。

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

主な起動引数は次のとおりである。

| 引数 | 既定値 | 用途 |
|---|---|---|
| `use_base`, `use_lidar`, `use_zed` | `true` | 駆動部、LiDAR、ZEDの有効化 |
| `odom_source` | `vio` | `vio` または `wheel`。`wheel` は車輪・IMUの推定値を使用 |
| `use_gnss` | `false` | GNSSの有効化 |
| `serial_port` | `/dev/rplidar` | LiDARのシリアルポート |
| `serial_number` | `10028118` | ZEDのシリアル番号 |
| `enable_joystick`, `rviz` | `true` | 操作入力と表示 |
| `robot_config` | パッケージ内の `robot.yaml` | 機体設定ファイル |

ZEDを使わない構成では、`use_zed:=false odom_source:=wheel` を指定する。GNSSを使用する場合は `use_gnss:=true` と必要に応じて `gnss_serial:=...` を指定する。GNSSデータのNav2への融合は、この起動には含まれない。

## 操作と停止

起動時は停止モードである。ジョイスティックの割当ては、Xが手動、Aが外部指令、Bが停止、Yが停止確認後のフリーである。手動走行には右スティックを使用する。モードの切替時と通信異常時は停止モードに戻る。復旧後に自動で走行を再開しない。

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
| `/base/state`、`/diagnostics` | モーター状態、電流・温度・異常の診断 |
| `/cmd_vel_external`、`/cmd_vel_teleop` | 外部走行指令、平滑化済み手動指令 |
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

Nav2・地図・waypointに関する確認は [experiment_cat のREADME](https://github.com/shimpeisasaki/experiment_cat) を参照する。

## ライセンス

本リポジトリは [Apache License, Version 2.0](LICENSE)（SPDX識別子: `Apache-2.0`）で提供する。外部依存パッケージには、それぞれのライセンスが適用される。
