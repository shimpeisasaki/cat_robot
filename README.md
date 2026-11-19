# CATロボット 機体運用マニュアル

`cat_robot` はモーター、LiDAR、ZED、任意のGNSS、機体モデル、オドメトリ、手動操作、安全停止、任意の画像撮影を管理する。`cat_navigation` は同じリポジトリ内で2D LiDAR SLAM、保存地図上の自己位置推定、Nav2、waypoint巡回を管理する。`cat_navigation` の起動時には機体起動も含まれるため、`cat_bringup` を別途起動しない。

7パッケージの責務と資料の正本は [資料索引](docs/README.md)、
外部指令・手動の走行許可・停止・再armは [制御契約](docs/control.md) を参照する。
現在のビルド対象、3オドメトリの起動入口、ZED画像制御、廃止引数の移行は
[現行起動手順](docs/startup.md)を正本とする。
ハードウェア起動と走行操作は、その都度明示的な実機操作の許可がある場合だけ実施する。

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
# 次にdocs/startup.mdの明示ソース指定でbuildし、そのinstallをsourceする。
```

`dependencies_lidar.repos` はLiDARドライバとemcl2だけを取得する。ZEDやGNSSも使用する場合は
`dependencies.repos`の対象とZED SDK・CUDAも用意する。現行wrapperにはnested
`d1vision_interfaces`も必要なので、[現行ビルド対象](docs/startup.md#前提とビルド)を使用する。
Nav2とslam_toolboxはHumbleの依存として用意する。機器・依存導入は今回の検証では実施していない。

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
| `lidar_source` | `rplidar` | `rplidar` または `horizon`。2D navigationは `rplidar` のみ |
| `odom_source` | `vio` | `vio` または `wheel`。`wheel` は車輪・IMUの推定値を使用 |
| `gnss_mode` | `off` | `off`：無効、`standalone`：単独測位、`rtk`：選択した基準局でRTK（既定は大学CIT） |
| `rtk_source` | `cit` | `cit`：大学内、`matsudo`：公開松戸局 |
| `serial_port` | `/dev/rplidar` | RPLIDARのシリアルポート |
| `rplidar_mount` | `front` | 現在位置 `RPLIDAR_F`。`custom` は `rplidar_x/y/z/roll/yaw`（m、rad） |
| `serial_number` | `10028118` | ZEDのシリアル番号 |
| `enable_joystick`, `rviz` | `true` | 操作入力と表示 |
| `enable_panorama` | `true` | 24方向のRGB画像を保存する撮影アクション。画像合成は行わない |
| `robot_config` | パッケージ内の `robot.yaml` | 機体設定ファイル |

ZEDを使わない構成では、`use_zed:=false odom_source:=wheel` を指定する。ZEDとGNSSを使う構成には、それぞれのROSパッケージが必要である。GNSSは `gnss_mode:=standalone`、大学CITを使うRTKは **`gnss_mode:=rtk` の一設定**で有効にする。RTKはNTRIPクライアントと受信機への補正入力もまとめて起動する。松戸局へ切り替える場合は `rtk_source:=matsudo` を追加する。大学の認証情報はGit管理外の `~/.config/ublox_dgnss/ntrip_cit.json` から読む。必要に応じて `gnss_serial:=...` を指定する。`mapping.launch.py`・`navigation.launch.py`からも同じ引数を使える。インターネット接続が必要で、RTK Fixは実機未確認。GNSSデータのNav2への融合は、この起動には含まれない。

### 現行LiDARと取付位置

2D地図作成・自己位置推定・navigationは前方RPLiDARの `/scan` を使う。
Horizonの起動、時刻対応、IMU補正とVIOの条件は [cat_bringup](cat_bringup/README.md) に記載する。

`RPLIDAR_F` はbase_linkから **前方3.19 mm**、Y=0、高さ254.35 mm、
roll=0、yaw=180°である。現URDFと
[2026-11-05の採用校正](../robot_sensor_tests/docs/2026-11-05_calibration.md)
に一致する。旧説明の33.19 mmは30 mm移動前の値であり、今回物理パラメーターは変更していない。
`custom` は `RPLIDAR_custom` フレームを使い、ドライバの `inverted` は `false` のまま、
角度の反転はTFと車体セクタフィルタで扱う。

VLP-16と後方RPLiDARの起動・機種選択・設定・専用依存は廃止した。
旧取付値、由来と履歴の参照先は [保守記録](docs/maintenance.md) に残す。

`RPLIDAR_F`（`rplidar_mount:=front`）の近距離除外は車体前方側270°を
0.2 m未満、後方側90°を0.6 m未満とする。距離はLiDAR原点基準。
`custom` は全周0.2 m未満とする。これはソフトウェアのマスク値であり、
センサの近距離測定能力を保証する値ではない。

`RPLIDAR_F` は車体に遮られる後方を未観測として扱うため、車体基準の後方90°を
測距値にかかわらずNaNにする。フィルタはC++ドライバ内で適用し、`/scan`を直接出す。
設定と校正時の無効化は[LiDARフィルタの正本](cat_bringup/README.md#lidar-filter)を参照。
bringup引数 `rplidar_front_blind_rear_angle_deg`（既定90）で変更できるが、
角度は実際の記録で検証する必要がある。customにはこの死角マスクを自動適用しない。

地形処理と復帰navigationの現行契約は
[recovery_navigation](../recovery_navigation/recovery_navigation/README.md) が所有する。
壁追従を含む比較アルゴリズムは外部指令interfaceを使い、catの共通駆動・安全部に依存を追加しない。

## 操作と停止

起動時は停止モードである。ジョイスティックの割当ては、Xが手動、Aが外部指令、Bが停止、Yが停止確認後のフリーである。手動モード中のRBは24方向のRGB画像の撮影を開始する。保存先は `~/ros2_ws/data/d1_vision` で、画像合成は行わない。手動走行には右スティックを使用する。モードの切替時と通信異常時は停止モードに戻る。復旧後に自動で走行を再開しない。撮影後も停止モードになるため、再び手動で走るにはXを押す。

手動の右スティックは縦で前後速度、横で旋回曲率を指定する。横だけではその場旋回せず、縦横を同時に倒すと旋回する。最大曲率は3.333333 1/mで、定常時の最小旋回半径は0.30 mである。縦入力を最大にしたときの速度上限は直進・後進で1.0 m/s、横入力最大の旋回で0.6 m/sとなり、横入力に応じて滑らかに変わる。縦入力が小さければ、その割合に応じて速度も下がる。最大角速度は約2.0 rad/s。スティックの不感領域は `joy_node` の5%を一度だけ適用する。

走行許可・状態・停止・手動モード・再armは `robot_control` にまとめた。
引数・前提・停止方法は [制御契約](docs/control.md#手動ツール) を正本とする。
`arm` は許可のみを行い、ゴールや速度指令を生成しないが、既存ゴールがあれば動き得る。

```bash
ros2 run cat_bringup robot_control help
ros2 run cat_bringup robot_control status
ros2 run cat_bringup robot_control brake
```

状態の確認には `/base_safety/status` と `/base/state` を使用する。`/base/state` のbit 0は両輪の通信正常、bit 1は両輪の停止を表す。`base_driver` は通信断、速度指令の途絶、動作許可の途絶を検出して停止する。`freewheel_on_shutdown` の初期設定は `true` であり、プロセス終了時はモーターのトルクが解除される。

## 主なトピックと設定

| トピック | 内容 |
|---|---|
| `/wheel/odom` | 車輪RPMから生成したオドメトリ |
| `/wheel_odom` | 車輪・IMU EKFによる推定値 |
| `/odom` | 選択した推定値。`odom → base_link` のTFも配信 |
| `/scan` | ドライバ内でフィルタしたLiDAR |
| `/zed/zed_node/rgb/color/raw/image/compressed` | `sensor_msgs/msg/CompressedImage`、未補正JPEG、品質90・672×376。起動時OFF、grab30fps・worker上限24Hz。実測周期は負荷で変動。VIO/IMUとは独立 |
| `/base/state`、`/diagnostics` | モーター状態、電流・温度・異常の診断 |
| `/cmd_vel_external`、`/cmd_vel_teleop`、`/cmd_vel_panorama` | 外部走行指令、平滑化済み手動指令、撮影旋回指令 |
| `/cmd_vel_safe`、`/base_safety/mode` | 安全制御後の指令、ドライバへの動作許可 |

機体寸法、モーターID、方向、シリアル設定は `cat_bringup/config/robot.yaml` に集約する。手動操作は `manual_control.yaml`、RPLIDARの近距離除外は `scan_filter.yaml`、車輪・IMU推定は `ekf.yaml` で設定する。ZEDの設定は `zed_vio.yaml` と `zed_sensors.yaml` を使い分ける。通常運転では生RPMトピックを公開しない。

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
| `/scan` がない | LiDARドライバ、`scan_filter.yaml`の設定 |
| 走行が許可されない | `/base_safety/status`、`/base/state`、ジョイスティックのB/Y入力 |

Nav2・地図・waypointに関する確認は [cat_navigation/README.md](cat_navigation/README.md) を参照する。ZEDを使わない地図作成は `ros2 launch cat_navigation mapping.launch.py`、保存地図でのナビゲーションは `ros2 launch cat_navigation navigation.launch.py` で起動する。両方とも既定で車輪オドメトリと2D LiDARを使用する。

ノード間の制御状態は `/base_safety/control_state`（`cat_control_interfaces/msg/ControlState`）を使用する。`mode` はブレーキ0・フリー1・走行2、`source` は `navigation`・`manual`・`panorama` で、停止時は空になる。`/base_safety/status` は従来どおり表示用文字列を配信する。

回帰テストは `colcon test --packages-select cat_bringup cat_capture cat_navigation` で実行できる。

任意の操作画面として [cat_operator_gui](cat_operator_gui/README.md) を利用できます。従来のCLI操作でも運用できます。

## ライセンス

本リポジトリは [Apache License, Version 2.0](LICENSE)（SPDX識別子: `Apache-2.0`）で提供する。外部依存パッケージには、それぞれのライセンスが適用される。
