# CAT robot

機体固有のハードウェア、センサ、TF、手動操作を管理します。Nav2・SLAMは別リポジトリのexperiment_catが担当します。

## ビルドと起動

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ddsm115_controller cat_bringup --symlink-install
source install/setup.bash
ros2 launch cat_bringup bringup.launch.py
```

依存ソースは `dependencies.repos` に記載しています。ワークスペースの `src/` に配置してください。

| 起動引数 | 既定値 | 用途 |
|---|---|---|
| `use_base`, `use_lidar`, `use_zed` | `true` | 車輪・LiDAR・ZED/IMUを有効化 |
| `use_gnss` | `false` | u-blox GNSSを有効化 |
| `gnss_serial` | 空 | GNSS受信機のUSBシリアル指定 |
| `odom_source` | `vio` | `vio`または`wheel`。wheelは車輪/IMU EKF |
| `serial_port` | `/dev/rplidar` | LiDARポート |
| `serial_number` | `10028118` | ZED Miniのシリアル番号 |
| `enable_joystick`, `rviz` | `true` | 操作・表示 |
| `enable_motor_tools` | `false` | 校正向け生RPM入出力・個別テレメトリ |

GNSSを含む全センサの起動は `use_gnss:=true`。GNSS配信は `/gnss/fix` と `/gnss/fix_velocity`。測位のNav2への融合やNTRIP接続は含みません。`use_zed:=false`なら`odom_source:=wheel`を指定します。

起動時はブレーキ。ジョイスティックはXで手動、Aで外部指令、Bで停止、Yで停止確認後フリー。右スティックは速度と曲率です。モード切替は停止状態を経由し、通信断から自動復帰しません。

外部指令は `/cmd_vel_external`。ジョイスティックを無効にした場合も自動走行は開始しません。準備後に明示的に許可します。

```bash
ros2 service call /base_safety/arm std_srvs/srv/Trigger '{}'
ros2 service call /base_safety/brake std_srvs/srv/Trigger '{}'
ros2 service call /base_safety/free std_srvs/srv/Trigger '{}'
```

## ファイルの担当

- `cat_bringup/config/robot.yaml`: シリアル、モーターID、方向、車輪寸法、odom共分散の唯一の設定元。
- `cat_bringup/config/manual_control.yaml`: 手動操作と速度平滑化。
- `cat_bringup/config/scan_filter.yaml`: 前方0.1 m・後方0.4 m以内をNaNで除外。
- `cat_bringup/config/ekf.yaml`: 車輪速度とIMU角速度の融合。
- `cat_bringup/config/zed_vio.yaml`, `zed_sensors.yaml`: VIO用／IMU用のカメラ設定。
- `cat_bringup/urdf/`, `meshes/`, `udev/`: 機体モデルとデバイス設定。
- `cat_bringup/launch/`: 公開入口はbringup。他は内部のセンサ・odom構成。
- `ddsm115_controller/src/`: シリアルドライバ、ベースドライバ、曲率操作。
- `ddsm115_controller/tools/`: テストGUI・ID設定・シリアル直接ブレーキ。

ZEDは既定で画像・点群・重複poseを配信しません。カメラを利用するアプリは `zed_config:=...` で必要な配信を有効にできます。

## 通常運転のインターフェース

| トピック | 用途 |
|---|---|
| `/wheel/odom` | 実測RPMを内部で換算した車輪odom。通信異常時は配信停止 |
| `/wheel_odom` | 車輪/IMU EKFの推定。TFは出さない |
| `/odom` | 選択した推定。odom_sourceだけが`odom → base_link` TFを配信 |
| `/scan` | フィルタ済みLiDAR。`/scan_raw`はフィルタ・校正用 |
| `/base/state` | UInt8。bit 0=両輪正常、bit 1=両輪停止。RPM値は配信しない |
| `/diagnostics` | 電流・温度・異常の1 Hz診断 |
| `/cmd_vel_external`, `/cmd_vel_teleop` | 外部／平滑化済み手動指令 |
| `/cmd_vel_safe` | base_safetyが選んだドライバ入力 |
| `/base_safety/mode` | ドライバの有効期限付き動作許可。0=停止、1=フリー、2=走行 |

`base_driver`が速度→RPM→シリアル→車輪odomを一括処理します。通常運転でRPMをノード間転送しません。ドライバは通信異常・指令途絶・許可途絶を検出し、フリーへの切替前には実測停止も確認します。終了時のフリー動作は `freewheel_on_shutdown` で設定します。

## 保守GUI

通常のbringupを終了してから起動してください。GUIは直接モーターを操作します。

```bash
ros2 launch ddsm115_controller motor_test_gui.launch.py
```

この起動だけは生RPM操作と個別テレメトリを有効にし、GUIがブレーキ／フリーを操作します。設定編集も上記の共通 `robot.yaml` に保存します。ID確認・変更、直接ブレーキは `tools/README.md` を参照してください。

## 移行

旧`velocity_control`と`two_wheels_robot`は`base_driver`へ統合しました。旧`robot_nav2.yaml`は`robot.yaml`、`zed_vio_test.yaml`は`zed_vio.yaml`へ変更しました。旧`command_mux`・`navigation_safety`の物理停止処理は`base_safety`に統一しています。

旧installには削除済み実行ファイルが残る場合があります。別環境で切り替える際は対象3パッケージ（ddsm115_controller、cat_bringup、experiment_cat）のbuild/installを作り直してください。
