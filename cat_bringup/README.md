# cat_bringup

実機のground/frontier探索向けに、`horizon.launch.py timestamp_mode:=host_offset` と
`bringup.launch.py odom_source:=vio odom_pose_mode:=spatial` を追加。
統合起動は [recovery_explorationの実機手順](../../recovery_exploration/HARDWARE_EXPLORATION.md)。
既定は従来どおりHorizonのセンサー時刻保持、オドメトリの平面化。
host_offsetでは元入力を `/livox/lidar_sensor_raw`・`/livox/imu_sensor_raw` に残し、
点群とIMUへ共通の固定offsetを適用する。1秒の初期化後も時計の飛び・逆行を監視し、
配信開始後の異常時は再起動まで配信を停止する。受信時刻に基づく近似であり外部時刻同期ではない。
spatialはZED VIO専用で、SDKの重力基準WORLD姿勢
`/zed/zed_node/pose_with_covariance` を使う。カメラ相対の `/zed/zed_node/odom` は
spatial入力に使わない。取付TFでbaseへ変換し、初期の並進・yawだけを原点化する。
起動時からroll/pitchを保持するため、水平床での開始は必要ない。
`zed_vio.yaml` はIMU融合・重力基準を有効、2D化・area memory・wrapper TF配信を無効にする。
WORLD poseの共分散配信を有効にし、速度は変換後base姿勢の差分から求める。
2026-11-11、カメラ相対odomを重力基準と扱い平地を12度の坂と誤判定する不具合を修正した。
校正済みHorizonの取付TFと地面判定しきい値は変更していない。
ZEDの実取付ずれの測定は別作業で、校正テストは `robot_sensor_tests` に置く。
坂での推定精度は実機未検証。時刻bridge/VIO停止後は機体を止めてstack全体を再起動する。

2026-11-10のオフライン確認：cat_bringupをfocused buildし、`CAT_TEST_HORIZON` を
unsetしてcolcon testを実施。62件中61件成功、明示opt-inの実機power試験1件はskip。
模擬点群/IMUで時刻対応と停止ラッチ、模擬ZED/取付TFで高さ・pitchの配信と
追跡断後の停止を確認した。ハードウェア・シミュレータ・RVizは起動していない。

2026-11-11の実機bringupで、サンプリング開始直後の時刻飛びが初期化を止める事象を確認。
未配信のwarmup中だけ仮のoffsetと両入力履歴を捨て、安定した1秒を待ち直すよう修正した。
配信開始には両入力が0.5秒以内に受信されていることも要求する。
配信開始後の再対応付けは禁止のまま。単体・模擬ROSテスト17件成功。
実機再確認では点群10Hz・IMU約50Hz、点群時刻差約0.10秒で20秒間受信できた。
`/livox/time_status` の `warmup_resets` で初期化待ち直し数を確認できる。

機体起動・設定・TF・センサ・停止制御の所有元です。
起動方法、設定一覧、公開トピックは[リポジトリREADME](../README.md)を参照してください。

## 3構成と設定の読み込み

ビルド・起動順・入口ごとの選択方法は[現行起動手順](../docs/startup.md)を正本とします。
`bringup.launch.py`は`odom_source:=wheel odom_pose_mode:=planar`（wheel＋IMU）、
`odom_source:=vio odom_pose_mode:=planar`、`odom_source:=vio odom_pose_mode:=spatial`を維持します。
wheel＋spatialは拒否します。mapping/navigationとGUIは`odom_pose_mode`を公開していません。
wheelのIMUは`imu_guard`が`/zed/zed_node/imu/data`から受け、[EKF設定](config/ekf.yaml)の
`/imu/ekf`へ渡します。新鮮なgyroとbase_linkへのTFが必要です。
`use_zed:=false`だけでは代わりのIMUは生成されません。

`zed_vio.yaml`と`zed_sensors.yaml`はCAT supervisor用profileです。
`cat_zed_common: zed_common.yaml`はROS/YAMLの暗黙合成ではなく、supervisorが
profile所在ディレクトリから読み、同じsection内ではprofile値を優先して明示的に合成します。
共通部はJPEG品質90、672×376、grab30fps、worker上限24Hz、既存の再接続値を保持します。
supervisorは一枚の一時overrideからtracking有効を判定し、同じfileをwrapperの
`ros_params_override_path`へ渡します。子process groupの終了まで保持し、その後削除します。
wrapper固有の共通・camera設定、CAT override、強制launch値の優先順は変わりません。
このprofileを直接wrapperへ渡さず、CATの`zed_config`から選んでください。
既存の独自一枚ROS YAMLは`zed_config:=/absolute/path/custom.yaml`で従来どおり使用でき、
`cat_zed_common`がなければ合成・書き換え・CAT既定値の注入を行いません。

## GNSS・RTK基準局の切り替え

既存の起動コマンドに **`gnss_mode:=rtk`** を指定すると、受信機と大学CIT局の
NTRIPクライアントをまとめて起動する。既定は `rtk_source:=cit`、接続先は `taroz.net:2101/CIT`。
認証はGit管理外の `~/.config/ublox_dgnss/ntrip_cit.json`（所有者だけ読み書き可能）から読む。
公開松戸局は `rtk_source:=matsudo` で選択できる。接続先は `geortk.jp:2101/Matsudo`、認証は不要。
通常の単独測位は `gnss_mode:=standalone`、GNSS無効は `gnss_mode:=off`。
同じ引数を `cat_navigation` のmapping/navigation launchにも指定できる。
既定は `off`。旧`use_gnss:=true`は`gnss_mode:=standalone`、
旧`use_gnss:=false`は`gnss_mode:=off`へ置き換えます。切り替えはlaunchの再起動時に適用します。

GNSS有効時の測位周期は5 Hz、衛星選別条件は受信機の保存値を使う。
RTKではROSのRTCM転送とUSB RTCM3入力を有効にし、単独測位では無効にする。
既存の `/gnss/fix`・`/gnss/fix_velocity` と `gnss_antenna_link` を使用する。
受信状況・品質は `/gnss/status`。時刻・有効フラグの解釈は
[ublox出力契約](../../ublox_dgnss/docs/gnss_output_contract.md) を参照する。
大学・松戸の各局からの補正受信とオフラインの起動構成は確認済み。
正常受信中は接続を保持し、切断時は30秒から最大300秒へ間隔を延ばして再接続する。
RTKの実機起動・Fix・精度は未確認。受信機起動だけでは自己位置リセットを有効にしない。
保存地図への変換と要求型リセットの接続は
[cat_navigationの正本](../cat_navigation/README.md#要求型gnss自己位置リセットの接続) を参照する。

VLP-16・後方RPLiDARの起動・設定・未使用URDFは廃止しました。
過去の取付値と参照先は [保守記録](../docs/maintenance.md) に残します。
`RPLIDAR_custom` は `rplidar_mount:=custom` のときだけ生成します。

ZEDは `zed_supervisor` がwrapperとコンテナを同じプロセスグループで管理します。
終了時はグループ全体を停止し、応答しない子プロセスにも段階的に終了シグナルを送ります。
同じカメラの二重起動と、既存の `/zed/zed_container`・`/zed/zed_node` がある状態での起動を拒否します。
VIO設定では、起動後60秒間odomが出ない場合、または一度出たodomが10秒途絶えた場合に
wrapper全体を停止して再起動します。`odom_source` が入力断を検出して停止した場合は、
ロボットを停止した状態でnavigation全体を再起動してください。

ZED画像は未補正rawをwrapper内でJPEG化した
`/zed/zed_node/rgb/color/raw/image/compressed`（`sensor_msgs/msg/CompressedImage`）
だけを配信します。VIO/wheel両設定で `video.raw_jpeg_only=true`、品質90です。
非圧縮Image・rect画像の配信口は生成しません。
画像配信は起動・再起動時ともOFFです。VIO/odom、wheel用IMUは
画像停止中も継続します。wheel構成は不要な深度・位置追跡を無効にしています。
JPEGの購読・録画にも明示的なON操作が必要です。
panorama_captureは元JPEGを保存し、GUIの画像録画presetとRVizはcompressed topicを使います。
D1の録画抽出・評価は[JPEG入力手順](../../d1vision_ros2/docs/JPEG_INPUT_JA.md)を参照してください。
これらの購読者は画像を自動ONにしません。
制御元は1つにし、利用終了後はOFFにしてください。SDKのカメラ開始停止ではありません。
サービス型の所有元は `d1vision_interfaces` です。新しいinstallをsourceして実行します。
D1を使う場合の唯一の制御元は[D1 manager](../../d1vision_ros2/docs/CONTINUOUS_D1_JA.md)です。
下記の手動操作はD1を使わない撮影・録画時に、単一clientから行います。

```bash
# 例: 0.2秒周期で未補正rawのJPEG配信を許可（撮影・VIOは継続）
ros2 service call /zed/zed_node/set_image_stream d1vision_interfaces/srv/SetImageStream \
  '{enabled: true, period_sec: 0.2}'
# OFFにも有効な周期が必要
ros2 service call /zed/zed_node/set_image_stream d1vision_interfaces/srv/SetImageStream \
  '{enabled: false, period_sec: 0.2}'
```

周期の上限・拒否条件・OFF応答とDDS到着の関係・ビルド方法は
[ZED画像配信契約](../../zed-ros2-wrapper/IMAGE_STREAM.md)を参照してください。

## Livox Horizon 単体起動

公式 [Livox-SDK/livox_ros2_driver](https://github.com/Livox-SDK/livox_ros2_driver)
を `src/livox_ros2_driver/` に独立したGitリポジトリとして配置しています。
Horizonはこの旧世代SDK用ROS 2ドライバの対応機種です。
`livox_ros_driver2` / SDK2 は別の対応機種向けなので使用しません。
SDKは `livox_sdk_vendor` がビルド領域へ取得・インストールします。
既存の `/usr/local` のSDK2を変更する必要はありません。
機体用のlaunch・設定・RVizはこのパッケージが所有します。

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --base-paths src --symlink-install --packages-up-to livox_ros2_driver cat_bringup
source install/setup.bash
ros2 launch cat_bringup horizon.launch.py show_rviz:=true
```

このlaunchはHorizonドライバ、IMU補正ノード、任意のRVizを起動します。
起動時はStandby/PowerSavingからNormalへ戻して測定し、Ctrl+C終了時は
Standbyへ切り替えます。コマンドのACKだけで成功とせず、実機の
`state=3` を最大4秒待って確認してからSDKを終了します。
確認失敗時は `NOT confirmed` エラーを出します。launchのSIGTERM猶予は8秒です。
`config:=...` でROSパラメータYAML、
`user_config:=...` で機体選択JSONを変更できます。

`horizon.yaml` の `manage_horizon_power: true` がこの動作を有効にします。
`horizon_idle_mode` は `standby`（標準、レーザー停止）または
`power_saving`（通信以外を停止するモード）です。
ただし現在の個体（firmware 6.10.0.0）ではPowerSavingコマンドに成功ACKが
返った後、通信断→初期化→Normalへ戻り、`state=2` を確認できませんでした。
ROSなしの公式SDK単体でも、Normal/Standbyの両方から再現しました。
そのため**現時点で確認済みなのはレーザー停止までで、内部走査モータの
停止は保証できません**。PowerSavingの実運用には原因調査が必要です。

通常終了後はStandbyを保持し、常駐するROSノードは残しません。
電源再投入後の初期モードは未検証です。強制終了（SIGKILL）やPC電源断では
終了処理を実行できないため、自動でStandbyへ移ることは保証しません。

現在の接続:

- USBハブのEthernetアダプタ: `enx6c1ff7cbb97f`。
- Horizon: `192.168.1.105`、broadcast code `3WEDH7600100051`。
- PC: `192.168.1.100/24` をNetworkManagerの既存接続
  `VLP16 USB Ethernet` に追加済み。VLP-16用 `192.168.0.100/24` も維持。
- JSONは上記broadcast codeだけを接続対象に指定します。
  USBハブはLivox製Hubとは異なるため、ドライバは直接LiDARモードです。

| トピック | 型 | 設定 |
|---|---|---|
| `/livox/lidar` | `sensor_msgs/msg/PointCloud2` | 10 Hz、x/y/z/intensity/tag/line |
| `/livox/imu_raw` | `sensor_msgs/msg/Imu` | SDK生値、加速度g単位 |
| `/livox/imu` | `sensor_msgs/msg/Imu` | ジャイロバイアス補正済み、加速度m/s² |

点群とIMUのframeは `livox_frame`。RVizもこのセンサー座標系で表示します。
URDFには `base_link -> livox_frame` の固定ジョイントを追加しています。
位置は `xyz=(-0.27096, 0, 0.43570)` m、姿勢は上下反転・下向き15.570度
（`rpy=(180°, +15.570°, 0)`、前方向き）です。
高さとpitchは2026-11-05の地面点群測定に基づく補正値です。
同日のユーザー判断で今回の取付校正は採用済みです。rollは指定により180度、
Xは壁評価が不合格のため従来値を維持しています。RPLIDAR_FのXは、
30 mm移動後の更新漏れを修正してユーザー指定の+3.19 mmとしました。
測定結果・採用条件・記録先は
[取付校正の採用記録](../../robot_sensor_tests/docs/2026-11-05_calibration.md)を参照してください。
機体bringupの `robot_state_publisher` がこのTFを配信します。
Horizon単体launchはセンサードライバ、IMU補正、任意のRVizを起動します。
モデルと実機点群を一緒に表示する場合は以下を使います。

```bash
ros2 launch cat_bringup bringup.launch.py use_base:=false use_zed:=false \
  gnss_mode:=off use_lidar:=true lidar_source:=horizon \
  enable_joystick:=false enable_panorama:=false rviz:=true
```

この構成ではRVizの固定座標系は `base_link`、`Horizon points` が
`/livox/lidar` を購読し、URDFの反転・傾斜を反映して表示します。
停止時のStandby確認はHorizon単体launchと共通です。
cat_bringupは点群・IMU・TFまでを担当し、地形処理やNav2のアルゴリズムは所有しません。

公式ドライバの生の時刻を保持しています。今回の未同期機ではstampが
機体起動からの相対時刻でした。後続の地形検出に接続する際は、ROS時刻への
対応付けまたはセンサー時刻同期が必要です。取付TFは上記URDFを使用します。
`horizon_imu` が公式IMUの加速度をgからm/s²へ変換し、
`config/horizon_imu.yaml` のジャイロバイアスを引きます。
補正後も重力は残し、姿勢は未推定として `orientation_covariance[0]=-1` を設定します。
生値と補正後は同じセンサー時刻を保持するため、ROS時刻との同期は別途必要です。
`imu_calibration:=...` で別の校正YAMLを指定できます。

静止ジャイロ校正は、Horizonを固定したまま以下を実行します。
60秒ウォームアップ、60秒校正、30秒検証を行い、生CSV・統計JSON・
校正YAMLを指定した新規ディレクトリへ保存します。合格したYAMLのみを
`config/horizon_imu.yaml` に反映し、Horizon launchを再起動します。

```bash
ros2 run cat_bringup horizon_imu_calibrate --output data/horizon_imu/NEW_SESSION
```

倒立でもジャイロの静止バイアスは測定できます。重力を加速度バイアスと
取り違えないため、加速度の平均は引きません。単一姿勢でのこの校正では、
加速度の3軸バイアス・感度・軸間ずれやIMU/LiDAR間の外部パラメータは求めません。

2026-11-03の静止校正: Horizon `3WEDH7600100051` を机上で倒立させ、
校正3,019サンプル、別区間の検証1,512サンプルで合格しました。
ジャイロバイアスは `[0.000799610, -0.000211304, -0.000059982]` rad/s。
検証区間の補正後平均は各軸とも0.004 deg/s未満でした。
生値・統計・校正YAMLは `data/horizon_imu/20261103T214614/` に保存しています。
温度依存・長時間ドリフト・動的精度は今回評価していません。

2026-11-03の確認: 公式ドライバcommit
`1565b976ab1905f911cd2a757fc32433422b701c` をROS 2 Humbleでビルド成功。
実機firmware `6.10.0.0` へ接続し、12秒の受信で点群122メッセージ
（約10 Hz）、IMU 600メッセージ（観測約50 Hz）を確認しました。
点群1フレーム24,000点中10,639点が非ゼロ座標でした。
RVizの起動と点群subscriptionも確認済み。地形検出精度の検証は未実施です。

ローカル修正: 公式ドライバのSIGINTハンドラがSDK・受信スレッドの終了前に
`exit()` して実機接続時のCtrl+Cでクラッシュしたため、ROS 2標準の
シグナル処理を使い、受信待ちを解除してスレッド終了後にSDKを破棄するよう
修正しています。点群・IMUのデータ処理は公式実装を維持しています。
修正後は10秒の再受信（点群93件・IMU467件、両方 `livox_frame`）と
Ctrl+C正常終了を確認し、対象機未接続で受信待ち中の正常終了も確認しました。

待機制御追加後の確認: 実機で起動→25点群受信→Standby状態確認→正常終了を
2回繰り返す回帰テストが成功しました。通常の `cat_bringup` テストも
45件・失敗0件（実機テスト1件は標準ではskip）でした。
実機回帰テストはレーザーを起動するため、明示的に実行する場合だけ以下を使います。

```bash
CAT_TEST_HORIZON=1 PYTHONNOUSERSITE=1 python3 -m pytest -q \
  src/cat_robot/cat_bringup/test/test_horizon_power.py
```

<a id="lidar-filter"></a>

## LiDARフィルタ

RPLiDARは`sllidar_ros2`のC++内でマスクし、`/scan`を直接配信する。
旧Python `sector_scan_filter`と`/scan_raw`へのリマップは廃止した。
機体設定は[config/scan_filter.yaml](config/scan_filter.yaml)の
`sllidar_node.ros__parameters`に置く。`sector_filter_enabled`はCATではtrue、
ドライバ単体ではfalse。変更は起動時に適用され、実行中の設定変更は行わない。

距離・角度設定と座標式は[ドライバ契約](../../sllidar_ros2/README.md#optional-vehicle-sector-filter)を参照。
frontはroll=0/yaw=pi、後方90度をNaN、前方270度は0.2 m未満をNaNとする。
customは取付roll/yawを引き継ぎ、全周0.2 m未満をNaN、死角除外は0度。
取付値はlaunch引数でYAMLを上書きし、rear最小距離と死角もframeに応じて上書きする。
`mounting_roll`/`mounting_yaw`は車体方向の判定だけに使い、配信座標・TFは変えない。

通常のbringupは`rplidar_filter_config`で別YAMLを指定できる。
内部の`rplidar_s1.launch.py`では同じ引数を`filter_config`と呼ぶ。
フィルタ前の測距が必要な校正では、YAMLのコピーで
`sector_filter_enabled: false`にして指定する。走行時の通常設定は有効のまま使う。
校正の手順は[robot_sensor_tests](../../robot_sensor_tests/README.md#rplidarの前後位置だけを校正する)を参照。
過去の`/scan_raw`を含むbag・校正記録は当時のまま保持する。
