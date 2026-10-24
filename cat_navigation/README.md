# CATロボット ナビゲーション運用マニュアル

`cat_navigation` はNav2、SLAM、emcl2による自己位置推定、地図、waypoint巡回を管理する。機体、センサ、手動操作、安全停止は同じリポジトリの `cat_bringup` が管理する。`navigation.launch.py` と `mapping.launch.py` は機体も起動するため、`cat_bringup` を別途起動しない。

実機確認（2026-10-13）：`use_zed:=true odom_source:=vio` でナビゲーションの走行を完走した。これは1回の完走確認であり、以前発生したRVizフリーズの原因までは特定していない。

## 導入とビルド

ROS 2 Humbleを使用する。リポジトリの取得、依存ソースの導入、デバイス設定は [cat_robotの導入手順](../README.md) を参照する。LiDARドライバとemcl2のソースは `cat_robot/dependencies_lidar.repos` で取得する。

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to cat_navigation
source install/setup.bash
```

新しい端末ではROS 2とワークスペースの `setup.bash` を再度読み込む。地図YAML、対応する画像、waypoint YAMLは `map/` 内で一緒に管理し、別PCへ展開するときも同じリポジトリの変更として共有する。`map/` に新しい地図を追加した後はビルドを実施し、パッケージ共有ディレクトリへ反映する。

## 保存地図を使ったナビゲーション

```bash
ros2 launch cat_navigation navigation.launch.py
```

既定は `use_zed:=false odom_source:=wheel` で、車輪オドメトリと2D LiDARを使う。ZEDのVIOを使う場合は `use_zed:=true odom_source:=vio` を指定する。撮影機能は既定で無効で、必要な場合は `enable_panorama:=true` を指定する。

`lidar_source:=both` でRPLIDARの `/scan` を使いながらVLP-16の `/velodyne_points` も起動できる。地図作成・自己位置推定・安全監視は引き続きRPLIDARの `/scan` を使うため、`vlp16` 単独はこの起動では選択できない。VLP-16は既定で移設後の `VLP16_B`（後方347.981 mm、高さ515.158 mm、上下反転、前傾20°、yaw=+90°）を使い、旧位置 `VLP16_F` に戻すには `vlp16_mount:=front` を指定する。RPLIDARは既定で移設後の `RPLIDAR_B`（後方309.5 mm、高さ497.35 mm、逆さま、yaw=+90°）を使う。旧位置 `RPLIDAR_F` に戻すには `rplidar_mount:=front` を指定する。`legacy` も旧位置の別名として使える。`custom` は `rplidar_x/y/z/roll/yaw` を指定する。

既定では `slam:=false` とし、以下のファイルをパッケージ内の `map/` から読み込む。

| 用途 | 既定ファイル | ROSトピック |
|---|---|---|
| emcl2の自己位置推定 | `map/19F_260928.yaml` と同名のPGM | `/localization/map` |
| Nav2の計画・costmap | `map/19F_260928_Nav.yaml` と同名のPGM | `/map` |
| 巡回ルート | `map/19F_waypoint.yaml` | `/waypoint_route/markers` |

巡回の中間点は半径1.0 m・向き±90°、最終点は半径0.5 m・向き±30°で到達と判定します。
位置と向きが同時に許容範囲内にある必要があります。1点だけのルートには最終点の条件を使います。
進捗判定は20秒以内に基準位置から0.1 m超移動することです。その場旋回は移動距離に含みません。
`waypoint_route` が `/navigate_to_pose` に1点ずつ送り、各ゴールのbehavior treeで
中間点用・最終点用のgoal checkerとDWB controllerを指定します。
到達後の0.2秒待機と、失敗した点を記録して次へ進む動作は維持しています。
通常のRVizゴールと、Nav2の `/follow_waypoints` に直接送るゴールには最終点の条件を使います。
中間点・最終点を使い分ける巡回は `/waypoint_route/start` から開始してください。

地図を変更する場合は `map:=/path/localization.yaml`、`navigation_map:=/path/navigation.yaml`、`waypoints_file:=/path/route.yaml` を指定する。`localization_map` は `map` の別名である。両地図とwaypointは同じ `map` 座標系、解像度、原点に揃える。地図YAMLの `image` には対応する画像ファイルを指定する。

保存地図での起動後、RVizの「2D Pose Estimate」で実際の機体位置と向きを設定する。`/navigation/ready`、`/base_safety/status` とRViz表示を確認し、機体が停止している状態を2秒以上維持してから走行を許可する。

```bash
ros2 service call /base_safety/arm std_srvs/srv/Trigger '{}'
```

`arm` は走行許可のみを行う。Nav2のゴール送信やwaypoint巡回の開始は別操作である。停止にはジョイスティックのBボタンまたは `/base_safety/brake` を使用する。

```bash
ros2 service call /base_safety/brake std_srvs/srv/Trigger '{}'
```

## waypoint巡回

ナビゲーション起動時にYAMLを読み込み、RVizの「Loaded waypoint route」に番号と向きを表示する。読込だけでは走行を開始しない。走行を許可した後、次の順序で開始する。

```bash
ros2 service call /base_safety/arm std_srvs/srv/Trigger '{}'
ros2 service call /waypoint_route/start std_srvs/srv/Trigger '{}'
```

既に走行許可済みの場合、`arm` の再実行は不要である。`/waypoint_route/start` の応答を確認する。状態と進捗は次で表示する。

```bash
ros2 topic echo /waypoint_route/status --qos-durability transient_local
```

巡回の取り消しと機体の停止は、それぞれ次の操作で行う。

```bash
ros2 service call /waypoint_route/cancel std_srvs/srv/Trigger '{}'
ros2 service call /base_safety/brake std_srvs/srv/Trigger '{}'
```

YAMLの形式は次のとおりである。`x` と `y` の単位はm、`yaw` はradとし、配列順に巡回する。ファイルを書き換えた場合はナビゲーションを再起動して読み込み直す。

```yaml
frame_id: map
waypoints:
  - {x: 0.0, y: 0.0, yaw: 0.0}
  - {x: 1.0, y: 0.0, yaw: 1.5708}
```

waypointを使用しない場合は `waypoints_file:=` を空にする。

## ロボットを使わないwaypoint編集

```bash
ros2 launch cat_navigation waypoint_editor.launch.py
```

既定ではナビゲーション用地図と `map/19F_waypoint.yaml` を開く。既存の地点も読み込まれる。RVizの「2D Goal Pose」で地点を順番に指定し、ドラッグ方向で向きを設定する。操作用サービスは別端末で呼び出す。

```bash
ros2 service call /waypoint_editor/undo std_srvs/srv/Trigger '{}'
ros2 service call /waypoint_editor/clear std_srvs/srv/Trigger '{}'
ros2 service call /waypoint_editor/save std_srvs/srv/Trigger '{}'
```

`undo` は最後の地点を取り消し、`clear` は編集中のリストを消去する。ファイルへの反映は `save` 実行時のみである。保存時は同名YAMLを上書きする。別ファイルを編集する場合は `map:=... input:=... output:=...` を指定する。`input:=` を空にすると空のリストから開始する。

## SLAMと地図の保存

手動操作で地図を作る場合は次を起動する。

```bash
ros2 launch cat_navigation mapping.launch.py
```

地図の保存は、ワークスペースを読み込んだ別端末で実行する。

```bash
cd ~/ros2_ws
ros2 run nav2_map_server map_saver_cli -f src/cat_robot/cat_navigation/map/new_map
```

この操作で `new_map.yaml` と地図画像が `src/cat_robot/cat_navigation/map/` に作成される。新しい地図をlaunchの既定値にする場合は、地図YAMLの `image`、launchのファイル名、地図間の座標を確認してからパッケージを再ビルドする。

SLAMしながらNav2を使用する場合は次を起動する。`navigation_map:=` を空にするとNav2もライブSLAM地図を使用する。既定の保存済みNav2地図を併用する場合は、SLAM地図と同じ座標であることを確認する。

```bash
ros2 launch cat_navigation navigation.launch.py slam:=true navigation_map:= waypoints_file:=
```

## 設定と障害時の確認

| ファイル | 主な設定 |
|---|---|
| `config/nav2_params.yaml` | Nav2、local/global costmap、衝突監視 |
| `config/slam_toolbox.yaml` | SLAMのスキャン処理と地図更新 |
| `config/emcl2.yaml` | 保存地図上の自己位置推定 |
| `map/` | 地図YAML・画像、waypoint YAML |

costmapのLiDAR障害物高さ上限はlocal/globalともに0.8 m、スキャン更新監視は0.2秒に設定している。`navigation_guard` は `/scan`、`/odom`、TF、衝突監視を確認し、結果を `/navigation/ready` に配信する。異常時は `base_safety` が停止し、ゴールを取り消す。復旧後の自動再許可は行わない。

| 状態 | 確認箇所 |
|---|---|
| 地図が表示されない | 地図YAMLの `image` と画像ファイル、`map/` のインストール状態 |
| RVizが起動しない | `navigation_rviz` 引数、起動ログ、画面表示環境 |
| 巡回が始まらない | `/waypoint_route/start` の応答、`/waypoint_route/status`、`/base_safety/status` |
| `/scan` の更新警告 | LiDARの接続、`/scan_raw` と `/scan` の配信状態 |
| 走行許可が解除される | `/navigation/ready`、`/base_safety/status`、モーター診断 |

機体側の通信と停止制御は [cat_robotのREADME](../README.md) を参照する。

`navigation_guard` の `map_tf_future_tolerance` は既定で0.3秒です。
EMCLが `transform_tolerance`（既定0.2秒）を加えて配信する `map -> odom` を許容します。
独自のEMCL設定を使う場合は、その未来幅に合わせてこのパラメータを設定してください。
`odom -> base_link` とセンサ入力の未来時刻許容は0.1秒のままです。

## ライセンス

本リポジトリは [Apache License, Version 2.0](../LICENSE)（SPDX識別子: `Apache-2.0`）で提供する。外部依存パッケージには、それぞれのライセンスが適用される。
