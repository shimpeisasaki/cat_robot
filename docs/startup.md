# 現行CAT・ZEDのビルドと起動

2026-11-19の入出力整理後の手順です。以下の機器起動・サービス操作は運用時の説明であり、
今回の検証では実行していません。今回の証拠は合成入力・模擬HTTP・localhost ROSに限定します。
校正・過去の実機評価は[保守記録](maintenance.md)と各日付付き記録に残します。

## 前提とビルド

実行場所は`~/ros2_ws`です。ROS 2 Humble、colcon、C++/Python開発環境、
Nav2、robot_localization、slam_toolbox、PyQt5（GUI使用時）と各package.xmlの依存が必要です。
ZED使用時は対応するZED SDK・CUDA・image_transport/compressed pluginが必要です。
今回の環境はSDK 5.4.1を使用し、SDK・CUDA・依存の導入や更新は行っていません。
D1の型はモデル不要ですが、実推論には[D1モデル準備](../../d1vision_ros2/docs/D1_MODEL_SETUP_JA.md)
の外部serverが別途必要です。CATやwrapperがモデルserverを起動することはありません。

既存overlayを読み込んでいない端末で、checkoutしたソースを明示します。
外側のD1 packageで探索が止まるため、nested interfaceも必ず指定します。
次は前方RPLiDAR・ZED・GNSS・GUI・D1を含むビルドで、機器は起動しません。
出力先が既存なら別名を選び、他のbuildと共有しないでください。

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
export CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2
colcon --log-base log/cat_current build --executor sequential \
  --build-base build/cat_current --install-base install/cat_current \
  --base-paths src/cat_robot src/d1vision_ros2 src/d1vision_ros2/d1vision_interfaces \
    src/zed-ros2-wrapper/zed_components src/zed-ros2-wrapper/zed_wrapper \
    src/sllidar_ros2 src/emcl2_ros2 src/ublox_dgnss src/gnss2map \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source install/cat_current/setup.bash
```

Horizonを使用する場合は別途`livox_ros2_driver`・`livox_interfaces`・`livox_sdk_vendor`が
必要です。既存Livox SDKがないとvendorの通常buildは外部取得を行います。
依存導入を禁止した今回のfresh検証ではこれらを対象に含めず、旧installも借用していません。
Horizonの導入・起動条件は[cat_bringup](../cat_bringup/README.md#livox-horizon-単体起動)を参照してください。
Horizon経路まで受け入れ済みとは扱いません。

各運用端末でも、workspace rootでHumble、上記installの順にsourceします。
検証用の正確な出力先・SHA・実行記録は[今回の検証](validation/cat_zed_refactor_20261119.md)を参照してください。

## オドメトリと起動所有権

以下は**実機ノードを起動する**コマンドです。実行場所は上記workspace root、
前提は対応機器の接続・設定・明示的な運転許可です。いずれも走行armとは別です。
bringup、mapping、navigationのうち所有元を一つ選び、重ねて起動しません。

| 構成 | bringupでの指定 | 入力と出力 |
| --- | --- | --- |
| wheel＋IMU | `odom_source:=wheel odom_pose_mode:=planar` | `/wheel/odom`と`imu_guard`経由の`/imu/ekf`を平面EKFへ。選択出力は`/odom` |
| 平面VIO | `odom_source:=vio odom_pose_mode:=planar` | `/zed/zed_node/odom`からbaseへ変換し、z/roll/pitchを除く |
| 空間VIO | `odom_source:=vio odom_pose_mode:=spatial` | `/zed/zed_node/pose_with_covariance`の重力基準WORLD姿勢。初期並進/yawのみ原点化し、高さ・傾きを保持 |

3構成とも`odom_source`が`odom -> base_link`の単一TFを所有します。
wrapperのodom/map TFは無効のままです。wheel＋spatialは拒否します。
wheel＋IMUには新鮮な`/zed/zed_node/imu/data`とbase_linkへのTFが必要です。
ZEDを使わない場合、同じ入口へ適切なIMUを外部供給する必要があります。
供給しない場合の従来の`wheel_only`退化は保持していますが、wheel＋IMU成立とは呼びません。
状態は`/localization/imu_status`で区別します。自動fallbackの方式や閾値は変更していません。

```bash
# wheel＋IMU。ZEDはROS gyroを出すwheel profileを選択する。
ros2 launch cat_bringup bringup.launch.py odom_source:=wheel odom_pose_mode:=planar
# 平面VIO（bringup既定: use_zed=true, odom_source=vio, odom_pose_mode=planar）
ros2 launch cat_bringup bringup.launch.py odom_source:=vio odom_pose_mode:=planar
# 空間VIO。mapping/navigationやGUIにはこのpose mode引数を追加していない。
ros2 launch cat_bringup bringup.launch.py odom_source:=vio odom_pose_mode:=spatial
```

mapping/navigationはbringupを含み、既定は`use_zed=false odom_source=wheel`です。
wheel＋IMUを使うなら`use_zed:=true odom_source:=wheel`、平面VIOなら
`use_zed:=true odom_source:=vio`を指定します。`odom_pose_mode`は両入口とも公開していません。
空間VIOの指定例をこれらの入口へ付け足さないでください。
地図、waypoint、GNSSリセット、停止は[cat_navigation](../cat_navigation/README.md)が正本です。

```bash
# 地図作成。base・センサー・SLAMを所有する。既定のpanoramaはbringup由来で有効。
ros2 launch cat_navigation mapping.launch.py use_zed:=true odom_source:=wheel
# 保存地図navigation。既定panoramaは無効。画像撮影が必要ならenable_panorama:=true。
ros2 launch cat_navigation navigation.launch.py use_zed:=true odom_source:=vio
```

GUIの設定キー・保存済みpresetは維持します。GUIで選択できる範囲と終了時の所有権は
[GUI README](../cat_operator_gui/README.md)を参照してください。

## ZED設定と画像のON/OFF

VIO/wheel profileの共通値は`zed_common.yaml`へ集約しました。
CAT supervisorがprofileの明示参照を解決し、一枚のoverrideをwrapperへ渡します。
独自の一枚ROS YAMLは従来の`zed_config:=/absolute/path/custom.yaml`で選べます。
詳細と優先順位は[cat_bringupの設定契約](../cat_bringup/README.md#3構成と設定の読み込み)を参照してください。

CATは`/zed/zed_node/rgb/color/raw/image/compressed`、
`sensor_msgs/msg/CompressedImage`、Reliable/Volatile/depth1を使います。
rawは未補正視野の意味で、非圧縮ではありません。元撮影stampと
`zed_left_camera_frame_optical`を保持します。品質90、672×376、grab30fps、
worker上限24Hzを維持します。要求周期は実測周期の保証ではありません。
有限正数かつ撮影周期以上が必要で、OFF要求にも有効周期が必要です。
wrapperの汎用Image/rectified/gray/left/right経路はCAT profile以外では維持します。

画像は起動・再起動時ともOFFです。購読、撮影、GUI監視・録画だけではONになりません。
VIOと必要なIMUは画像OFFでも継続します。制御元を次のいずれか一つにします。

- D1使用時: [D1 manager](../../d1vision_ros2/docs/CONTINUOUS_D1_JA.md)が
  設問ID付き`/d1/set_task`を受け、wrapperへ周期を指定してON/OFFし、JPEGを
  復号・再圧縮せず模擬/実推論先へ送り、元撮影header付き`/d1/result`を配信します。
  別の手動clientを重ねません。現行`area_avoidance`は配線確認に使える既存設問であり、
  探索用設問の完成を意味しません。新設問作成・探索側接続は保留です。
- D1なしの撮影・録画時: [単一手動clientのON/OFF操作](../cat_bringup/README.md)を使用します。
  撮影側へ自動ON/OFFを追加していません。[保存・取消・drain](../cat_capture/README.md)も参照してください。

状態・拒否・DDS遅延到着・再起動OFFの詳細は
[wrapper画像契約](../../zed-ros2-wrapper/IMAGE_STREAM.md)を正本とします。
機体停止・明示再armは[制御契約](control.md)、VIO/時刻停止ラッチ後のstack再起動は
[cat_bringup](../cat_bringup/README.md)に従います。起動・再起動でarmしません。

## 廃止入口の移行

| 旧入口 | 現行 |
| --- | --- |
| `use_gnss:=true` | `gnss_mode:=standalone` |
| `use_gnss:=false` | `gnss_mode:=off` |
| `rplidar_mount:=legacy` | `rplidar_mount:=front` |
| `rgb_transport:=raw`とImage/BGR | JPEG `CompressedImage` topicへ移行 |
| `rgb_transport:=compressed` | 指定不要。明示topicはノードの`rgb_topic`で維持 |

`gnss_mode:=rtk rtk_source:=cit|matsudo`、`rplidar_mount:=custom`、
`localization_map`、地図引数、センサー選択とGUI設定キーは現役です。
GNSSの受信機選択と要求型自己位置リセットは別設定のままです。
旧探索のImage/BGR直接D1呼出しは現行D1 mainと非対応で、互換adapterは復活させません。
camera-free/shared探索まで非対応と一般化しません。
