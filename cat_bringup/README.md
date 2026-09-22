# cat_bringup

Nav2の経路計画・AMCL・SLAMに依存しない、センサ・TF・オドメトリ・制御の共通ROSパッケージです。
`nav2_velocity_smoother`とそのlifecycle managerのみ単独利用します。

```bash
cd ~/ros2_ws
colcon build --packages-select ddsm115_controller cat_bringup --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=123
ros2 launch cat_bringup bringup.launch.py
```

ジョイスティックはデフォルトで有効です。起動時はブレーキ、Xで手動、Aで外部入力、
Bでブレーキ、Yでフリーです。Aは高速手動モードではありません。
入力切替時はゼロ指令をスムーサーへ送り、並進・角速度の出力がゼロになったら切り替えます。
実測RPMの停止待ちや切替用のブレーキは使いません。切替前の指令は破棄します。
Bと入力機器の通信断は通常の切替とは別にブレーキをかけます。

```
/joy → curvature_teleop → /cmd_vel_teleop ─┐
外部システム            → /cmd_vel ────────┴→ command_mux
command_mux → /cmd_vel_selected → velocity_smoother → /cmd_vel_smoothed
command_mux（平滑化出力を監視）→ /cmd_vel_safe → two_wheels_robot
```

外部システムは`/cmd_vel`へTwistを継続配信してください（30 Hz推奨、0.3秒無入力で減速）。
非選択入力へは自動フォールバックしません。各入力は発行元1つに限定します。
`/cmd_vel_safe`は共通基盤だけが発行します。ジョイスティック入力にrawトピックは不要です。
センサ・TF・odomのトピック名は従来と同じです。

```bash
# ジョイスティックなし：外部入力を自動選択（テストプログラム用）
ros2 launch cat_bringup bringup.launch.py enable_joystick:=false
# VIOを使わずwheel + IMUの推定を使う
ros2 launch cat_bringup bringup.launch.py odom_source:=wheel
ros2 topic echo /command_mux/status
ros2 service call /command_mux/external std_srvs/srv/Trigger '{}'
ros2 service call /command_mux/brake std_srvs/srv/Trigger '{}'
```

設定の変更先はこのパッケージの`config/`です。従来の`experiment_nav2 bringup.launch.py`
は互換入口として本launchを呼びます。Nav2のnavigation.launch.pyは引き続き専用の
衝突監視・安全ゲートを使うため、通常bringupと同時起動しないでください。
共通資産はNav2側からも参照します。旧パッケージの設定コピーは過去の実験launchとの互換用です。
本パッケージは`ddsm115_controller`とともに`cat_robot`リポジトリで管理します。
外部ドライバーはリポジトリ直下の`dependencies.repos`で取得します。
