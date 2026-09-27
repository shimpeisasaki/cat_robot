# ddsm115_controller

- `base_driver.cpp`: 速度指令、シリアル交換、停止制御、odom・診断配信。
- `motor_control.cpp`: DDSM115のシリアルプロトコル。
- `motor_channel.hpp`: モーターチャネルの所有と終了時処理。
- `wheel_odometry.hpp`: 差動二輪の指令変換・車輪積分。
- `safety_lease.hpp`: 動作許可の期限と再許可。
- `curvature_teleop.cpp`: ジョイスティックから曲率速度指令への変換。
- `tools/`: 保守GUI、ID確認・設定、直接ブレーキ。

機体設定は `cat_bringup/config/robot.yaml` に集約しています。
通常の起動とトピックは[リポジトリREADME](../README.md)、保守は[tools](tools/README.md)を参照してください。
