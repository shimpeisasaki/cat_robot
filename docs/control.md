# 外部指令・走行許可・停止

正本実装は `cat_bringup/scripts/base_safety` と
`ddsm115_controller/include/ddsm115_controller/safety_lease.hpp`。
機体起動・実機操作は明示的に許可された場合だけ行う。以下は運用手順の記述である。

## 外部クライアントの契約

| Interface | 型・意味 |
| --- | --- |
| `/cmd_vel_external` | `geometry_msgs/msg/Twist`。外部方式はこの入口だけを使う |
| `/base_safety/control_state` | `cat_control_interfaces/msg/ControlState`。reliable/transient_local、20 Hz。BRAKE=0、FREE=1、DRIVE=2 |
| `/base_safety/status` | `std_msgs/msg/String`。表示用。文言を制御判定に使わない |
| `/base/state` | `std_msgs/msg/UInt8`。bit 0=両輪通信・異常状態が正常、bit 1=両輪停止 |
| `/navigation/ready` | `std_msgs/msg/Bool`。`require_navigation=true` のとき必要 |
| `/base_safety/arm` | `std_srvs/srv/Trigger`。明示的な外部走行許可 |
| `/base_safety/brake` | `std_srvs/srv/Trigger`。直ちにゼロ・BRAKE、保留要求も取消 |
| `/base_safety/manual`, `/base_safety/free` | `std_srvs/srv/Trigger`。手動要求・停止確認後のトルク解除 |

`mode=DRIVE` かつ `source=navigation` のとき外部指令を選択する。
`source=manual|panorama` は別所有者で、停止中は空文字列。
control_stateは安全ノードの要求状態であり、driver内部の実動作許可を保証する測定値ではない。
driver lease切れではcontrol_stateがDRIVEでもdriverが停止ラッチを保持し得る。
現在の採用枝に `/base/drive_enabled` はなく、復帰には下記の明示手順を使う。

全Twist成分は有限、`abs(linear.x)<=2 m/s`、`abs(angular.z)<=3 rad/s`、
`abs(linear.y)<=0.001`。通常クライアントは他成分をゼロにする。
許可後に到着した指令だけを使用し、指令齢0.3秒超でゼロを出力する。
これはBRAKEへの遷移ではなく、許可が残れば新しい指令を受けられる。
取消・所有権/走行許可の喪失時はクライアント自身も指令生成を止め、
ラッチ停止が必要なら `/base_safety/brake` を明示的に要求する。
方式の起動・再起動でarmしない。Nav2、壁追従、探索等を同時に配信させない。

`/cmd_vel_external`、`/cmd_vel_teleop`、`/cmd_vel_panorama`、
`/cmd_vel_safe`、`/base_safety/mode` の各publisherが複数になると停止する。
`/cmd_vel_safe` と `/base_safety/mode` は安全ノード専用。
クライアントはdriverの直接指令・lease・RPM経路を使わない。

armの前提：

- BRAKEでfree要求がなく、正常・静止状態が連続2秒以上。
- motor feedbackが0.3秒以内で正常。joystick有効時はJoyが0.5秒以内でB/Yを解除。
- `require_navigation=true` ではreadiness=trueが0.3秒以内。
  センサー・TF・localizationの条件はcat_navigationのguardが所有する。
- 最新の新鮮な外部指令が非ゼロでないこと。arm以前の指令は再生しない。

cat_navigationのguardは `/cmd_vel_external` の唯一のpublisherが
`collision_monitor` であることも要求する。壁追従は既存Nav2起動へ重ねず、
外部方式側が用意するbringup構成・センサー監視を使う。

`/base_safety/request_navigation` はGUI/Aボタン相当の保留handover要求で、
正常静止2秒後に有効化し得る。単なる状態照会・自動再armとして使わない。
driver leaseのタイムアウト・異常はnon-driveを受信するまで復帰しない。
基本安全の時刻異常・通信異常・停止ラッチをクライアント側で迂回しない。

## 手動ツール

Humbleとworkspaceのsetupを読み込んだ端末で使用する。
`ros2 run cat_bringup robot_control help` はROS graphに接続せず用途を表示する。
各実行は1回の照会または明示サービス要求で、自動許可・再試行・速度送信は行わない。

| 引数 | 用途・前提・結果 |
| --- | --- |
| `status` | control_stateを1件表示。publisherがなければ待つ。Ctrl+Cで照会終了 |
| `motors` | motor bitを1件表示。3=正常かつ静止。走行許可そのものではない |
| `brake` | 即時停止要求。Bボタンと同じラッチ停止 |
| `arm` | 上記前提で外部走行を明示許可。既存ゴールがあれば動き得る |
| `manual` | joystick有効時、健康な静止確認後に手動走行を要求。X相当 |
| `free` | 停止確認後にトルク解除。坂で転がり得る。Y相当 |

停止は `ros2 run cat_bringup robot_control brake`。
再armは、まず `brake` を要求して、`status`・`motors` と異常原因を確認する。
外部方式の非ゼロ指令と不要な既存ゴールを止め、正常静止2秒を維持してから、
操作者が別の実行として `arm` を要求する。サービス応答の `success` と `message` を読む。
失敗時は原因を解消し、勝手に再試行しない。
Horizon時刻/VIO停止ラッチはarmでは解除されないため、
[bringupの復旧手順](../cat_bringup/README.md) に従い停止したままstackを再起動する。
freeから直接armは不可。brakeを経由する。
