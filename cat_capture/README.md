# cat_capture

機体を旋回させ、ZEDのRGB画像を指定枚数だけ保存するROS 2アクションです。画像合成は行いません。このパッケージとアクション定義 `cat_panorama_interfaces` は `cat_robot` リポジトリに含まれます。

## 起動と操作

`cat_bringup` では既定で有効です。次のように起動します。

```bash
ros2 launch cat_bringup bringup.launch.py
```

Xで手動モードにした後、RB（既定のボタン番号5）を押すと停止し、2秒の静止確認後に360度・24枚の旋回撮影をします。終了後はブレーキモードです。B/X/Yで中止できます。外部ノードからは角度と枚数も指定できます。

```bash
ros2 action send_goal /panorama/capture \
  cat_panorama_interfaces/action/CapturePanorama '{frames: 0, angle_deg: 0.0}' --feedback

ros2 action send_goal /panorama/capture \
  cat_panorama_interfaces/action/CapturePanorama '{frames: 10, angle_deg: 120.0}' --feedback
```

`frames: 0` は設定枚数の24枚、`angle_deg: 0` は360度撮影です。`angle_deg` を指定した場合、開始時の向きを中心に、右へ半分の角度だけ旋回してから左へ指定角度を撮影し、最後に開始時の向きへ戻ります。方向を反転する前には減速・停止し、車輪の停止フィードバックを0.5秒確認します。例えば120度・10枚なら各12度区間の中央で撮影します。`frames` は3～255、`angle_deg` は0より大きく360以下です。結果の `path` は保存先ディレクトリです。同時に実行できるアクションは1件です。

旋回の最低指令は `minimum_angular_speed`（既定0.15 rad/s）です。小さすぎる指令がモータの丸め・摩擦で動かず、最後の数度で撮影が時間切れになるのを防ぎます。

探索から使う場合はアクションの `recovery_observation: true` を指定します。この場合は既存のナビゲーション制御モードを維持し、旋回指令を `/recovery/observation_command` へ送ります。`recovery_navigation` 側の観測許可、旋回クリアランス、入力の鮮度、機体の安全ゲートが必要です。完了後もナビゲーションモードを維持します。通常のRB操作と上記のアクション要求は従来どおり撮影モードへ切り替え、終了後にブレーキをかけます。

## 保存先と依存関係

既定の保存先は `~/ros2_ws/data/d1_vision/YYYYMMDD_HHMMSS_角度_枚数/` です。画像時刻に対応する `/odom` の向きから、最初の保存画像を0度とした角度を求め、`f00_000.0deg_rgb.jpg` のように保存します。JPEG入力の元バイト列を保存し、品質の変更・再圧縮はしません。角度は0.1度単位に丸めます。`panorama.png` は生成しません。保存先の親ディレクトリはlaunch引数 `panorama_output_dir:=/path/to/data` で変更できます。

`panorama_rotate` は `/odom`、`/base/state`、`/base_safety/control_state` を監視し、専用の `/cmd_vel_panorama` を通じて旋回します。`panorama_capture` はZEDの `/zed/zed_node/rgb/color/raw/image/compressed`（CompressedImage、Reliable）を使用します。
入力はJPEG専用です。`rgb_transport`、旧Image/BGR入力、再圧縮経路は廃止しました。
ノードの`rgb_topic`パラメータによる明示指定も`CompressedImage`のJPEG topicとして扱います。
依存・ビルド順は[現行起動手順](../docs/startup.md)を参照してください。

撮影済みの各JPEGのファイル名、画像時刻、画像フレーム、補間した `/odom` 位置と向きは `frames.json` に保存します。探索はこの記録から各画像の撮影位置を復元します。

撮影完了時に `/panorama_capture/status` で保存先を通知します。旧探索の24枚D1判定への接続は
現行D1では非対応・保留です。撮影サービス・actionは維持します。JPEG保存はオフライン検証、
旋回走行と実カメラの統合は実機未検証です。

開始サービスがタイムアウトした場合は遅れて届く応答を追跡し、成立した撮影セッションを取り消します。取り消しが確認されるまでは、新しい撮影要求を受け付けません。

## 画像配信とJPEG入力

撮影用サービス・アクションはCATに残し、旋回制御・安全条件は変更しません。
ZEDは起動・再起動時に画像OFFなので、撮影前に単一の制御元で配信ONが必要です。
[CATのZED配信操作](../cat_bringup/README.md)に従い、撮影・購読だけでONになるとは扱わないでください。
D1管理ノードが画像制御元の場合、別の利用者からON/OFFを重ねないでください。

JPEGは形式・先頭末尾マーカーを検査し、保存時の全画素復号は行いません。
`frames.json`には従来の撮影時刻・frame・odomに加え、`image_storage: source_jpeg`と
`image_format`を保存します。録画時のJPEG品質は推測しません。
既存の撮影データや評価条件は書き換えません。
移行は[オフライン検証](../docs/validation/jpeg_input_20261118.md)のみで、旋回走行・実カメラは未検証です。
