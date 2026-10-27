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

探索から使う場合はアクションの `recovery_observation: true` を指定します。この場合は既存のナビゲーション制御モードを維持し、旋回指令を `/recovery/observation_command` へ送ります。`recovery_navigation` 側の観測許可、旋回クリアランス、入力の鮮度、機体の安全ゲートが必要です。完了後もナビゲーションモードを維持します。通常のRB操作と上記のアクション要求は従来どおり撮影モードへ切り替え、終了後にブレーキをかけます。

## 保存先と依存関係

既定の保存先は `~/ros2_ws/data/d1_vision/YYYYMMDD_HHMMSS_角度_枚数/` です。画像時刻に対応する `/odom` の向きから、最初の保存画像を0度とした角度を求め、`f00_000.0deg_rgb.jpg` のようにJPEG品質95で保存します。角度は0.1度単位に丸めます。`panorama.png` は生成しません。保存先の親ディレクトリはlaunch引数 `panorama_output_dir:=/path/to/data` で変更できます。

`panorama_rotate` は `/odom`、`/base/state`、`/base_safety/control_state` を監視し、専用の `/cmd_vel_panorama` を通じて旋回します。`panorama_capture` はZEDの `/zed/zed_node/rgb/color/raw/image` を使用します。ROS依存パッケージは `package.xml` に記載しています。新しいPCでは `cat_robot` を取得し、`rosdep install --from-paths src --ignore-src -r -y` と `colcon build --symlink-install --packages-up-to cat_bringup` で構築できます。

撮影済みの各JPEGのファイル名、画像時刻、画像フレーム、補間した `/odom` 位置と向きは `frames.json` に保存します。探索はこの記録から各画像の撮影位置を復元します。

撮影完了時に `/panorama_capture/status` で保存先を通知します。オンラインの画像判定は保存済みの24枚を読み込みます。このJPEG保存とオンライン連携は実機未検証です。

開始サービスがタイムアウトした場合は遅れて届く応答を追跡し、成立した撮影セッションを取り消します。取り消しが確認されるまでは、新しい撮影要求を受け付けません。
