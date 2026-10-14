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

## 保存先と依存関係

既定の保存先は `~/ros2_ws/data/d1_vision/mmdd_hh:mm_角度_枚数/` です。`f00_rgb.png` から順にRGB画像を保存します。`panorama.png` は生成しません。保存先の親ディレクトリはlaunch引数 `panorama_output_dir:=/path/to/data` で変更できます。

`panorama_rotate` は `/odom`、`/base/state`、`/base_safety/control_state` を監視し、専用の `/cmd_vel_panorama` を通じて旋回します。`panorama_capture` はZEDの `/zed/zed_node/rgb/color/raw/image` を使用します。ROS依存パッケージは `package.xml` に記載しています。新しいPCでは `cat_robot` を取得し、`rosdep install --from-paths src --ignore-src -r -y` と `colcon build --symlink-install --packages-up-to cat_bringup` で構築できます。

保存に成功したRGB画像は撮影番号とセッション名を付けて `/panorama_capture/frame` にも配信します。オンラインの画像判定はこのトピックから24枚を受け取れます。

この画像配信と点群を使わない撮影は実機未検証です。

開始サービスがタイムアウトした場合は遅れて届く応答を追跡し、成立した撮影セッションを取り消します。取り消しが確認されるまでは、新しい撮影要求を受け付けません。
