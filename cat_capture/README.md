# cat_capture

機体を一周旋回させ、ZEDのRGB画像を指定枚数だけ保存するROS 2アクションです。画像合成は行いません。このパッケージとアクション定義 `cat_panorama_interfaces` は `cat_robot` リポジトリに含まれ、旧 `cat_panorama` は不要です。

## 起動と操作

`cat_bringup` では既定で無効です。撮影時だけ次のように起動します。

```bash
ros2 launch cat_bringup bringup.launch.py enable_panorama:=true
```

Xで手動モードにした後、RB（既定のボタン番号5）を押すと停止し、2秒の静止確認後に旋回撮影します。終了後はブレーキモードです。B/X/Yで中止できます。外部ノードからも同じ撮影を要求できます。

```bash
ros2 action send_goal /panorama/capture \
  cat_panorama_interfaces/action/CapturePanorama '{frames: 0}' --feedback
```

`frames: 0` は設定枚数の12枚、3～255は今回だけの枚数です。結果の `path` は保存先ディレクトリです。同時に実行できるアクションは1件です。

## 保存先と依存関係

既定の保存先は `~/ros2_ws/data/panorama/panorama_mmdd_hhmm_ss/` です。`f00_017.png` のような撮影順・角度付きファイルを保存します。`panorama.png` は生成しません。保存先はlaunch引数 `panorama_output_dir:=/path/to/data` で変更できます。

`panorama_rotate` は `/odom`、`/base/state`、`/base_safety/status` を監視し、専用の `/cmd_vel_panorama` を通じて旋回します。`panorama_capture` はZEDの `/zed/zed_node/rgb/color/rect/image` を使用します。ROS依存パッケージは `package.xml` に記載しています。新しいPCでは `cat_robot` を取得し、`rosdep install --from-paths src --ignore-src -r -y` と `colcon build --symlink-install --packages-up-to cat_bringup` で構築できます。
