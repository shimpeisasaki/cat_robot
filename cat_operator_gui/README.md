# Optional CAT Operator GUI

ロボット接続PCで使うPyQt5の操作画面です。通常のCLI運用に追加して使う任意のパッケージです。

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to cat_operator_gui
source install/setup.bash
ros2 run cat_operator_gui cat_operator_gui
```

「運転・地図」でセンサー・手動操作、オンライン地図作成、ナビゲーション、オフライン地図作成を
選んで起動します。地図と経路は別ウィンドウのRVizで確認し、初期位置と目的地もRVizで指定します。
起動後の走行許可は別操作で、既存の`base_safety`サービスを使います。モータ試験GUIは独立です。

設定画面で機能の使用、オドメトリ源、接続ポート、地図・YAML、保存先を選びます。
各操作の「詳細」は実行に使うコマンドと主要な参照ファイルを表示します。起動後は設定欄を変更しても
起動中の引数は変わりません。機体の停止・終了時動作は選択した機体YAMLの設定に従います。
起動時に主要なYAMLのコピーもログに保存し、詳細画面から参照できます。
GUI設定は`~/.config/cat_operator_gui/settings.yaml`、実行ログと設定スナップショットは
`<データ保存先>/gui_sessions/`に保存します。`--settings-file`で設定保存先を変更できます。

「rosbag収録」で標準プリセットを選び、詳細設定でトピックをチェックします。全トピックでは
後から現れるトピックも対象です。独自プリセットも保存できます。収録は`<データ保存先>/bags/`に
保存し、終了時に選択トピックと起動時設定を`gui_recording.json`に記録します。モードと収録は
同時に動かせます。GUIを閉じるとGUIが起動した処理を終了し、bagの書き出しを待ちます。
GUIは外部から起動したプロセスを終了しません。

オフライン地図作成には`/scan`、`/odom`、`/tf`、`/tf_static`を含むbagを選びます。
実機と異なるROS_DOMAIN_ID（既定80）でSLAMと再生を起動し、モータや実機ドライバは起動しません。
記録済みのmap座標系TFを除外し、再生を一時停止状態で起動します。「再開」で開始し、速度変更・
一時停止・停止ができます。「再生停止」はbag再生だけを終了し、作成済み地図を残します。
再生が終わっても地図作成モードは残るため、RVizで確認してから「地図を保存」でYAMLとPGMを
保存します。保存先は`<データ保存先>/maps/<地図名>/`です。
再実行する場合はモードを停止して起動し直します。既存の同名地図は上書きしません。

GPU監視は`nvidia-smi`を使います。NVIDIA GPUやドライバが使えない場合は取得不可と表示します。
状態画面の受信経過時間はトピックの更新確認です。LiDARやカメラを挿しただけの状態と、
ドライバが動いてデータを配信している状態を区別できます。D1の評価ツールはこのGUIに含みません。

```bash
colcon test --packages-select cat_operator_gui
# ハードウェアを起動せず画面だけ確認
ros2 run cat_operator_gui cat_operator_gui --no-ros
```
