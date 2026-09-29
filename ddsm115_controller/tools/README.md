# モーター保守ツール

通常のbringupを終了し、同じシリアルポートを複数プロセスで開かない状態で使用します。

| ツール | 用途 |
|---|---|
| `motor_test_gui` | 単輪RPM・速度指令、フィードバック、機体設定の編集 |
| `check_motor_id` | モーターIDの確認 |
| `check_motor_channels` | 左右シリアルチャネルの確認 |
| `set_motor_id` | モーターIDの変更 |
| `direct_motor_brake.py` | ROSの制御経路を通さないシリアル直接ブレーキ |

GUIは `ros2 launch ddsm115_controller motor_test_gui.launch.py` で必要なドライバと一緒に起動します。他は `ros2 run ddsm115_controller <名前>` で起動できます。シリアルポート等の引数は各ツール冒頭の定義を参照してください。

`enable_motor_tools:=true` のドライバだけが `/ddsm115/rpm_cmd`、`rpm_fb`、`cur_fb`、`temp_fb`、`error`、`online_id` を有効化します。通常運転では不要です。
