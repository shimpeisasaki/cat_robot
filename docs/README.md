# cat_robot 資料索引と責務

起動・接続・設定の入口は [README](../README.md)。7パッケージの境界は以下とする。

| パッケージ | 所有する責務 | 正本・設定 |
| --- | --- | --- |
| `ddsm115_controller` | シリアル通信、車輪駆動、車輪odom、手動曲率入力、driver lease | [README](../ddsm115_controller/README.md)、[保守ツール](../ddsm115_controller/tools/README.md) |
| `cat_bringup` | 機体モデル、センサー起動、Horizon時刻、IMU補正、VIO/wheel選択、安全指令選択 | [README](../cat_bringup/README.md)、`cat_bringup/config/`、[制御契約](control.md) |
| `cat_control_interfaces` | brake/free/driveと指令元の構造化状態 | [`ControlState.msg`](../cat_control_interfaces/msg/ControlState.msg) |
| `cat_capture` | RGB保存、撮影旋回とactionの寿命、画像時刻とodom metadata | [README](../cat_capture/README.md) |
| `cat_panorama_interfaces` | 撮影actionの要求・結果・進捗 | [`CapturePanorama.action`](../cat_panorama_interfaces/action/CapturePanorama.action) |
| `cat_navigation` | 2D SLAM、emcl2、Nav2、地図・waypoint、navigation readiness | [README](../cat_navigation/README.md)、`cat_navigation/config/` |
| `cat_operator_gui` | 上記公開interfaceを使う操作・表示、オフライン地図作成UI | [README](../cat_operator_gui/README.md) |

`mapping.launch.py` と `navigation.launch.py` は機体起動を含む。
bringupを重ねて起動しない。cat_navigationは基本安全を所有せず、
機体側が制御モードと停止の最終判断を保持する。
壁追従・探索・D1の方式選択は別repoから外部指令interfaceを消費する。

- [外部指令と手動操作](control.md)：トピック、許可、停止、再armの前提。
- [保守と採用履歴](maintenance.md)：検証手順、評価記録、廃止機器、panorama枝の判断。
- [2026-11-05採用校正](../../robot_sensor_tests/docs/2026-11-05_calibration.md)：
  取付値と否定的結果。生データを複製せず元の記録を参照する。

- [要求型GNSSリセットの接続](../cat_navigation/README.md#要求型gnss自己位置リセットの接続)：
  受信機起動・座標変換・リセット有効化の区別。各機能の正本への入口。
- [GNSS実装時のオフライン検証](validation/gnss_reset_20261117.md)、
  [GNSS文書再点検](validation/gnss_docs_review_20261117.md)：日付と対象SHAを持つ別々の記録。
