# 2026-11-18 CAT画像入力のJPEG移行

旋回撮影のサービス・アクションは`cat_panorama_interfaces`に維持し、
旋回・停止制御を変更せず、`cat_capture`の画像入力だけをJPEG対応した。
元JPEGを再圧縮せず保存し、GUI録画presetとRViz画像表示もcompressed topicを使う。
配信ON/OFFを購読側から自動実行しない。

操作の正本：[cat_capture](../../cat_capture/README.md)、
[cat_bringup](../../cat_bringup/README.md)、[GUI](../../cat_operator_gui/README.md)。

SHA・dirty diff・環境・fixture・正確なコマンド・終了コード・試験件数・install確認・
未確認条件は[D1側の共通検証記録](../../../d1vision_ros2/docs/JPEG_MIGRATION_VALIDATION_20261118.md)に集約。
完全ログはworkspaceの`log/d1_jpeg_migration_20261118/`。
CATの対象HEADは`5f539ffb5872f79a4d3e784e572d9bbb236101b7`、
branchは`feat/d1-image-interfaces`、未コミット差分を含む。
開始時に存在したZED設定とbringup文書の差分を保持した。

オフラインpytestは25 cases成功、0 fail/skip（colcon集計は5 suite結果を加え30）。
インストール先だけでCAT撮影型・JPEG保存・GUI preset・RViz・品質90設定を確認。
実カメラ、表示・収録、旋回走行、D1認識精度は未確認。実機受入ではない。
