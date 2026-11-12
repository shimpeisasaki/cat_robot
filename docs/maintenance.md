# 保守・検証と採用履歴

## 変更と記録

機能・interface・起動引数・設定・操作を変えたら、その正本とREADMEの導線を同じ変更で更新する。
機能資料は [索引](README.md) から参照し、過去評価は当時の条件を保って追記する。
結果にはrepo、完全SHA、実行時差分、OS/ROS/Pythonと依存、入力、正確なコマンド、
exit code、成功/失敗/skip件数、ログ保存先、未確認条件を記録する。
コード変更後へ古い結果を付け替えず、オフライン・simulation・実機を区別する。

bag・画像・点群・重み・生ログはまずパス、サイズ、metadata、既存集計を読む。
必要な疑問を決めてからファイル・topic・時間・件数・読み込み量を絞る。
全件展開・再生・base64化・ログ全文の会話転記をしない。評価・校正・ライセンスを削除しない。
ソース検索からbuild/install/log/data/models/cacheを外す。完全ログは生成物領域へ保存し、
会話には原因と必要な前後だけを示す。pipe使用時も元の終了状態を保持する。

## 安全な検証

fixtureとsubprocess・外部接続を読んでから実行する。通常のビルド・テスト例：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to cat_navigation
source install/setup.bash
unset CAT_TEST_HORIZON
colcon test --packages-select cat_bringup cat_capture cat_navigation
colcon test-result --verbose
```

ビルドは対象checkoutを入力にする。共同作業では担当者がビルドを一元化し、
同じbuild/installを並行更新しない。古いinstallだけの成功を新ソースの成功と扱わない。
Horizon実機power試験はopt-inで、オフライン時は1件skipする。
synthetic ROS試験はlocalhostのdomain 211（clock）と212（spatial VIO）を使う。
実センサー、速度指令、launch、simulation、bag再生を使わない。
CLIのhelp/不正引数/transport終了状態はfake ros2で確認し、実サービスを呼ばない。

## 2026-11-12 機器廃止

ユーザーQ2によりVLP-16と後方RPLiDARの今後の利用を廃止。
起動入口・機種選択・設定・専用power/filter・RViz・テスト・URDF不要定義・
Velodyne依存を一緒に撤去した。前方RPLiDAR、customの共有セクタ処理、
Horizon・VIO・GNSS・安全状態機械は保持する。
別repoのdriverや記録データはこの変更では削除しない。

削除前の実装・手順は `cffc4b40bf214484a9cf2bd38a8c8a93d6da9337` の
`cat_bringup/launch/vlp16.launch.py`、`config/vlp16.yaml`、
`urdf/experiment_robot.urdf.xacro`、root READMEから参照できる。
当時の取付値（現在の起動設定ではない）：

| 旧frame | base_linkからxyz (m) | 姿勢 |
| --- | --- | --- |
| `VLP16_F` | (0.0186, 0, 0.207) | 前傾7度 |
| `VLP16_B` | (-0.347981, 0, 0.515158) | 上下反転、前傾20度、yaw=-90度 |
| `RPLIDAR_B` | (-0.3095, 0, 0.49735) | roll=180度、yaw=+90度 |

VLPの旧IP=192.168.0.201、PC=192.168.0.100、port=2368、600 rpm。
Horizonの現接続名 `VLP16 USB Ethernet` はPCに残る接続プロファイル名で、
名称だけを理由に削除・ネットワーク再設定しない。校正の採用・否定的結果は
[元記録](../../robot_sensor_tests/docs/2026-11-05_calibration.md) を保持する。

## panorama枝の個別評価

採用基準は `cffc4b40bf214484a9cf2bd38a8c8a93d6da9337`。
`backup/panorama-20261009` の独自3コミットを確認した。
`f10c1fa4af90da4d01b478a4dbcbab79ddee7425` の同一モード再選択を維持する挙動は
現基準にも存在するため、再取り込みしない。
`78d1b2bab0cd040fe9b8930542d0870bec7ef3af` の要求毎の再armは同枝の次commitで戻されている。
driver実許可の新topicは必須interfaceではなく、今回は既存のbrake→明示armをツール化した。
表示する制御要求とdriver内部許可の違いは [制御契約](control.md) に明記する。
`40f851318135d017f7cd30c1c95a214b87d7079b` のquiet launchはログ既定・driver起動構成を
変えるため一括採用しない。現行の診断出力とSLLidar起動構成を維持する。
