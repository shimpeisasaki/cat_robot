# 2026-11-19 CAT/ZED入出力整理の検証

仕様はworkspaceの`docs/refactoring/CAT_ZED_REFACTOR_SPEC_20261119.md`、
手順は同階層の`OMO_CAT_ZED_WORK_ORDER_20261119.md`。
現行の運用正本は[起動手順](../startup.md)。
完全ログ・実行script・変更前diffはworkspaceの
`log/cat_zed_refactor_20261119_172436/`に保存する。
workspace文書・logは独立Git repoに追跡されていない。

## 入力と制限

| repo | 開始時完全SHA |
| --- | --- |
| cat_robot | `4d75c419855fcec28290118cb58ddf0f067acf2d` |
| zed-ros2-wrapper | `3878297b762883c94010978269b050f4c5e78ace` |
| d1vision_ros2 | `1b72f5bf286e9c5f2462ceb891930e7b759e08a2` |
| recovery_exploration | `8a9590790ea952fa0736c372d8bf70cab5188f70` |

開始時4 repoともclean。D1 mainは変更しない。他3 repoは実HEADから
`refactor/cat-zed-20261119-172436`へ分離した。
Humbleのみの`env -i`環境、Python3.10、SDK5.4.1、最大2 build workers、
colcon sequential。詳細環境は`environment-before.txt`と各build cache/log。
実機ノード・motor・model server・実推論・simulation・bag再生・依存導入は行わない。
既存retry・アルゴリズム・速度・鮮度・取付値を変えない。

Livox SDK header/static libraryが見つからず、vendorの自動取得は禁止のため、
Livox3 packagesをfresh対象から外した。旧installのLivoxを借用していない。
Horizon経路とmotor driver実行は未検証。LSP各serverも未導入であり、
診断cleanとは主張しない。依存追加で隠さない。

## 変更前証拠

`run_phase.sh baseline`は明示sourceの21 packagesをfresh buildしexit0。
`baseline/test_result.log`は202 tests、0 errors、0 failures、既存の実機power試験skip1。
CAT・wrapper・D1・driver各選択のexitもすべて0。
wrapperは`image_stream_gate`/`raw_jpeg_publisher`、D1は合成manager/manager_ros/image等、
driverは実ノードを起動しない差動駆動/lease GTestだけを明示した。

`snapshot.py`はinstalled YAMLとlaunchの宣言・値伝播を保存した。
`surface_smoke.py`はソース外cwd・source PYTHONPATHなし・domain89/localhostで
実サービスによるJPEG撮影、元バイト・撮影header・odom補間・frames.json、
既定/custom topic、SetTask→ON/0.2秒→JPEG→模擬HTTP→OK TaskResult→OFFを確認した。
`baseline/surface_clean.log`にPASSとノード・worker・socket・一時画像の清掃receiptがある。

設定の全値・型を変更前YAML fixtureで固定した。
追加保持契約は実装前35 pass、新common解決と入口廃止の契約3 fail、
別実行のJPEG専用selector契約2 fail。意図した箇所以外の失敗ではない。
`contracts_before.log`、`jpeg_contract_before.log`を保存する。

## 実装段階

JPEG専用化後は撮影・metadata・取消drain・通常brake要求・購読型/QoSの23件が成功。
設定共通化後は全値/型・独自file・profile優先・一時file清掃と既存odom計算の35件が成功。
GNSS mode・front/custom・上位からの伝播は17件成功。
それぞれ`jpeg_after.log`、`config_after.log`、`launch_after_fixed.log`。
旧BGRの試験入力をJPEGへ移し、保存名・metadataの検査は維持した。

途中の試験harness不具合も保存した。Humble setupはnounset非対応、
launch source.locationは解決前には表示文字列、最初のsurfaceはuser-site NumPyと
cv_bridgeのABI警告を出した。pipefailのみ、public launch解決API、
`PYTHONNOUSERSITE=1`へ修正した。製品の失敗として読み替えない。

## 変更後受入

`run_phase.sh after`はbaselineとは別のfresh出力へ同じ21 packagesをbuildしてexit0。
当初集計は230 tests、0 errors/failures、既存skip1だった。
追加点検でGUIが共通YAMLを起動時コピーへ含めない回帰を再現し、
`plans.py`が明示参照の共通fileも記録するよう修正した。独自一枚fileは変更しない。
`gui_common_before.log`は共通case1 fail/standalone1 pass。
修正後のGUI12件と同packageのfocused rebuild/installed CTestはすべてexit0。
最終集計は`after/test_result_final.log`の232 tests、0 errors、0 failures、既存skip1。
これは全package再buildの追加回数ではなく、fresh出力内の当該packageだけの追試である。

`after/surface.log`でinstalled撮影とD1模擬HTTPの全シナリオ成功・清掃を確認した。
`after/installed_checks_final.log`は全21 package/moduleのfresh解決、GUI3モードの
command生成と実際の設定コピー、全YAML値/型・launch既定値/転送・wrapper層の同値性を確認。
GUIコピーは実ProcessManagerへ空jobsを渡し、processを作らずに記録fileの元bytesを検証した。
一時画像、mock HTTP socket/thread、node/executor、GUIコピーは各receiptで終了・削除確認済み。
safe help3件もexit0。実機commandは実行していない。

wrapper優先順位は通常VIO、wheel、独自一枚file、inline override、汎用profileを比較した。
navigationのRewrittenYamlは一時path名が毎回変わるため、比較器を生成内容の比較へ修正した。
旧path比較の失敗は`after/acceptance.log`、修正成功は`after/installed_checks_fixed.log`へ保持。
source構文・実行属性・相対文書linkは`audit_files.py`で検査する。
推定/安全/TF・wrapper実装・D1が変更されていない証拠は`preserved-code.log`。

独立最終検証・local commit・清掃の確定結果は、同じlog配下と
workspaceの`docs/refactoring/reports/2026-11-19_cat-zed-refactor.md`へ記録する。
