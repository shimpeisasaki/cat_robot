# cat_bringup

機体起動・設定・TF・センサ・停止制御の所有元です。
起動方法、設定一覧、公開トピックは[リポジトリREADME](../README.md)を参照してください。

`bringup.launch.py` は既定でROSノードの警告・エラーを表示します。詳しい起動ログが必要な場合は `log_level:=info`、ZEDのみ詳しく見る場合は `zed_log_level:=info` を指定してください。
