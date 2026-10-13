"""Entry point for the optional local operator GUI."""

import argparse
import os
from pathlib import Path
import signal
import sys

from ament_index_python.packages import get_package_share_directory
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication
import yaml

from .plans import defaults
from .window import OperatorWindow


def main():
    parser = argparse.ArgumentParser(description='CAT robot optional operator GUI')
    parser.add_argument('--settings-file', default=str(
        Path.home() / '.config/cat_operator_gui/settings.yaml'))
    parser.add_argument('--no-ros', action='store_true', help='画面の確認のみ。ROS操作は無効')
    args = parser.parse_args()
    shares = {name: get_package_share_directory(name)
              for name in ('cat_operator_gui', 'cat_bringup', 'cat_navigation')}
    settings = defaults(shares)
    if settings.offline_domain == int(os.environ.get('ROS_DOMAIN_ID', '0')):
        settings.offline_domain = 81
    settings_path = Path(args.settings_file).expanduser()
    if settings_path.exists():
        document = yaml.safe_load(settings_path.read_text(encoding='utf-8')) or {}
        if not isinstance(document, dict) or set(document) - set(settings.__dataclass_fields__):
            parser.error('GUI設定YAMLの形式またはキーが不正です')
        settings.apply(document)
    application = QApplication(sys.argv[:1])
    window = OperatorWindow(settings, shares, settings_path, args.no_ros)
    signal.signal(signal.SIGINT, lambda *_: window.close())
    signal.signal(signal.SIGTERM, lambda *_: window.close())
    window.show()
    # Keep Python signal handling responsive while Qt is idle.
    signal_timer = QTimer()
    signal_timer.timeout.connect(lambda: None)
    signal_timer.start(200)
    return application.exec_()
