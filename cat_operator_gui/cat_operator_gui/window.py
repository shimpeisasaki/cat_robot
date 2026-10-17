"""Local Qt console; existing launch files and services own robot behavior."""

from dataclasses import asdict
from datetime import datetime
import json
import os
from pathlib import Path
import re
import time
import uuid
from zoneinfo import ZoneInfo
import yaml

from PyQt5.QtCore import QTimer, QUrl
from PyQt5.QtGui import QDesktopServices
from PyQt5.QtWidgets import (
    QCheckBox, QComboBox, QDialog, QDialogButtonBox, QDoubleSpinBox, QFileDialog,
    QFormLayout, QGroupBox, QHBoxLayout, QInputDialog, QLabel, QLineEdit,
    QListWidget, QListWidgetItem, QMainWindow, QMessageBox, QPlainTextEdit,
    QPushButton, QSpinBox, QTableWidget, QTableWidgetItem, QTabWidget,
    QVBoxLayout, QWidget,
)
from PyQt5.QtCore import Qt

from .monitor import PcMonitor
from .plans import (IMAGE_TOPIC, MAPPING_TOPICS, PRESETS, Settings,
                    mode_plan, record_plan, save_map_plan)
from .processes import ProcessManager


MODES = {'センサー・手動操作': 'sensors', 'オンライン地図作成': 'mapping',
         'ナビゲーション': 'navigation', 'オフライン地図作成': 'offline'}


def timestamp():
    return datetime.now(ZoneInfo('Asia/Tokyo')).strftime('%Y%m%d_%H%M%S')


def safe_name(value):
    if not value or not re.fullmatch(r'[\w-]+', value):
        raise ValueError('名前には文字・数字・_・-を使用してください')
    return value


class DetailsDialog(QDialog):
    def __init__(self, plan, parent):
        super().__init__(parent)
        self.setWindowTitle('操作の詳細')
        self.resize(900, 560)
        layout = QVBoxLayout(self)
        text = QPlainTextEdit(plan.text())
        text.setReadOnly(True)
        layout.addWidget(text)
        self.files = QComboBox()
        for name, path in plan.sources.items():
            if Path(path).is_file():
                self.files.addItem(f'{name}: {path}', path)
        buttons = QHBoxLayout()
        buttons.addWidget(self.files)
        open_button = QPushButton('ファイルを開く')
        open_button.setEnabled(self.files.count() > 0)
        open_button.clicked.connect(self.open_file)
        buttons.addWidget(open_button)
        layout.addLayout(buttons)
        close = QDialogButtonBox(QDialogButtonBox.Close)
        close.rejected.connect(self.reject)
        layout.addWidget(close)

    def open_file(self):
        QDesktopServices.openUrl(QUrl.fromLocalFile(self.files.currentData()))


class TopicsDialog(QDialog):
    def __init__(self, graph, topics, excluded, record_all, parent):
        super().__init__(parent)
        self.setWindowTitle('記録トピックの詳細設定')
        self.resize(850, 600)
        layout = QVBoxLayout(self)
        layout.addWidget(QLabel('チェックしたトピックを収録から除外します。全トピックでは後から現れるトピックも収録します。'))
        self.list = QListWidget()
        for name in sorted((set(graph) if record_all else set(topics)) | set(excluded)):
            entry = graph.get(name, {})
            available = entry.get('publishers', 0)
            kind = ', '.join(entry.get('types', []))
            item = QListWidgetItem(f'{name}  [{available} publisher]  {kind}')
            item.setData(Qt.UserRole, name)
            item.setFlags(item.flags() | Qt.ItemIsUserCheckable)
            item.setCheckState(Qt.Checked if name in excluded else Qt.Unchecked)
            self.list.addItem(item)
        layout.addWidget(self.list)
        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

    def exclusions(self):
        return [self.list.item(i).data(Qt.UserRole) for i in range(self.list.count())
                if self.list.item(i).checkState() == Qt.Checked]


class OperatorWindow(QMainWindow):
    def __init__(self, settings, shares, settings_path, no_ros=False):
        super().__init__()
        self.settings = settings
        self.shares = shares
        self.settings_path = Path(settings_path)
        self.live_domain = int(os.environ.get('ROS_DOMAIN_ID', '0'))
        self.no_ros = no_ros
        self.live = None
        self.offline = None
        self.live_snapshot = {}
        self.offline_snapshot = {}
        self.active_settings = None
        self.active_plan = None
        self.pending_plan = None
        self.record_selection = None
        self.active_record_plan = None
        self.closing = False
        self.last_device_update = 0.0
        self.stopping_mode = False
        self.fields = {}
        self.selected_topics = set()
        self.excluded_topics = set()
        self.record_all = True
        self.setWindowTitle('CAT Operator Console')
        self.resize(1120, 880)
        self.build_ui()
        self.log_dir = Path(settings.data_dir).expanduser() / 'gui_sessions' / (
            timestamp() + '-' + uuid.uuid4().hex[:6])
        self.manager = ProcessManager(self.log_dir, self.append_log, self.job_event)
        self.pc = PcMonitor(str(Path(settings.data_dir).expanduser()))
        self.pc.updated.connect(self.pc_updated)
        self.pc.start()
        if not no_ros:
            from .ros_bridge import RosBridge
            try:
                self.live = RosBridge()
                self.live.snapshot.connect(self.live_updated)
                self.connect_bridge(self.live)
            except Exception as exc:
                self.no_ros = True
                self.notify(f'ROS初期化に失敗しました。操作を無効にします: {exc}')
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(150)
        self.load_widgets(settings)
        self.statusBar().showMessage(f'ログ: {self.log_dir}')

    def add_path(self, form, label, key, directory=False):
        row = QWidget()
        box = QHBoxLayout(row)
        box.setContentsMargins(0, 0, 0, 0)
        edit = QLineEdit()
        button = QPushButton('選択')
        button.clicked.connect(lambda: self.browse(edit, directory))
        box.addWidget(edit)
        box.addWidget(button)
        form.addRow(label, row)
        self.fields[key] = edit
        return edit

    def browse(self, field, directory):
        current = field.text() or self.fields['data_dir'].text()
        if directory:
            value = QFileDialog.getExistingDirectory(self, 'フォルダを選択', current)
        else:
            value, _ = QFileDialog.getOpenFileName(
                self, 'ファイルを選択', current, 'YAML (*.yaml *.yml);;すべて (*)')
        if value:
            field.setText(value)

    def button(self, text, callback, layout):
        button = QPushButton(text)
        button.clicked.connect(callback)
        layout.addWidget(button)
        return button

    def build_ui(self):
        central = QWidget()
        root = QVBoxLayout(central)
        self.summary = QLabel('起動待ち')
        self.summary.setWordWrap(True)
        root.addWidget(self.summary)
        self.pc_label = QLabel('PC監視: 取得中')
        self.pc_label.setWordWrap(True)
        root.addWidget(self.pc_label)
        self.device_label = QLabel('デバイス状態: 取得中')
        self.device_label.setWordWrap(True)
        root.addWidget(self.device_label)
        top = QHBoxLayout()
        brake = self.button('ブレーキ', lambda: self.trigger('/base_safety/brake'), top)
        brake.setStyleSheet('background:#b52e35;color:white;font-weight:bold;padding:8px')
        self.button('手動走行許可', lambda: self.trigger('/base_safety/manual'), top)
        self.button('ナビ走行許可', lambda: self.trigger('/base_safety/request_navigation'), top)
        self.button('巡回開始', lambda: self.trigger('/waypoint_route/start'), top)
        self.button('巡回取消＋ブレーキ', self.cancel_route, top)
        root.addLayout(top)
        tabs = QTabWidget()
        root.addWidget(tabs)
        self.setCentralWidget(central)

        operations = QWidget()
        ops = QVBoxLayout(operations)
        mode_row = QHBoxLayout()
        self.mode = QComboBox()
        for title, mode in MODES.items():
            self.mode.addItem(title, mode)
        mode_row.addWidget(self.mode)
        self.start_button = self.button('起動', self.start_mode, mode_row)
        self.stop_button = self.button('モード停止', self.stop_mode, mode_row)
        self.button('詳細', lambda: self.details('mode'), mode_row)
        ops.addLayout(mode_row)
        maps = QGroupBox('ナビゲーションの入力')
        form = QFormLayout(maps)
        self.add_path(form, '自己位置推定用地図', 'localization_map')
        self.add_path(form, 'ナビゲーション用地図', 'navigation_map')
        self.add_path(form, '巡回ルート（任意）', 'waypoints')
        ops.addWidget(maps)
        offline = QGroupBox('オフライン地図作成')
        form = QFormLayout(offline)
        self.add_path(form, '再生するrosbagフォルダ', 'bag_path', True)
        rate_row = QHBoxLayout()
        self.rate = QDoubleSpinBox()
        self.rate.setRange(.05, 5.0)
        self.rate.setSingleStep(.25)
        self.rate.setValue(1.0)
        rate_row.addWidget(self.rate)
        self.button('速度を適用', self.set_rate, rate_row)
        self.button('一時停止', lambda: self.playback('pause'), rate_row)
        self.button('再開', lambda: self.playback('resume'), rate_row)
        self.button('再生停止', lambda: self.manager.stop('playback'), rate_row)
        form.addRow('再生速度', rate_row)
        self.playback_label = QLabel('再生停止')
        form.addRow('状態', self.playback_label)
        ops.addWidget(offline)
        save = QGroupBox('地図保存')
        row = QHBoxLayout(save)
        self.map_name = QLineEdit('map_' + timestamp())
        row.addWidget(QLabel('地図名'))
        row.addWidget(self.map_name)
        self.button('地図を保存', self.save_map, row)
        self.button('詳細', lambda: self.details('save_map'), row)
        self.button('保存先を開く', self.open_maps, row)
        ops.addWidget(save)
        ops.addWidget(QLabel('地図・経路の表示、初期位置・目的地の指定はRVizで操作します。'))
        ops.addStretch()
        tabs.addTab(operations, '運転・地図')

        recording = QWidget()
        rec = QVBoxLayout(recording)
        preset_row = QHBoxLayout()
        self.presets = QComboBox()
        self.presets.addItems(list(PRESETS))
        self.presets.currentTextChanged.connect(self.select_preset)
        preset_row.addWidget(self.presets)
        self.button('詳細設定（トピック）', self.edit_topics, preset_row)
        self.button('独自プリセットを保存', self.save_preset, preset_row)
        rec.addLayout(preset_row)
        self.topic_summary = QLabel()
        self.topic_summary.setWordWrap(True)
        rec.addWidget(self.topic_summary)
        name_row = QHBoxLayout()
        name_row.addWidget(QLabel('収録名'))
        self.bag_name = QLineEdit('run_' + timestamp())
        name_row.addWidget(self.bag_name)
        self.button('収録開始', self.start_recording, name_row)
        self.button('収録停止', lambda: self.manager.stop('record'), name_row)
        self.button('詳細', lambda: self.details('record'), name_row)
        rec.addLayout(name_row)
        self.record_label = QLabel('未収録')
        self.record_label.setWordWrap(True)
        rec.addWidget(self.record_label)
        rec.addWidget(QLabel('設定や保存先を変えても、起動済み処理の設定は変わりません。'))
        rec.addStretch()
        tabs.addTab(recording, 'rosbag収録')

        settings = QWidget()
        form = QFormLayout(settings)
        sensor_row = QHBoxLayout()
        for key, label in [('use_base', 'モータ'), ('use_zed', 'ZED'), ('use_lidar', 'LiDAR'),
                           ('rviz', 'RVizを開く')]:
            widget = QCheckBox(label)
            self.fields[key] = widget
            sensor_row.addWidget(widget)
        form.addRow('使用する機能', sensor_row)
        self.odom = QComboBox()
        self.odom.addItems(['wheel', 'vio'])
        form.addRow('オドメトリ源', self.odom)
        for key, title in [('lidar_port', 'LiDARポート'), ('camera_serial', 'ZEDシリアル番号')]:
            edit = QLineEdit()
            self.fields[key] = edit
            form.addRow(title, edit)
        self.add_path(form, 'データ保存先', 'data_dir', True)
        for key, title in [('robot_yaml', '機体YAML'), ('manual_yaml', '手動操作YAML'),
                           ('zed_yaml', 'ZED YAML（空欄なら自動）'), ('nav_yaml', 'Nav2 YAML'),
                           ('slam_yaml', 'SLAM YAML'), ('localization_yaml', '自己位置推定YAML')]:
            self.add_path(form, title, key)
        self.domain = QSpinBox()
        self.domain.setRange(0, 232)
        form.addRow('オフライン用ROS_DOMAIN_ID', self.domain)
        settings_row = QHBoxLayout()
        self.button('GUI設定を保存', self.save_settings, settings_row)
        self.button('GUI設定を読み直す', self.reload_settings, settings_row)
        self.button('設定を別名保存', self.save_settings_as, settings_row)
        self.button('設定ファイルを選択', self.load_settings_file, settings_row)
        self.button('ログフォルダを開く', self.open_logs, settings_row)
        form.addRow(settings_row)
        tabs.addTab(settings, '設定')

        status = QWidget()
        state_layout = QVBoxLayout(status)
        self.states = QTableWidget(0, 5)
        self.states.setHorizontalHeaderLabels(['トピック', '配信数', '受信経過秒', 'Hz', '状態'])
        self.states.horizontalHeader().setStretchLastSection(True)
        self.states.setColumnWidth(0, 320)
        state_layout.addWidget(self.states)
        self.logs = QPlainTextEdit()
        self.logs.setReadOnly(True)
        self.logs.setMaximumBlockCount(2500)
        state_layout.addWidget(self.logs)
        tabs.addTab(status, '状態・ログ')

    def load_widgets(self, settings):
        for key, widget in self.fields.items():
            if isinstance(widget, QCheckBox):
                widget.setChecked(getattr(settings, key))
            else:
                widget.setText(str(getattr(settings, key)))
        self.mode.setCurrentIndex(max(0, self.mode.findData(settings.mode)))
        self.odom.setCurrentText(settings.odom_source)
        self.domain.setValue(settings.offline_domain)
        self.rate.setValue(settings.playback_rate)
        self.settings.custom_presets = settings.custom_presets
        self.presets.blockSignals(True)
        self.presets.clear()
        self.presets.addItems(list(PRESETS) + list(settings.custom_presets))
        self.presets.blockSignals(False)
        self.select_preset(self.presets.currentText())

    def collect(self):
        settings = Settings(**asdict(self.settings))
        for key, widget in self.fields.items():
            setattr(settings, key, widget.isChecked() if isinstance(widget, QCheckBox)
                    else widget.text().strip())
        settings.mode = self.mode.currentData()
        settings.odom_source = self.odom.currentText()
        settings.offline_domain = self.domain.value()
        settings.playback_rate = self.rate.value()
        if not settings.data_dir:
            raise ValueError('データ保存先を指定してください')
        settings.data_dir = str(Path(settings.data_dir).expanduser().resolve())
        return settings

    def notify(self, text):
        self.statusBar().showMessage(text, 15000)
        if hasattr(self, 'logs'):
            self.logs.appendPlainText(text)

    def error(self, exc):
        self.notify(str(exc))
        QMessageBox.warning(self, '操作を実行できません', str(exc))

    def append_log(self, name, text):
        self.logs.appendPlainText(f'[{name}] {text.rstrip()}')

    def connect_bridge(self, bridge):
        bridge.message.connect(self.notify)
        bridge.service_done.connect(self.service_done)

    def start_mode(self):
        if self.closing or self.manager.jobs or self.pending_plan:
            self.error('実行中の処理を停止してからモードを起動してください')
            return
        try:
            settings = self.collect()
            plan = mode_plan(settings, self.shares, self.live_domain)
            if self.no_ros:
                raise ValueError('--no-rosは画面確認用です。起動操作は無効です')
            if settings.mode == 'offline':
                from .ros_bridge import RosBridge
                self.pending_plan = (settings, plan)
                self.offline = RosBridge(settings.offline_domain, offline=True)
                self.connect_bridge(self.offline)
                self.offline.snapshot.connect(self.offline_updated)
                self.notify('オフライン用ROSドメインを確認中')
                return
            if not self.live_snapshot:
                raise ValueError('ROSの状態取得を待ってから起動してください')
            conflicts = set(self.live_snapshot.get('nodes', [])) & {
                'base_driver', 'slam_toolbox', 'motor_test_gui', 'controller_server', 'map_server',
                'sllidar_node', 'zed_node', 'sector_scan_filter', 'base_safety'}
            if conflicts:
                raise ValueError(f'既に起動した関連ノードがあります: {sorted(conflicts)}')
            self.begin_mode(settings, plan)
        except Exception as exc:
            self.error(exc)

    def begin_mode(self, settings, plan):
        self.active_settings = settings
        self.active_plan = plan
        self.settings = settings
        settings.save(self.settings_path)
        settings.save(self.log_dir / ('mode_' + timestamp() + '.yaml'))
        self.manager.start(plan)
        self.stopping_mode = False

    def offline_updated(self, snapshot):
        self.offline_snapshot = snapshot
        if self.pending_plan:
            settings, plan = self.pending_plan
            self.pending_plan = None
            foreign = set(snapshot['nodes']) - {'cat_operator_monitor', '_ros2cli_daemon_0'}
            foreign = {name for name in foreign if not name.startswith('_ros2cli')}
            if foreign:
                self.close_offline()
                self.error(f'オフライン用ドメインが使用中です: {sorted(foreign)}')
                return
            try:
                self.begin_mode(settings, plan)
            except Exception as exc:
                self.close_offline()
                self.error(exc)
        paused = snapshot.get('paused')
        player = 'rosbag2_player' in snapshot.get('nodes', [])
        self.playback_label.setText('一時停止' if paused and player else
                                    '再生中' if player else '再生終了／待機')
        if self.active_settings and self.active_settings.mode == 'offline':
            self.render_states(snapshot)

    def live_updated(self, snapshot):
        self.live_snapshot = snapshot
        if not self.active_settings or self.active_settings.mode != 'offline':
            self.render_states(snapshot)

    def render_states(self, snapshot):
        topics = snapshot.get('topics', {})
        states = snapshot.get('states', {})
        names = [IMAGE_TOPIC, '/scan', '/odom', '/tf', '/tf_static', '/map', '/base/state',
                 '/base_safety/control_state', '/navigation/ready', '/waypoint_route/status']
        self.states.setRowCount(len(names))
        for row, name in enumerate(names):
            state = states.get(name, {})
            age = state.get('age')
            status = '未受信' if age is None else '更新あり' if age < 2 else '更新停止'
            if name == '/tf_static' and age is not None:
                status = '受信済み（固定TF）'
            if name == '/base/state' and age is not None and age < 1:
                status = '通信正常' if state.get('value', 0) & 1 else '通信異常'
            if 'reason' in state:
                status = f"mode={state['mode']} {state['source']} {state['reason']}"
            elif 'value' in state:
                status += f": {state['value']}"
            values = [name, str(topics.get(name, {}).get('publishers', 0)),
                      '-' if age is None else f'{age:.1f}', str(state.get('hz', '-')), status]
            for column, value in enumerate(values):
                self.states.setItem(row, column, QTableWidgetItem(value))

    def trigger(self, service):
        if self.live is None or (self.active_settings and self.active_settings.mode == 'offline'):
            self.notify('実機サービス操作は現在無効です')
            return
        self.live.request(service)

    def cancel_route(self):
        self.trigger('/waypoint_route/cancel')
        self.trigger('/base_safety/brake')

    def service_done(self, tag, success, text):
        self.notify(f'{tag}: {"成功" if success else "失敗"} — {text}')
        if tag == 'stop_mode':
            self.manager.stop('mode')
        elif tag == 'close_gui':
            self.manager.stop_all()

    def stop_mode(self):
        if self.pending_plan:
            self.pending_plan = None
            self.close_offline()
        if 'mode' not in self.manager.jobs or self.stopping_mode:
            return
        self.stopping_mode = True
        self.manager.stop('record')
        self.manager.stop('save_map')
        if self.active_settings and self.active_settings.mode == 'offline' and self.offline:
            self.manager.stop('playback')
            self.manager.stop('mode')
        elif self.live:
            self.live.request('/base_safety/brake', tag='stop_mode')
            QTimer.singleShot(6000, self.stop_mode_fallback)
        else:
            self.manager.stop('mode')

    def stop_mode_fallback(self):
        if self.stopping_mode:
            self.manager.stop('mode')

    def playback(self, action):
        if self.offline and 'mode' in self.manager.jobs:
            self.offline.request(f'/rosbag2_player/{action}', action)

    def set_rate(self):
        if self.offline and 'mode' in self.manager.jobs:
            self.offline.request('/rosbag2_player/set_rate', 'rate', self.rate.value())

    def map_plan(self):
        name = safe_name(self.map_name.text().strip())
        settings = self.active_settings or self.collect()
        output = Path(settings.data_dir).expanduser() / 'maps' / name / name
        domain = settings.offline_domain if settings.mode == 'offline' else None
        return save_map_plan(output, domain)

    def save_map(self):
        try:
            if (not self.active_settings or self.active_settings.mode not in ('mapping', 'offline')
                    or 'mode' not in self.manager.jobs or self.stopping_mode):
                raise ValueError('地図作成モードを起動してください')
            snapshot = (self.offline_snapshot if self.active_settings.mode == 'offline'
                        else self.live_snapshot)
            if not snapshot.get('states', {}).get('/map'):
                raise ValueError('まだ地図が生成されていません')
            plan = self.map_plan()
            Path(plan.sources['出力名']).parent.mkdir(parents=True, exist_ok=True)
            self.manager.start(plan)
        except Exception as exc:
            self.error(exc)

    def record_plan(self):
        name = safe_name(self.bag_name.text().strip())
        output = Path(self.fields['data_dir'].text()).expanduser() / 'bags' / name
        return record_plan(output, self.selected_topics, self.record_all, self.excluded_topics)

    def start_recording(self):
        try:
            if self.no_ros or self.closing:
                raise ValueError('ROS収録は現在無効です')
            if self.active_settings and self.active_settings.mode == 'offline':
                raise ValueError('オフライン地図作成中の実機収録は行いません')
            if 'record' in self.manager.jobs:
                raise ValueError('既に収録中です')
            plan = self.record_plan()
            Path(plan.sources['保存先']).parent.mkdir(parents=True, exist_ok=True)
            self.record_selection = {
                'preset': self.presets.currentText(), 'topics': sorted(self.selected_topics),
                'all_topics': self.record_all, 'excluded_topics': sorted(self.excluded_topics),
                'settings': asdict(self.collect()),
                'started_at': timestamp(), 'output': plan.sources['保存先']}
            (self.log_dir / ('record_' + timestamp() + '.json')).write_text(
                json.dumps(self.record_selection, ensure_ascii=False, indent=2), encoding='utf-8')
            self.manager.start(plan)
            self.active_record_plan = plan
            self.record_label.setText('収録中: ' + plan.sources['保存先'])
        except Exception as exc:
            self.error(exc)

    def select_preset(self, name):
        self.record_all = name == '全トピック'
        self.selected_topics = set(PRESETS.get(name, self.settings.custom_presets.get(name, [])))
        self.excluded_topics = set()
        self.update_topic_summary()

    def update_topic_summary(self):
        text = ('全トピックを記録' if self.record_all else
                '収録対象:\n' + '\n'.join(sorted(self.selected_topics - self.excluded_topics)))
        if self.excluded_topics:
            text += '\n除外:\n' + '\n'.join(sorted(self.excluded_topics))
        missing = (set(MAPPING_TOPICS) & self.excluded_topics if self.record_all else
                   set(MAPPING_TOPICS) - (self.selected_topics - self.excluded_topics))
        if missing:
            text += '\nオフライン地図作成に必要なトピック未選択: ' + ', '.join(sorted(missing))
        self.topic_summary.setText(text)

    def edit_topics(self):
        dialog = TopicsDialog(self.live_snapshot.get('topics', {}), self.selected_topics,
                              self.excluded_topics, self.record_all, self)
        if dialog.exec_() == QDialog.Accepted:
            self.excluded_topics = set(dialog.exclusions())
            self.update_topic_summary()

    def save_preset(self):
        if self.record_all:
            self.error('独自プリセットには個別トピックを選択してください')
            return
        name, accepted = QInputDialog.getText(self, '独自プリセット', '名前')
        name = name.strip()
        if accepted and name:
            if name in PRESETS:
                self.error('標準プリセットと異なる名前にしてください')
                return
            self.settings.custom_presets[name] = sorted(self.selected_topics - self.excluded_topics)
            if self.presets.findText(name) < 0:
                self.presets.addItem(name)
            self.presets.setCurrentText(name)
            self.save_settings()

    def save_settings(self):
        try:
            self.settings = self.collect()
            self.settings.save(self.settings_path)
            self.pc.directory = str(Path(self.settings.data_dir).expanduser())
            self.notify(f'GUI設定を保存: {self.settings_path}')
        except Exception as exc:
            self.error(exc)

    def reload_settings(self):
        try:
            self.settings = Settings.load(self.settings_path)
            self.load_widgets(self.settings)
        except Exception as exc:
            self.error(exc)

    def save_settings_as(self):
        value, _ = QFileDialog.getSaveFileName(
            self, 'GUI設定を別名保存', str(self.settings_path), 'YAML (*.yaml)')
        if value:
            self.settings_path = Path(value)
            self.save_settings()

    def load_settings_file(self):
        value, _ = QFileDialog.getOpenFileName(
            self, 'GUI設定を選択', str(self.settings_path.parent), 'YAML (*.yaml *.yml)')
        if value:
            try:
                settings = Settings.load(value)
                self.settings = settings
                self.settings_path = Path(value)
                self.load_widgets(settings)
                self.notify(f'GUI設定を選択: {value}')
            except Exception as exc:
                self.error(exc)

    def details(self, kind):
        try:
            if kind == 'mode':
                plan = self.active_plan if 'mode' in self.manager.jobs else mode_plan(
                    self.collect(), self.shares, self.live_domain)
            elif kind == 'record':
                if 'record' in self.manager.jobs and self.record_selection:
                    plan = self.active_record_plan
                else:
                    plan = self.record_plan()
            else:
                plan = self.map_plan()
            DetailsDialog(plan, self).exec_()
        except Exception as exc:
            self.error(exc)

    def open_maps(self):
        path = Path(self.fields['data_dir'].text()).expanduser() / 'maps'
        path.mkdir(parents=True, exist_ok=True)
        QDesktopServices.openUrl(QUrl.fromLocalFile(str(path)))

    def open_logs(self):
        QDesktopServices.openUrl(QUrl.fromLocalFile(str(self.log_dir)))

    def pc_updated(self, info):
        free = '-' if info['disk_free_gb'] is None else f"{info['disk_free_gb']} GB"
        self.pc_label.setText(f"CPU {info['cpu']}% / RAM {info['memory']}% / 空き容量 {free}\n"
                              'GPU [名前, 使用率%, 使用MiB, 全MiB, 温度°C]: ' + info['gpu'])

    def job_event(self, name, event, code):
        self.notify(f'{name}: {event}' + (f' ({code})' if code is not None else ''))
        if event != 'finished':
            return
        if name == 'mode':
            self.manager.stop('record')
            self.manager.stop('playback')
            self.active_settings = None
            self.active_plan = None
            self.stopping_mode = False
            self.close_offline()
        if name == 'record' and self.record_selection:
            selection = self.record_selection.copy()
            selection['ended_at'] = timestamp()
            selection['exit_code'] = code
            output = Path(selection['output'])
            if output.is_dir():
                (output / 'gui_recording.json').write_text(
                    json.dumps(selection, ensure_ascii=False, indent=2), encoding='utf-8')
            self.record_label.setText(f'収録終了 ({code}): {output}')
            self.bag_name.setText('run_' + timestamp())
        if name == 'save_map' and code == 0:
            self.notify('地図保存が完了しました。保存先フォルダでYAMLと画像を確認できます。')
            self.map_name.setText('map_' + timestamp())

    def tick(self):
        self.manager.tick()
        names = ', '.join(self.manager.jobs) or 'なし'
        state = self.live_snapshot.get('states', {}).get('/base_safety/control_state', {})
        reason = state.get('reason', '安全制御状態は未受信')
        if state.get('age', 0) > 2:
            reason = '更新停止: ' + reason
        self.summary.setText(f'GUIが起動した処理: {names}\n実機: {reason}')
        self.start_button.setEnabled(
            not self.manager.jobs and not self.pending_plan and not self.closing)
        self.stop_button.setEnabled('mode' in self.manager.jobs or self.pending_plan is not None)
        now = time.monotonic()
        if now - self.last_device_update > 2:
            self.last_device_update = now
            self.update_devices()
        if self.closing and not self.manager.jobs:
            self.close()

    def update_devices(self):
        paths = {'LiDAR': self.fields['lidar_port'].text()}
        try:
            robot_path = Path(self.fields['robot_yaml'].text()).expanduser()
            document = yaml.safe_load(robot_path.read_text(encoding='utf-8'))
            robot = document['base_driver']['ros__parameters']
            paths.update({'左モータ': robot['left_usb_dev'], '右モータ': robot['right_usb_dev']})
        except (OSError, KeyError, TypeError, yaml.YAMLError):
            pass
        text = ' / '.join(f'{name}: {"検出" if value and Path(value).exists() else "未検出"}'
                          for name, value in paths.items())
        states = self.live_snapshot.get('states', {})
        for name, topic in [('LiDAR配信', '/scan'), ('カメラ配信', IMAGE_TOPIC)]:
            active = states.get(topic, {}).get('age', 999) < 2
            text += f' / {name}: {"更新あり" if active else "未受信・停止"}'
        self.device_label.setText(text)

    def close_offline(self):
        if self.offline:
            self.offline.close()
            self.offline = None
        self.offline_snapshot = {}

    def closeEvent(self, event):
        self.pending_plan = None
        if self.manager.jobs and not self.closing:
            self.closing = True
            self.pending_plan = None
            self.notify('終了処理中。収録の書き出しと起動したプロセスの終了を待っています。')
            if 'mode' in self.manager.jobs and self.live and self.active_settings.mode != 'offline':
                self.live.request('/base_safety/brake', tag='close_gui')
                QTimer.singleShot(6000, self.close_fallback)
            else:
                self.manager.stop_all()
            event.ignore()
            return
        if self.manager.jobs:
            event.ignore()
            return
        self.timer.stop()
        self.close_offline()
        if self.live:
            self.live.close()
            self.live = None
        self.pc.close()
        event.accept()

    def close_fallback(self):
        if self.closing:
            self.manager.stop_all()
