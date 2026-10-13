"""Validated commands and their configuration sources, shared by execution and details."""

from dataclasses import asdict, dataclass, field
from pathlib import Path
import shlex
import math

import yaml


IMAGE_TOPIC = '/zed/zed_node/rgb/color/rect/image'
MAPPING_TOPICS = ['/scan', '/odom', '/tf', '/tf_static']
PRESETS = {
    '地図作成': MAPPING_TOPICS,
    '走行・ナビゲーション': MAPPING_TOPICS + [
        '/scan_raw', '/wheel/odom', '/map', '/localization/map', '/joy',
        '/cmd_vel_safe', '/base/state', '/diagnostics', '/base_safety/control_state',
        '/navigation/ready', '/waypoint_route/status'],
    'ZED画像評価': MAPPING_TOPICS + [IMAGE_TOPIC, '/zed/zed_node/rgb/color/rect/camera_info'],
    '全トピック': [],
}


@dataclass
class Settings:
    mode: str = 'sensors'
    use_base: bool = True
    use_zed: bool = False
    use_lidar: bool = True
    odom_source: str = 'wheel'
    lidar_port: str = '/dev/rplidar'
    camera_serial: str = '10028118'
    rviz: bool = True
    localization_map: str = ''
    navigation_map: str = ''
    waypoints: str = ''
    robot_yaml: str = ''
    manual_yaml: str = ''
    zed_yaml: str = ''
    nav_yaml: str = ''
    slam_yaml: str = ''
    localization_yaml: str = ''
    bag_path: str = ''
    data_dir: str = str(Path.home() / 'ros2_ws' / 'data')
    playback_rate: float = 1.0
    offline_domain: int = 80
    custom_presets: dict = field(default_factory=dict)

    def save(self, path):
        path = Path(path).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix('.tmp')
        temporary.write_text(yaml.safe_dump(asdict(self), allow_unicode=True), encoding='utf-8')
        temporary.replace(path)

    @classmethod
    def load(cls, path):
        path = Path(path).expanduser()
        if not path.exists():
            return cls()
        document = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
        if not isinstance(document, dict):
            raise ValueError('GUI設定はYAMLの辞書で指定してください')
        return cls().apply(document)

    def apply(self, document):
        unknown = set(document) - set(self.__dataclass_fields__)
        if unknown:
            raise ValueError(f'不明なGUI設定: {sorted(unknown)}')
        for key, value in document.items():
            expected = type(getattr(self, key))
            valid = (isinstance(value, (int, float)) and not isinstance(value, bool)
                     if expected is float else type(value) is expected)
            if not valid:
                raise ValueError(f'GUI設定の型が不正です: {key}')
        if not math.isfinite(document.get('playback_rate', self.playback_rate)):
            raise ValueError('再生速度が不正です')
        for name, topics in document.get('custom_presets', {}).items():
            bad_topic = (isinstance(topics, list) and any(
                not isinstance(topic, str) or not topic.startswith('/') for topic in topics))
            if (not isinstance(name, str) or not isinstance(topics, list)
                    or bad_topic):
                raise ValueError('独自プリセットの形式が不正です')
        for key, value in document.items():
            setattr(self, key, value)
        return self


@dataclass
class JobSpec:
    name: str
    command: list
    env: dict = field(default_factory=dict)


@dataclass
class Plan:
    title: str
    jobs: list
    sources: dict
    notes: list = field(default_factory=list)

    def text(self):
        lines = [self.title, '', '実行コマンド:']
        for job in self.jobs:
            prefix = ' '.join(f'{key}={shlex.quote(str(value))}' for key, value in job.env.items())
            lines.append(f'{job.name}: {prefix} {shlex.join(job.command)}'.strip())
        lines += ['', '参照ファイル:']
        lines += [f'{name}: {path}' for name, path in self.sources.items()]
        lines += ['', *self.notes]
        return '\n'.join(lines)


def defaults(shares):
    settings = Settings()
    base, nav = Path(shares['cat_bringup']), Path(shares['cat_navigation'])
    for name, value in {
        'robot_yaml': base / 'config/robot.yaml',
        'manual_yaml': base / 'config/manual_control.yaml',
        'nav_yaml': nav / 'config/nav2_params.yaml',
        'slam_yaml': nav / 'config/slam_toolbox.yaml',
        'localization_yaml': nav / 'config/emcl2.yaml',
        'localization_map': nav / 'map/19F_260928.yaml',
        'navigation_map': nav / 'map/19F_260928_Nav.yaml',
    }.items():
        setattr(settings, name, str(value))
    return settings


def yaml_file(value):
    path = Path(value).expanduser().resolve()
    if not value or not path.is_file():
        raise ValueError(f'YAMLファイルがありません: {value}')
    document = yaml.safe_load(path.read_text(encoding='utf-8'))
    if not isinstance(document, dict):
        raise ValueError(f'YAMLの形式が不正です: {path}')
    return str(path), document


def map_file(value):
    path, document = yaml_file(value)
    image = Path(document.get('image', '')).expanduser()
    if not image.is_absolute():
        image = Path(path).parent / image
    if not document.get('image') or not image.is_file():
        raise ValueError(f'地図画像がありません: {image}')
    try:
        geometry = (float(document['resolution']), *map(float, document['origin']))
        if len(geometry) != 4 or geometry[0] <= 0 or not all(map(math.isfinite, geometry)):
            raise ValueError()
    except (KeyError, TypeError, ValueError):
        raise ValueError(f'地図のresolution/originが不正です: {path}') from None
    return path, geometry


def bag_info(value):
    bag = Path(value).expanduser().resolve()
    if not (bag / 'metadata.yaml').is_file():
        raise ValueError(f'rosbagのmetadata.yamlがありません: {bag}')
    metadata = yaml.safe_load((bag / 'metadata.yaml').read_text(encoding='utf-8'))
    info = metadata['rosbag2_bagfile_information']
    topics = {item['topic_metadata']['name']: item['topic_metadata']['type']
              for item in info['topics_with_message_count'] if item['message_count'] > 0}
    missing = set(MAPPING_TOPICS) - set(topics)
    if missing:
        raise ValueError(f'オフライン地図作成に必要なトピックがありません: {sorted(missing)}')
    expected = {'/scan': 'sensor_msgs/msg/LaserScan', '/odom': 'nav_msgs/msg/Odometry',
                '/tf': 'tf2_msgs/msg/TFMessage', '/tf_static': 'tf2_msgs/msg/TFMessage'}
    if any(topics[name] != kind for name, kind in expected.items()):
        raise ValueError('rosbag内の地図作成トピックの型が一致しません')
    return str(bag), info


def mode_plan(settings, shares, live_domain=0):
    s = settings
    sources = {}
    if s.mode == 'offline':
        if not 0 <= s.offline_domain <= 232 or s.offline_domain == live_domain:
            raise ValueError('オフライン用ROS_DOMAIN_IDは実機と異なる0〜232にしてください')
        bag, _ = bag_info(s.bag_path)
        slam, _ = yaml_file(s.slam_yaml)
        if s.playback_rate <= 0:
            raise ValueError('再生速度は0より大きくしてください')
        sources = {'GUI launch': str(
            Path(shares['cat_operator_gui']) / 'launch/offline_mapping.launch.py'),
            'SLAM YAML': slam, 'rosbag': bag}
        qos = str(Path(shares['cat_operator_gui']) / 'config/offline_qos.yaml')
        sources['再生QoS YAML'] = qos
        command = ['ros2', 'launch', 'cat_operator_gui', 'offline_mapping.launch.py',
                   f'slam_params_file:={slam}', f'rviz:={str(s.rviz).lower()}']
        playback = ['ros2', 'bag', 'play', bag, '--clock', '30', '--start-paused',
                    '--disable-keyboard-controls', '--rate', str(s.playback_rate),
                    '--topics', *MAPPING_TOPICS, '--remap', '/tf:=/offline/raw_tf',
                    '/tf_static:=/offline/raw_tf_static', '--qos-profile-overrides-path', qos]
        env = {'ROS_DOMAIN_ID': str(s.offline_domain)}
        return Plan('オフライン地図作成', [JobSpec('mode', command, env),
                    JobSpec('playback', playback, env)], sources,
                    ['実機と別のROSドメインを使用します。実機ドライバは起動しません。',
                     'bagの記録済みmap座標系のTFを除外し、SLAMがmap→odomを生成します。',
                     '再生は一時停止状態で起動します。「再開」で処理を開始します。'])
    if s.mode not in ('sensors', 'mapping', 'navigation'):
        raise ValueError(f'不明な運転モード: {s.mode}')
    if s.odom_source not in ('wheel', 'vio'):
        raise ValueError('オドメトリ源はwheelまたはvioです')
    if s.odom_source == 'vio' and not s.use_zed:
        raise ValueError('VIOにはZEDを有効にしてください')
    if s.mode != 'sensors' and (not s.use_base or not s.use_lidar):
        raise ValueError('地図作成・ナビゲーションにはモータとLiDARが必要です')
    if s.use_lidar and not s.lidar_port:
        raise ValueError('LiDARポートを指定してください')
    if s.use_zed and not s.camera_serial.isdigit():
        raise ValueError('ZEDシリアル番号は数字で指定してください')
    robot, _ = yaml_file(s.robot_yaml)
    manual, _ = yaml_file(s.manual_yaml)
    sources.update({'機体 YAML': robot, '手動操作 YAML': manual})
    args = {'robot_config': robot, 'manual_config': manual,
            'use_zed': str(s.use_zed).lower(), 'odom_source': s.odom_source,
            'serial_port': s.lidar_port, 'serial_number': s.camera_serial}
    if s.use_zed:
        zed_name = 'zed_vio.yaml' if s.odom_source == 'vio' else 'zed_sensors.yaml'
        zed = s.zed_yaml or str(Path(shares['cat_bringup']) / 'config' / zed_name)
        args['zed_config'], _ = yaml_file(zed)
        sources['ZED YAML'] = args['zed_config']
    if s.mode == 'sensors':
        package, launch = 'cat_bringup', 'bringup.launch.py'
        args.update(use_base=str(s.use_base).lower(), use_lidar=str(s.use_lidar).lower(),
                    rviz=str(s.rviz).lower(), enable_panorama='false', enable_motor_tools='false')
    else:
        package = 'cat_navigation'
        sources['内部の機体 launch'] = str(Path(shares['cat_bringup']) / 'launch/bringup.launch.py')
        if s.mode == 'mapping':
            launch = 'mapping.launch.py'
            args['slam_params_file'], _ = yaml_file(s.slam_yaml)
            args['mapping_rviz'] = str(s.rviz).lower()
            sources['SLAM YAML'] = args['slam_params_file']
        else:
            launch = 'navigation.launch.py'
            args['localization_map'], first = map_file(s.localization_map)
            args['navigation_map'], second = map_file(s.navigation_map)
            if any(abs(a - b) > 1e-6 for a, b in zip(first, second)):
                raise ValueError('２枚の地図のresolution/originを揃えてください')
            args['params_file'], _ = yaml_file(s.nav_yaml)
            args['localization_params_file'], _ = yaml_file(s.localization_yaml)
            args['waypoints_file'] = (str(Path(s.waypoints).expanduser().resolve())
                                      if s.waypoints else '')
            if s.waypoints:
                yaml_file(s.waypoints)
            args.update(slam='false', navigation_rviz=str(s.rviz).lower(), enable_panorama='false')
            sources.update({'自己位置推定地図': args['localization_map'],
                            'ナビゲーション地図': args['navigation_map'],
                            'Nav2 YAML': args['params_file'],
                            '自己位置推定 YAML': args['localization_params_file']})
            if s.waypoints:
                sources['巡回ルート YAML'] = args['waypoints_file']
    sources['起動 launch'] = str(Path(shares[package]) / 'launch' / launch)
    if s.use_lidar:
        sources['LiDAR launch'] = str(Path(shares['cat_bringup']) / 'launch/rplidar_s1.launch.py')
        sources['LiDARフィルタ YAML'] = str(Path(shares['cat_bringup']) / 'config/scan_filter.yaml')
    if s.use_zed:
        sources['ZED launch'] = str(Path(shares['cat_bringup']) / 'launch/zed_sensors.launch.py')
    if s.mode == 'navigation':
        sources['自己位置推定 launch'] = str(
            Path(shares['cat_navigation']) / 'launch/localization.launch.py')
    command = ['ros2', 'launch', package, launch]
    command += [f'{key}:={value}' for key, value in args.items()]
    return Plan(s.mode, [JobSpec('mode', command)], sources,
                ['走行許可は起動後の別操作です。機体の終了時動作は機体YAMLの設定に従います。',
                 '依存パッケージのlaunchでは、各パッケージの既定設定も使われます。'])


def record_plan(output, topics, record_all=False):
    output = Path(output).expanduser().resolve()
    if output.exists():
        raise ValueError(f'保存先が既に存在します: {output}')
    topics = sorted(set(topics))
    if not record_all and not topics:
        raise ValueError('記録するトピックを選択してください')
    command = ['ros2', 'bag', 'record', '--node-name', 'cat_gui_recorder', '-o', str(output)]
    command += ['--all'] if record_all else topics
    return Plan('rosbag収録', [JobSpec('record', command)], {'保存先': str(output)},
                ['選択したトピックを収録します。終了時にはbagの書き出し完了を待ちます。'])


def save_map_plan(output, domain=None):
    output = Path(output).expanduser().resolve()
    if not output.name or output.name in ('.', '..'):
        raise ValueError('地図名を指定してください')
    if output.with_suffix('.yaml').exists() or output.with_suffix('.pgm').exists():
        raise ValueError(f'同名の地図があります: {output}')
    command = ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', str(output),
               '--ros-args', '-p', 'save_map_timeout:=10.0']
    environment = {} if domain is None else {'ROS_DOMAIN_ID': str(domain)}
    return Plan('地図保存', [JobSpec('save_map', command, environment)], {'出力名': str(output)},
                ['現在の/mapをYAML＋PGMに保存します。ナビゲーション用地図は別途指定します。'])
