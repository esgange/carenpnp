import json
import os
import re
import subprocess
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

try:
    import yaml
except Exception:  # pragma: no cover - launch-time dependency
    yaml = None


DEFAULT_ORBBEC_LAUNCH_ARGS = {
    'device_preset': 'High Accuracy',
    'enable_color': True,
    'enable_depth': True,
    'depth_registration': True,
    'align_target_stream': 'COLOR',
    'align_mode': 'SW',
    'enable_frame_sync': True,
    'enable_temporal_filter': True,
    'color_width': 848,
    'color_height': 480,
    'color_fps': 30,
    'depth_width': 848,
    'depth_height': 480,
    'depth_fps': 30,
    'enable_point_cloud': False,
}
DEFAULT_SCAN_TIMEOUT_SEC = 8.0
DEFAULT_WATCHDOG_STARTUP_TIMEOUT_SEC = 20.0
DEFAULT_WATCHDOG_HEALTH_TIMEOUT_SEC = 5.0
DEFAULT_WATCHDOG_CHECK_PERIOD_SEC = 1.0
DEFAULT_WATCHDOG_RESTART_DELAY_SEC = 3.0
DEFAULT_WATCHDOG_RESTART_BACKOFF_MAX_SEC = 30.0
SERIAL_LABEL_RE = re.compile(
    r'(?:serial(?:\s+number)?|serial_number|sn)\s*[:=]\s*([A-Za-z0-9_.:-]+)',
    re.IGNORECASE,
)
GENERIC_SERIAL_RE = re.compile(r'\b[A-Za-z0-9][A-Za-z0-9_.:-]{5,}\b')


def _workspace_root() -> Path:
    def looks_like_root(path: Path) -> bool:
        return (
            (path / 'src').exists() and
            (
                (path / 'README.md').exists()
                or (path / 'src' / 'dobot_msgs_v4').exists()
            )
        )

    for name in ('DOBOT_PICKN_PLACE_ROOT', 'DOBOT_WORKSPACE_ROOT'):
        value = os.environ.get(name)
        if value:
            return Path(value).expanduser().resolve()

    for start in (Path.cwd(), Path(__file__).resolve()):
        path = start.expanduser().resolve()
        if path.is_file():
            path = path.parent
        for candidate in (path, *path.parents):
            if looks_like_root(candidate):
                return candidate
    return Path.cwd().resolve()


def _repo_path(*parts: str) -> str:
    return str(_workspace_root().joinpath(*parts))


def _ros_domain_action():
    import importlib.util

    helper_candidates = []
    for parent in Path(__file__).resolve().parents:
        helper_candidates.extend([
            parent / 'src' / 'dobot_bringup_v4' / 'launch' / 'ros_domain.py',
            parent / 'install' / 'cr_robot_ros2' / 'share' / 'cr_robot_ros2' / 'launch' / 'ros_domain.py',
            parent / 'cr_robot_ros2' / 'share' / 'cr_robot_ros2' / 'launch' / 'ros_domain.py',
            parent / 'share' / 'cr_robot_ros2' / 'launch' / 'ros_domain.py',
        ])

    for helper_path in helper_candidates:
        if helper_path.exists():
            spec = importlib.util.spec_from_file_location('_dobot_ros_domain', helper_path)
            if spec is None or spec.loader is None:
                continue
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            return module.ros_domain_action()

    raise RuntimeError('Could not find ros_domain.py helper for ROS_DOMAIN_ID')


def _to_bool_text(value: object) -> str:
    if isinstance(value, bool):
        return 'true' if value else 'false'
    return str(value)


def _to_bool(value: object, default: bool = False) -> bool:
    if isinstance(value, bool):
        return value
    text = str(value).strip().lower()
    if text in ('1', 'true', 'yes', 'on'):
        return True
    if text in ('0', 'false', 'no', 'off'):
        return False
    return default


def _to_positive_float(value: object, default: float) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return default
    return parsed if parsed > 0.0 else default


def _extract_serial_numbers(text: str) -> list[str]:
    found = []
    for match in SERIAL_LABEL_RE.finditer(text):
        candidate = match.group(1).strip().strip(',;')
        if candidate and candidate not in found:
            found.append(candidate)
    if found:
        return found

    stop_words = {
        'orbbec',
        'camera',
        'device',
        'serial',
        'number',
        'version',
        'firmware',
        'connected',
        'product',
    }
    for match in GENERIC_SERIAL_RE.finditer(text):
        candidate = match.group(0).strip().strip(',;')
        if candidate.lower() in stop_words:
            continue
        if candidate not in found:
            found.append(candidate)
    return found


def _scan_connected_serials(timeout_sec: float) -> tuple[int, list[str], str]:
    command = ['ros2', 'run', 'orbbec_camera', 'list_devices_node']
    try:
        result = subprocess.run(
            command,
            check=False,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=timeout_sec,
        )
        output = (result.stdout or '').strip()
        error = (result.stderr or '').strip()
        combined = '\n'.join(part for part in (output, error) if part).strip()
        return result.returncode, _extract_serial_numbers(combined), combined
    except subprocess.TimeoutExpired as exc:
        combined = '\n'.join(
            part.decode(errors='replace') if isinstance(part, bytes) else part
            for part in (exc.stdout, exc.stderr)
            if part
        ).strip()
        combined = (combined + '\n\nScan timed out.').strip()
        return -1, _extract_serial_numbers(combined), combined
    except Exception as exc:  # noqa: BLE001 - launch setup should report and abort cleanly
        return -1, [], f'Failed to run ros2 device scan: {exc}'


def _load_config(path: Path) -> dict:
    if yaml is None:
        raise RuntimeError('PyYAML is required to read Orbbec camera config.')
    if not path.exists():
        raise RuntimeError(f'Orbbec camera config does not exist: {path}')
    with path.open('r', encoding='utf-8') as infile:
        payload = yaml.safe_load(infile)
    return payload if isinstance(payload, dict) else {}


def _camera_pairs(payload: dict) -> list[dict[str, str]]:
    cameras = payload.get('cameras', [])
    if not isinstance(cameras, list):
        return []
    pairs = []
    for index, camera in enumerate(cameras, start=1):
        if not isinstance(camera, dict):
            continue
        slot = str(camera.get('slot', index)).strip() or str(index)
        serial_number = str(camera.get('serial_number', '')).strip()
        camera_name = str(camera.get('camera_name', '')).strip()
        if serial_number and camera_name:
            pairs.append({
                'slot': slot,
                'serial_number': serial_number,
                'camera_name': camera_name,
            })
    return pairs


def _selected_pairs(pairs: list[dict[str, str]], selected_text: str) -> list[dict[str, str]]:
    selected = selected_text.strip()
    if not selected or selected.lower() == 'all':
        return pairs
    tokens = {token.strip() for token in selected.split(',') if token.strip()}
    if not tokens:
        return pairs
    return [
        pair for pair in pairs
        if pair['slot'] in tokens or pair['camera_name'] in tokens or pair['serial_number'] in tokens
    ]


def _launch_setup(context, *args, **kwargs):
    del args, kwargs
    config_file = Path(LaunchConfiguration('config_file').perform(context)).expanduser()
    launch_file = LaunchConfiguration('orbbec_launch_file').perform(context).strip()
    enabled_cameras = LaunchConfiguration('enabled_cameras').perform(context)
    device_num_override = LaunchConfiguration('device_num').perform(context).strip()
    require_connected = _to_bool(LaunchConfiguration('require_connected').perform(context), True)
    scan_timeout_sec = _to_positive_float(
        LaunchConfiguration('scan_timeout_sec').perform(context),
        DEFAULT_SCAN_TIMEOUT_SEC,
    )
    watchdog_enabled = _to_bool(
        LaunchConfiguration('watchdog_enabled').perform(context),
        True,
    )
    payload = _load_config(config_file)
    pairs = _selected_pairs(_camera_pairs(payload), enabled_cameras)
    if not pairs:
        raise RuntimeError(
            f'No launchable Orbbec camera mappings found in {config_file} '
            f'for enabled_cameras={enabled_cameras!r}.'
        )

    connection_messages = []
    if require_connected:
        return_code, serials, scan_output = _scan_connected_serials(scan_timeout_sec)
        connected_serials = set(serials)
        disconnected_pairs = [pair for pair in pairs if pair['serial_number'] not in connected_serials]
        pairs = [pair for pair in pairs if pair['serial_number'] in connected_serials]
        if not pairs:
            configured = ', '.join(
                f'{pair["camera_name"]}({pair["serial_number"]})'
                for pair in disconnected_pairs
            )
            detected = ', '.join(serials) if serials else 'none'
            detail = f' Scan exit code={return_code}.' if return_code not in (0, None) else ''
            if scan_output:
                detail += f' Scan output: {scan_output}'
            raise RuntimeError(
                'No selected configured Orbbec cameras are connected; not starting camera nodes. '
                f'Configured: {configured or "none"}. Detected: {detected}.{detail}'
            )
        if disconnected_pairs:
            skipped = ', '.join(
                f'{pair["camera_name"]}({pair["serial_number"]})'
                for pair in disconnected_pairs
            )
            connection_messages.append(
                'Skipping disconnected Orbbec camera(s): ' + skipped
            )
        connection_messages.append(
            'Launching connected Orbbec camera(s): ' +
            ', '.join(f'{pair["camera_name"]}({pair["serial_number"]})' for pair in pairs)
        )

    config_launch_args = payload.get('orbbec_launch_args', {})
    launch_args = dict(DEFAULT_ORBBEC_LAUNCH_ARGS)
    if isinstance(config_launch_args, dict):
        launch_args.update(config_launch_args)

    for key in DEFAULT_ORBBEC_LAUNCH_ARGS:
        override = LaunchConfiguration(key).perform(context).strip()
        if override:
            launch_args[key] = override

    device_num = device_num_override or str(len(pairs))
    orbbec_launch_path = Path(get_package_share_directory('orbbec_camera')) / 'launch' / launch_file
    if not orbbec_launch_path.exists():
        raise RuntimeError(f'Orbbec launch file does not exist: {orbbec_launch_path}')

    actions = [LogInfo(msg=message) for message in connection_messages]
    if watchdog_enabled:
        watchdog_namespace = LaunchConfiguration('watchdog_namespace').perform(context).strip()
        actions.append(
            Node(
                package='orbbec_camera_launcher',
                executable='camera_watchdog',
                namespace=watchdog_namespace,
                name='supervisor',
                output='screen',
                parameters=[{
                    'camera_names': [pair['camera_name'] for pair in pairs],
                    'serial_numbers': [pair['serial_number'] for pair in pairs],
                    'orbbec_launch_file': launch_file,
                    'device_num': device_num,
                    'launch_args_json': json.dumps(launch_args),
                    'workspace_root': str(_workspace_root()),
                    'startup_timeout_sec': _to_positive_float(
                        LaunchConfiguration('watchdog_startup_timeout_sec').perform(context),
                        DEFAULT_WATCHDOG_STARTUP_TIMEOUT_SEC,
                    ),
                    'health_timeout_sec': _to_positive_float(
                        LaunchConfiguration('watchdog_health_timeout_sec').perform(context),
                        DEFAULT_WATCHDOG_HEALTH_TIMEOUT_SEC,
                    ),
                    'check_period_sec': _to_positive_float(
                        LaunchConfiguration('watchdog_check_period_sec').perform(context),
                        DEFAULT_WATCHDOG_CHECK_PERIOD_SEC,
                    ),
                    'restart_delay_sec': _to_positive_float(
                        LaunchConfiguration('watchdog_restart_delay_sec').perform(context),
                        DEFAULT_WATCHDOG_RESTART_DELAY_SEC,
                    ),
                    'restart_backoff_max_sec': _to_positive_float(
                        LaunchConfiguration('watchdog_restart_backoff_max_sec').perform(context),
                        DEFAULT_WATCHDOG_RESTART_BACKOFF_MAX_SEC,
                    ),
                }],
            )
        )
        return actions

    for pair in pairs:
        include_args = {
            'camera_name': pair['camera_name'],
            'serial_number': pair['serial_number'],
            'device_num': device_num,
        }
        include_args.update({key: _to_bool_text(value) for key, value in launch_args.items()})
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(orbbec_launch_path)),
                launch_arguments=include_args.items(),
            )
        )
    return actions


def generate_launch_description():
    launch_arguments = [
        DeclareLaunchArgument(
            'config_file',
            default_value=_repo_path('config', 'camera_bringup', 'orbbec_cameras.yaml'),
        ),
        DeclareLaunchArgument('enabled_cameras', default_value='all'),
        DeclareLaunchArgument('orbbec_launch_file', default_value='gemini_330_series.launch.py'),
        DeclareLaunchArgument('device_num', default_value=''),
        DeclareLaunchArgument('require_connected', default_value='true'),
        DeclareLaunchArgument('scan_timeout_sec', default_value=str(DEFAULT_SCAN_TIMEOUT_SEC)),
        DeclareLaunchArgument('watchdog_enabled', default_value='true'),
        DeclareLaunchArgument('watchdog_namespace', default_value='camera_watchdog'),
        DeclareLaunchArgument(
            'watchdog_startup_timeout_sec',
            default_value=str(DEFAULT_WATCHDOG_STARTUP_TIMEOUT_SEC),
        ),
        DeclareLaunchArgument(
            'watchdog_health_timeout_sec',
            default_value=str(DEFAULT_WATCHDOG_HEALTH_TIMEOUT_SEC),
        ),
        DeclareLaunchArgument(
            'watchdog_check_period_sec',
            default_value=str(DEFAULT_WATCHDOG_CHECK_PERIOD_SEC),
        ),
        DeclareLaunchArgument(
            'watchdog_restart_delay_sec',
            default_value=str(DEFAULT_WATCHDOG_RESTART_DELAY_SEC),
        ),
        DeclareLaunchArgument(
            'watchdog_restart_backoff_max_sec',
            default_value=str(DEFAULT_WATCHDOG_RESTART_BACKOFF_MAX_SEC),
        ),
    ]
    for key in DEFAULT_ORBBEC_LAUNCH_ARGS:
        launch_arguments.append(DeclareLaunchArgument(key, default_value=''))

    return LaunchDescription([
        _ros_domain_action(),
        *launch_arguments,
        OpaqueFunction(function=_launch_setup),
    ])
