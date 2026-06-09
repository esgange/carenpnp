from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_path
from pathlib import Path
import json
import os


def _ros_bool(value, *, default=False) -> bool:
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    if isinstance(value, int):
        if value in (0, 1):
            return bool(value)
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in ('1', 'true', 'yes', 'on'):
            return True
        if normalized in ('0', 'false', 'no', 'off'):
            return False
    raise ValueError("[cr_robot_ros2] `ros_localhost_only` must be true/false or 1/0.")


def _workspace_root() -> Path:
    def looks_like_root(path: Path) -> bool:
        return (
            (path / 'config' / 'robot_bringup' / 'param.json').exists()
            or (path / 'src' / 'dobot_msgs_v4').exists()
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


def _default_config_path() -> str:
    workspace_config = _workspace_root() / 'config' / 'robot_bringup' / 'param.json'
    if workspace_config.exists():
        return str(workspace_config)
    return str(get_package_share_path('cr_robot_ros2') / 'config' / 'param.json')


def _default_station_config_path() -> str:
    return str(_workspace_root() / 'station_config')


def _load_station_config(path: Path) -> dict[str, str]:
    settings: dict[str, str] = {}
    with open(path, 'r', encoding='utf-8') as stream:
        for raw_line in stream:
            line = raw_line.strip()
            if not line or line.startswith('#'):
                continue
            if line.startswith('export '):
                line = line[len('export '):].strip()
            if '=' not in line:
                continue
            key, value = line.split('=', 1)
            value = value.strip()
            if len(value) >= 2 and value[0] == value[-1] and value[0] in ("'", '"'):
                value = value[1:-1]
            settings[key.strip()] = value
    return settings


def _station_value(settings: dict[str, str], *names: str, default=None):
    for name in names:
        value = settings.get(name)
        if value not in (None, ''):
            return value
    return default


def _station_has_robot_bringup(settings: dict[str, str]) -> bool:
    return any(
        key in settings
        for key in ('ROBOT_IP_ADDRESS', 'ROBOT_TYPE', 'ROBOT_NUMBER', 'ROS_LOCALHOST_ONLY')
    )


def _station_config_to_bringup_config(settings: dict[str, str]) -> dict:
    robot_ip = _station_value(settings, 'ROBOT_IP_ADDRESS', 'ip_address')
    if not robot_ip:
        raise KeyError("[cr_robot_ros2] Missing required station_config key `ROBOT_IP_ADDRESS`.")

    return {
        'robot_number': int(_station_value(settings, 'ROBOT_NUMBER', 'robot_number', default=1)),
        'ros_localhost_only': _station_value(
            settings,
            'ROS_LOCALHOST_ONLY',
            'ros_localhost_only',
            default=False,
        ),
        'node_info': [
            {
                'ip_address': robot_ip,
                'robot_type': _station_value(settings, 'ROBOT_TYPE', 'robot_type', default='cr5'),
            }
        ],
    }


def _launch_setup(context, *args, **kwargs):
    station_config_path = Path(LaunchConfiguration('station_config').perform(context)).expanduser()
    cfg = None
    if station_config_path.exists():
        station_settings = _load_station_config(station_config_path)
        if _station_has_robot_bringup(station_settings):
            cfg = _station_config_to_bringup_config(station_settings)

    if cfg is None:
        config_path = Path(LaunchConfiguration('config').perform(context)).expanduser()
        if not config_path.exists():
            raise FileNotFoundError(f"[cr_robot_ros2] param.json not found at: {config_path}")

        with open(config_path, 'r') as f:
            cfg = json.load(f)

    # Read high-level config
    robot_number = int(cfg.get('robot_number', 1))
    current_robot = int(cfg.get('current_robot', 1))
    ros_domain_id_value = cfg.get('ros_domain_id')
    ros_domain_id = None
    if ros_domain_id_value is not None and not (
        isinstance(ros_domain_id_value, str) and not ros_domain_id_value.strip()
    ):
        ros_domain_id = int(ros_domain_id_value)
        if ros_domain_id < 0 or ros_domain_id > 232:
            raise ValueError("[cr_robot_ros2] `ros_domain_id` must be between 0 and 232.")
    ros_localhost_only = '1' if _ros_bool(cfg.get('ros_localhost_only'), default=False) else '0'

    node_info = cfg.get('node_info', [])
    if not isinstance(node_info, list) or len(node_info) == 0:
        raise ValueError("[cr_robot_ros2] `node_info` must be a non-empty list in param.json.")

    # Clamp index (1-indexed in the JSON)
    idx = max(0, min(current_robot - 1, len(node_info) - 1))
    ni = node_info[idx]

    # Required + optional per-robot fields
    try:
        robot_ip = ni['ip_address']
    except KeyError:
        raise KeyError("[cr_robot_ros2] Missing required field `ip_address` in node_info entry.")

    params = {
        'robot_ip_address': robot_ip,
        'robot_type': ni.get('robot_type', 'cr5'),
        'trajectory_duration': float(ni.get('trajectory_duration', 0.3)),
        'robot_node_name': ni.get('robot_node_name', 'dobot_bringup_ros2'),
        'robot_number': robot_number,
    }

    # Bringup node (name set from JSON; parameters only from JSON)
    bringup = Node(
        package='cr_robot_ros2',
        executable='cr_robot_ros2_node',
        name=params['robot_node_name'],
        output='screen',
        parameters=[params],
    )

    env_actions = [
        SetEnvironmentVariable(name='ROS_LOCALHOST_ONLY', value=ros_localhost_only),
    ]
    if ros_domain_id is not None:
        env_actions.insert(0, SetEnvironmentVariable(name='ROS_DOMAIN_ID', value=str(ros_domain_id)))

    return env_actions + [bringup]

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


def generate_launch_description():
    return LaunchDescription([
        _ros_domain_action(),
        DeclareLaunchArgument(
            'config',
            default_value=_default_config_path(),
            description='Path to the param.json containing robot connection info.'
        ),
        DeclareLaunchArgument(
            'station_config',
            default_value=_default_station_config_path(),
            description='Path to station_config containing robot connection info.'
        ),
        OpaqueFunction(function=_launch_setup),
    ])
