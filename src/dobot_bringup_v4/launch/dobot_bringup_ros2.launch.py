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
            (path / 'config' / 'dobot_bringup_v4' / 'param.json').exists()
            or (path / 'src' / 'dobot_msgs_v4').exists()
            or (path / 'docker-compose.yml').exists()
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
    workspace_config = _workspace_root() / 'config' / 'dobot_bringup_v4' / 'param.json'
    if workspace_config.exists():
        return str(workspace_config)
    return str(get_package_share_path('cr_robot_ros2') / 'config' / 'param.json')


def _launch_setup(context, *args, **kwargs):
    config_path = Path(LaunchConfiguration('config').perform(context)).expanduser()
    if not config_path.exists():
        raise FileNotFoundError(f"[cr_robot_ros2] param.json not found at: {config_path}")

    with open(config_path, 'r') as f:
        cfg = json.load(f)

    # Read high-level config
    robot_number = int(cfg.get('robot_number', 1))
    current_robot = int(cfg.get('current_robot', 1))
    ros_domain_id = int(cfg.get('ros_domain_id', 0))
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

    return [
        SetEnvironmentVariable(name='ROS_DOMAIN_ID', value=str(ros_domain_id)),
        SetEnvironmentVariable(name='ROS_LOCALHOST_ONLY', value=ros_localhost_only),
        bringup,
    ]

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
        OpaqueFunction(function=_launch_setup),
    ])
