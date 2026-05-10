from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch_ros.actions import Node
import json
import os
from pathlib import Path


def _workspace_root() -> Path:
    def looks_like_root(path: Path) -> bool:
        return (
            (path / 'src').exists() and
            (
                (path / 'README.md').exists()
                or (path / 'docker-compose.yml').exists()
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


def _bringup_config_path() -> Path:
    workspace_config = _workspace_root() / 'config' / 'dobot_bringup_v4' / 'param.json'
    if workspace_config.exists():
        return workspace_config
    bringup_share = get_package_share_path('cr_robot_ros2')
    return bringup_share / 'config' / 'param.json'

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
    cfg_path = _bringup_config_path()
    cfg = json.loads(cfg_path.read_text())
    idx = int(cfg.get('current_robot', 1)) - 1
    robot_type = cfg['node_info'][idx].get('robot_type', 'cr5')

    rviz_share = get_package_share_path('dobot_rviz')
    urdf_path = rviz_share / f'urdf/{robot_type}_robot.urdf'
    if not urdf_path.exists():
        urdf_path = rviz_share / 'urdf/cr5_robot.urdf'  # fallback

    rviz_config = rviz_share / 'rviz/urdf.rviz'

    # Feed REAL joint states into robot_state_publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': urdf_path.read_text()}],
        remappings=[('joint_states', '/joint_states_robot')],  # << key line
    )

    # Optional: anchor base_link to a world frame you can build on
    world_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_base',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link']
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', str(rviz_config)],
    )

    # Do NOT start joint_state_publisher or joint_state_publisher_gui here
    return LaunchDescription([
        _ros_domain_action(),
        rsp,
        world_to_base,
        rviz,
    ])
