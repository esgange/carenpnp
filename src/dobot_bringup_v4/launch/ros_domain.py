from __future__ import annotations

import json
import os
from pathlib import Path

from launch.actions import OpaqueFunction, SetEnvironmentVariable


ROS_DOMAIN_ID_MIN = 0
ROS_DOMAIN_ID_MAX = 232


def _looks_like_workspace_root(path: Path) -> bool:
    return (
        (path / 'station_config').exists()
        or
        (path / 'config' / 'robot_bringup' / 'param.json').exists()
        or (path / 'src' / 'dobot_msgs_v4').exists()
    )


def _workspace_root() -> Path:
    for name in ('DOBOT_PICKN_PLACE_ROOT', 'DOBOT_WORKSPACE_ROOT'):
        value = os.environ.get(name)
        if value:
            return Path(value).expanduser().resolve()

    for start in (Path(__file__).resolve(), Path.cwd()):
        path = start.expanduser().resolve()
        if path.is_file():
            path = path.parent
        for candidate in (path, *path.parents):
            if _looks_like_workspace_root(candidate):
                return candidate
    return Path.cwd().resolve()


def _default_config_path() -> Path:
    workspace_config = _workspace_root() / 'config' / 'robot_bringup' / 'param.json'
    if workspace_config.exists():
        return workspace_config

    package_config = Path(__file__).resolve().parent.parent / 'config' / 'param.json'
    return package_config


def _default_station_config_path() -> Path:
    return _workspace_root() / 'station_config'


def _load_station_config(path: Path) -> dict[str, str]:
    settings: dict[str, str] = {}
    if not path.exists():
        return settings
    for raw_line in path.read_text(encoding='utf-8').splitlines():
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


def _bool_to_ros(value, *, default=False) -> str:
    if value is None:
        return '1' if default else '0'
    if isinstance(value, bool):
        return '1' if value else '0'
    if isinstance(value, int) and value in (0, 1):
        return str(value)
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in ('1', 'true', 'yes', 'on'):
            return '1'
        if normalized in ('0', 'false', 'no', 'off'):
            return '0'
    raise ValueError(
        f'ros_localhost_only must be true/false or 1/0; got {value!r}'
    )


def ros_domain_id(config_path: str | Path | None = None) -> str:
    path = Path(config_path).expanduser() if config_path is not None else _default_config_path()
    if not path.exists():
        return ''

    data = json.loads(path.read_text())
    value = data.get('ros_domain_id')
    if value is None or (isinstance(value, str) and not value.strip()):
        return ''
    domain_id = int(value)
    if domain_id < ROS_DOMAIN_ID_MIN or domain_id > ROS_DOMAIN_ID_MAX:
        raise ValueError(
            f'ros_domain_id must be between {ROS_DOMAIN_ID_MIN} and '
            f'{ROS_DOMAIN_ID_MAX}; got {domain_id} from {path}'
        )
    return str(domain_id)


def ros_localhost_only(config_path: str | Path | None = None) -> str:
    station_settings = _load_station_config(_default_station_config_path())
    station_value = station_settings.get('ROS_LOCALHOST_ONLY')
    if station_value not in (None, ''):
        return _bool_to_ros(station_value)

    path = Path(config_path).expanduser() if config_path is not None else _default_config_path()
    if not path.exists():
        return '0'

    data = json.loads(path.read_text())
    try:
        return _bool_to_ros(data.get('ros_localhost_only', False))
    except ValueError as exc:
        raise ValueError(f'{exc} from {path}') from exc


def ros_domain_env(config_path: str | Path | None = None) -> dict[str, str]:
    env = {
        'ROS_LOCALHOST_ONLY': ros_localhost_only(config_path),
    }
    domain_id = ros_domain_id(config_path)
    if domain_id:
        env['ROS_DOMAIN_ID'] = domain_id
    return env


def ros_domain_action(config_path: str | Path | None = None) -> OpaqueFunction:
    def _set_environment(_context):
        env = ros_domain_env(config_path)
        os.environ.update(env)
        return [
            SetEnvironmentVariable(name=name, value=value)
            for name, value in env.items()
        ]

    os.environ.update(ros_domain_env(config_path))
    return OpaqueFunction(function=_set_environment)
