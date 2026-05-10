from __future__ import annotations

import json
import os
from pathlib import Path

from launch.actions import OpaqueFunction, SetEnvironmentVariable


ROS_DOMAIN_ID_MIN = 0
ROS_DOMAIN_ID_MAX = 232


def _looks_like_workspace_root(path: Path) -> bool:
    return (
        (path / 'config' / 'dobot_bringup_v4' / 'param.json').exists()
        or (path / 'src' / 'dobot_msgs_v4').exists()
        or (path / 'docker-compose.yml').exists()
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
    workspace_config = _workspace_root() / 'config' / 'dobot_bringup_v4' / 'param.json'
    if workspace_config.exists():
        return workspace_config

    package_config = Path(__file__).resolve().parent.parent / 'config' / 'param.json'
    return package_config


def ros_domain_id(config_path: str | Path | None = None) -> str:
    path = Path(config_path).expanduser() if config_path is not None else _default_config_path()
    if not path.exists():
        return '0'

    data = json.loads(path.read_text())
    value = data.get('ros_domain_id', 0)
    domain_id = int(value)
    if domain_id < ROS_DOMAIN_ID_MIN or domain_id > ROS_DOMAIN_ID_MAX:
        raise ValueError(
            f'ros_domain_id must be between {ROS_DOMAIN_ID_MIN} and '
            f'{ROS_DOMAIN_ID_MAX}; got {domain_id} from {path}'
        )
    return str(domain_id)


def ros_localhost_only(config_path: str | Path | None = None) -> str:
    path = Path(config_path).expanduser() if config_path is not None else _default_config_path()
    if not path.exists():
        return '0'

    data = json.loads(path.read_text())
    value = data.get('ros_localhost_only', False)
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
        f'ros_localhost_only must be true/false or 1/0; got {value!r} from {path}'
    )


def ros_domain_env(config_path: str | Path | None = None) -> dict[str, str]:
    return {
        'ROS_DOMAIN_ID': ros_domain_id(config_path),
        'ROS_LOCALHOST_ONLY': ros_localhost_only(config_path),
    }


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
