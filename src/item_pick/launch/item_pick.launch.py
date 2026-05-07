import os
from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node


def _item_pick_pythonpath() -> str:
    paths = []
    for parent in Path(__file__).resolve().parents:
        build_path = parent / 'build' / 'item_pick'
        source_path = parent / 'src' / 'item_pick'
        if build_path.exists() or source_path.exists():
            if build_path.exists():
                paths.append(str(build_path))
            if source_path.exists():
                paths.append(str(source_path))
            break

    current_pythonpath = os.environ.get('PYTHONPATH', '')
    if current_pythonpath:
        paths.append(current_pythonpath)
    return os.pathsep.join(paths)


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='item_pick',
            executable='item_pick',
            name='item_pick',
            output='screen',
            additional_env={
                'PYTHONPATH': _item_pick_pythonpath(),
            },
        ),
    ])
