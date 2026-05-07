import os
from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node


def _pick_cycle_pythonpath() -> str:
    paths = []
    for parent in Path(__file__).resolve().parents:
        build_path = parent / 'build' / 'pick_cycle'
        source_path = parent / 'src' / 'pick_cycle'
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
            package='pick_cycle',
            executable='pick_cycle_gui',
            name='pick_cycle_gui',
            output='screen',
            additional_env={
                'PYTHONPATH': _pick_cycle_pythonpath(),
            },
        ),
    ])
