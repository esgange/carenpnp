import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

try:
    import yaml
except Exception:  # pragma: no cover - launch-time dependency
    yaml = None


CAMERA_FORWARD_ARGS = (
    'device_preset',
    'enable_color',
    'enable_depth',
    'depth_registration',
    'align_target_stream',
    'align_mode',
    'enable_frame_sync',
    'enable_temporal_filter',
    'color_width',
    'color_height',
    'color_fps',
    'depth_width',
    'depth_height',
    'depth_fps',
    'enable_point_cloud',
)


OPTIONAL_NODE_PARAMETER_OVERRIDES = {
    'item_pick': {
        'item_pick_tf_only_mode': ('item_pick.parameters.tf_only_mode', 'tf_only_mode'),
        'item_pose_wait_timeout_sec': ('item_pick.parameters.item_pose_wait_timeout_sec', 'item_pose_wait_timeout_sec'),
        'item_pick_speed_mm_s': ('item_pick.parameters.ee_intercept_speed_mm_s', 'ee_intercept_speed_mm_s'),
        'item_x_offset_mm': ('item_pick.parameters.item_x_offset_mm', 'item_x_offset_mm'),
        'item_y_offset_mm': ('item_pick.parameters.item_y_offset_mm', 'item_y_offset_mm'),
        'item_standoff_z_mm': ('item_pick.parameters.item_standoff_z_mm', 'item_standoff_z_mm'),
        'item_approach_z_up_mm': ('item_pick.parameters.approach_z_up_mm', 'approach_z_up_mm'),
        'item_final_z_up_mm': ('item_pick.parameters.final_z_up_mm', 'final_z_up_mm'),
        'pre_pick_settling_time_sec': ('item_pick.parameters.pre_pick_settling_time_sec', 'pre_pick_settling_time_sec'),
        'pick_settling_time_sec': ('item_pick.parameters.pick_settling_time_sec', 'pick_settling_time_sec'),
        'item_command_hysteresis_sec': ('item_pick.parameters.command_hysteresis_sec', 'command_hysteresis_sec'),
    },
    'tray_intercept': {
        'tray_intercept_tf_only_mode': ('tray_intercept.parameters.tf_only_mode', 'tf_only_mode'),
        'tray_vector_wait_timeout_sec': ('tray_intercept.parameters.tray_vector_wait_timeout_sec', 'tray_vector_wait_timeout_sec'),
        'tray_intercept_speed_mm_s': ('tray_intercept.parameters.ee_intercept_speed_mm_s', 'ee_intercept_speed_mm_s'),
        'tray_ee_final_pose_angle_deg': ('tray_intercept.parameters.ee_final_pose_angle_deg', 'ee_final_pose_angle_deg'),
        'tray_intercept_x_offset_mm': ('tray_intercept.parameters.tray_intercept_x_offset_mm', 'tray_intercept_x_offset_mm'),
        'tray_intercept_y_offset_mm': ('tray_intercept.parameters.tray_intercept_y_offset_mm', 'tray_intercept_y_offset_mm'),
        'tray_standoff_z_mm': ('tray_intercept.parameters.tray_standoff_z_mm', 'tray_standoff_z_mm'),
        'tray_follow_distance_mm': ('tray_intercept.parameters.follow_distance_mm', 'follow_distance_mm'),
        'tray_post_follow_z_up_mm': ('tray_intercept.parameters.post_follow_z_up_mm', 'post_follow_z_up_mm'),
        'tray_release_grip_enabled': ('tray_intercept.parameters.release_grip_enabled', 'release_grip_enabled'),
        'tray_command_hysteresis_sec': ('tray_intercept.parameters.command_hysteresis_sec', 'command_hysteresis_sec'),
    },
}


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


def _package_launch(package: str, launch_file: str) -> str:
    return str(Path(get_package_share_directory(package)) / 'launch' / launch_file)


def _include(package: str, launch_file: str, launch_arguments: dict[str, str]):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_package_launch(package, launch_file)),
        launch_arguments=launch_arguments.items(),
    )


def _launch_value(context, name: str) -> str:
    return LaunchConfiguration(name).perform(context).strip()


def _to_bool(value: str) -> bool:
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def _stringify(value) -> str:
    if isinstance(value, bool):
        return 'true' if value else 'false'
    return str(value)


def _load_settings(path: Path) -> dict:
    if yaml is None:
        raise RuntimeError('PyYAML is required to read the headless runtime settings file.')
    if not path.exists():
        raise RuntimeError(f'Headless runtime settings file does not exist: {path}')
    with path.open('r', encoding='utf-8') as infile:
        payload = yaml.safe_load(infile)
    if not isinstance(payload, dict):
        raise RuntimeError(f'Headless runtime settings file is not a YAML map: {path}')
    return payload


def _nested(settings: dict, dotted_key: str):
    current = settings
    for part in dotted_key.split('.'):
        if not isinstance(current, dict) or part not in current:
            return None
        current = current[part]
    return current


def _path_text(value, workspace_root: Path) -> str:
    text = _stringify(value).strip()
    if not text:
        return ''
    expanded = os.path.expandvars(text)
    path = Path(expanded).expanduser()
    if not path.is_absolute():
        path = workspace_root / path
    return str(path.resolve())


def _setting(
    context,
    settings: dict,
    workspace_root: Path,
    arg_name: str,
    dotted_key: str,
    *,
    required: bool = True,
    path: bool = False,
) -> str:
    override = _launch_value(context, arg_name)
    if override:
        return _path_text(override, workspace_root) if path else override

    value = _nested(settings, dotted_key)
    if value is None:
        if required:
            raise RuntimeError(
                f'Missing required headless runtime setting "{dotted_key}" '
                f'or launch override "{arg_name}:=<value>".'
            )
        return ''

    text = _path_text(value, workspace_root) if path else _stringify(value).strip()
    if required and not text:
        raise RuntimeError(
            f'Empty required headless runtime setting "{dotted_key}" '
            f'or launch override "{arg_name}:=<value>".'
        )
    return text


def _optional_setting(context, settings: dict, workspace_root: Path, arg_name: str, dotted_key: str, *, path: bool = False) -> str:
    return _setting(context, settings, workspace_root, arg_name, dotted_key, required=False, path=path)


def _setting_with_fallback(
    context,
    settings: dict,
    workspace_root: Path,
    arg_name: str,
    dotted_key: str,
    fallback_arg_name: str,
    fallback_dotted_key: str,
    *,
    path: bool = False,
) -> str:
    configured = _optional_setting(context, settings, workspace_root, arg_name, dotted_key, path=path)
    if configured:
        return configured
    return _setting(context, settings, workspace_root, fallback_arg_name, fallback_dotted_key, path=path)


def _optional_setting_with_fallback(
    context,
    settings: dict,
    workspace_root: Path,
    arg_name: str,
    dotted_key: str,
    fallback_arg_name: str,
    fallback_dotted_key: str,
    *,
    path: bool = False,
) -> str:
    configured = _optional_setting(context, settings, workspace_root, arg_name, dotted_key, path=path)
    if configured:
        return configured
    return _optional_setting(context, settings, workspace_root, fallback_arg_name, fallback_dotted_key, path=path)


def _flag(context, settings: dict, workspace_root: Path, arg_name: str, dotted_key: str) -> bool:
    return _to_bool(_setting(context, settings, workspace_root, arg_name, dotted_key))


def _existing_file(path_text: str, label: str) -> str:
    if path_text and not Path(path_text).is_file():
        raise RuntimeError(f'{label} does not exist or is not a file: {path_text}')
    return path_text


def _yaml_has_top_level_key(path: Path, key: str) -> bool:
    try:
        with path.open('r', encoding='utf-8') as infile:
            payload = yaml.safe_load(infile)
    except Exception as exc:
        raise RuntimeError(f'Failed to read YAML profile {path}: {exc}') from exc
    return isinstance(payload, dict) and key in payload


def _single_profile_in_dir(profiles_dir: str, profile_key: str, label: str) -> str:
    base = Path(profiles_dir)
    matches = []
    if base.exists() and base.is_dir():
        for path in sorted(base.iterdir()):
            if path.is_file() and path.suffix.lower() in ('.yaml', '.yml') and _yaml_has_top_level_key(path, profile_key):
                matches.append(path.resolve())
    if len(matches) == 1:
        return str(matches[0])
    if not matches:
        raise RuntimeError(f'{label} selected profile not found in {base}')
    raise RuntimeError(f'{label} selected profile is ambiguous in {base}: ' + ', '.join(path.name for path in matches))


def _selected_profile_path(
    context,
    settings: dict,
    workspace_root: Path,
    mode: str,
    arg_name: str,
    dotted_key: str,
    profiles_dir: str,
    profile_key: str,
    label: str,
) -> str:
    configured = _optional_setting(context, settings, workspace_root, arg_name, dotted_key, path=True)
    if configured:
        return _existing_file(configured, f'{label} selected profile')
    if mode == 'online':
        return _single_profile_in_dir(profiles_dir, profile_key, label)
    return ''


def _profiles_dir(context, settings: dict, workspace_root: Path, mode: str, arg_name: str, dotted_key: str, fallback_arg: str, fallback_key: str) -> str:
    configured = _optional_setting(context, settings, workspace_root, arg_name, dotted_key, path=True)
    if configured:
        return configured
    if mode == 'online':
        return _setting(context, settings, workspace_root, 'runtime_dir', 'runtime_dir', path=True)
    return _setting(context, settings, workspace_root, fallback_arg, fallback_key, path=True)


def _add_optional_passthrough(
    context,
    settings: dict,
    launch_arguments: dict[str, str],
    arg_name: str,
    dotted_key: str,
    child_arg_name: str,
) -> None:
    value = _launch_value(context, arg_name)
    if not value:
        configured = _nested(settings, dotted_key)
        if configured is None or _stringify(configured).strip() == '':
            return
        value = _stringify(configured).strip()
    launch_arguments[child_arg_name] = value


def _launch_setup(context, *args, **kwargs):
    del args, kwargs
    workspace_root = _workspace_root()
    settings_path = Path(_launch_value(context, 'runtime_settings_file')).expanduser()
    if not settings_path.is_absolute():
        settings_path = workspace_root / settings_path
    settings = _load_settings(settings_path.resolve())

    mode = _setting(context, settings, workspace_root, 'mode', 'mode').lower()
    if mode not in ('online', 'offline'):
        raise RuntimeError(f'mode must be "online" or "offline", got {mode!r}')

    item_profiles_dir = _profiles_dir(
        context,
        settings,
        workspace_root,
        mode,
        'item_profiles_dir',
        'profiles.item_profiles_dir',
        'offline_item_profiles_dir',
        'offline_item_profiles_dir',
    )
    tray_profiles_dir = _profiles_dir(
        context,
        settings,
        workspace_root,
        mode,
        'tray_profiles_dir',
        'profiles.tray_profiles_dir',
        'offline_tray_profiles_dir',
        'offline_tray_profiles_dir',
    )
    load_runtime_settings = _setting(
        context,
        settings,
        workspace_root,
        'load_runtime_settings',
        'launch.load_runtime_settings',
    )
    start_visualization = _setting(
        context,
        settings,
        workspace_root,
        'start_visualization',
        'launch.start_visualization',
    )

    common_camera_args = {
        'color_topic': _setting(context, settings, workspace_root, 'color_topic', 'common.color_topic'),
        'depth_topic': _setting(context, settings, workspace_root, 'depth_topic', 'common.depth_topic'),
        'camera_info_topic': _setting(context, settings, workspace_root, 'camera_info_topic', 'common.camera_info_topic'),
        'camera_control_service_root': _setting(
            context,
            settings,
            workspace_root,
            'camera_control_service_root',
            'common.camera_control_service_root',
        ),
    }
    item_camera_args = {
        'color_topic': _setting_with_fallback(
            context,
            settings,
            workspace_root,
            'item_color_topic',
            'item_detect.color_topic',
            'color_topic',
            'common.color_topic',
        ),
        'depth_topic': _setting_with_fallback(
            context,
            settings,
            workspace_root,
            'item_depth_topic',
            'item_detect.depth_topic',
            'depth_topic',
            'common.depth_topic',
        ),
        'camera_info_topic': _setting_with_fallback(
            context,
            settings,
            workspace_root,
            'item_camera_info_topic',
            'item_detect.camera_info_topic',
            'camera_info_topic',
            'common.camera_info_topic',
        ),
        'camera_control_service_root': _setting_with_fallback(
            context,
            settings,
            workspace_root,
            'item_camera_control_service_root',
            'item_detect.camera_control_service_root',
            'camera_control_service_root',
            'common.camera_control_service_root',
        ),
    }
    common_calibration_args = {
        'use_calibration': _setting(context, settings, workspace_root, 'use_calibration', 'common.use_calibration'),
        'calibration_dir': _setting(context, settings, workspace_root, 'calibration_dir', 'common.calibration_dir', path=True),
        'calibration_file': _optional_setting(context, settings, workspace_root, 'calibration_file', 'common.calibration_file', path=True),
        'parent_frame': _setting(context, settings, workspace_root, 'calibration_parent_frame', 'common.calibration_parent_frame'),
        'child_frame': _setting(context, settings, workspace_root, 'calibration_child_frame', 'common.calibration_child_frame'),
        'camera_frame': _optional_setting(context, settings, workspace_root, 'camera_frame', 'common.camera_frame'),
    }
    item_calibration_args = {
        'use_calibration': _setting_with_fallback(
            context,
            settings,
            workspace_root,
            'item_use_calibration',
            'item_detect.use_calibration',
            'use_calibration',
            'common.use_calibration',
        ),
        'calibration_dir': _setting_with_fallback(
            context,
            settings,
            workspace_root,
            'item_calibration_dir',
            'item_detect.calibration_dir',
            'calibration_dir',
            'common.calibration_dir',
            path=True,
        ),
        'calibration_file': _optional_setting_with_fallback(
            context,
            settings,
            workspace_root,
            'item_calibration_file',
            'item_detect.calibration_file',
            'calibration_file',
            'common.calibration_file',
            path=True,
        ),
        'parent_frame': _setting_with_fallback(
            context,
            settings,
            workspace_root,
            'item_calibration_parent_frame',
            'item_detect.calibration_parent_frame',
            'calibration_parent_frame',
            'common.calibration_parent_frame',
        ),
        'child_frame': _setting_with_fallback(
            context,
            settings,
            workspace_root,
            'item_calibration_child_frame',
            'item_detect.calibration_child_frame',
            'calibration_child_frame',
            'common.calibration_child_frame',
        ),
        'camera_frame': _optional_setting_with_fallback(
            context,
            settings,
            workspace_root,
            'item_camera_frame',
            'item_detect.camera_frame',
            'camera_frame',
            'common.camera_frame',
        ),
    }

    actions = []
    if _flag(context, settings, workspace_root, 'launch_cameras', 'launch.cameras'):
        camera_config_file = _existing_file(
            _setting(context, settings, workspace_root, 'camera_config_file', 'camera.config_file', path=True),
            'Camera runtime settings file',
        )
        camera_args = {
            'config_file': camera_config_file,
            'enabled_cameras': _setting(context, settings, workspace_root, 'enabled_cameras', 'camera.enabled_cameras'),
            'orbbec_launch_file': _setting(context, settings, workspace_root, 'orbbec_launch_file', 'camera.orbbec_launch_file'),
            'device_num': _optional_setting(context, settings, workspace_root, 'camera_device_num', 'camera.device_num'),
        }
        for key in CAMERA_FORWARD_ARGS:
            value = _launch_value(context, f'camera_{key}')
            if not value:
                configured = _nested(settings, f'camera.launch_args.{key}')
                if configured is not None and _stringify(configured).strip() != '':
                    value = _stringify(configured).strip()
            if value:
                camera_args[key] = value
        actions.append(_include('orbbec_camera_launcher', 'camera_headless.launch.py', camera_args))

    if _flag(context, settings, workspace_root, 'launch_item_detect', 'launch.item_detect'):
        actions.append(_include('item_perception', 'item_detect.launch.py', {
            'params_file': _optional_setting(context, settings, workspace_root, 'item_detect_params_file', 'item_detect.params_file', path=True),
            'profiles_dir': item_profiles_dir,
            'selected_profile_path': _selected_profile_path(
                context,
                settings,
                workspace_root,
                mode,
                'item_selected_profile_path',
                'item_detect.selected_profile_path',
                item_profiles_dir,
                'item_detect',
                'Item detect',
            ),
            'runtime_settings_file': _existing_file(
                _setting(context, settings, workspace_root, 'item_detect_runtime_settings_file', 'item_detect.runtime_settings_file', path=True),
                'Item detect runtime settings file',
            ),
            'selected_profile_export_file': _setting(
                context,
                settings,
                workspace_root,
                'item_selected_profile_export_file',
                'item_detect.selected_profile_export_file',
                path=True,
            ),
            'bin_pose_topic': _setting(context, settings, workspace_root, 'item_pose_topic', 'item_detect.pose_topic'),
            'bin_item_pose_array_topic': _setting(
                context,
                settings,
                workspace_root,
                'bin_item_pose_array_topic',
                'item_detect.pose_array_topic',
            ),
            'start_visualization': start_visualization,
            'headless': 'true',
            **item_camera_args,
            **item_calibration_args,
        }))

    if _flag(context, settings, workspace_root, 'launch_item_pick', 'launch.item_pick'):
        item_pick_args = {
            'headless': 'true',
            'runtime_settings_file': _existing_file(
                _setting(context, settings, workspace_root, 'item_pick_runtime_settings_file', 'item_pick.runtime_settings_file', path=True),
                'Item pick runtime settings file',
            ),
            'load_runtime_settings': load_runtime_settings,
            'item_profile_state_file': _setting(
                context,
                settings,
                workspace_root,
                'item_selected_profile_export_file',
                'item_detect.selected_profile_export_file',
                path=True,
            ),
            'motion_service_root': _setting(context, settings, workspace_root, 'motion_service_root', 'common.motion_service_root'),
            'gripper_do_service': _optional_setting(context, settings, workspace_root, 'gripper_do_service', 'common.gripper_do_service'),
            'item_pose_topic': _setting(context, settings, workspace_root, 'item_pose_topic', 'item_detect.pose_topic'),
            'start_sequence_service': _setting(
                context,
                settings,
                workspace_root,
                'item_pick_start_sequence_service',
                'item_pick.start_sequence_service',
            ),
            'track_service': _setting(context, settings, workspace_root, 'item_pick_track_service', 'item_pick.track_service'),
            'track_status_service': _setting(
                context,
                settings,
                workspace_root,
                'item_pick_track_status_service',
                'item_pick.track_status_service',
            ),
            'item_seek_complete_service': _setting(
                context,
                settings,
                workspace_root,
                'item_seek_complete_service',
                'item_detect.seek_complete_service',
            ),
            'robot_goal_frame_id': _setting(context, settings, workspace_root, 'robot_goal_frame_id', 'common.robot_goal_frame_id'),
            'robot_gripper_frame_id': _setting(context, settings, workspace_root, 'robot_gripper_frame_id', 'common.robot_gripper_frame_id'),
            'camera_safety_frame_id': item_calibration_args['child_frame'],
        }
        for arg_name, (dotted_key, child_arg_name) in OPTIONAL_NODE_PARAMETER_OVERRIDES['item_pick'].items():
            _add_optional_passthrough(context, settings, item_pick_args, arg_name, dotted_key, child_arg_name)
        actions.append(_include('item_pick', 'item_pick.launch.py', item_pick_args))

    if _flag(context, settings, workspace_root, 'launch_tray_detect', 'launch.tray_detect'):
        actions.append(_include('tray_perception', 'tray_detect.launch.py', {
            'params_file': _optional_setting(context, settings, workspace_root, 'tray_detect_params_file', 'tray_detect.params_file', path=True),
            'profiles_dir': tray_profiles_dir,
            'selected_profile_path': _selected_profile_path(
                context,
                settings,
                workspace_root,
                mode,
                'tray_selected_profile_path',
                'tray_detect.selected_profile_path',
                tray_profiles_dir,
                'tray_detect',
                'Tray detect',
            ),
            'runtime_settings_file': _existing_file(
                _setting(context, settings, workspace_root, 'tray_detect_runtime_settings_file', 'tray_detect.runtime_settings_file', path=True),
                'Tray detect runtime settings file',
            ),
            'tray_pose_topic': _setting(context, settings, workspace_root, 'tray_pose_topic', 'tray_detect.pose_topic'),
            'tray_vector_topic': _setting(context, settings, workspace_root, 'tray_vector_topic', 'tray_detect.vector_topic'),
            'start_visualization': start_visualization,
            'headless': 'true',
            **common_camera_args,
            **common_calibration_args,
        }))

    if _flag(context, settings, workspace_root, 'launch_tray_intercept', 'launch.tray_intercept'):
        tray_intercept_args = {
            'headless': 'true',
            'runtime_settings_file': _existing_file(
                _setting(
                    context,
                    settings,
                    workspace_root,
                    'tray_intercept_runtime_settings_file',
                    'tray_intercept.runtime_settings_file',
                    path=True,
                ),
                'Tray intercept runtime settings file',
            ),
            'load_runtime_settings': load_runtime_settings,
            'motion_service_root': _setting(context, settings, workspace_root, 'motion_service_root', 'common.motion_service_root'),
            'tray_vector_topic': _setting(context, settings, workspace_root, 'tray_vector_topic', 'tray_detect.vector_topic'),
            'tray_axis_overlay_topic': _setting(
                context,
                settings,
                workspace_root,
                'tray_axis_overlay_topic',
                'tray_intercept.axis_overlay_topic',
            ),
            'start_sequence_service': _setting(
                context,
                settings,
                workspace_root,
                'tray_intercept_start_sequence_service',
                'tray_intercept.start_sequence_service',
            ),
            'track_service': _setting(
                context,
                settings,
                workspace_root,
                'tray_intercept_track_service',
                'tray_intercept.track_service',
            ),
            'track_status_service': _setting(
                context,
                settings,
                workspace_root,
                'tray_intercept_track_status_service',
                'tray_intercept.track_status_service',
            ),
            'tray_dimensions_service': _setting(
                context,
                settings,
                workspace_root,
                'tray_dimensions_service',
                'tray_detect.dimensions_service',
            ),
            'tray_seek_complete_service': _setting(
                context,
                settings,
                workspace_root,
                'tray_seek_complete_service',
                'tray_detect.seek_complete_service',
            ),
            'robot_goal_frame_id': _setting(context, settings, workspace_root, 'robot_goal_frame_id', 'common.robot_goal_frame_id'),
        }
        for arg_name, (dotted_key, child_arg_name) in OPTIONAL_NODE_PARAMETER_OVERRIDES['tray_intercept'].items():
            _add_optional_passthrough(context, settings, tray_intercept_args, arg_name, dotted_key, child_arg_name)
        actions.append(_include('tray_intercept', 'tray_intercept.launch.py', tray_intercept_args))

    if _flag(context, settings, workspace_root, 'launch_rviz', 'launch.rviz'):
        actions.append(_include('dobot_rviz', 'dobot_rviz.launch.py', {}))

    return actions


def _override_arg(name: str) -> DeclareLaunchArgument:
    return DeclareLaunchArgument(name, default_value='')


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            'runtime_settings_file',
            default_value=_repo_path('config', 'robot_cell_orchestrator', 'robot_runtime_headless_settings.yaml'),
        ),
        *[
            _override_arg(name)
            for name in (
                'mode',
                'runtime_dir',
                'offline_item_profiles_dir',
                'offline_tray_profiles_dir',
                'item_profiles_dir',
                'tray_profiles_dir',
                'launch_cameras',
                'launch_item_detect',
                'launch_item_pick',
                'launch_tray_detect',
                'launch_tray_intercept',
                'launch_rviz',
                'load_runtime_settings',
                'start_visualization',
                'camera_config_file',
                'enabled_cameras',
                'orbbec_launch_file',
                'camera_device_num',
                'color_topic',
                'depth_topic',
                'camera_info_topic',
                'camera_control_service_root',
                'use_calibration',
                'calibration_dir',
                'calibration_file',
                'calibration_parent_frame',
                'calibration_child_frame',
                'camera_frame',
                'robot_goal_frame_id',
                'robot_gripper_frame_id',
                'motion_service_root',
                'gripper_do_service',
                'item_detect_params_file',
                'item_selected_profile_path',
                'item_detect_runtime_settings_file',
                'item_selected_profile_export_file',
                'item_color_topic',
                'item_depth_topic',
                'item_camera_info_topic',
                'item_camera_control_service_root',
                'item_use_calibration',
                'item_calibration_dir',
                'item_calibration_file',
                'item_calibration_parent_frame',
                'item_calibration_child_frame',
                'item_camera_frame',
                'item_pose_topic',
                'bin_item_pose_array_topic',
                'item_seek_complete_service',
                'item_pick_runtime_settings_file',
                'item_pick_start_sequence_service',
                'item_pick_track_service',
                'item_pick_track_status_service',
                'item_pick_tf_only_mode',
                'item_pose_wait_timeout_sec',
                'item_pick_speed_mm_s',
                'item_x_offset_mm',
                'item_y_offset_mm',
                'item_standoff_z_mm',
                'item_approach_z_up_mm',
                'item_final_z_up_mm',
                'pre_pick_settling_time_sec',
                'pick_settling_time_sec',
                'item_command_hysteresis_sec',
                'tray_detect_params_file',
                'tray_selected_profile_path',
                'tray_detect_runtime_settings_file',
                'tray_pose_topic',
                'tray_vector_topic',
                'tray_axis_overlay_topic',
                'tray_dimensions_service',
                'tray_seek_complete_service',
                'tray_intercept_runtime_settings_file',
                'tray_intercept_start_sequence_service',
                'tray_intercept_track_service',
                'tray_intercept_track_status_service',
                'tray_intercept_tf_only_mode',
                'tray_vector_wait_timeout_sec',
                'tray_intercept_speed_mm_s',
                'tray_ee_final_pose_angle_deg',
                'tray_intercept_x_offset_mm',
                'tray_intercept_y_offset_mm',
                'tray_standoff_z_mm',
                'tray_follow_distance_mm',
                'tray_post_follow_z_up_mm',
                'tray_release_grip_enabled',
                'tray_command_hysteresis_sec',
            )
        ],
    ]
    for key in CAMERA_FORWARD_ARGS:
        args.append(_override_arg(f'camera_{key}'))

    return LaunchDescription([
        *args,
        OpaqueFunction(function=_launch_setup),
    ])
