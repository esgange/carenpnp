from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from datetime import datetime


def generate_launch_description() -> LaunchDescription:
    stamp = datetime.now().strftime('%d%m%Y')
    default_output_file = f'~/DOBOT_pickn_place/calibration/relmovl_speed_calibration_{stamp}.json'

    service_root_arg = DeclareLaunchArgument(
        'service_root',
        default_value='/dobot_bringup_ros2/srv',
        description='Dashboard/motion service root',
    )
    tcp_topic_arg = DeclareLaunchArgument(
        'tcp_topic',
        default_value='dobot_msgs_v4/msg/ToolVectorActual',
        description='Raw TCP stream topic',
    )
    scripts_dir_arg = DeclareLaunchArgument(
        'scripts_dir',
        default_value='~/.ros/motion_debug_scripts',
        description='Directory containing calibration scripts',
    )
    script_names_csv_arg = DeclareLaunchArgument(
        'script_names_csv',
        default_value='x_calibrate,y_calibrate,z_calibrate',
        description='Comma-separated script names (without .json)',
    )
    startup_cp_arg = DeclareLaunchArgument(
        'startup_cp',
        default_value='100',
        description='CP percentage applied before script execution',
    )
    startup_speed_factor_arg = DeclareLaunchArgument(
        'startup_speed_factor',
        default_value='50',
        description='SpeedFactor percentage applied before script execution',
    )
    goal_tolerance_mm_arg = DeclareLaunchArgument(
        'goal_tolerance_mm',
        default_value='2.0',
        description='Target tolerance per commanded MovL point',
    )
    settle_time_sec_arg = DeclareLaunchArgument(
        'settle_time_sec',
        default_value='0.15',
        description='Extra capture time after target hit',
    )
    segment_timeout_sec_arg = DeclareLaunchArgument(
        'segment_timeout_sec',
        default_value='20.0',
        description='Max wait per MovL segment',
    )
    output_file_arg = DeclareLaunchArgument(
        'output_file',
        default_value=default_output_file,
        description='Generated calibration JSON path',
    )
    save_raw_trace_arg = DeclareLaunchArgument(
        'save_raw_trace',
        default_value='true',
        description='Save raw TCP trace CSV',
    )
    raw_trace_file_arg = DeclareLaunchArgument(
        'raw_trace_file',
        default_value='',
        description='Optional explicit raw TCP trace CSV path',
    )
    skip_first_point_arg = DeclareLaunchArgument(
        'skip_first_point_in_script',
        default_value='true',
        description='Skip first point in each script for sampling (pre-position move)',
    )
    exclude_v_percents_arg = DeclareLaunchArgument(
        'exclude_v_percents_csv',
        default_value='100',
        description='Comma-separated v%% values to exclude from calibration fit',
    )
    min_command_distance_arg = DeclareLaunchArgument(
        'min_command_distance_mm',
        default_value='10.0',
        description='Minimum commanded axis travel required for a valid sample',
    )
    min_measured_distance_arg = DeclareLaunchArgument(
        'min_measured_distance_mm',
        default_value='5.0',
        description='Minimum measured axis travel required for a valid sample',
    )
    min_travel_ratio_arg = DeclareLaunchArgument(
        'min_travel_ratio',
        default_value='0.25',
        description='Minimum measured/commanded travel ratio required for a valid sample',
    )
    exclude_plateau_from_fit_arg = DeclareLaunchArgument(
        'exclude_plateau_from_fit',
        default_value='true',
        description='Auto-detect and exclude saturated plateau samples from fit',
    )
    plateau_min_samples_arg = DeclareLaunchArgument(
        'plateau_min_samples',
        default_value='5',
        description='Minimum number of speed samples needed for plateau detection',
    )
    plateau_min_speed_ratio_arg = DeclareLaunchArgument(
        'plateau_min_speed_ratio',
        default_value='0.95',
        description='Minimum speed ratio to max speed before plateau detection can start',
    )
    plateau_max_gain_ratio_arg = DeclareLaunchArgument(
        'plateau_max_gain_ratio',
        default_value='0.20',
        description='Maximum gain ratio vs baseline gain to classify plateau',
    )
    plateau_min_consecutive_steps_arg = DeclareLaunchArgument(
        'plateau_min_consecutive_steps',
        default_value='2',
        description='Consecutive near-zero-gain steps required to detect plateau start',
    )

    node = Node(
        package='movement_calibration',
        executable='movement_calibration',
        name='movement_calibration',
        output='screen',
        parameters=[
            {
                'service_root': LaunchConfiguration('service_root'),
                'tcp_topic': LaunchConfiguration('tcp_topic'),
                'scripts_dir': LaunchConfiguration('scripts_dir'),
                'script_names_csv': LaunchConfiguration('script_names_csv'),
                'startup_cp': LaunchConfiguration('startup_cp'),
                'startup_speed_factor': LaunchConfiguration('startup_speed_factor'),
                'goal_tolerance_mm': LaunchConfiguration('goal_tolerance_mm'),
                'settle_time_sec': LaunchConfiguration('settle_time_sec'),
                'segment_timeout_sec': LaunchConfiguration('segment_timeout_sec'),
                'output_file': LaunchConfiguration('output_file'),
                'save_raw_trace': LaunchConfiguration('save_raw_trace'),
                'raw_trace_file': LaunchConfiguration('raw_trace_file'),
                'skip_first_point_in_script': LaunchConfiguration('skip_first_point_in_script'),
                'exclude_v_percents_csv': LaunchConfiguration('exclude_v_percents_csv'),
                'min_command_distance_mm': LaunchConfiguration('min_command_distance_mm'),
                'min_measured_distance_mm': LaunchConfiguration('min_measured_distance_mm'),
                'min_travel_ratio': LaunchConfiguration('min_travel_ratio'),
                'exclude_plateau_from_fit': LaunchConfiguration('exclude_plateau_from_fit'),
                'plateau_min_samples': LaunchConfiguration('plateau_min_samples'),
                'plateau_min_speed_ratio': LaunchConfiguration('plateau_min_speed_ratio'),
                'plateau_max_gain_ratio': LaunchConfiguration('plateau_max_gain_ratio'),
                'plateau_min_consecutive_steps': LaunchConfiguration('plateau_min_consecutive_steps'),
            }
        ],
    )

    return LaunchDescription(
        [
            service_root_arg,
            tcp_topic_arg,
            scripts_dir_arg,
            script_names_csv_arg,
            startup_cp_arg,
            startup_speed_factor_arg,
            goal_tolerance_mm_arg,
            settle_time_sec_arg,
            segment_timeout_sec_arg,
            output_file_arg,
            save_raw_trace_arg,
            raw_trace_file_arg,
            skip_first_point_arg,
            exclude_v_percents_arg,
            min_command_distance_arg,
            min_measured_distance_arg,
            min_travel_ratio_arg,
            exclude_plateau_from_fit_arg,
            plateau_min_samples_arg,
            plateau_min_speed_ratio_arg,
            plateau_max_gain_ratio_arg,
            plateau_min_consecutive_steps_arg,
            node,
        ]
    )
