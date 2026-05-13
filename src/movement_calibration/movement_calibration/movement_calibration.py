import csv
import json
import math
import os
import re
import sys
import threading
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node

from dobot_msgs_v4.msg import ToolVectorActual
from dobot_msgs_v4.srv import CP, MovL, SpeedFactor


SCRIPT_POINT_PATTERN = re.compile(
    r'^\s*(?:POINT(?:\s+\d+(?:/\d+)?)?\s+)?(MovJ|MovL)\s*:\s*(.+?)\s*$',
    re.IGNORECASE,
)
TRANSLATION_AXES = ('x', 'y', 'z')


def workspace_root() -> Path:
    def looks_like_root(path: Path) -> bool:
        return (
            (path / 'src').exists() and
            (
                (path / 'README.md').exists()
                or (path / 'docker-compose.yml').exists()
                or (path / 'src' / 'dobot_msgs_v4').exists()
            )
        )

    def find_from(start: Path) -> Path | None:
        path = start.expanduser().resolve()
        if path.is_file():
            path = path.parent
        for candidate in (path, *path.parents):
            if looks_like_root(candidate):
                return candidate
        return None

    for name in ('DOBOT_PICKN_PLACE_ROOT', 'DOBOT_WORKSPACE_ROOT'):
        value = os.environ.get(name)
        if value:
            return find_from(Path(value)) or Path(value).expanduser().resolve()

    candidates = [Path.cwd(), Path(__file__).resolve()]
    for name in ('COLCON_PREFIX_PATH', 'AMENT_PREFIX_PATH'):
        for token in os.environ.get(name, '').split(os.pathsep):
            if not token:
                continue
            prefix = Path(token)
            candidates.append(prefix)
            if 'install' in prefix.parts:
                candidates.append(Path(*prefix.parts[:prefix.parts.index('install')]))

    for candidate in candidates:
        found = find_from(candidate)
        if found is not None:
            return found
    return Path.cwd().resolve()


def workspace_path(*parts: str) -> Path:
    return workspace_root().joinpath(*parts)


def _default_output_file_path() -> Path:
    stamp = datetime.now().strftime('%d%m%Y')
    filename = f'relmovl_speed_calibration_{stamp}.json'
    calibration_dir = workspace_path('calibration')
    try:
        calibration_dir.mkdir(parents=True, exist_ok=True)
        return calibration_dir / filename
    except Exception:
        return workspace_path('calibration', 'relmovl_speed_calibration.json')


@dataclass(frozen=True)
class TcpSample:
    stamp_monotonic: float
    x: float
    y: float
    z: float
    rx: float
    ry: float
    rz: float

    def axis_value(self, axis_index: int) -> float:
        if axis_index == 0:
            return self.x
        if axis_index == 1:
            return self.y
        if axis_index == 2:
            return self.z
        raise IndexError(f'Invalid translation axis index {axis_index}')


@dataclass(frozen=True)
class ScriptMove:
    script_name: str
    point_index: int
    mode: bool
    target: tuple[float, float, float, float, float, float]
    v_percent: Optional[int]
    a_percent: Optional[int]
    use_tool: bool
    raw_command: str


@dataclass(frozen=True)
class SegmentMeasurement:
    script_name: str
    point_index: int
    axis_name: str
    v_percent: int
    a_percent: Optional[int]
    command_distance_mm: float
    measured_distance_mm: float
    elapsed_sec: float
    speed_mm_s: float
    mean_speed_mm_s: float
    peak_speed_mm_s: float
    reached_target: bool
    sample_count: int
    start_axis_mm: float
    target_axis_mm: float
    final_axis_mm: float


class MovementCalibrationNode(Node):
    def __init__(self) -> None:
        super().__init__('movement_calibration')
        self._exit_code = 1

        self._service_root = self.declare_parameter(
            'service_root', '/dobot_bringup_ros2/srv').value
        self._tcp_topic = self.declare_parameter(
            'tcp_topic', 'dobot_msgs_v4/msg/ToolVectorActual').value
        self._scripts_dir = Path(
            str(self.declare_parameter(
                'scripts_dir',
                str(workspace_path('config', 'motion_calibrate')),
            ).value)
        ).expanduser()
        self._script_names_csv = str(self.declare_parameter(
            'script_names_csv', 'x_calibrate,y_calibrate,z_calibrate').value)
        default_output_file = str(_default_output_file_path())
        output_file_param = str(
            self.declare_parameter('output_file', default_output_file).value
        ).strip()
        self._output_file = Path(output_file_param if output_file_param else default_output_file).expanduser()
        self._raw_trace_file = str(self.declare_parameter('raw_trace_file', '').value).strip()

        requested_startup_cp = int(self.declare_parameter('startup_cp', 100).value)
        requested_startup_speed_factor = int(self.declare_parameter('startup_speed_factor', 50).value)
        self._startup_cp = max(1, min(100, requested_startup_cp))
        self._startup_speed_factor = max(1, min(100, requested_startup_speed_factor))
        if requested_startup_cp != self._startup_cp:
            self.get_logger().warn(
                f'startup_cp out of range ({requested_startup_cp}); clamped to {self._startup_cp}.'
            )
        if requested_startup_speed_factor != self._startup_speed_factor:
            self.get_logger().warn(
                'startup_speed_factor out of range '
                f'({requested_startup_speed_factor}); clamped to {self._startup_speed_factor}.'
            )
        self.get_logger().info(
            f'Using startup settings: CP={self._startup_cp}, '
            f'SpeedFactor={self._startup_speed_factor}.'
        )
        self._goal_tolerance_mm = float(self.declare_parameter('goal_tolerance_mm', 2.0).value)
        self._settle_time_sec = float(self.declare_parameter('settle_time_sec', 0.15).value)
        self._segment_timeout_sec = float(self.declare_parameter('segment_timeout_sec', 20.0).value)
        self._fresh_tcp_timeout_sec = float(self.declare_parameter('fresh_tcp_timeout_sec', 5.0).value)
        self._service_wait_timeout_sec = float(self.declare_parameter('service_wait_timeout_sec', 15.0).value)
        self._service_call_timeout_sec = float(self.declare_parameter('service_call_timeout_sec', 8.0).value)
        self._skip_first_point_in_script = bool(
            self.declare_parameter('skip_first_point_in_script', True).value)
        self._save_raw_trace = bool(self.declare_parameter('save_raw_trace', True).value)
        exclude_v_raw_value = self.declare_parameter(
            'exclude_v_percents_csv',
            '100',
            ParameterDescriptor(dynamic_typing=True),
        ).value
        self._exclude_v_percents = self._parse_v_percent_exclusions(
            self._normalize_exclude_v_param(exclude_v_raw_value)
        )
        self._min_command_distance_mm = max(
            0.0, float(self.declare_parameter('min_command_distance_mm', 10.0).value))
        self._min_measured_distance_mm = max(
            0.0, float(self.declare_parameter('min_measured_distance_mm', 5.0).value))
        self._min_travel_ratio = min(
            1.0, max(0.0, float(self.declare_parameter('min_travel_ratio', 0.25).value)))
        self._exclude_plateau_from_fit = bool(
            self.declare_parameter('exclude_plateau_from_fit', True).value)
        self._plateau_min_samples = max(
            4, int(self.declare_parameter('plateau_min_samples', 5).value))
        self._plateau_min_speed_ratio = min(
            1.0, max(0.0, float(self.declare_parameter('plateau_min_speed_ratio', 0.95).value)))
        self._plateau_max_gain_ratio = max(
            0.0, float(self.declare_parameter('plateau_max_gain_ratio', 0.20).value))
        self._plateau_min_consecutive_steps = max(
            1, int(self.declare_parameter('plateau_min_consecutive_steps', 2).value))

        self._cp_client = self.create_client(CP, f'{self._service_root}/CP')
        self._speed_factor_client = self.create_client(
            SpeedFactor, f'{self._service_root}/SpeedFactor')
        self._movl_client = self.create_client(MovL, f'{self._service_root}/MovL')
        self.create_subscription(ToolVectorActual, self._tcp_topic, self._tcp_callback, 50)

        self._lock = threading.Lock()
        self._latest_tcp_sample: Optional[TcpSample] = None
        self._latest_tcp_seq = 0

        self._started = False
        self._start_timer = self.create_timer(0.2, self._start_once)

    def _start_once(self) -> None:
        if self._started:
            return
        self._started = True
        self._start_timer.cancel()
        worker = threading.Thread(target=self._run_worker, daemon=True)
        worker.start()

    def _tcp_callback(self, msg: ToolVectorActual) -> None:
        sample = TcpSample(
            stamp_monotonic=time.monotonic(),
            x=float(msg.x),
            y=float(msg.y),
            z=float(msg.z),
            rx=float(msg.rx),
            ry=float(msg.ry),
            rz=float(msg.rz),
        )
        with self._lock:
            self._latest_tcp_sample = sample
            self._latest_tcp_seq += 1

    def _run_worker(self) -> None:
        try:
            success = self._run_calibration_sequence()
            if success:
                self.get_logger().info('Movement calibration completed.')
                self._exit_code = 0
            else:
                self.get_logger().error('Movement calibration failed.')
                self._exit_code = 1
        except Exception as exc:
            self.get_logger().error(f'Movement calibration crashed: {exc}')
            self._exit_code = 1
        finally:
            rclpy.shutdown()

    @property
    def exit_code(self) -> int:
        return int(self._exit_code)

    def _run_calibration_sequence(self) -> bool:
        script_names = self._parse_script_names(self._script_names_csv)
        if not script_names:
            self.get_logger().error('No script names provided in script_names_csv.')
            return False
        missing_script_paths = self._find_missing_script_paths(script_names)
        if missing_script_paths:
            self._prompt_missing_scripts(missing_script_paths)
            return False

        self.get_logger().info(
            f'Calibration scripts={script_names}, scripts_dir="{self._scripts_dir}"')
        self.get_logger().info(
            f'Startup settings: CP={self._startup_cp}, SpeedFactor={self._startup_speed_factor}')
        self.get_logger().info(
            'Sample filters: '
            f'exclude_v={sorted(self._exclude_v_percents)}, '
            f'min_cmd_dist={self._min_command_distance_mm:.1f}mm, '
            f'min_measured_dist={self._min_measured_distance_mm:.1f}mm, '
            f'min_travel_ratio={self._min_travel_ratio:.2f}'
        )
        self.get_logger().info(
            'Fit settings: '
            f'plateau_exclusion={self._exclude_plateau_from_fit}, '
            f'plateau_min_samples={self._plateau_min_samples}, '
            f'plateau_min_speed_ratio={self._plateau_min_speed_ratio:.2f}, '
            f'plateau_max_gain_ratio={self._plateau_max_gain_ratio:.2f}, '
            f'plateau_min_consecutive_steps={self._plateau_min_consecutive_steps}'
        )

        if not self._wait_for_service(self._cp_client, 'CP'):
            return False
        if not self._wait_for_service(self._speed_factor_client, 'SpeedFactor'):
            return False
        if not self._wait_for_service(self._movl_client, 'MovL'):
            return False
        if self._wait_for_fresh_tcp_sample() is None:
            return False

        cp_request = CP.Request()
        cp_request.r = max(1, min(100, int(self._startup_cp)))
        cp_response = self._call_service(self._cp_client, cp_request, f'CP({cp_request.r})')
        if cp_response is None or int(getattr(cp_response, 'res', -1)) < 0:
            return False

        sf_request = SpeedFactor.Request()
        sf_request.ratio = max(1, min(100, int(self._startup_speed_factor)))
        sf_response = self._call_service(
            self._speed_factor_client, sf_request, f'SpeedFactor({sf_request.ratio})')
        if sf_response is None or int(getattr(sf_response, 'res', -1)) < 0:
            return False

        all_measurements: list[SegmentMeasurement] = []
        raw_trace_rows: list[list[object]] = []

        for script_name in script_names:
            script_path = self._scripts_dir / f'{script_name}.json'
            moves = self._load_script_moves(script_name, script_path)
            if not moves:
                self.get_logger().warn(f'Skipping "{script_name}" (no MovL points).')
                continue

            self.get_logger().info(
                f'Running script "{script_name}" with {len(moves)} MovL points...')

            for move_idx, move in enumerate(moves):
                collect_sample = not (self._skip_first_point_in_script and move_idx == 0)
                measurement = self._execute_move_and_measure(move, collect_sample, raw_trace_rows)
                if measurement is None:
                    return False
                if not collect_sample:
                    continue
                include_measurement, rejection_reason = self._should_include_measurement(measurement)
                if not include_measurement:
                    self.get_logger().warn(
                        f'[{measurement.script_name} #{measurement.point_index}] ignored for fit: '
                        f'{rejection_reason} '
                        f'(axis={measurement.axis_name}, v={measurement.v_percent}%, '
                        f'cmd={measurement.command_distance_mm:.2f}mm, '
                        f'meas={measurement.measured_distance_mm:.2f}mm, '
                        f'elapsed={measurement.elapsed_sec:.3f}s, '
                        f'reached={measurement.reached_target})'
                    )
                    continue
                all_measurements.append(measurement)
                self.get_logger().info(
                    f'[{measurement.script_name} #{measurement.point_index}] axis={measurement.axis_name} '
                    f'v={measurement.v_percent}% speed={measurement.speed_mm_s:.2f} mm/s '
                    f'elapsed={measurement.elapsed_sec:.3f}s reached={measurement.reached_target}')

        if not all_measurements:
            self.get_logger().error('No valid measured segments were collected.')
            return False

        payload = self._build_calibration_payload(script_names, all_measurements)
        self._write_json(self._output_file, payload)
        self.get_logger().info(f'Wrote calibration file: {self._output_file}')

        if self._save_raw_trace and raw_trace_rows:
            raw_trace_path = self._resolve_raw_trace_path()
            self._write_raw_trace_csv(raw_trace_path, raw_trace_rows)
            self.get_logger().info(f'Wrote raw TCP trace: {raw_trace_path}')

        return True

    def _parse_script_names(self, csv_value: str) -> list[str]:
        names: list[str] = []
        for token in csv_value.split(','):
            name = token.strip()
            if not name:
                continue
            if name.lower().endswith('.json'):
                name = name[:-5]
            if name and name not in names:
                names.append(name)
        return names

    def _parse_v_percent_exclusions(self, csv_value: str) -> set[int]:
        excluded: set[int] = set()
        for token in csv_value.split(','):
            value_text = token.strip()
            if not value_text:
                continue
            try:
                value = int(round(float(value_text)))
            except ValueError:
                continue
            if 1 <= value <= 100:
                excluded.add(value)
        return excluded

    def _normalize_exclude_v_param(self, raw_value: object) -> str:
        if isinstance(raw_value, str):
            return raw_value
        if isinstance(raw_value, bool):
            return '1' if raw_value else '0'
        if isinstance(raw_value, (int, float)):
            return str(int(raw_value))
        if isinstance(raw_value, (list, tuple)):
            tokens: list[str] = []
            for item in raw_value:
                if isinstance(item, str):
                    text = item.strip()
                    if text:
                        tokens.append(text)
                    continue
                if isinstance(item, bool):
                    tokens.append('1' if item else '0')
                    continue
                if isinstance(item, (int, float)):
                    tokens.append(str(int(item)))
            return ','.join(tokens)
        return str(raw_value)

    def _should_include_measurement(self, measurement: SegmentMeasurement) -> tuple[bool, str]:
        if not measurement.reached_target:
            return False, 'target not reached'
        if measurement.v_percent in self._exclude_v_percents:
            return False, f'v={measurement.v_percent}% is excluded'

        command_abs_mm = abs(measurement.command_distance_mm)
        measured_abs_mm = abs(measurement.measured_distance_mm)
        if command_abs_mm < self._min_command_distance_mm:
            return False, (
                f'command distance {command_abs_mm:.2f}mm < min_command_distance_mm '
                f'{self._min_command_distance_mm:.2f}mm'
            )
        if measured_abs_mm < self._min_measured_distance_mm:
            return False, (
                f'measured distance {measured_abs_mm:.2f}mm < min_measured_distance_mm '
                f'{self._min_measured_distance_mm:.2f}mm'
            )
        if command_abs_mm > 1e-6:
            travel_ratio = measured_abs_mm / command_abs_mm
            if travel_ratio < self._min_travel_ratio:
                return False, (
                    f'travel ratio {travel_ratio:.2f} < min_travel_ratio '
                    f'{self._min_travel_ratio:.2f}'
                )
        return True, 'ok'

    def _find_missing_script_paths(self, script_names: list[str]) -> list[Path]:
        missing_paths: list[Path] = []
        for script_name in script_names:
            script_path = self._scripts_dir / f'{script_name}.json'
            if not script_path.exists():
                missing_paths.append(script_path)
        return missing_paths

    def _prompt_missing_scripts(self, missing_script_paths: list[Path]) -> None:
        expected_defaults = {'x_calibrate', 'y_calibrate', 'z_calibrate'}
        configured = {name.strip() for name in self._parse_script_names(self._script_names_csv)}
        if configured == expected_defaults and len(missing_script_paths) == 3:
            self.get_logger().error(
                'No X/Y/Z calibration scripts created yet. '
                'Expected files: x_calibrate.json, y_calibrate.json, z_calibrate.json'
            )
        else:
            missing_names = ', '.join(path.name for path in missing_script_paths)
            self.get_logger().error(f'Missing calibration script files: {missing_names}')

        self.get_logger().error(
            f'Create the scripts in motion_debug GUI under "{self._scripts_dir}", then rerun movement_calibration.'
        )
        if not sys.stdin or not sys.stdin.isatty():
            return
        try:
            input('Movement calibration cannot start. Create scripts and press Enter to exit...')
        except EOFError:
            return

    def _load_script_moves(self, script_name: str, script_path: Path) -> list[ScriptMove]:
        if not script_path.exists():
            self.get_logger().error(f'Script not found: {script_path}')
            return []

        try:
            with open(script_path, 'r', encoding='utf-8') as script_file:
                root = json.load(script_file)
        except Exception as exc:
            self.get_logger().error(f'Failed to read script "{script_path}": {exc}')
            return []

        points = root.get('points')
        if not isinstance(points, list):
            self.get_logger().error(f'Script "{script_path}" has no valid points list.')
            return []

        moves: list[ScriptMove] = []
        for index, point in enumerate(points, start=1):
            move = self._parse_script_point(script_name, index, point)
            if move is not None:
                moves.append(move)
        return moves

    def _parse_script_point(
        self,
        script_name: str,
        point_index: int,
        point,
    ) -> Optional[ScriptMove]:
        if not isinstance(point, dict):
            return None

        mode = bool(point.get('mode', False))
        use_tool = bool(point.get('use_tool', False))

        motion_type = str(point.get('motion_type', '')).strip()
        values = point.get('values')
        motion_args = point.get('motion_args')

        if motion_type and isinstance(values, list):
            if motion_type.strip().lower() not in {'movl', 'moll'}:
                return None
            if len(values) != 6:
                return None
            try:
                target = tuple(float(value) for value in values)
            except (TypeError, ValueError):
                return None
            arg_tokens = [str(arg).strip() for arg in motion_args] if isinstance(motion_args, list) else []
            v_percent, a_percent = self._extract_v_a_from_tokens(arg_tokens)
            raw_command = f'MovL: {",".join(f"{value:.3f}" for value in target)}'
            return ScriptMove(
                script_name=script_name,
                point_index=point_index,
                mode=mode,
                target=target,
                v_percent=v_percent,
                a_percent=a_percent,
                use_tool=use_tool,
                raw_command=raw_command,
            )

        command = str(point.get('command', '')).strip()
        match = SCRIPT_POINT_PATTERN.match(command)
        if match is None:
            return None
        if match.group(1).strip().lower() != 'movl':
            return None

        payload = match.group(2).strip()
        tokens = [token.strip() for token in payload.split(',') if token.strip()]
        if len(tokens) < 6:
            return None
        try:
            target = tuple(float(token) for token in tokens[:6])
        except ValueError:
            return None

        v_percent, a_percent = self._extract_v_a_from_tokens(tokens[6:])
        return ScriptMove(
            script_name=script_name,
            point_index=point_index,
            mode=mode,
            target=target,
            v_percent=v_percent,
            a_percent=a_percent,
            use_tool=use_tool,
            raw_command=command,
        )

    def _extract_v_a_from_tokens(self, tokens: list[str]) -> tuple[Optional[int], Optional[int]]:
        v_percent: Optional[int] = None
        a_percent: Optional[int] = None
        for token in tokens:
            for sub_token in token.replace(';', ',').split(','):
                key_value = sub_token.strip()
                if '=' not in key_value:
                    continue
                key, raw_value = key_value.split('=', 1)
                key = key.strip().lower()
                raw_value = raw_value.strip()
                if not key or not raw_value:
                    continue
                try:
                    value = int(round(float(raw_value)))
                except ValueError:
                    continue
                value = max(1, min(100, value))
                if key in {'v', 'speed', 'speed_l', 'speedl'}:
                    v_percent = value
                elif key in {'a', 'acc', 'acc_l', 'accl'}:
                    a_percent = value
        return v_percent, a_percent

    def _wait_for_service(self, client, service_name: str) -> bool:
        start_time = time.monotonic()
        while rclpy.ok():
            if client.wait_for_service(timeout_sec=0.3):
                return True
            if (time.monotonic() - start_time) >= self._service_wait_timeout_sec:
                break
        self.get_logger().error(f'Timeout waiting for {service_name} service.')
        return False

    def _call_service(self, client, request, label: str):
        self.get_logger().info(f'SEND {label}')
        future = client.call_async(request)
        start_time = time.monotonic()
        while rclpy.ok() and not future.done():
            if (time.monotonic() - start_time) >= self._service_call_timeout_sec:
                self.get_logger().error(f'Timeout waiting response for {label}')
                return None
            time.sleep(0.01)

        exception = future.exception()
        if exception is not None:
            self.get_logger().error(f'Exception from {label}: {exception}')
            return None

        response = future.result()
        if response is None:
            self.get_logger().error(f'No response from {label}')
            return None

        result_code = int(getattr(response, 'res', -1))
        robot_return = str(getattr(response, 'robot_return', '')).strip()
        if result_code < 0:
            if robot_return:
                self.get_logger().error(f'FAIL {label}: res={result_code}, robot_return={robot_return}')
            else:
                self.get_logger().error(f'FAIL {label}: res={result_code}')
            return response

        if robot_return:
            self.get_logger().info(f'OK {label}: {robot_return}')
        else:
            self.get_logger().info(f'OK {label}')
        return response

    def _wait_for_fresh_tcp_sample(self) -> Optional[TcpSample]:
        start_time = time.monotonic()
        while rclpy.ok():
            sample, _ = self._snapshot_tcp()
            if sample is not None:
                age = time.monotonic() - sample.stamp_monotonic
                if age <= 1.0:
                    return sample
            if (time.monotonic() - start_time) >= self._fresh_tcp_timeout_sec:
                break
            time.sleep(0.02)

        self.get_logger().error(
            f'No fresh TCP samples on topic "{self._tcp_topic}" within {self._fresh_tcp_timeout_sec:.1f}s.')
        return None

    def _snapshot_tcp(self) -> tuple[Optional[TcpSample], int]:
        with self._lock:
            return self._latest_tcp_sample, self._latest_tcp_seq

    def _dominant_axis(self, start_sample: TcpSample, move: ScriptMove) -> tuple[int, str, float]:
        target_xyz = move.target[:3]
        deltas = [
            float(target_xyz[0]) - start_sample.x,
            float(target_xyz[1]) - start_sample.y,
            float(target_xyz[2]) - start_sample.z,
        ]
        dominant_index = max(range(3), key=lambda idx: abs(deltas[idx]))
        return dominant_index, TRANSLATION_AXES[dominant_index], deltas[dominant_index]

    def _build_movl_request(self, move: ScriptMove) -> MovL.Request:
        request = MovL.Request()
        request.mode = bool(move.mode)
        request.a = move.target[0]
        request.b = move.target[1]
        request.c = move.target[2]
        request.d = move.target[3]
        request.e = move.target[4]
        request.f = move.target[5]

        args: list[str] = []
        if move.v_percent is not None:
            args.append(f'v={move.v_percent}')
        if move.a_percent is not None:
            args.append(f'a={move.a_percent}')
        if move.use_tool:
            args.append('tool=1')
        request.param_value = [','.join(args)] if args else []
        return request

    def _execute_move_and_measure(
        self,
        move: ScriptMove,
        collect_sample: bool,
        raw_trace_rows: list[list[object]],
    ) -> Optional[SegmentMeasurement]:
        start_sample = self._wait_for_fresh_tcp_sample()
        if start_sample is None:
            return None
        start_seq = self._snapshot_tcp()[1]

        axis_index, axis_name, command_distance_mm = self._dominant_axis(start_sample, move)
        target_axis_mm = move.target[axis_index]

        request = self._build_movl_request(move)
        point_label = f'{move.script_name}#{move.point_index}'
        v_text = f'v={move.v_percent}' if move.v_percent is not None else 'v=?'
        a_text = f'a={move.a_percent}' if move.a_percent is not None else 'a=?'
        send_label = (
            f'{point_label} MovL target=({request.a:.3f},{request.b:.3f},{request.c:.3f},'
            f'{request.d:.3f},{request.e:.3f},{request.f:.3f}) {v_text},{a_text}'
        )

        response = self._call_service(self._movl_client, request, send_label)
        if response is None or int(getattr(response, 'res', -1)) < 0:
            return None

        segment_samples: list[TcpSample] = []
        reached_target = False
        last_seq = start_seq
        segment_start_monotonic = start_sample.stamp_monotonic
        deadline = time.monotonic() + self._segment_timeout_sec

        while rclpy.ok():
            now = time.monotonic()
            if now > deadline:
                break

            latest_sample, latest_seq = self._snapshot_tcp()
            if latest_sample is None:
                time.sleep(0.01)
                continue

            if latest_seq == last_seq:
                time.sleep(0.005)
                continue

            last_seq = latest_seq
            segment_samples.append(latest_sample)
            if self._save_raw_trace:
                raw_trace_rows.append([
                    move.script_name,
                    move.point_index,
                    axis_name,
                    move.v_percent if move.v_percent is not None else '',
                    move.a_percent if move.a_percent is not None else '',
                    latest_sample.stamp_monotonic - segment_start_monotonic,
                    latest_sample.x,
                    latest_sample.y,
                    latest_sample.z,
                    latest_sample.rx,
                    latest_sample.ry,
                    latest_sample.rz,
                ])

            axis_value = latest_sample.axis_value(axis_index)
            if abs(axis_value - target_axis_mm) <= self._goal_tolerance_mm:
                reached_target = True
                break

        if reached_target and self._settle_time_sec > 1e-6:
            settle_deadline = time.monotonic() + self._settle_time_sec
            while rclpy.ok() and time.monotonic() < settle_deadline:
                latest_sample, latest_seq = self._snapshot_tcp()
                if latest_sample is None or latest_seq == last_seq:
                    time.sleep(0.005)
                    continue
                last_seq = latest_seq
                segment_samples.append(latest_sample)
                if self._save_raw_trace:
                    raw_trace_rows.append([
                        move.script_name,
                        move.point_index,
                        axis_name,
                        move.v_percent if move.v_percent is not None else '',
                        move.a_percent if move.a_percent is not None else '',
                        latest_sample.stamp_monotonic - segment_start_monotonic,
                        latest_sample.x,
                        latest_sample.y,
                        latest_sample.z,
                        latest_sample.rx,
                        latest_sample.ry,
                        latest_sample.rz,
                    ])

        final_sample = segment_samples[-1] if segment_samples else start_sample
        final_axis_mm = final_sample.axis_value(axis_index)
        measured_distance_mm = final_axis_mm - start_sample.axis_value(axis_index)
        elapsed_sec = max(0.0, final_sample.stamp_monotonic - start_sample.stamp_monotonic)

        speed_mm_s, mean_speed_mm_s, peak_speed_mm_s = self._estimate_speed_mm_s(
            start_sample=start_sample,
            segment_samples=segment_samples,
            axis_index=axis_index,
            target_axis_mm=target_axis_mm,
        )

        if speed_mm_s is None:
            speed_mm_s = 0.0
        if mean_speed_mm_s is None:
            mean_speed_mm_s = 0.0
        if peak_speed_mm_s is None:
            peak_speed_mm_s = 0.0

        v_percent = move.v_percent if move.v_percent is not None else 0
        return SegmentMeasurement(
            script_name=move.script_name,
            point_index=move.point_index,
            axis_name=axis_name,
            v_percent=v_percent,
            a_percent=move.a_percent,
            command_distance_mm=command_distance_mm,
            measured_distance_mm=measured_distance_mm,
            elapsed_sec=elapsed_sec,
            speed_mm_s=speed_mm_s,
            mean_speed_mm_s=mean_speed_mm_s,
            peak_speed_mm_s=peak_speed_mm_s,
            reached_target=reached_target,
            sample_count=len(segment_samples),
            start_axis_mm=start_sample.axis_value(axis_index),
            target_axis_mm=target_axis_mm,
            final_axis_mm=final_axis_mm,
        )

    def _estimate_speed_mm_s(
        self,
        start_sample: TcpSample,
        segment_samples: list[TcpSample],
        axis_index: int,
        target_axis_mm: float,
    ) -> tuple[Optional[float], Optional[float], Optional[float]]:
        if not segment_samples:
            return None, None, None

        series: list[tuple[float, float]] = [
            (start_sample.stamp_monotonic, start_sample.axis_value(axis_index))
        ]
        series.extend((sample.stamp_monotonic, sample.axis_value(axis_index)) for sample in segment_samples)
        if len(series) < 2:
            return None, None, None

        abs_velocities: list[float] = []
        for index in range(1, len(series)):
            prev_t, prev_pos = series[index - 1]
            now_t, now_pos = series[index]
            dt = now_t - prev_t
            if dt <= 1e-6:
                continue
            abs_velocities.append(abs((now_pos - prev_pos) / dt))

        if not abs_velocities:
            return None, None, None

        mean_speed = sum(abs_velocities) / float(len(abs_velocities))
        peak_speed = max(abs_velocities)
        start_axis_mm = series[0][1]
        command_delta_mm = target_axis_mm - start_axis_mm
        progress_speed = self._estimate_progress_band_speed(series, start_axis_mm, command_delta_mm)
        if progress_speed is not None:
            return progress_speed, mean_speed, peak_speed

        sorted_velocities = sorted(abs_velocities)
        lower_idx = int(math.floor(0.20 * (len(sorted_velocities) - 1)))
        upper_idx = int(math.ceil(0.80 * (len(sorted_velocities) - 1)))
        core = sorted_velocities[lower_idx: upper_idx + 1]
        if not core:
            core = sorted_velocities
        core_mean = sum(core) / float(len(core))
        return core_mean, mean_speed, peak_speed

    def _estimate_progress_band_speed(
        self,
        series: list[tuple[float, float]],
        start_axis_mm: float,
        command_delta_mm: float,
    ) -> Optional[float]:
        if abs(command_delta_mm) < 1e-3:
            return None

        t10 = self._progress_crossing_time(series, start_axis_mm, command_delta_mm, 0.10)
        t90 = self._progress_crossing_time(series, start_axis_mm, command_delta_mm, 0.90)
        if t10 is None or t90 is None or t90 <= t10:
            return None

        return abs((0.80 * command_delta_mm) / (t90 - t10))

    def _progress_crossing_time(
        self,
        series: list[tuple[float, float]],
        start_axis_mm: float,
        command_delta_mm: float,
        threshold: float,
    ) -> Optional[float]:
        def _progress(pos_mm: float) -> float:
            return (pos_mm - start_axis_mm) / command_delta_mm

        prev_time, prev_pos = series[0]
        prev_progress = _progress(prev_pos)
        if prev_progress >= threshold:
            return prev_time

        for now_time, now_pos in series[1:]:
            now_progress = _progress(now_pos)
            if now_progress >= threshold:
                span = now_progress - prev_progress
                if abs(span) <= 1e-9:
                    return now_time
                ratio = (threshold - prev_progress) / span
                ratio = max(0.0, min(1.0, ratio))
                return prev_time + ratio * (now_time - prev_time)
            prev_time = now_time
            prev_progress = now_progress

        return None

    def _build_calibration_payload(
        self,
        script_names: list[str],
        measurements: list[SegmentMeasurement],
    ) -> dict:
        grouped: dict[str, dict[int, list[float]]] = {axis: {} for axis in TRANSLATION_AXES}
        for measurement in measurements:
            if measurement.v_percent <= 0:
                continue
            axis_group = grouped.setdefault(measurement.axis_name, {})
            axis_group.setdefault(measurement.v_percent, []).append(measurement.speed_mm_s)

        axis_models: dict[str, dict] = {}
        global_group: dict[int, list[float]] = {}

        for axis_name in TRANSLATION_AXES:
            axis_group = grouped.get(axis_name, {})
            axis_samples: list[dict] = []
            for v_percent in sorted(axis_group.keys()):
                speeds = axis_group[v_percent]
                if not speeds:
                    continue
                mean_speed = sum(speeds) / float(len(speeds))
                variance = 0.0
                if len(speeds) > 1:
                    variance = sum((speed - mean_speed) ** 2 for speed in speeds) / float(len(speeds))
                std_dev = math.sqrt(variance)
                axis_samples.append({
                    'v_percent': int(v_percent),
                    'speed_mm_s': mean_speed,
                    'std_mm_s': std_dev,
                    'count': len(speeds),
                })
                global_group.setdefault(int(v_percent), []).extend(speeds)

            fit_samples, plateau_info = self._exclude_plateau_samples(axis_samples)
            fit_points = [
                (float(sample['v_percent']), float(sample['speed_mm_s'])) for sample in fit_samples
            ]
            fit = self._linear_fit(fit_points)
            suggested_max_speed = 0.0
            if axis_samples:
                observed_max_speed = max(sample['speed_mm_s'] for sample in axis_samples)
                suggested_max_speed = observed_max_speed
                predicted_v100 = self._interpolate_speed_for_v(fit_samples, 100.0)
                if predicted_v100 is not None:
                    if bool(plateau_info.get('excluded')):
                        suggested_max_speed = min(observed_max_speed, predicted_v100)
                    else:
                        suggested_max_speed = max(observed_max_speed, predicted_v100)
            if bool(plateau_info.get('excluded')):
                self.get_logger().info(
                    f'Plateau excluded for axis "{axis_name}": '
                    f'start_v={plateau_info.get("start_v_percent")}, '
                    f'excluded={plateau_info.get("excluded_count")} sample(s).'
                )

            axis_models[axis_name] = {
                'samples': axis_samples,
                'fit_samples': fit_samples,
                'fit': fit,
                'suggested_max_speed_mm_s': suggested_max_speed,
                'plateau_exclusion': plateau_info,
            }

        global_samples: list[dict] = []
        for v_percent in sorted(global_group.keys()):
            speeds = global_group[v_percent]
            if not speeds:
                continue
            mean_speed = sum(speeds) / float(len(speeds))
            variance = 0.0
            if len(speeds) > 1:
                variance = sum((speed - mean_speed) ** 2 for speed in speeds) / float(len(speeds))
            global_samples.append({
                'v_percent': int(v_percent),
                'speed_mm_s': mean_speed,
                'std_mm_s': math.sqrt(variance),
                'count': len(speeds),
            })

        global_fit_samples, global_plateau_info = self._exclude_plateau_samples(global_samples)
        if bool(global_plateau_info.get('excluded')):
            self.get_logger().info(
                'Plateau excluded for global model: '
                f'start_v={global_plateau_info.get("start_v_percent")}, '
                f'excluded={global_plateau_info.get("excluded_count")} sample(s).'
            )

        payload = {
            'format_version': 1,
            'generated_at': datetime.now(timezone.utc).isoformat(),
            'service_root': self._service_root,
            'tcp_topic': self._tcp_topic,
            'scripts_dir': str(self._scripts_dir),
            'script_names': script_names,
            'startup_cp': int(self._startup_cp),
            'startup_speed_factor': int(self._startup_speed_factor),
            'applied_startup_settings': {
                'cp_percent': int(self._startup_cp),
                'speed_factor_percent': int(self._startup_speed_factor),
            },
            'goal_tolerance_mm': float(self._goal_tolerance_mm),
            'segment_timeout_sec': float(self._segment_timeout_sec),
            'measurement_filters': {
                'exclude_v_percents': sorted(self._exclude_v_percents),
                'min_command_distance_mm': float(self._min_command_distance_mm),
                'min_measured_distance_mm': float(self._min_measured_distance_mm),
                'min_travel_ratio': float(self._min_travel_ratio),
            },
            'fit_settings': {
                'exclude_plateau_from_fit': bool(self._exclude_plateau_from_fit),
                'plateau_min_samples': int(self._plateau_min_samples),
                'plateau_min_speed_ratio': float(self._plateau_min_speed_ratio),
                'plateau_max_gain_ratio': float(self._plateau_max_gain_ratio),
                'plateau_min_consecutive_steps': int(self._plateau_min_consecutive_steps),
            },
            'axis_models': axis_models,
            'global_model': {
                'samples': global_samples,
                'fit_samples': global_fit_samples,
                'fit': self._linear_fit([
                    (float(item['v_percent']), float(item['speed_mm_s'])) for item in global_fit_samples
                ]),
                'plateau_exclusion': global_plateau_info,
            },
            'raw_segments': [self._measurement_to_dict(measurement) for measurement in measurements],
        }
        return payload

    def _exclude_plateau_samples(self, samples: list[dict]) -> tuple[list[dict], dict]:
        if not samples:
            return [], {
                'enabled': bool(self._exclude_plateau_from_fit),
                'excluded': False,
                'reason': 'no_samples',
            }

        if not self._exclude_plateau_from_fit:
            return list(samples), {
                'enabled': False,
                'excluded': False,
                'reason': 'disabled',
            }

        if len(samples) < self._plateau_min_samples:
            return list(samples), {
                'enabled': True,
                'excluded': False,
                'reason': f'insufficient_samples:{len(samples)}<{self._plateau_min_samples}',
            }

        speeds = [float(sample.get('speed_mm_s', 0.0)) for sample in samples]
        v_percents = [float(sample.get('v_percent', 0.0)) for sample in samples]
        max_speed = max(speeds)
        if max_speed <= 1e-6:
            return list(samples), {
                'enabled': True,
                'excluded': False,
                'reason': 'non_positive_speeds',
            }

        slopes: list[float] = []
        for index in range(1, len(samples)):
            dv = v_percents[index] - v_percents[index - 1]
            if dv <= 1e-6:
                slopes.append(0.0)
                continue
            ds = speeds[index] - speeds[index - 1]
            slopes.append(ds / dv)

        positive_slopes = sorted((slope for slope in slopes if slope > 1e-6), reverse=True)
        if not positive_slopes:
            return list(samples), {
                'enabled': True,
                'excluded': False,
                'reason': 'no_positive_gain',
            }

        top_count = min(3, len(positive_slopes))
        baseline_gain = sum(positive_slopes[:top_count]) / float(top_count)
        if baseline_gain <= 1e-6:
            return list(samples), {
                'enabled': True,
                'excluded': False,
                'reason': 'low_baseline_gain',
            }
        gain_threshold = baseline_gain * self._plateau_max_gain_ratio

        required_steps = self._plateau_min_consecutive_steps
        for sample_index in range(1, len(samples)):
            speed_ratio = speeds[sample_index] / max_speed
            if speed_ratio < self._plateau_min_speed_ratio:
                continue

            slope_start = sample_index - 1
            slope_end = slope_start + required_steps
            if slope_end > len(slopes):
                continue
            window = slopes[slope_start:slope_end]
            if all(abs(slope) <= gain_threshold for slope in window):
                fit_samples = list(samples[:sample_index])
                if len(fit_samples) < 2:
                    return list(samples), {
                        'enabled': True,
                        'excluded': False,
                        'reason': 'not_enough_fit_points_after_exclusion',
                    }
                return fit_samples, {
                    'enabled': True,
                    'excluded': True,
                    'reason': 'plateau_detected',
                    'start_index': int(sample_index),
                    'start_v_percent': int(round(v_percents[sample_index])),
                    'start_speed_mm_s': float(speeds[sample_index]),
                    'excluded_count': int(len(samples) - len(fit_samples)),
                    'baseline_gain_mm_s_per_v': float(baseline_gain),
                    'gain_threshold_mm_s_per_v': float(gain_threshold),
                }

        return list(samples), {
            'enabled': True,
            'excluded': False,
            'reason': 'plateau_not_detected',
            'baseline_gain_mm_s_per_v': float(baseline_gain),
            'gain_threshold_mm_s_per_v': float(gain_threshold),
        }

    def _measurement_to_dict(self, measurement: SegmentMeasurement) -> dict:
        return {
            'script_name': measurement.script_name,
            'point_index': measurement.point_index,
            'axis': measurement.axis_name,
            'v_percent': measurement.v_percent,
            'a_percent': measurement.a_percent,
            'command_distance_mm': measurement.command_distance_mm,
            'measured_distance_mm': measurement.measured_distance_mm,
            'elapsed_sec': measurement.elapsed_sec,
            'speed_mm_s': measurement.speed_mm_s,
            'mean_speed_mm_s': measurement.mean_speed_mm_s,
            'peak_speed_mm_s': measurement.peak_speed_mm_s,
            'reached_target': measurement.reached_target,
            'sample_count': measurement.sample_count,
            'start_axis_mm': measurement.start_axis_mm,
            'target_axis_mm': measurement.target_axis_mm,
            'final_axis_mm': measurement.final_axis_mm,
        }

    def _linear_fit(self, points: list[tuple[float, float]]) -> dict:
        if len(points) < 2:
            return {
                'slope': 0.0,
                'intercept': points[0][1] if points else 0.0,
                'r2': 0.0,
            }

        x_mean = sum(point[0] for point in points) / float(len(points))
        y_mean = sum(point[1] for point in points) / float(len(points))

        ss_xy = sum((x - x_mean) * (y - y_mean) for x, y in points)
        ss_xx = sum((x - x_mean) ** 2 for x, _ in points)
        if ss_xx <= 1e-12:
            slope = 0.0
        else:
            slope = ss_xy / ss_xx
        intercept = y_mean - slope * x_mean

        ss_tot = sum((y - y_mean) ** 2 for _, y in points)
        ss_res = sum((y - (slope * x + intercept)) ** 2 for x, y in points)
        r2 = 0.0 if ss_tot <= 1e-12 else max(0.0, min(1.0, 1.0 - (ss_res / ss_tot)))
        return {
            'slope': slope,
            'intercept': intercept,
            'r2': r2,
        }

    def _interpolate_speed_for_v(self, samples: list[dict], target_v: float) -> Optional[float]:
        if not samples:
            return None
        if len(samples) == 1:
            return float(samples[0]['speed_mm_s'])

        ordered = sorted(samples, key=lambda item: float(item['v_percent']))
        if target_v <= float(ordered[0]['v_percent']):
            return float(ordered[0]['speed_mm_s'])
        if target_v >= float(ordered[-1]['v_percent']):
            return float(ordered[-1]['speed_mm_s'])

        for index in range(1, len(ordered)):
            left = ordered[index - 1]
            right = ordered[index]
            left_v = float(left['v_percent'])
            right_v = float(right['v_percent'])
            if target_v > right_v:
                continue
            span = right_v - left_v
            if span <= 1e-9:
                return float(left['speed_mm_s'])
            ratio = (target_v - left_v) / span
            return float(left['speed_mm_s']) + ratio * (
                float(right['speed_mm_s']) - float(left['speed_mm_s']))
        return None

    def _write_json(self, path: Path, payload: dict) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        with open(path, 'w', encoding='utf-8') as output_file:
            json.dump(payload, output_file, indent=2)
            output_file.write('\n')

    def _resolve_raw_trace_path(self) -> Path:
        if self._raw_trace_file:
            return Path(self._raw_trace_file).expanduser()
        return self._output_file.with_name(self._output_file.stem + '_tcp_trace.csv')

    def _write_raw_trace_csv(self, path: Path, rows: list[list[object]]) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        with open(path, 'w', encoding='utf-8', newline='') as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow([
                'script_name',
                'point_index',
                'axis',
                'v_percent',
                'a_percent',
                't_rel_sec',
                'x_mm',
                'y_mm',
                'z_mm',
                'rx_deg',
                'ry_deg',
                'rz_deg',
            ])
            writer.writerows(rows)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MovementCalibrationNode()
    exit_code = 1
    try:
        rclpy.spin(node)
        exit_code = node.exit_code
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == '__main__':
    main()
