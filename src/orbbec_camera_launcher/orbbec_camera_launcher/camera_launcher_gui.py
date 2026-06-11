from __future__ import annotations

import os
import re
import shlex
import shutil
import signal
import subprocess
import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import messagebox, scrolledtext, ttk

try:
    import yaml
except ImportError:  # pragma: no cover - runtime fallback for minimal installs
    yaml = None


CONFIG_PATH = Path('config/camera_bringup/orbbec_cameras.yaml')
SCAN_TIMEOUT_SEC = 8.0
CAMERA_COUNT = 2
AUTO_LAUNCH_DELAY_MS = 600
PROCESS_STOP_TIMEOUT_SEC = 3.0
PROCESS_TERMINAL_PID_WAIT_SEC = 2.0
CAMERA_READY_TIMEOUT_SEC = 15.0
ROS_TOPIC_QUERY_TIMEOUT_SEC = 3.0

DEFAULT_ORBBEC_LAUNCH_ARGS = {
    'device_preset': 'High Accuracy',
    'enable_color': True,
    'enable_depth': True,
    'depth_registration': True,
    'align_target_stream': 'COLOR',
    'align_mode': 'SW',
    'enable_frame_sync': True,
    'enable_temporal_filter': True,
    'color_width': 848,
    'color_height': 480,
    'color_fps': 30,
    'depth_width': 848,
    'depth_height': 480,
    'depth_fps': 30,
    'enable_point_cloud': False,
}

SERIAL_LABEL_RE = re.compile(
    r'(?:serial(?:\s+number)?|serial_number|sn)\s*[:=]\s*([A-Za-z0-9_.:-]+)',
    re.IGNORECASE,
)
GENERIC_SERIAL_RE = re.compile(r'\b[A-Za-z0-9][A-Za-z0-9_.:-]{5,}\b')
CAMERA_NAME_RE = re.compile(r'^[A-Za-z][A-Za-z0-9_]*$')


def workspace_path(*parts: str) -> Path:
    for env_name in ('DOBOT_PICKN_PLACE_ROOT', 'DOBOT_WORKSPACE_ROOT'):
        value = os.environ.get(env_name)
        if value:
            return Path(value).expanduser().resolve().joinpath(*parts)

    candidates = [Path.cwd(), Path(__file__).resolve()]
    for token in os.environ.get('PYTHONPATH', '').split(os.pathsep):
        if token:
            candidates.append(Path(token))

    for start in candidates:
        path = start.expanduser().resolve()
        if path.is_file():
            path = path.parent
        for parent in (path, *path.parents):
            if (parent / 'src').exists() and (parent / 'config').exists():
                return parent.joinpath(*parts)
            if parent.name == 'install' and parent.parent.exists():
                return parent.parent.joinpath(*parts)

    return Path.cwd().resolve().joinpath(*parts)


def workspace_root() -> Path:
    return workspace_path()


def shell_join(args: list[str]) -> str:
    return ' '.join(shlex.quote(str(arg)) for arg in args)


def ros_sourced_shell_command(cmd: list[str]) -> str:
    root = shlex.quote(str(workspace_root()))
    return (
        'set -e; '
        f'ROOT="${{DOBOT_PICKN_PLACE_ROOT:-{root}}}"; '
        'cd "$ROOT"; '
        'if [ -f /opt/ros/humble/setup.bash ]; then source /opt/ros/humble/setup.bash; fi; '
        'if [ -f "$ROOT/install/setup.bash" ]; then source "$ROOT/install/setup.bash"; fi; '
        'if [ -f "$ROOT/third_party/.venv/bin/activate" ]; then source "$ROOT/third_party/.venv/bin/activate"; fi; '
        f'exec {shell_join(cmd)}'
    )


def visible_terminal_command(title: str, shell_command: str) -> list[str] | None:
    if shutil.which('gnome-terminal'):
        return [
            'gnome-terminal',
            '--wait',
            '--title',
            title,
            '--',
            'bash',
            '-lc',
            shell_command,
        ]
    if shutil.which('xfce4-terminal'):
        return [
            'xfce4-terminal',
            '--disable-server',
            '--title',
            title,
            '--command',
            f'bash -lc {shlex.quote(shell_command)}',
        ]
    if shutil.which('xterm'):
        return ['xterm', '-T', title, '-e', 'bash', '-lc', shell_command]
    if shutil.which('konsole'):
        return [
            'konsole',
            '--nofork',
            '--workdir',
            str(workspace_root()),
            '-p',
            f'tabtitle={title}',
            '-e',
            'bash',
            '-lc',
            shell_command,
        ]
    if shutil.which('mate-terminal'):
        return [
            'mate-terminal',
            '--disable-factory',
            '--title',
            title,
            '--',
            'bash',
            '-lc',
            shell_command,
        ]
    return None


def safe_process_label(text: str) -> str:
    chars = [ch.lower() if ch.isalnum() else '_' for ch in text]
    label = ''.join(chars).strip('_')
    while '__' in label:
        label = label.replace('__', '_')
    return label[:64] or 'process'


def _load_yaml(path: Path) -> dict:
    if not path.exists():
        return {}
    try:
        with path.open('r', encoding='utf-8') as infile:
            if yaml is not None:
                data = yaml.safe_load(infile)
            else:
                data = _simple_yaml_load(infile.read())
    except Exception:
        return {}
    return data if isinstance(data, dict) else {}


def _simple_yaml_load(text: str) -> dict:
    cameras = []
    current = None
    launch_args = {}
    section = None
    for raw_line in text.splitlines():
        line = raw_line.strip()
        if not line or line.startswith('#'):
            continue
        if line == 'cameras:':
            section = 'cameras'
            continue
        if line == 'orbbec_launch_args:':
            section = 'orbbec_launch_args'
            if current is not None:
                cameras.append(current)
                current = None
            continue
        if section == 'orbbec_launch_args':
            if ':' not in line:
                continue
            key, value = line.split(':', 1)
            launch_args[key.strip()] = _parse_scalar(value.strip().strip('"\''))
            continue
        if section != 'cameras':
            continue
        if line.startswith('- '):
            if current is not None:
                cameras.append(current)
            current = {}
            line = line[2:].strip()
        if ':' not in line or current is None:
            continue
        key, value = line.split(':', 1)
        current[key.strip()] = value.strip().strip('"\'')
    if current is not None:
        cameras.append(current)
    return {'cameras': cameras, 'orbbec_launch_args': launch_args}


def _parse_scalar(value: str) -> object:
    lowered = value.lower()
    if lowered == 'true':
        return True
    if lowered == 'false':
        return False
    try:
        return int(value)
    except ValueError:
        return value


def _write_yaml(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if yaml is not None:
        with path.open('w', encoding='utf-8') as outfile:
            yaml.safe_dump(payload, outfile, sort_keys=False)
        return

    with path.open('w', encoding='utf-8') as outfile:
        outfile.write('cameras:\n')
        for camera in payload.get('cameras', []):
            outfile.write(f'  - slot: {camera.get("slot", "")}\n')
            outfile.write(f'    serial_number: "{camera.get("serial_number", "")}"\n')
            outfile.write(f'    camera_name: "{camera.get("camera_name", "")}"\n')
        outfile.write('orbbec_launch_args:\n')
        for key, value in payload.get('orbbec_launch_args', {}).items():
            outfile.write(f'  {key}: {_format_yaml_scalar(value)}\n')


def _format_yaml_scalar(value: object) -> str:
    if isinstance(value, bool):
        return 'true' if value else 'false'
    return str(value)


def _load_orbbec_launch_args(config_path: Path) -> dict:
    payload = _load_yaml(config_path)
    launch_args = payload.get('orbbec_launch_args', {})
    if not isinstance(launch_args, dict):
        return dict(DEFAULT_ORBBEC_LAUNCH_ARGS)
    merged = dict(DEFAULT_ORBBEC_LAUNCH_ARGS)
    for key, value in launch_args.items():
        merged[str(key)] = value
    return merged


def _ros_arg_value(value: object) -> str:
    if isinstance(value, bool):
        return 'true' if value else 'false'
    return str(value)


def _orbbec_launch_cli_args(config_path: Path) -> list[str]:
    return [
        f'{key}:={_ros_arg_value(value)}'
        for key, value in _load_orbbec_launch_args(config_path).items()
    ]


def _extract_serial_numbers(text: str) -> list[str]:
    found = []

    for match in SERIAL_LABEL_RE.finditer(text):
        candidate = match.group(1).strip().strip(',;')
        if candidate and candidate not in found:
            found.append(candidate)

    if found:
        return found

    stop_words = {
        'orbbec',
        'camera',
        'device',
        'serial',
        'number',
        'version',
        'firmware',
        'connected',
        'product',
    }
    for match in GENERIC_SERIAL_RE.finditer(text):
        candidate = match.group(0).strip().strip(',;')
        if candidate.lower() in stop_words:
            continue
        if candidate not in found:
            found.append(candidate)
    return found


class CameraLauncherApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.config_path = workspace_path(*CONFIG_PATH.parts)
        self.serial_vars = [tk.StringVar() for _ in range(CAMERA_COUNT)]
        self.name_vars = [tk.StringVar() for _ in range(CAMERA_COUNT)]
        self.status_var = tk.StringVar(value='No camera data yet')
        self.config_var = tk.StringVar(value=f'Config: {self.config_path}')
        self.scan_status_var = tk.StringVar(value='Scan has not run yet')
        self.running_var = tk.StringVar(value='Cameras stopped')

        self.launch_processes: list[subprocess.Popen] = []
        self.launch_process_groups: dict[int, int | None] = {}
        self.process_labels: dict[int, str] = {}
        self.process_pairs: dict[int, dict[str, str]] = {}
        self.ready_process_pids: set[int] = set()
        self.pending_launch_pairs: list[dict[str, str]] = []
        self.sequential_device_num = 1
        self.sequential_launch_failures: list[str] = []
        self.process_poll_active = False
        self.detected_serials: list[str] = []
        self.scanning = False
        self.launching = False
        self.launch_stop_requested = False
        self.closing = False

        self.scan_button = None
        self.copy_first_button = None
        self.use_slot_buttons = []
        self.save_button = None
        self.launch_button = None
        self.stop_button = None
        self.scan_text = None

        self._build_ui()
        self._load_config()
        self._bind_validation()
        self._update_ui_state()
        self.root.protocol('WM_DELETE_WINDOW', self._on_close)
        self.root.after(AUTO_LAUNCH_DELAY_MS, self._auto_launch_from_config)

    def _build_ui(self) -> None:
        self.root.title('Orbbec Camera Launcher')
        self.root.geometry('780x610')
        self.root.minsize(720, 560)

        style = ttk.Style()
        style.configure('Status.TLabel', foreground='#204a87')

        outer = ttk.Frame(self.root, padding=14)
        outer.pack(fill='both', expand=True)
        outer.columnconfigure(0, weight=1)
        outer.rowconfigure(2, weight=1)

        header = ttk.Frame(outer)
        header.grid(row=0, column=0, sticky='ew')
        header.columnconfigure(0, weight=1)

        ttk.Label(
            header,
            text='Orbbec Gemini 335 Camera Launcher',
            font=('TkDefaultFont', 13, 'bold'),
        ).grid(row=0, column=0, sticky='w')
        ttk.Label(header, textvariable=self.status_var, style='Status.TLabel').grid(
            row=1, column=0, sticky='w', pady=(4, 0)
        )
        ttk.Label(header, textvariable=self.config_var).grid(row=2, column=0, sticky='w', pady=(4, 10))

        mapping = ttk.LabelFrame(outer, text='Camera SN / Topic Prefix', padding=10)
        mapping.grid(row=1, column=0, sticky='ew')
        mapping.columnconfigure(1, weight=1)
        mapping.columnconfigure(3, weight=1)

        ttk.Label(mapping, text='Slot').grid(row=0, column=0, sticky='w', padx=(0, 8))
        ttk.Label(mapping, text='Serial Number').grid(row=0, column=1, sticky='w', padx=(0, 8))
        ttk.Label(mapping, text='Camera Name').grid(row=0, column=3, sticky='w', padx=(0, 8))

        defaults = ('bin_camera', 'robot_camera')
        for index in range(CAMERA_COUNT):
            row = index + 1
            self.name_vars[index].set(defaults[index])
            ttk.Label(mapping, text=f'Camera {row}').grid(row=row, column=0, sticky='w', pady=5, padx=(0, 8))
            ttk.Entry(mapping, textvariable=self.serial_vars[index], width=28).grid(
                row=row, column=1, sticky='ew', pady=5, padx=(0, 8)
            )
            ttk.Button(
                mapping,
                text='Paste',
                command=lambda slot=index: self._paste_serial(slot),
            ).grid(row=row, column=2, sticky='w', pady=5, padx=(0, 12))
            ttk.Entry(mapping, textvariable=self.name_vars[index], width=24).grid(
                row=row, column=3, sticky='ew', pady=5, padx=(0, 8)
            )

        actions = ttk.Frame(mapping)
        actions.grid(row=3, column=0, columnspan=4, sticky='ew', pady=(8, 0))
        actions.columnconfigure(2, weight=1)

        self.save_button = tk.Button(
            actions,
            text='Save Mapping',
            width=14,
            command=self._save_config,
        )
        self.save_button.grid(row=0, column=0, padx=(0, 8))

        self.launch_button = tk.Button(
            actions,
            text='Launch Cameras',
            width=16,
            fg='white',
            command=self._launch_cameras,
        )
        self.launch_button.grid(row=0, column=1, padx=(0, 8))

        self.stop_button = tk.Button(
            actions,
            text='Stop Cameras',
            width=14,
            bg='#8b1a1a',
            fg='white',
            activebackground='#b22222',
            command=self._stop_cameras,
        )
        self.stop_button.grid(row=0, column=2, sticky='w')

        ttk.Label(actions, textvariable=self.running_var).grid(row=0, column=3, sticky='e')

        scan = ttk.LabelFrame(outer, text='Scan Camera', padding=10)
        scan.grid(row=2, column=0, sticky='nsew', pady=(10, 0))
        scan.columnconfigure(0, weight=1)
        scan.rowconfigure(2, weight=1)

        scan_actions = ttk.Frame(scan)
        scan_actions.grid(row=0, column=0, sticky='ew')
        scan_actions.columnconfigure(5, weight=1)

        self.scan_button = tk.Button(
            scan_actions,
            text='Scan Camera',
            width=14,
            bg='#2f5f8f',
            fg='white',
            activebackground='#3973ac',
            command=self._scan_camera,
        )
        self.scan_button.grid(row=0, column=0, padx=(0, 8))

        self.copy_first_button = ttk.Button(
            scan_actions,
            text='Copy First SN',
            command=self._copy_first_serial,
        )
        self.copy_first_button.grid(row=0, column=1, padx=(0, 8))

        for slot in range(CAMERA_COUNT):
            button = ttk.Button(
                scan_actions,
                text=f'Use In Camera {slot + 1}',
                command=lambda index=slot: self._use_first_serial(index),
            )
            button.grid(row=0, column=slot + 2, padx=(0, 8))
            self.use_slot_buttons.append(button)

        ttk.Label(scan, textvariable=self.scan_status_var).grid(row=1, column=0, sticky='w', pady=(8, 6))
        self.scan_text = scrolledtext.ScrolledText(scan, height=12, wrap='word')
        self.scan_text.grid(row=2, column=0, sticky='nsew')
        self._set_scan_text('No scan result yet.')

    def _bind_validation(self) -> None:
        for var in [*self.serial_vars, *self.name_vars]:
            var.trace_add('write', lambda *_args: self._update_ui_state())

    def _load_config(self) -> None:
        payload = _load_yaml(self.config_path)
        cameras = payload.get('cameras', [])
        if not isinstance(cameras, list):
            return

        for camera in cameras:
            if not isinstance(camera, dict):
                continue
            try:
                slot = int(camera.get('slot', 0)) - 1
            except (TypeError, ValueError):
                continue
            if slot < 0 or slot >= CAMERA_COUNT:
                continue
            self.serial_vars[slot].set(str(camera.get('serial_number', '')).strip())
            camera_name = str(camera.get('camera_name', '')).strip()
            if camera_name:
                self.name_vars[slot].set(camera_name)

    def _camera_pairs(self) -> list[dict[str, str]]:
        pairs = []
        for index in range(CAMERA_COUNT):
            pairs.append({
                'slot': str(index + 1),
                'serial_number': self.serial_vars[index].get().strip(),
                'camera_name': self.name_vars[index].get().strip(),
            })
        return pairs

    def _mapping_errors(self, require_all: bool = False) -> list[str]:
        pairs = self._camera_pairs()
        errors = []
        configured_serials = []
        configured_names = []

        for pair in pairs:
            slot = pair['slot']
            has_serial = bool(pair['serial_number'])
            has_name = bool(pair['camera_name'])
            if require_all and not has_serial:
                errors.append(f'Camera {slot} SN is empty')
            if require_all and not has_name:
                errors.append(f'Camera {slot} name is empty')
            elif has_serial and not has_name:
                errors.append(f'Camera {slot} name is empty but SN is set')
            elif has_name and not CAMERA_NAME_RE.match(pair['camera_name']):
                errors.append(f'Camera {slot} name must use letters, numbers, and underscores')
            if has_serial and has_name:
                configured_serials.append(pair['serial_number'])
                configured_names.append(pair['camera_name'])

        if len(configured_serials) != len(set(configured_serials)):
            errors.append('Serial numbers must be unique')
        if len(configured_names) != len(set(configured_names)):
            errors.append('Camera names must be unique')

        return errors

    def _validation_errors(self) -> list[str]:
        return self._mapping_errors(require_all=True)

    def _has_complete_camera_data(self) -> bool:
        return not self._validation_errors()

    def _configured_camera_pairs(self) -> list[dict[str, str]]:
        return [
            pair for pair in self._camera_pairs()
            if pair['serial_number'] and pair['camera_name']
        ]

    def _unconfigured_camera_pairs(self) -> list[dict[str, str]]:
        return [
            pair for pair in self._camera_pairs()
            if not (pair['serial_number'] and pair['camera_name'])
        ]

    def _has_launchable_camera_data(self) -> bool:
        return bool(self._configured_camera_pairs()) and not self._mapping_errors()

    def _slot_label(self, pair: dict[str, str]) -> str:
        name = pair.get('camera_name') or '<missing_name>'
        serial = pair.get('serial_number') or '<missing_sn>'
        return f'Camera {pair.get("slot", "?")} {name} (SN {serial})'

    def _update_ui_state(self) -> None:
        complete = self._has_complete_camera_data()
        launchable = self._has_launchable_camera_data()
        errors = self._mapping_errors()
        running = self._any_process_running()
        starting = self._any_process_starting()

        if self.launching and starting:
            self.status_var.set('Launching cameras one at a time; waiting for current camera topics...')
        elif self.launching and running:
            self.status_var.set('Camera verified; preparing next configured camera...')
        elif self.launching:
            self.status_var.set('Checking configured cameras...')
        elif starting:
            self.status_var.set('Waiting for camera topics...')
        elif running:
            self.status_var.set('Camera launch verified')
        elif complete:
            self.status_var.set('Camera data ready')
        elif launchable:
            self.status_var.set('Partial camera data ready')
        elif errors:
            self.status_var.set(errors[0])
        else:
            self.status_var.set('No camera data yet')

        save_state = tk.NORMAL if not errors and not running and not self.launching else tk.DISABLED
        launch_state = tk.NORMAL if launchable and not running and not self.launching else tk.DISABLED
        # Keep Stop available even when no tracked camera is running so the operator
        # always has an obvious close/cleanup action.
        stop_state = tk.NORMAL

        if self.save_button is not None:
            self.save_button.configure(state=save_state)

        if self.launch_button is not None:
            if launch_state == tk.NORMAL:
                self.launch_button.configure(
                    state=tk.NORMAL,
                    bg='#1f7a1f',
                    activebackground='#2e8b57',
                    fg='white',
                )
            else:
                self.launch_button.configure(
                    state=tk.DISABLED,
                    bg='#777777',
                    activebackground='#777777',
                    fg='white',
                )

        if self.stop_button is not None:
            self.stop_button.configure(state=stop_state)

        serial_button_state = tk.NORMAL if self.detected_serials else tk.DISABLED
        if self.copy_first_button is not None:
            self.copy_first_button.configure(state=serial_button_state)
        for button in self.use_slot_buttons:
            button.configure(state=serial_button_state)

    def _set_scan_text(self, text: str) -> None:
        if self.scan_text is None:
            return
        self.scan_text.configure(state=tk.NORMAL)
        self.scan_text.delete('1.0', tk.END)
        self.scan_text.insert(tk.END, text)
        self.scan_text.configure(state=tk.NORMAL)

    def _append_scan_text(self, text: str) -> None:
        if self.scan_text is None:
            return
        self.scan_text.configure(state=tk.NORMAL)
        self.scan_text.insert(tk.END, text)
        self.scan_text.see(tk.END)

    def _scan_camera(self) -> None:
        if self.scanning:
            return
        self.scanning = True
        self.detected_serials = []
        self.scan_status_var.set('Scanning...')
        self._set_scan_text('Scanning Orbbec devices...\n')
        if self.scan_button is not None:
            self.scan_button.configure(state=tk.DISABLED)
        self._update_ui_state()

        thread = threading.Thread(target=self._scan_worker, daemon=True)
        thread.start()

    def _run_device_scan(self) -> tuple[int, list[str], str]:
        command = ['ros2', 'run', 'orbbec_camera', 'list_devices_node']
        try:
            result = subprocess.run(
                command,
                check=False,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=SCAN_TIMEOUT_SEC,
            )
            output = (result.stdout or '').strip()
            error = (result.stderr or '').strip()
            combined = '\n'.join(part for part in (output, error) if part).strip()
            serials = _extract_serial_numbers(combined)
            return_code = result.returncode
        except subprocess.TimeoutExpired as exc:
            combined = '\n'.join(
                part.decode(errors='replace') if isinstance(part, bytes) else part
                for part in (exc.stdout, exc.stderr)
                if part
            ).strip()
            serials = _extract_serial_numbers(combined)
            return_code = -1
            combined = (combined + '\n\nScan timed out.').strip()
        except Exception as exc:  # noqa: BLE001
            combined = f'Failed to run ros2 device scan: {exc}'
            serials = []
            return_code = -1

        return return_code, serials, combined

    def _scan_worker(self) -> None:
        return_code, serials, combined = self._run_device_scan()
        self.root.after(0, lambda: self._finish_scan(return_code, serials, combined))

    def _finish_scan(self, return_code: int, serials: list[str], output: str) -> None:
        self.scanning = False
        self.detected_serials = serials

        if self.scan_button is not None:
            self.scan_button.configure(state=tk.NORMAL)

        if serials:
            detected_lines = '\n'.join(serials)
            self.scan_status_var.set(f'Detected {len(serials)} serial number(s)')
            text = f'Detected serial numbers:\n{detected_lines}\n'
            if output:
                text += f'\nRaw scan output:\n{output}\n'
        else:
            if output:
                self.scan_status_var.set('No serial number found in scan output')
                text = f'No serial number found.\n\nRaw scan output:\n{output}\n'
            else:
                self.scan_status_var.set('No Orbbec camera data returned')
                text = 'No camera data yet.\n'

        if return_code not in (0, None):
            text += f'\nScan command exit code: {return_code}\n'

        self._set_scan_text(text)
        self._update_ui_state()

    def _copy_first_serial(self) -> None:
        if not self.detected_serials:
            return
        self.root.clipboard_clear()
        self.root.clipboard_append(self.detected_serials[0])
        self.scan_status_var.set(f'Copied SN: {self.detected_serials[0]}')

    def _use_first_serial(self, slot: int) -> None:
        if not self.detected_serials:
            return
        self.serial_vars[slot].set(self.detected_serials[0])
        self.scan_status_var.set(f'Camera {slot + 1} SN set to {self.detected_serials[0]}')

    def _paste_serial(self, slot: int) -> None:
        try:
            text = self.root.clipboard_get().strip()
        except tk.TclError:
            self.scan_status_var.set('Clipboard is empty')
            return
        self.serial_vars[slot].set(text)

    def _save_config(self) -> bool:
        errors = self._mapping_errors()
        if errors:
            messagebox.showerror('Camera mapping incomplete', '\n'.join(errors))
            self._update_ui_state()
            return False

        payload = {
            'cameras': [
                {
                    'slot': int(pair['slot']),
                    'serial_number': pair['serial_number'],
                    'camera_name': pair['camera_name'],
                }
                for pair in self._camera_pairs()
            ],
            'orbbec_launch_args': _load_orbbec_launch_args(self.config_path),
        }
        try:
            _write_yaml(self.config_path, payload)
        except Exception as exc:  # noqa: BLE001
            messagebox.showerror('Save failed', f'Failed to save camera mapping:\n{exc}')
            return False

        self.status_var.set(f'Saved mapping: {self.config_path}')
        return True

    def _auto_launch_from_config(self) -> None:
        if self.closing or self._any_process_running() or self.launching:
            return
        self._launch_cameras(auto=True)

    def _launch_cameras(self, auto: bool = False) -> None:
        if self._any_process_running() or self.launching:
            return

        errors = self._mapping_errors()
        if errors:
            title = 'Auto launch skipped' if auto else 'Camera mapping incomplete'
            if not auto:
                messagebox.showwarning(title, '\n'.join(errors))
            self._append_scan_text('\nCamera launch skipped:\n' + '\n'.join(errors) + '\n')
            self._update_ui_state()
            return

        pairs = self._configured_camera_pairs()
        if not pairs:
            message = 'No configured camera slots have both serial number and camera name.'
            if auto:
                self._append_scan_text(f'\nAuto launch skipped: {message}\n')
            else:
                messagebox.showerror('No cameras configured', message)
                self._append_scan_text(f'\n{message}\n')
            self._update_ui_state()
            return

        if not auto and not self._save_config():
            return

        self.launch_stop_requested = False
        self.launching = True
        self.running_var.set('Checking configured cameras...')
        self.scan_status_var.set('Checking connected cameras before launch...')
        self._append_scan_text('\nChecking connected cameras before launch...\n')
        self._update_ui_state()
        thread = threading.Thread(target=self._launch_scan_worker, args=(pairs, auto), daemon=True)
        thread.start()

    def _launch_scan_worker(self, pairs: list[dict[str, str]], auto: bool) -> None:
        return_code, serials, output = self._run_device_scan()
        self.root.after(0, lambda: self._finish_launch_scan(return_code, serials, output, pairs, auto))

    def _finish_launch_scan(
        self,
        return_code: int,
        serials: list[str],
        output: str,
        pairs: list[dict[str, str]],
        auto: bool,
    ) -> None:
        if self.closing or self.launch_stop_requested:
            self.launching = False
            self._update_ui_state()
            return

        self.detected_serials = serials
        connected = set(serials)
        launch_pairs = [pair for pair in pairs if pair['serial_number'] in connected]
        missing_connected = [pair for pair in pairs if pair['serial_number'] not in connected]
        unconfigured = self._unconfigured_camera_pairs()

        summary_lines = []
        if serials:
            summary_lines.append('Detected serial numbers:')
            summary_lines.extend(f'  {serial}' for serial in serials)
        elif output:
            summary_lines.append('No configured Orbbec serials were detected.')
        else:
            summary_lines.append('No Orbbec camera data returned.')
        if output:
            summary_lines.append('')
            summary_lines.append('Raw scan output:')
            summary_lines.append(output)
        if return_code not in (0, None):
            summary_lines.append(f'Scan command exit code: {return_code}')
        self._set_scan_text('\n'.join(summary_lines) + '\n')

        warnings = []
        if unconfigured:
            warnings.append('Unconfigured slot(s):')
            warnings.extend(f'  {self._slot_label(pair)}' for pair in unconfigured)
        if missing_connected:
            warnings.append('Configured camera(s) not connected:')
            warnings.extend(f'  {self._slot_label(pair)}' for pair in missing_connected)

        if not launch_pairs:
            self.launching = False
            self.running_var.set('No configured cameras running')
            self.status_var.set('No camera connected')
            if serials:
                message = 'No configured cameras are connected. No camera nodes were launched.'
            else:
                message = 'No camera connected. No camera nodes were launched.'
            if warnings:
                message = message + '\n\n' + '\n'.join(warnings)
            if not auto:
                messagebox.showwarning('No camera connected', message)
            self._append_scan_text('\n' + message + '\n')
            self._update_ui_state()
            self.status_var.set('No camera connected')
            return

        if missing_connected:
            connected_lines = '\n'.join(f'  {self._slot_label(pair)}' for pair in launch_pairs)
            missing_lines = '\n'.join(f'  {self._slot_label(pair)}' for pair in missing_connected)
            camera_word = 'camera is' if len(launch_pairs) == 1 else 'cameras are'
            message = (
                f'Only {len(launch_pairs)} configured {camera_word} connected.\n\n'
                f'Connected and will launch:\n{connected_lines}\n\n'
                f'Missing and will not launch:\n{missing_lines}'
            )
            if unconfigured:
                message += '\n\n' + '\n'.join(warnings[: 1 + len(unconfigured)])
            if not auto:
                messagebox.showwarning('Partial camera startup', message)
            else:
                self.status_var.set('Auto launching connected configured camera(s)')
            self._append_scan_text('\n' + message + '\n')
        elif warnings:
            message = '\n'.join(warnings) + '\n\nLaunching detected configured camera(s).'
            if not auto:
                messagebox.showwarning('Partial camera startup', message)
            else:
                self.status_var.set('Auto launching connected configured camera(s)')
            self._append_scan_text('\n' + message + '\n')

        self._launch_camera_pairs(launch_pairs)

    def _managed_terminal_shell_command(
        self,
        shell_command: str,
        pid_file: Path,
        display_command: str,
        label: str,
    ) -> str:
        quoted_label = shlex.quote(label)
        return (
            'set -e; '
            'printf "========================================\\n"; '
            f'printf "RUNNING CAMERA NODE: %s\\n" {quoted_label}; '
            'printf "========================================\\n\\n"; '
            f'printf "Command: %s\\n\\n" {shlex.quote(display_command)}; '
            f'printf "This terminal belongs to %s. Keep it open while the camera is running.\\n\\n" {quoted_label}; '
            f'echo $$ > {shlex.quote(str(pid_file))}; '
            f'{shell_command}'
        )

    def _runtime_process_pid_file(self, label: str) -> Path:
        pid_dir = workspace_path('runtime', 'orbbec_camera_launcher_pids')
        pid_dir.mkdir(parents=True, exist_ok=True)
        filename = f'{safe_process_label(label)}_{time.monotonic_ns()}.pid'
        return pid_dir / filename

    def _wait_for_managed_process_group(
        self,
        pid_file: Path,
        terminal_process: subprocess.Popen,
    ) -> int | None:
        deadline = time.monotonic() + PROCESS_TERMINAL_PID_WAIT_SEC
        while time.monotonic() < deadline:
            try:
                if pid_file.exists():
                    text = pid_file.read_text(encoding='utf-8').strip()
                    if text:
                        pid = int(text)
                        return os.getpgid(pid) if hasattr(os, 'getpgid') else None
            except (OSError, ValueError, ProcessLookupError):
                return None
            if terminal_process.poll() is not None:
                break
            time.sleep(0.05)
        return None

    def _capture_process_group(self, process: subprocess.Popen) -> int | None:
        if not hasattr(os, 'getpgid'):
            return None
        try:
            return os.getpgid(process.pid)
        except ProcessLookupError:
            return None
        except Exception:
            return None

    def _start_visible_terminal_process(
        self,
        label: str,
        shell_command: str,
        display_command: str,
    ) -> tuple[subprocess.Popen, int | None] | None:
        terminal_title = f'Orbbec Camera - {label}'
        pid_file = self._runtime_process_pid_file(label)
        managed_command = self._managed_terminal_shell_command(shell_command, pid_file, display_command, label)
        terminal_cmd = visible_terminal_command(terminal_title, managed_command)
        if terminal_cmd is None:
            message = (
                'No supported terminal emulator found; refusing to launch hidden camera process. '
                'Install gnome-terminal, xterm, xfce4-terminal, konsole, or mate-terminal.'
            )
            self._append_scan_text(f'\nCamera launch skipped: {message}\n')
            return None
        try:
            process = subprocess.Popen(
                terminal_cmd,
                cwd=str(workspace_root()),
                env=os.environ.copy(),
                start_new_session=hasattr(os, 'setsid'),
            )
            pgid = self._wait_for_managed_process_group(pid_file, process)
            return process, pgid or self._capture_process_group(process)
        except Exception as exc:
            self._append_scan_text(f'\nFailed to open terminal for {label}: {exc}\n')
            return None
        finally:
            try:
                pid_file.unlink()
            except FileNotFoundError:
                pass
            except Exception:
                pass

    def _launch_camera_pairs(self, pairs: list[dict[str, str]]) -> None:
        # Launch one camera at a time. The next camera starts only after the
        # current camera publishes the required color/depth topics.
        self.pending_launch_pairs = list(pairs)
        self.sequential_device_num = max(1, len(pairs))
        self.sequential_launch_failures: list[str] = []
        self._append_scan_text('\nLaunching connected configured cameras one at a time.\n')
        self._launch_next_camera_pair()

    def _launch_next_camera_pair(self) -> None:
        if self.closing:
            return
        if self.launch_stop_requested:
            self.pending_launch_pairs = []
            self._finish_sequential_launch()
            return

        pending = getattr(self, 'pending_launch_pairs', [])
        if not pending:
            self._finish_sequential_launch()
            return

        if self._any_process_running() and not self.launching:
            self._append_scan_text('\nLaunch blocked because a camera node is already running.\n')
            self._finish_sequential_launch()
            return

        pair = pending.pop(0)
        self.pending_launch_pairs = pending
        label = self._slot_label(pair)
        orbbec_launch_args = _orbbec_launch_cli_args(self.config_path)
        command = [
            'ros2',
            'launch',
            'orbbec_camera_launcher',
            'camera_headless.launch.py',
            f'config_file:={self.config_path}',
            f'enabled_cameras:={pair["camera_name"]}',
            f'device_num:={getattr(self, "sequential_device_num", max(1, len(pending) + 1))}',
            f'watchdog_namespace:=camera_watchdog/{pair["camera_name"]}',
            *orbbec_launch_args,
        ]
        display_command = shell_join(command)
        shell_command = ros_sourced_shell_command(command)

        self.running_var.set(f'Starting {label}...')
        self.scan_status_var.set(f'Starting {label}; waiting for image topics...')
        self._append_scan_text(
            f'\nStarting {label} in its own terminal.\n'
            f'Waiting for topics before launching the next camera:\n'
            f'  /{pair["camera_name"]}/color/image_raw\n'
            f'  /{pair["camera_name"]}/depth/image_raw\n'
            f'Command: {display_command}\n'
        )
        self._update_ui_state()

        started = self._start_visible_terminal_process(label, shell_command, display_command)
        if started is None:
            reason = (
                f'{label}: could not start visible terminal. '
                'Install gnome-terminal, xterm, xfce4-terminal, konsole, or mate-terminal.'
            )
            self.sequential_launch_failures.append(reason)
            messagebox.showerror('Launch failed', reason)
            self.pending_launch_pairs = []
            self._finish_sequential_launch()
            return

        process, pgid = started
        self.launch_processes.append(process)
        self.launch_process_groups[process.pid] = pgid
        self.process_labels[process.pid] = label
        self.process_pairs[process.pid] = dict(pair)
        self._update_running_status()
        self._update_ui_state()
        self._ensure_process_polling()
        self._start_single_camera_launch_validation(process, pair)

    def _finish_sequential_launch(self) -> None:
        self.launching = False
        failures = getattr(self, 'sequential_launch_failures', [])
        if failures:
            self._append_scan_text('\nCamera launch issue(s):\n' + '\n'.join(f'  {item}' for item in failures) + '\n')
            if not self._any_process_running():
                messagebox.showwarning('Camera launch completed with issues', '\n'.join(failures))
        elif self._any_process_running():
            self._append_scan_text('\nConfigured connected camera node(s) are running.\n')
        else:
            self._append_scan_text('\nNo camera nodes were launched.\n')
        self._update_running_status()
        self._update_ui_state()

    def _start_single_camera_launch_validation(
        self,
        process: subprocess.Popen,
        pair: dict[str, str],
    ) -> None:
        snapshot = (process, dict(pair))
        thread = threading.Thread(target=self._single_camera_launch_validation_worker, args=snapshot, daemon=True)
        thread.start()

    def _single_camera_launch_validation_worker(
        self,
        process: subprocess.Popen,
        pair: dict[str, str],
    ) -> None:
        pid = process.pid
        label = self._slot_label(pair)
        required = self._required_camera_topics(pair)
        last_missing = set(required)
        last_query_error = ''
        deadline = time.monotonic() + CAMERA_READY_TIMEOUT_SEC

        while not self.closing and time.monotonic() < deadline:
            if not self._process_alive(process):
                reason = f'launch process exited with code {process.poll()}'
                self.root.after(0, lambda pid=pid, reason=reason: self._finish_single_camera_launch_validation(pid, False, reason))
                return

            return_code, topics, query_output = self._query_ros_topics()
            if return_code not in (0, None):
                last_query_error = query_output or f'ros2 topic list exited with code {return_code}'

            missing = required - topics
            last_missing = missing
            if required and not missing:
                self.root.after(0, lambda pid=pid: self._finish_single_camera_launch_validation(pid, True, ''))
                return

            time.sleep(0.5)

        if self.closing:
            return

        reason = 'required topics did not appear'
        if last_missing:
            reason += ': ' + ', '.join(sorted(last_missing))
        if last_query_error:
            reason += f' ({last_query_error})'
        self.root.after(0, lambda pid=pid, reason=reason: self._finish_single_camera_launch_validation(pid, False, reason))

    def _finish_single_camera_launch_validation(self, pid: int, ready: bool, reason: str) -> None:
        if self.closing or self.launch_stop_requested:
            return

        process = next((item for item in self.launch_processes if item.pid == pid), None)
        label = self.process_labels.get(pid, f'camera process {pid}')

        if ready and process is not None and self._process_alive(process):
            self.ready_process_pids.add(pid)
            self._append_scan_text(f'\n{label} is running. Required image topics are available.\n')
            self.scan_status_var.set(f'{label} is running')
            self._update_running_status()
            self._update_ui_state()
            self._launch_next_camera_pair()
            return

        failure = f'{label}: {reason or "camera did not become ready"}'
        self.sequential_launch_failures.append(failure)
        self._append_scan_text(f'\nCamera launch validation failed:\n  {failure}\n')
        if process is not None:
            pgid = self.launch_process_groups.pop(pid, None)
            self._terminate_process(process, pgid)
            self.launch_processes = [item for item in self.launch_processes if item.pid != pid]
        self.process_labels.pop(pid, None)
        self.process_pairs.pop(pid, None)
        self.ready_process_pids.discard(pid)
        self.scan_status_var.set(f'{label} failed; continuing with remaining camera(s)')
        messagebox.showwarning('Camera launch failed', failure)
        self._update_running_status()
        self._update_ui_state()
        self._launch_next_camera_pair()

    def _required_camera_topics(self, pair: dict[str, str]) -> set[str]:
        camera_name = pair.get('camera_name', '').strip()
        if not camera_name:
            return set()
        return {
            f'/{camera_name}/color/image_raw',
            f'/{camera_name}/depth/image_raw',
        }

    def _query_ros_topics(self) -> tuple[int, set[str], str]:
        command = ros_sourced_shell_command(['ros2', 'topic', 'list'])
        try:
            result = subprocess.run(
                ['bash', '-lc', command],
                check=False,
                cwd=str(workspace_root()),
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=ROS_TOPIC_QUERY_TIMEOUT_SEC,
            )
        except subprocess.TimeoutExpired:
            return -1, set(), 'ros2 topic list timed out'
        except Exception as exc:  # noqa: BLE001
            return -1, set(), f'Failed to run ros2 topic list: {exc}'

        output = (result.stdout or '').strip()
        error = (result.stderr or '').strip()
        combined = '\n'.join(part for part in (output, error) if part).strip()
        topics = {line.strip() for line in output.splitlines() if line.strip().startswith('/')}
        return result.returncode, topics, combined

    def _start_camera_launch_validation(self, processes: list[subprocess.Popen]) -> None:
        snapshot = [(process, dict(self.process_pairs.get(process.pid, {}))) for process in processes]
        thread = threading.Thread(target=self._camera_launch_validation_worker, args=(snapshot,), daemon=True)
        thread.start()

    def _camera_launch_validation_worker(
        self,
        launches: list[tuple[subprocess.Popen, dict[str, str]]],
    ) -> None:
        pending = {process.pid: (process, pair) for process, pair in launches}
        failures: dict[int, str] = {}
        last_missing: dict[int, set[str]] = {}
        last_query_error = ''
        deadline = time.monotonic() + CAMERA_READY_TIMEOUT_SEC

        while pending and not self.closing and time.monotonic() < deadline:
            for pid, (process, _pair) in list(pending.items()):
                if not self._process_alive(process):
                    failures[pid] = f'launch process exited with code {process.poll()}'
                    pending.pop(pid, None)

            if not pending:
                break

            return_code, topics, query_output = self._query_ros_topics()
            if return_code not in (0, None):
                last_query_error = query_output or f'ros2 topic list exited with code {return_code}'

            ready: list[tuple[int, str]] = []
            for pid, (_process, pair) in list(pending.items()):
                required = self._required_camera_topics(pair)
                missing = required - topics
                last_missing[pid] = missing
                if required and not missing:
                    ready.append((pid, self._slot_label(pair)))
                    pending.pop(pid, None)

            if ready:
                self.root.after(0, lambda ready_items=ready: self._mark_camera_launch_ready(ready_items))

            if pending:
                time.sleep(0.5)

        if pending and not self.closing:
            for pid, (_process, pair) in pending.items():
                missing = sorted(last_missing.get(pid) or self._required_camera_topics(pair))
                reason = 'required topics did not appear'
                if missing:
                    reason += ': ' + ', '.join(missing)
                if last_query_error:
                    reason += f' ({last_query_error})'
                failures[pid] = reason

        if failures and not self.closing:
            self.root.after(0, lambda failed=dict(failures): self._finish_camera_launch_validation(failed))

    def _mark_camera_launch_ready(self, ready_items: list[tuple[int, str]]) -> None:
        messages = []
        for pid, label in ready_items:
            if pid not in self.process_pairs:
                continue
            self.ready_process_pids.add(pid)
            messages.append(f'  {label}')
        if messages:
            self._append_scan_text('\nCamera topic validation passed:\n' + '\n'.join(messages) + '\n')
            self.scan_status_var.set('Camera topics verified')
            self._update_running_status()
            self._update_ui_state()

    def _finish_camera_launch_validation(self, failures: dict[int, str]) -> None:
        if self.closing:
            return

        failed_lines = []
        for pid, reason in failures.items():
            process = next((item for item in self.launch_processes if item.pid == pid), None)
            if process is None:
                continue
            label = self._process_label(process)
            failed_lines.append(f'  {label}: {reason}')
            pgid = self.launch_process_groups.pop(pid, None)
            self._terminate_process(process, pgid)
            self.launch_processes = [item for item in self.launch_processes if item.pid != pid]
            self.process_labels.pop(pid, None)
            self.process_pairs.pop(pid, None)
            self.ready_process_pids.discard(pid)

        if failed_lines:
            self._append_scan_text('\nCamera launch validation failed:\n' + '\n'.join(failed_lines) + '\n')
            messagebox.showwarning('Camera launch failed', '\n'.join(failed_lines))

        self._update_running_status()
        self._update_ui_state()

    def _process_group_alive(self, pgid: int | None) -> bool:
        if pgid is None or not hasattr(os, 'killpg'):
            return False
        try:
            os.killpg(pgid, 0)
            return True
        except ProcessLookupError:
            return False
        except PermissionError:
            return True
        except Exception:
            return False

    def _process_alive(self, process: subprocess.Popen) -> bool:
        return (
            process.poll() is None
            or self._process_group_alive(self.launch_process_groups.get(process.pid))
        )

    def _running_processes(self) -> list[subprocess.Popen]:
        return [process for process in self.launch_processes if self._process_alive(process)]

    def _ready_processes(self) -> list[subprocess.Popen]:
        return [process for process in self._running_processes() if process.pid in self.ready_process_pids]

    def _starting_processes(self) -> list[subprocess.Popen]:
        return [process for process in self._running_processes() if process.pid not in self.ready_process_pids]

    def _process_label(self, process: subprocess.Popen) -> str:
        return self.process_labels.get(process.pid, f'camera process {process.pid}')

    def _update_running_status(self) -> None:
        running = self._running_processes()
        if not running:
            self.running_var.set('Cameras stopped')
            self.status_var.set('No camera launch processes are running')
            return
        ready = self._ready_processes()
        starting = self._starting_processes()
        if starting and ready:
            ready_labels = ', '.join(self._process_label(process) for process in ready)
            starting_labels = ', '.join(self._process_label(process) for process in starting)
            self.running_var.set(f'{len(ready)} camera(s) verified, {len(starting)} starting')
            self.status_var.set(f'Verified: {ready_labels}; waiting: {starting_labels}')
            return
        if starting:
            labels = ', '.join(self._process_label(process) for process in starting)
            self.running_var.set(f'{len(starting)} camera process(es) starting')
            self.status_var.set(f'Waiting for camera topics: {labels}')
            return
        labels = ', '.join(self._process_label(process) for process in running)
        self.running_var.set(f'{len(ready)} camera(s) verified')
        self.status_var.set(f'{len(ready)} camera(s) verified: {labels}')

    def _ensure_process_polling(self) -> None:
        if self.process_poll_active:
            return
        self.process_poll_active = True
        self.root.after(1000, self._poll_processes)

    def _poll_processes(self) -> None:
        if self.closing:
            self.process_poll_active = False
            return
        stopped = [process for process in self.launch_processes if process.poll() is not None]
        if stopped:
            stopped_lines = []
            for process in stopped:
                label = self._process_label(process)
                pgid = self.launch_process_groups.get(process.pid)
                if self._process_group_alive(pgid):
                    stopped_lines.append(
                        f'  {label} launch terminal exited but camera nodes were still running; cleaning up'
                    )
                    self._terminate_process(process, pgid)
                else:
                    stopped_lines.append(f'  {label} exited with code {process.poll()}')
            self._append_scan_text('\nCamera process stopped:\n' + '\n'.join(stopped_lines) + '\n')
            stopped_pids = {process.pid for process in stopped}
            self.launch_processes = [process for process in self.launch_processes if process.pid not in stopped_pids]
            for pid in stopped_pids:
                self.launch_process_groups.pop(pid, None)
                self.process_labels.pop(pid, None)
                self.process_pairs.pop(pid, None)
                self.ready_process_pids.discard(pid)
        running_count = len(self._running_processes())
        if running_count:
            self._update_running_status()
            self.root.after(1000, self._poll_processes)
        elif self.launch_processes or stopped:
            self.running_var.set('Cameras stopped')
            self.launch_processes = []
            self.launch_process_groups = {}
            self.process_labels = {}
            self.process_pairs = {}
            self.ready_process_pids = set()
            self.process_poll_active = False
            self._update_ui_state()
        else:
            self.process_poll_active = False

    def _any_process_running(self) -> bool:
        return any(self._process_alive(process) for process in self.launch_processes)

    def _any_process_starting(self) -> bool:
        return any(
            self._process_alive(process) and process.pid not in self.ready_process_pids
            for process in self.launch_processes
        )

    def _stop_cameras(self) -> None:
        self.launch_stop_requested = True
        self.pending_launch_pairs = []
        self.launching = False
        if not self.launch_processes and not self._any_process_running():
            self.running_var.set('Cameras stopped')
            self._append_scan_text('\nNo camera nodes are currently running.\n')
            self._update_ui_state()
            self.status_var.set('No camera nodes are currently running')
            return
        self.running_var.set('Stopping cameras...')
        for process in list(self.launch_processes):
            pgid = self.launch_process_groups.pop(process.pid, None)
            self._terminate_process(process, pgid)
        self.launch_processes = []
        self.launch_process_groups = {}
        self.process_labels = {}
        self.process_pairs = {}
        self.ready_process_pids = set()
        self.running_var.set('Cameras stopped')
        self._append_scan_text('\nStopped camera launch processes and child camera nodes.\n')
        self._update_ui_state()

    def _send_process_signal(
        self,
        process: subprocess.Popen,
        pgid: int | None,
        sig: signal.Signals,
    ) -> bool:
        if pgid is not None and hasattr(os, 'killpg'):
            try:
                os.killpg(pgid, sig)
                return True
            except ProcessLookupError:
                pass
        if process.poll() is None:
            try:
                process.send_signal(sig)
                return True
            except ProcessLookupError:
                pass
        return False

    def _wait_process_or_group_stopped(
        self,
        process: subprocess.Popen,
        pgid: int | None,
        timeout_sec: float,
    ) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            parent_running = process.poll() is None
            group_running = self._process_group_alive(pgid)
            if not parent_running and not group_running:
                return True
            time.sleep(0.05)
        return process.poll() is not None and not self._process_group_alive(pgid)

    def _terminate_process(self, process: subprocess.Popen, pgid: int | None = None) -> None:
        if process.poll() is not None and not self._process_group_alive(pgid):
            return

        self._send_process_signal(process, pgid, signal.SIGINT)
        if self._wait_process_or_group_stopped(process, pgid, PROCESS_STOP_TIMEOUT_SEC):
            return

        self._send_process_signal(process, pgid, signal.SIGTERM)
        if self._wait_process_or_group_stopped(process, pgid, PROCESS_STOP_TIMEOUT_SEC):
            return

        self._send_process_signal(process, pgid, signal.SIGKILL)
        self._wait_process_or_group_stopped(process, pgid, 1.0)

    def _on_close(self) -> None:
        self.closing = True
        if self._any_process_running():
            self._stop_cameras()
        self.root.destroy()


def main() -> None:
    root = tk.Tk()
    app = CameraLauncherApp(root)
    root.mainloop()


if __name__ == '__main__':
    main()
