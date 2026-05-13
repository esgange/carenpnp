from __future__ import annotations

import os
import re
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
LAUNCH_STARTUP_GRACE_MS = 5500

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
        self.process_labels: dict[int, str] = {}
        self.detected_serials: list[str] = []
        self.scanning = False
        self.launching = False
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

        if self.launching:
            self.status_var.set('Checking configured cameras...')
        elif running:
            self.status_var.set('Camera launch processes are running')
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
        stop_state = tk.NORMAL if running else tk.DISABLED

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
            messagebox.showwarning(title, '\n'.join(errors))
            self._append_scan_text('\nCamera launch skipped:\n' + '\n'.join(errors) + '\n')
            self._update_ui_state()
            return

        pairs = self._configured_camera_pairs()
        if not pairs:
            message = 'No configured camera slots have both serial number and camera name.'
            if auto:
                messagebox.showwarning('Auto launch skipped', message)
            else:
                messagebox.showerror('No cameras configured', message)
            self._append_scan_text(f'\n{message}\n')
            self._update_ui_state()
            return

        if not auto and not self._save_config():
            return

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
        if self.closing:
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
            warnings.append('Configured camera(s) not detected:')
            warnings.extend(f'  {self._slot_label(pair)}' for pair in missing_connected)

        if not launch_pairs:
            self.launching = False
            self.running_var.set('No configured cameras running')
            self.status_var.set('No configured cameras detected')
            if warnings:
                messagebox.showwarning('No configured cameras detected', '\n'.join(warnings))
            else:
                messagebox.showwarning('No configured cameras detected', 'No configured cameras were detected.')
            self._update_ui_state()
            return

        if warnings:
            messagebox.showwarning(
                'Partial camera startup',
                '\n'.join(warnings) + '\n\nLaunching detected configured camera(s).',
            )

        self._launch_camera_pairs(launch_pairs)

    def _launch_camera_pairs(self, pairs: list[dict[str, str]]) -> None:
        orbbec_launch_args = _orbbec_launch_cli_args(self.config_path)
        launched: list[subprocess.Popen] = []
        device_num = max(1, len(pairs))
        try:
            for pair in pairs:
                command = [
                    'ros2',
                    'launch',
                    'orbbec_camera',
                    'gemini_330_series.launch.py',
                    f'camera_name:={pair["camera_name"]}',
                    f'serial_number:={pair["serial_number"]}',
                    f'device_num:={device_num}',
                    *orbbec_launch_args,
                ]
                process = subprocess.Popen(command, preexec_fn=os.setsid)
                launched.append(process)
                self.process_labels[process.pid] = self._slot_label(pair)
                self._append_scan_text(
                    f'\nLaunched {pair["camera_name"]} with SN {pair["serial_number"]}.\n'
                )
        except Exception as exc:  # noqa: BLE001
            for process in launched:
                self._terminate_process(process)
            messagebox.showerror('Launch failed', f'Failed to launch cameras:\n{exc}')
            self.launch_processes = []
            self.process_labels = {}
            self.launching = False
            self._update_ui_state()
            return

        self.launch_processes = launched
        self.launching = False
        self._update_running_status()
        self._update_ui_state()
        self.root.after(LAUNCH_STARTUP_GRACE_MS, lambda processes=launched: self._check_startup_status(processes))
        self._poll_processes()

    def _running_processes(self) -> list[subprocess.Popen]:
        return [process for process in self.launch_processes if process.poll() is None]

    def _process_label(self, process: subprocess.Popen) -> str:
        return self.process_labels.get(process.pid, f'camera process {process.pid}')

    def _update_running_status(self) -> None:
        running = self._running_processes()
        if not running:
            self.running_var.set('Cameras stopped')
            self.status_var.set('No camera launch processes are running')
            return
        labels = ', '.join(self._process_label(process) for process in running)
        self.running_var.set(f'{len(running)} camera process(es) running')
        self.status_var.set(f'{len(running)} camera(s) running: {labels}')

    def _check_startup_status(self, processes: list[subprocess.Popen]) -> None:
        if self.closing:
            return
        failed = [process for process in processes if process.poll() is not None]
        running = [process for process in processes if process.poll() is None]
        if failed:
            failed_lines = [
                f'  {self._process_label(process)} exited with code {process.poll()}'
                for process in failed
            ]
            self._append_scan_text('\nCamera launch failure detected:\n' + '\n'.join(failed_lines) + '\n')
            self.launch_processes = [process for process in self.launch_processes if process.poll() is None]
            self._update_running_status()
            message = 'Camera launch failed:\n' + '\n'.join(failed_lines)
            if running:
                running_lines = [f'  {self._process_label(process)}' for process in running]
                message += '\n\nStill running:\n' + '\n'.join(running_lines)
            else:
                message += '\n\nNo configured cameras are running.'
            messagebox.showwarning('Camera launch status', message)
            self._update_ui_state()

    def _poll_processes(self) -> None:
        if self.closing:
            return
        stopped = [process for process in self.launch_processes if process.poll() is not None]
        if stopped:
            stopped_lines = [
                f'  {self._process_label(process)} exited with code {process.poll()}'
                for process in stopped
            ]
            self._append_scan_text('\nCamera process stopped:\n' + '\n'.join(stopped_lines) + '\n')
            self.launch_processes = [process for process in self.launch_processes if process.poll() is None]
        running_count = len(self._running_processes())
        if running_count:
            self._update_running_status()
            self.root.after(1000, self._poll_processes)
        elif self.launch_processes or stopped:
            self.running_var.set('Cameras stopped')
            self.launch_processes = []
            self.process_labels = {}
            self._update_ui_state()

    def _any_process_running(self) -> bool:
        return any(process.poll() is None for process in self.launch_processes)

    def _stop_cameras(self) -> None:
        if not self.launch_processes:
            self._update_ui_state()
            return
        self.running_var.set('Stopping cameras...')
        for process in list(self.launch_processes):
            self._terminate_process(process)
        self.launch_processes = []
        self.process_labels = {}
        self.running_var.set('Cameras stopped')
        self._append_scan_text('\nStopped camera launch processes.\n')
        self._update_ui_state()

    def _terminate_process(self, process: subprocess.Popen) -> None:
        if process.poll() is not None:
            return
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGINT)
        except Exception:
            process.terminate()

        deadline = time.monotonic() + 4.0
        while process.poll() is None and time.monotonic() < deadline:
            time.sleep(0.1)

        if process.poll() is None:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            except Exception:
                process.terminate()

        deadline = time.monotonic() + 2.0
        while process.poll() is None and time.monotonic() < deadline:
            time.sleep(0.1)

        if process.poll() is None:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGKILL)
            except Exception:
                process.kill()

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
