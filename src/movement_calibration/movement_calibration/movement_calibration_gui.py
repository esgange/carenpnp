import hashlib
import json
import os
import queue
import re
import signal
import subprocess
import sys
import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import filedialog
from tkinter.scrolledtext import ScrolledText


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


DEFAULT_SCRIPTS_DIR = workspace_path('config', 'motion_debug_scripts')


def _default_output_file() -> Path:
    stamp = time.strftime('%d%m%Y')
    return workspace_path('calibration', f'relmovl_speed_calibration_{stamp}.json')


DEFAULT_OUTPUT_FILE = _default_output_file()
DEFAULT_SCRIPT_NAMES = ('x_calibrate', 'y_calibrate', 'z_calibrate')
DEFAULT_STARTUP_CP = 100
DEFAULT_STARTUP_SPEED_FACTOR = 50
SCRIPT_POINT_PATTERN = re.compile(
    r'^\s*(?:POINT(?:\s+\d+(?:/\d+)?)?\s+)?(MovJ|MovL)\s*:\s*(.+?)\s*$',
    re.IGNORECASE,
)


class MovementCalibrationMiniGui:
    def __init__(self) -> None:
        self.root = tk.Tk()
        self.root.title('Movement Calibration')
        self.root.geometry('860x640')
        self.root.minsize(760, 580)

        self._process: subprocess.Popen | None = None
        self._log_queue: queue.Queue[str] = queue.Queue()
        self._closed = False
        self._ui_running_mode = False
        self._run_start_time_sec: float | None = None
        self._run_prev_output_hash: str | None = None
        self._last_run_success: bool | None = None

        container = tk.Frame(self.root, padx=12, pady=12)
        container.pack(fill=tk.BOTH, expand=True)

        info_text = (
            'Script requirements:\n'
            '- Start script editor first: ros2 launch motion_debug motion_debug.launch.py\n'
            '- Select the script folder below (default shown).\n'
            '- Required script files: x_calibrate.json, y_calibrate.json, z_calibrate.json\n'
            '- Each script should include MovL points for that axis.\n'
            '- Use multiple speed factors; suggested v=5..60 in intervals of 5.\n'
            '- Startup CP/SF come from node params (startup_cp/startup_speed_factor).\n'
            '- Fit defaults: exclude v=100, ignore non-travel, and auto-remove speed plateau from fit.\n'
            '\n'
            'Output calibration file:\n'
            f'- {DEFAULT_OUTPUT_FILE}\n'
            '\n'
            'Press "Run Calibration" to start.'
        )
        self.info_view = tk.Text(
            container,
            height=12,
            wrap=tk.WORD,
            state=tk.NORMAL,
        )
        self.info_view.pack(fill=tk.X)
        self.info_view.insert('1.0', info_text)
        self.info_view.configure(state=tk.DISABLED)

        scripts_dir_row = tk.Frame(container)
        scripts_dir_row.pack(fill=tk.X, pady=(8, 6))
        tk.Label(scripts_dir_row, text='Scripts folder').pack(side=tk.LEFT)
        self.scripts_dir_var = tk.StringVar(value=str(DEFAULT_SCRIPTS_DIR))
        self.scripts_dir_entry = tk.Entry(scripts_dir_row, textvariable=self.scripts_dir_var)
        self.scripts_dir_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(8, 6))
        self.browse_scripts_dir_button = tk.Button(
            scripts_dir_row,
            text='Browse',
            command=self._browse_scripts_dir,
            width=10,
        )
        self.browse_scripts_dir_button.pack(side=tk.RIGHT)

        startup_row = tk.Frame(container)
        startup_row.pack(fill=tk.X, pady=(0, 8))
        tk.Label(startup_row, text='Startup CP (%)').pack(side=tk.LEFT)
        self.startup_cp_var = tk.IntVar(value=DEFAULT_STARTUP_CP)
        self.startup_cp_spinbox = tk.Spinbox(
            startup_row,
            from_=1,
            to=100,
            increment=1,
            textvariable=self.startup_cp_var,
            width=6,
            justify='center',
        )
        self.startup_cp_spinbox.pack(side=tk.LEFT, padx=(8, 18))
        tk.Label(startup_row, text='Startup SpeedFactor (%)').pack(side=tk.LEFT)
        self.startup_speed_factor_var = tk.IntVar(value=DEFAULT_STARTUP_SPEED_FACTOR)
        self.startup_speed_factor_spinbox = tk.Spinbox(
            startup_row,
            from_=1,
            to=100,
            increment=1,
            textvariable=self.startup_speed_factor_var,
            width=6,
            justify='center',
        )
        self.startup_speed_factor_spinbox.pack(side=tk.LEFT, padx=(8, 0))

        button_row = tk.Frame(container)
        button_row.pack(fill=tk.X, pady=(10, 8))

        self.run_button = tk.Button(
            button_row,
            text='Run Calibration',
            command=self._run_calibration_clicked,
            width=18,
            height=1,
        )
        self.run_button.pack(side=tk.LEFT)
        self._run_button_default_style = {
            'bg': self.run_button.cget('bg'),
            'activebackground': self.run_button.cget('activebackground'),
            'fg': self.run_button.cget('fg'),
            'activeforeground': self.run_button.cget('activeforeground'),
            'relief': self.run_button.cget('relief'),
            'state': tk.NORMAL,
        }

        self.status_var = tk.StringVar(value='Ready.')
        status_label = tk.Label(
            button_row,
            textvariable=self.status_var,
            anchor='w',
            justify=tk.LEFT,
        )
        status_label.pack(side=tk.LEFT, padx=(10, 0), fill=tk.X, expand=True)

        self.log_view = ScrolledText(
            container,
            height=18,
            wrap=tk.WORD,
            state=tk.DISABLED,
        )
        self.log_view.pack(fill=tk.BOTH, expand=True)

        self.root.protocol('WM_DELETE_WINDOW', self._on_close)
        self._set_running(False)
        self.root.after(100, self._poll_output)

    def _append_log(self, line: str) -> None:
        self.log_view.configure(state=tk.NORMAL)
        self.log_view.insert(tk.END, line.rstrip() + '\n')
        self.log_view.see(tk.END)
        self.log_view.configure(state=tk.DISABLED)

    def _set_running(self, running: bool) -> None:
        self._ui_running_mode = running
        if running:
            self.run_button.configure(
                text='Stop Calibration',
                command=self._stop_calibration_clicked,
                state=tk.NORMAL,
                bg='#dc2626',
                activebackground='#b91c1c',
                fg='white',
                activeforeground='white',
                relief=tk.SUNKEN,
            )
            self.browse_scripts_dir_button.configure(state=tk.DISABLED)
            self.scripts_dir_entry.configure(state=tk.DISABLED)
            self.startup_cp_spinbox.configure(state=tk.DISABLED)
            self.startup_speed_factor_spinbox.configure(state=tk.DISABLED)
            return

        self.run_button.configure(
            text='Run Calibration',
            command=self._run_calibration_clicked,
            state=self._run_button_default_style['state'],
            bg=self._run_button_default_style['bg'],
            activebackground=self._run_button_default_style['activebackground'],
            fg=self._run_button_default_style['fg'],
            activeforeground=self._run_button_default_style['activeforeground'],
            relief=self._run_button_default_style['relief'],
        )
        self.browse_scripts_dir_button.configure(state=tk.NORMAL)
        self.scripts_dir_entry.configure(state=tk.NORMAL)
        self.startup_cp_spinbox.configure(state=tk.NORMAL)
        self.startup_speed_factor_spinbox.configure(state=tk.NORMAL)

    def _read_percent_field(self, var: tk.IntVar, field_name: str) -> int | None:
        try:
            value = int(var.get())
        except (ValueError, tk.TclError):
            self.status_var.set(f'Invalid {field_name}. Use 1..100.')
            self._append_log(f'ERROR: invalid {field_name} value.')
            return None
        if value < 1 or value > 100:
            self.status_var.set(f'Invalid {field_name}. Use 1..100.')
            self._append_log(f'ERROR: {field_name}={value} is out of range (1..100).')
            return None
        return value

    def _selected_scripts_dir(self) -> Path:
        raw_value = self.scripts_dir_var.get().strip()
        if not raw_value:
            return DEFAULT_SCRIPTS_DIR
        return Path(raw_value).expanduser()

    def _browse_scripts_dir(self) -> None:
        initial_dir = self._selected_scripts_dir()
        selected_dir = filedialog.askdirectory(
            parent=self.root,
            title='Select movement calibration scripts folder',
            initialdir=str(initial_dir),
            mustexist=True,
        )
        if not selected_dir:
            return
        selected_path = str(Path(selected_dir).expanduser())
        self.scripts_dir_var.set(selected_path)
        self.status_var.set('Scripts folder updated.')
        self._append_log(f'Scripts folder set to "{selected_path}"')

    def _run_calibration_clicked(self) -> None:
        if self._process is not None and self._process.poll() is None:
            self.status_var.set('Calibration already running...')
            return

        scripts_dir = self._selected_scripts_dir()
        startup_cp = self._read_percent_field(self.startup_cp_var, 'startup CP')
        if startup_cp is None:
            return
        startup_speed_factor = self._read_percent_field(
            self.startup_speed_factor_var, 'startup SpeedFactor')
        if startup_speed_factor is None:
            return

        preflight_ok, preflight_messages = self._preflight_scripts(scripts_dir)
        for line in preflight_messages:
            self._append_log(line)
        if not preflight_ok:
            self.status_var.set('Calibration blocked. Fix script files first.')
            return

        self._run_start_time_sec = time.time()
        self._run_prev_output_hash = self._output_file_hash()
        self._last_run_success = None
        self._set_running(True)
        self.status_var.set('Starting movement calibration...')
        self._append_log('=== Starting movement calibration ===')

        command = [
            sys.executable,
            '-m',
            'movement_calibration.movement_calibration',
            '--ros-args',
            '--log-level',
            'movement_calibration:=info',
            '-p',
            f'scripts_dir:={scripts_dir}',
            '-p',
            f'script_names_csv:={",".join(DEFAULT_SCRIPT_NAMES)}',
            '-p',
            f'output_file:={DEFAULT_OUTPUT_FILE}',
            '-p',
            f'startup_cp:={startup_cp}',
            '-p',
            f'startup_speed_factor:={startup_speed_factor}',
        ]
        self._append_log(
            f'Running with scripts_dir="{scripts_dir}", '
            f'script_names={",".join(DEFAULT_SCRIPT_NAMES)}, '
            f'startup_cp={startup_cp}, startup_speed_factor={startup_speed_factor}'
        )
        env = dict(os.environ)
        env['PYTHONUNBUFFERED'] = '1'
        try:
            self._process = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                env=env,
            )
        except Exception as exc:
            self._set_running(False)
            self.status_var.set('Failed to start movement_calibration.')
            self._append_log(f'Failed to start process: {exc}')
            return

        reader = threading.Thread(target=self._read_process_output, daemon=True)
        reader.start()

    def _stop_calibration_clicked(self) -> None:
        process = self._process
        if process is None or process.poll() is not None:
            self.status_var.set('Calibration is not running.')
            self._set_running(False)
            return

        self.status_var.set('Stopping movement calibration...')
        self._append_log('=== Stop requested by user ===')
        try:
            process.send_signal(signal.SIGINT)
        except Exception as exc:
            self._append_log(f'Failed to send stop signal: {exc}')
            return
        self.root.after(1500, self._force_terminate_if_running)

    def _force_terminate_if_running(self) -> None:
        process = self._process
        if process is None or process.poll() is not None:
            return
        self._append_log('Calibration still running after SIGINT; sending SIGTERM...')
        try:
            process.terminate()
        except Exception as exc:
            self._append_log(f'Failed to terminate process: {exc}')

    def _output_file_hash(self) -> str | None:
        try:
            with open(DEFAULT_OUTPUT_FILE, 'rb') as calibration_file:
                data = calibration_file.read()
        except (FileNotFoundError, OSError):
            return None
        return hashlib.sha256(data).hexdigest()

    def _preflight_scripts(self, scripts_dir: Path) -> tuple[bool, list[str]]:
        messages: list[str] = []
        ok = True
        if not scripts_dir.exists():
            return False, [f'Preflight ERROR: scripts folder does not exist: {scripts_dir}']
        if not scripts_dir.is_dir():
            return False, [f'Preflight ERROR: scripts folder is not a directory: {scripts_dir}']
        for script_name in DEFAULT_SCRIPT_NAMES:
            script_path = scripts_dir / f'{script_name}.json'
            if not script_path.exists():
                messages.append(f'Preflight ERROR: missing script {script_path}')
                ok = False
                continue

            try:
                with open(script_path, 'r', encoding='utf-8') as script_file:
                    root = json.load(script_file)
            except Exception as exc:
                messages.append(f'Preflight ERROR: cannot read {script_path}: {exc}')
                ok = False
                continue

            points = root.get('points')
            if not isinstance(points, list):
                messages.append(f'Preflight ERROR: {script_path} has no "points" list.')
                ok = False
                continue

            movl_count = 0
            for point in points:
                if not isinstance(point, dict):
                    continue
                motion_type = str(point.get('motion_type', '')).strip().lower()
                values = point.get('values')
                if motion_type in {'movl', 'moll'} and isinstance(values, list) and len(values) == 6:
                    movl_count += 1
                    continue
                command = str(point.get('command', '')).strip()
                match = SCRIPT_POINT_PATTERN.match(command)
                if match is not None and match.group(1).strip().lower() == 'movl':
                    movl_count += 1

            messages.append(f'Preflight: {script_name}.json MovL points={movl_count}')
            if movl_count < 2:
                messages.append(
                    f'Preflight ERROR: {script_name}.json needs at least 2 MovL points '
                    '(first is startup move, next points are measured).'
                )
                ok = False

        return ok, messages

    def _validate_new_calibration_file(self) -> tuple[bool, str]:
        if not DEFAULT_OUTPUT_FILE.exists():
            return False, f'Calibration file missing: {DEFAULT_OUTPUT_FILE}'

        try:
            with open(DEFAULT_OUTPUT_FILE, 'r', encoding='utf-8') as calibration_file:
                payload = json.load(calibration_file)
        except Exception as exc:
            return False, f'Calibration file is not valid JSON: {exc}'

        if not isinstance(payload, dict):
            return False, 'Calibration file format is invalid.'
        if not isinstance(payload.get('axis_models'), dict):
            return False, 'Calibration file missing axis_models.'
        raw_segments = payload.get('raw_segments')
        if not isinstance(raw_segments, list) or not raw_segments:
            return False, 'Calibration file has no measured segments.'
        current_hash = self._output_file_hash()
        if current_hash is None:
            return False, 'Calibration file could not be read for hash check.'
        if self._run_prev_output_hash is not None and current_hash == self._run_prev_output_hash:
            return False, 'Calibration file content unchanged after this run.'
        return True, ''

    def _read_process_output(self) -> None:
        process = self._process
        if process is None or process.stdout is None:
            return
        for line in process.stdout:
            self._log_queue.put(line)
        return_code = process.wait()
        self._log_queue.put(f'=== movement_calibration exited with code {return_code} ===\n')
        if return_code == 0:
            file_ok, reason = self._validate_new_calibration_file()
            self._last_run_success = file_ok
            if not file_ok:
                self._log_queue.put(
                    f'Calibration run returned success, but output file validation failed: {reason}\n'
                )
                self._log_queue.put('Treating run as failed.\n')
                return
            self._log_queue.put(
                f'Calibration finished. Check output file: {DEFAULT_OUTPUT_FILE}\n'
            )
            return
        else:
            self._last_run_success = False
            self._log_queue.put('Calibration failed. Check log output above.\n')

    def _poll_output(self) -> None:
        had_updates = False
        while True:
            try:
                line = self._log_queue.get_nowait()
            except queue.Empty:
                break
            had_updates = True
            self._append_log(line)

        if self._process is not None and self._process.poll() is not None:
            if self._ui_running_mode:
                self._set_running(False)
                if self._last_run_success is True:
                    self.status_var.set('Calibration completed.')
                else:
                    self.status_var.set('Calibration failed. Check log output.')

        if not self._closed:
            self.root.after(100, self._poll_output)
        elif had_updates:
            self.root.update_idletasks()

    def _on_close(self) -> None:
        self._closed = True
        process = self._process
        if process is not None and process.poll() is None:
            try:
                process.send_signal(signal.SIGTERM)
            except Exception:
                pass
        self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()


def main() -> None:
    app = MovementCalibrationMiniGui()
    app.run()
