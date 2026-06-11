from __future__ import annotations

import fnmatch
import os
from pathlib import Path


SHARED_ROBOT_IP_ADDRESS = "192.168.200.1"


def _looks_like_workspace_root(path: Path) -> bool:
    return (
        (path / "station_config").exists()
        or (path / "src" / "dobot_msgs_v4").exists()
        or (path / "README.md").exists()
    )


def workspace_root() -> Path:
    for name in ("DOBOT_PICKN_PLACE_ROOT", "DOBOT_WORKSPACE_ROOT"):
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


def _station_config_value(*keys: str) -> str:
    path = workspace_root() / "station_config"
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except OSError:
        return ""

    values: dict[str, str] = {}
    for raw_line in lines:
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        if line.startswith("export "):
            line = line[len("export "):].strip()
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        value = value.strip()
        if len(value) >= 2 and value[0] == value[-1] and value[0] in ("'", '"'):
            value = value[1:-1]
        values[key.strip()] = value.strip()

    for key in keys:
        value = values.get(key, "")
        if value:
            return value
    return ""


def resolve_robot_ip_address(value: str = "") -> str:
    requested = str(value or "").strip()
    if requested:
        return requested
    env_ip = os.environ.get("ROBOT_IP_ADDRESS", "").strip()
    if env_ip:
        return env_ip
    return _station_config_value("ROBOT_IP_ADDRESS", "ip_address")


def requires_manual_selection(robot_ip_address: str) -> bool:
    return str(robot_ip_address or "").strip() == SHARED_ROBOT_IP_ADDRESS


def calibration_file_is_usable(path: str | Path) -> bool:
    try:
        candidate = Path(path).expanduser()
        return candidate.exists() and candidate.is_file() and candidate.stat().st_size > 0
    except OSError:
        return False


def choose_required_calibration(
    *,
    calibration_dir: str | Path,
    filename_pattern: str,
    calibration_label: str,
    launch_label: str,
    robot_ip_address: str,
    launch_argument_name: str = "calibration_file",
) -> str:
    initial_dir = Path(calibration_dir).expanduser()
    if not initial_dir.exists():
        initial_dir = workspace_root() / "calibration"

    try:
        import tkinter as tk
        from tkinter import filedialog

        root = tk.Tk()
        root.withdraw()
        root.attributes("-topmost", True)
        try:
            selected = filedialog.askopenfilename(
                parent=root,
                title=f"Select {calibration_label} for {launch_label}",
                initialdir=str(initial_dir),
                filetypes=(
                    (calibration_label, filename_pattern),
                    ("YAML calibration", "*.yaml"),
                ),
            )
        finally:
            root.destroy()
    except Exception as exc:
        raise RuntimeError(
            f"[{launch_label}] Robot IP {robot_ip_address} requires manual "
            f"{calibration_label} selection, but the file chooser could not open: {exc}. "
            f"Provide the file explicitly with {launch_argument_name}:=<path>."
        ) from exc

    if not selected:
        raise RuntimeError(
            f"[{launch_label}] Robot IP {robot_ip_address} requires manual "
            f"{calibration_label} selection; no file was selected. "
            f"Relaunch and choose a file, or provide {launch_argument_name}:=<path>."
        )

    selected_path = Path(selected).expanduser().resolve()
    if not fnmatch.fnmatch(selected_path.name, filename_pattern):
        raise RuntimeError(
            f"[{launch_label}] Selected file does not match {filename_pattern!r}: "
            f"{selected_path}"
        )
    if not calibration_file_is_usable(selected_path):
        raise RuntimeError(
            f"[{launch_label}] Selected calibration file is missing or empty: "
            f"{selected_path}"
        )
    return str(selected_path)
