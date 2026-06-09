"""Runtime program payload parsing for online Robot Cell Orchestrator.

The current online contract sends local teach filenames and tray placement to
Robot Cell Orchestrator. Older helpers for direct YAML materialization remain in this module
for test/dev compatibility, but production load flow calls
``parse_runtime_program_selection`` and lets Robot Cell Orchestrator own the runtime folder.
"""

from __future__ import annotations

import base64
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional

RUNTIME_FILE_KEYS = ("runtime_files", "program_files", "teach_files")
YAML_SUFFIXES = (".yaml", ".yml")


@dataclass(frozen=True)
class RuntimeProgramSelection:
    qqc_id: str
    bin_teach_file: str
    item_teach_file: str
    tray_teach_file: str
    tray_x_mm: float
    tray_y_mm: float
    tray_rz_deg: float

    @property
    def requested_files(self) -> list[str]:
        return [self.bin_teach_file, self.item_teach_file, self.tray_teach_file]


def parse_runtime_program_selection(
    payload: dict[str, Any],
) -> tuple[RuntimeProgramSelection | None, str | None]:
    """Parse the new load_program contract.

    Accepted shape:

    ``qqc_id`` plus teach names either as direct keys
    (``bin_teach_file``, ``item_teach_file``, ``tray_teach_file``) or inside a
    ``teach_files`` / ``teach`` / ``program`` map. Tray placement can be direct
    (``tray_x_mm`` / ``tray_y_mm`` / ``tray_rz_deg`` or ``x`` / ``y`` / ``rz``)
    or nested in ``tray_position`` / ``tray_placement`` / ``placement``.
    """
    qqc_id = _string_value(payload, "qqc_id", "program_id")
    if not qqc_id:
        return None, "missing_qqc_id"

    teach_maps = _nested_maps(payload, "teach_files", "teach", "program")
    bin_teach_file = _string_value(
        payload,
        "bin_teach_file",
        "bin_file",
        "bin_teach",
        "bin",
        nested=teach_maps,
    )
    item_teach_file = _string_value(
        payload,
        "item_teach_file",
        "item_file",
        "item_detect_file",
        "item_detect",
        "item_teach",
        "item",
        nested=teach_maps,
    )
    tray_teach_file = _string_value(
        payload,
        "tray_teach_file",
        "tray_file",
        "tray_detect_file",
        "tray_detect",
        "tray_teach",
        "tray",
        nested=teach_maps,
    )
    missing = [
        name for name, value in (
            ("bin_teach_file", bin_teach_file),
            ("item_teach_file", item_teach_file),
            ("tray_teach_file", tray_teach_file),
        )
        if not value
    ]
    if missing:
        return None, "missing_" + "_".join(missing)

    placement_maps = _nested_maps(payload, "tray_position", "tray_placement", "placement")
    tray_x, x_reason = _float_value(
        payload,
        "tray_x_mm",
        "tray_x",
        "x_mm",
        "x",
        nested=placement_maps,
        label="tray_x_mm",
    )
    tray_y, y_reason = _float_value(
        payload,
        "tray_y_mm",
        "tray_y",
        "y_mm",
        "y",
        nested=placement_maps,
        label="tray_y_mm",
    )
    tray_rz, rz_reason = _float_value(
        payload,
        "tray_rz_deg",
        "tray_rz",
        "rz_deg",
        "rz",
        nested=placement_maps,
        label="tray_rz_deg",
    )
    for reason in (x_reason, y_reason, rz_reason):
        if reason is not None:
            return None, reason

    return RuntimeProgramSelection(
        qqc_id=qqc_id,
        bin_teach_file=bin_teach_file,
        item_teach_file=item_teach_file,
        tray_teach_file=tray_teach_file,
        tray_x_mm=float(tray_x),
        tray_y_mm=float(tray_y),
        tray_rz_deg=float(tray_rz),
    ), None


def _nested_maps(payload: dict[str, Any], *keys: str) -> list[dict[str, Any]]:
    maps: list[dict[str, Any]] = []
    for key in keys:
        value = payload.get(key)
        if isinstance(value, dict):
            maps.append(value)
    return maps


def _string_value(
    payload: dict[str, Any],
    *keys: str,
    nested: list[dict[str, Any]] | None = None,
) -> str:
    for source in (payload, *(nested or [])):
        for key in keys:
            value = source.get(key)
            if value is None:
                continue
            text = str(value).strip()
            if text:
                return text
    return ""


def _float_value(
    payload: dict[str, Any],
    *keys: str,
    nested: list[dict[str, Any]] | None = None,
    label: str,
) -> tuple[float | None, str | None]:
    raw = _string_value(payload, *keys, nested=nested)
    if not raw:
        return None, f"missing_{label}"
    try:
        return float(raw), None
    except ValueError:
        return None, f"invalid_{label}"


def materialize_runtime_program(payload: dict[str, Any], runtime_dir: str) -> dict[str, Any]:
    """Replace runtime YAMLs with files carried by ``payload``.

    The accepted payload shape is intentionally broad to preserve the
    external contract while the master side settles:

    - ``runtime_files`` / ``program_files`` / ``teach_files``
    - list of objects, or map of filename -> object/content
    - content in ``content_b64``, ``b64``, or plain text ``content``
    """
    raw_files = _runtime_file_payload(payload)
    entries, err = _normalise_runtime_file_entries(raw_files)
    if err is not None:
        return {"status": "error", "reason": err}

    destination_dir = Path(runtime_dir).expanduser()
    try:
        destination_dir.mkdir(parents=True, exist_ok=True)
        for existing in destination_dir.iterdir():
            if existing.is_file() and existing.suffix.lower() in YAML_SUFFIXES:
                existing.unlink()

        written: list[str] = []
        for name, body in entries:
            destination = destination_dir / name
            destination.write_bytes(body)
            written.append(name)
    except Exception as exc:
        return {"status": "error", "reason": f"runtime_write_failed: {exc}"}

    return {"status": "ok", "files": written}


def _runtime_file_payload(payload: dict[str, Any]) -> Any:
    for key in RUNTIME_FILE_KEYS:
        value = payload.get(key)
        if value:
            return value
    return None


def _normalise_runtime_file_entries(
    raw_files: Any,
) -> tuple[list[tuple[str, bytes]], Optional[str]]:
    if raw_files is None:
        return [], "missing_runtime_files"

    if isinstance(raw_files, dict):
        raw_entries = [
            {"filename": filename, **value}
            if isinstance(value, dict)
            else {"filename": filename, "content": value}
            for filename, value in raw_files.items()
        ]
    elif isinstance(raw_files, list):
        raw_entries = raw_files
    else:
        return [], "runtime_files_not_list_or_map"

    if not raw_entries:
        return [], "runtime_files_empty"

    entries: list[tuple[str, bytes]] = []
    seen_names: set[str] = set()
    for index, entry in enumerate(raw_entries):
        if not isinstance(entry, dict):
            return [], f"runtime_file_{index}_not_object"

        raw_name = entry.get("filename") or entry.get("name") or entry.get("path")
        if not raw_name:
            return [], f"runtime_file_{index}_missing_filename"

        raw_text = str(raw_name).replace("\\", "/")
        name = Path(raw_text).name
        if name in ("", ".", "..") or "/" in raw_text:
            return [], f"runtime_file_{index}_unsafe_filename"
        if Path(name).suffix.lower() not in YAML_SUFFIXES:
            return [], f"runtime_file_{index}_not_yaml"
        if name in seen_names:
            return [], f"runtime_file_{index}_duplicate_filename"
        seen_names.add(name)

        if "content_b64" in entry:
            try:
                body = base64.b64decode(str(entry["content_b64"]), validate=True)
            except Exception as exc:
                return [], f"runtime_file_{index}_bad_base64: {exc}"
        elif "b64" in entry:
            try:
                body = base64.b64decode(str(entry["b64"]), validate=True)
            except Exception as exc:
                return [], f"runtime_file_{index}_bad_base64: {exc}"
        elif "content" in entry:
            content = entry["content"]
            body = content if isinstance(content, bytes) else str(content).encode("utf-8")
        else:
            return [], f"runtime_file_{index}_missing_content"

        entries.append((name, body))

    return entries, None


__all__ = [
    "RuntimeProgramSelection",
    "materialize_runtime_program",
    "parse_runtime_program_selection",
]
