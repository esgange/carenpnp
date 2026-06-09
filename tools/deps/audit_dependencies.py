#!/usr/bin/env python3
"""Audit workspace dependencies and write the offline third-party manifest."""

from __future__ import annotations

import argparse
import ast
import hashlib
import os
import re
import subprocess
import sys
import xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import Path

try:
    import yaml
except ImportError:  # pragma: no cover
    yaml = None


ROOT = Path(__file__).resolve().parents[2]
APT_LIST = ROOT / "tools" / "deps" / "apt-packages.freeze.txt"
DEFAULT_OUTPUT = ROOT / "third_party" / "manifest.yaml"
THIRD_PARTY_HASH_EXTS = {
    ".deb",
    ".gz",
    ".lock",
    ".whl",
    ".pt",
    ".onnx",
    ".yaml",
    ".yml",
    ".txt",
    ".zip",
}
THIRD_PARTY_HASH_NAMES = {
    "SHA256SUMS",
}
THIRD_PARTY_SKIP_PARTS = {
    ".apt-cache",
    ".apt-state",
    ".git",
    ".venv",
    "__pycache__",
}
STD_PY_MODULES = {
    "__future__",
    "argparse",
    "ast",
    "collections",
    "copy",
    "csv",
    "dataclasses",
    "datetime",
    "enum",
    "functools",
    "glob",
    "hashlib",
    "importlib",
    "itertools",
    "json",
    "math",
    "os",
    "pathlib",
    "queue",
    "re",
    "shutil",
    "signal",
    "statistics",
    "subprocess",
    "sys",
    "tempfile",
    "threading",
    "time",
    "traceback",
    "typing",
    "xml",
}


def run(command: list[str]) -> str:
    try:
        return subprocess.check_output(command, text=True, stderr=subprocess.DEVNULL).strip()
    except Exception:
        return ""


def read_apt_specs() -> list[str]:
    specs = []
    if not APT_LIST.exists():
        return specs
    for raw in APT_LIST.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if line and not line.startswith("#"):
            specs.append(line)
    return specs


def apt_package_name(spec: str) -> str:
    return spec.split("=", 1)[0].strip()


def dpkg_versions(names: list[str]) -> dict[str, str]:
    versions = {}
    for name in names:
        output = run(["dpkg-query", "-W", "-f=${Version}", name])
        if output:
            versions[name] = output
    return versions


def package_xml_deps() -> dict[str, dict[str, list[str]]]:
    packages: dict[str, dict[str, list[str]]] = {}
    for path in sorted((ROOT / "src").glob("*/package.xml")):
        root = ET.parse(path).getroot()
        name = root.findtext("name") or path.parent.name
        groups: dict[str, list[str]] = defaultdict(list)
        for tag in (
            "buildtool_depend",
            "build_depend",
            "build_export_depend",
            "exec_depend",
            "depend",
            "test_depend",
        ):
            for elem in root.findall(tag):
                if elem.text and elem.text.strip():
                    groups[tag].append(elem.text.strip())
        packages[name] = {key: sorted(set(value)) for key, value in sorted(groups.items())}
    return packages


def cmake_packages() -> dict[str, list[str]]:
    found: dict[str, list[str]] = {}
    pattern = re.compile(r"find_package\(([^)\s]+)")
    for path in sorted((ROOT / "src").glob("*/CMakeLists.txt")):
        matches = sorted(set(pattern.findall(path.read_text(encoding="utf-8", errors="ignore"))))
        if matches:
            found[path.parent.name] = matches
    return found


def python_imports() -> dict[str, list[str]]:
    imports: dict[str, list[str]] = {}
    for path in sorted((ROOT / "src").rglob("*.py")):
        if {"build", "install", "log"} & set(path.parts):
            continue
        try:
            tree = ast.parse(path.read_text(encoding="utf-8"))
        except Exception:
            continue
        names = set()
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                names.update(alias.name.split(".", 1)[0] for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module:
                names.add(node.module.split(".", 1)[0])
        external = sorted(name for name in names if name not in STD_PY_MODULES)
        if external:
            imports[str(path.relative_to(ROOT))] = external
    return imports


def requirement_lines(path: Path) -> list[str]:
    if not path.exists():
        return []
    lines = []
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if line and not line.startswith("#"):
            lines.append(line)
    return lines


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as infile:
        for chunk in iter(lambda: infile.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def third_party_assets() -> list[dict[str, object]]:
    assets = []
    for path in sorted((ROOT / "third_party").rglob("*")):
        if not path.is_file() or path.name == ".gitkeep":
            continue
        if path == DEFAULT_OUTPUT:
            continue
        if THIRD_PARTY_SKIP_PARTS & set(path.parts):
            continue
        if path.suffix not in THIRD_PARTY_HASH_EXTS and path.name not in THIRD_PARTY_HASH_NAMES:
            continue
        assets.append(
            {
                "path": str(path.relative_to(ROOT)),
                "size_bytes": path.stat().st_size,
                "sha256": sha256(path),
            }
        )
    return assets


def sam2_git_info() -> dict[str, str]:
    sam2 = ROOT / "third_party" / "sam2"
    if not sam2.exists():
        return {}
    return {
        "path": "third_party/sam2",
        "remote": run(["git", "-C", str(sam2), "remote", "get-url", "origin"]),
        "commit": run(["git", "-C", str(sam2), "rev-parse", "HEAD"]),
    }


def write_manifest(data: dict, output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    if yaml is None:
        import json

        output.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
        return
    output.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")


def build_manifest() -> dict:
    apt_specs = read_apt_specs()
    apt_names = [apt_package_name(spec) for spec in apt_specs]
    return {
        "target": {
            "os": "Ubuntu 22.04",
            "ros_distro": "humble",
            "python": "3.10",
            "ai_acceleration": "cpu",
        },
        "apt": {
            "spec_file": str(APT_LIST.relative_to(ROOT)),
            "requested_specs": apt_specs,
            "downloaded_lock": requirement_lines(ROOT / "third_party" / "debs" / "apt-downloaded.lock"),
            "installed_versions": dpkg_versions(apt_names),
        },
        "python": {
            "requirements": requirement_lines(ROOT / "requirements.txt"),
            "lock": requirement_lines(ROOT / "requirements.lock.txt"),
            "venv": "third_party/.venv",
        },
        "workspace": {
            "package_xml_dependencies": package_xml_deps(),
            "cmake_find_packages": cmake_packages(),
            "python_imports": python_imports(),
        },
        "third_party": {
            "sam2": sam2_git_info(),
            "assets": third_party_assets(),
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=DEFAULT_OUTPUT,
        help="Manifest path to write. Defaults to third_party/manifest.yaml.",
    )
    parser.add_argument("--check", action="store_true", help="Print manifest to stdout only.")
    args = parser.parse_args()

    data = build_manifest()
    if args.check:
        if yaml is not None:
            print(yaml.safe_dump(data, sort_keys=False))
        else:
            import json

            print(json.dumps(data, indent=2))
        return 0
    write_manifest(data, args.output)
    print(f"Wrote {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
