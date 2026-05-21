"""Cell External Bridge runtime configuration.

Single dataclass, root ``station_config`` loader, optional YAML overlay, and
env-var overrides. See README's "Configuration reference" table for the
canonical knob list.
"""

from __future__ import annotations

import dataclasses
import logging
import os
from pathlib import Path
from typing import Any, Callable, Mapping

try:
    import yaml  # type: ignore
except ImportError:
    yaml = None  # YAML overlay is optional; env-var-only setups still work.

logger = logging.getLogger("cell_external_bridge.config")

_STATION_CONFIG_ENV = "STATION_CONFIG_PATH"
_LEGACY_ROBOT_CONFIG_ENV = "ROBOT_CONFIG_PATH"

_YAML_KEYS = (
    "belt_id", "edge_node_id", "robot_arm_id", "arm_number",
    "rabbitmq_url", "runtime_dir", "exchange_name",
    "robot_cell_orchestrator_load_program_service",
    "robot_cell_orchestrator_validate_service", "robot_cell_orchestrator_start_service",
    "robot_cell_orchestrator_place_service",
    "robot_cell_orchestrator_events_topic", "robot_cell_orchestrator_phase_timeout_s",
    "offline_debug", "datalog_dir",
)

_LEGACY_YAML_KEY_ALIASES = {
    "pick_cycle_load_program_service": "robot_cell_orchestrator_load_program_service",
    "pick_cycle_validate_service": "robot_cell_orchestrator_validate_service",
    "pick_cycle_start_service": "robot_cell_orchestrator_start_service",
    "pick_cycle_place_service": "robot_cell_orchestrator_place_service",
    "pick_cycle_events_topic": "robot_cell_orchestrator_events_topic",
    "pick_cycle_phase_timeout_s": "robot_cell_orchestrator_phase_timeout_s",
}

_ENV_STR_OVERRIDES = {
    "belt_id": ("BELT_ID",),
    "edge_node_id": ("CELL_BRIDGE_ID", "EDGE_NODE_ID"),
    "robot_arm_id": ("ROBOT_ARM_ID",),
    "rabbitmq_url": ("RABBITMQ_URL",),
    "runtime_dir": ("RUNTIME_DIR",),
    "exchange_name": ("RABBITMQ_EXCHANGE",),
    "simulate_mode": ("CELL_BRIDGE_SIMULATE_MODE", "EDGE_SIMULATE_MODE"),
    "robot_cell_orchestrator_load_program_service": (
        "ROBOT_CELL_ORCHESTRATOR_LOAD_PROGRAM_SERVICE",
        "PICK_CYCLE_LOAD_PROGRAM_SERVICE",
    ),
    "robot_cell_orchestrator_validate_service": (
        "ROBOT_CELL_ORCHESTRATOR_VALIDATE_SERVICE",
        "PICK_CYCLE_VALIDATE_SERVICE",
    ),
    "robot_cell_orchestrator_start_service": (
        "ROBOT_CELL_ORCHESTRATOR_START_SERVICE",
        "PICK_CYCLE_START_SERVICE",
    ),
    "robot_cell_orchestrator_place_service": (
        "ROBOT_CELL_ORCHESTRATOR_PLACE_SERVICE",
        "PICK_CYCLE_PLACE_SERVICE",
    ),
    "robot_cell_orchestrator_events_topic": (
        "ROBOT_CELL_ORCHESTRATOR_EVENTS_TOPIC",
        "PICK_CYCLE_EVENTS_TOPIC",
    ),
    "datalog_dir": ("CELL_BRIDGE_DATALOG_DIR",),
}


@dataclasses.dataclass
class Config:
    """Runtime configuration for one Cell External Bridge process.

    All fields default to safe dev values. Production overrides usually come
    from the root ``station_config`` file and from the optional YAML file at
    ``CELL_BRIDGE_CONFIG_PATH``. Environment variables still win for one-off
    shell overrides. Nothing in this dataclass is robot-specific.
    The ``edge_node_id`` field name remains part of the external message
    envelope for compatibility with the conveyor/master contract.
    """

    belt_id: str = "belt-a"
    edge_node_id: str = "cell_bridge_01"
    robot_arm_id: str = "arm_01"
    arm_number: int = 1
    rabbitmq_url: str = "amqp://guest:guest@localhost:5672/"
    runtime_dir: str = "/ws/runtime"
    exchange_name: str = "catarm.events"
    simulate_mode: str = ""
    # Liveness signal the conveyor server uses for run-assignment readiness.
    # Set to 0 to disable (test-only).
    heartbeat_interval_s: float = 5.0

    # Best-effort eager warmup of the ROS bridge BEFORE the per-arm cmd
    # queue consumer is started. Without this, a ``cmd.pick`` queued by
    # the conveyor while the arm was offline gets delivered the instant
    # the queue is bound -- typically a few hundred ms after the
    # background ``ros2 launch`` calls fire, long before the trigger
    # services have registered. The warmup waits up to this many seconds
    # for the bridge's startup gate to pass; on timeout the controller
    # logs a warning and starts consuming anyway. Set to 0 to disable.
    bridge_warmup_timeout_s: float = 30.0
    robot_cell_orchestrator_phase_timeout_s: float = 120.0
    robot_cell_orchestrator_load_program_service: str = "robot_cell_orchestrator/load_online_program"
    robot_cell_orchestrator_validate_service: str = "robot_cell_orchestrator/validate_online_program"
    robot_cell_orchestrator_start_service: str = "robot_cell_orchestrator/start_online"
    robot_cell_orchestrator_place_service: str = "robot_cell_orchestrator/place_online"
    robot_cell_orchestrator_events_topic: str = "robot_cell_orchestrator/events"
    offline_debug: bool = False
    datalog_dir: str = "debug files/cell_external_bridge"

    @property
    def cmd_queue_name(self) -> str:
        return f"edge.{self.belt_id}.{self.arm_number}.cmd"

    @property
    def state_queue_name(self) -> str:
        return f"edge.{self.belt_id}.{self.arm_number}.notify"

    @property
    def cmd_routing_key_prefix(self) -> str:
        return f"belt.{self.belt_id}.arm.{self.arm_number}.cmd"

    @property
    def status_routing_key(self) -> str:
        return f"belt.{self.belt_id}.arm.{self.arm_number}.status"

    @property
    def telemetry_routing_key(self) -> str:
        return f"belt.{self.belt_id}.arm.{self.arm_number}.telemetry"

    @property
    def state_changed_routing_key(self) -> str:
        return f"belt.{self.belt_id}.conveyor.state.changed"

    @classmethod
    def from_env(cls) -> "Config":
        """Resolve defaults < YAML overlay < station_config < environment."""
        cfg = cls()
        station_config = _load_station_config()
        yaml_path = _first_value(
            ("CELL_BRIDGE_CONFIG_PATH", "EDGE_CONFIG_PATH"),
            os.environ,
            station_config,
        )
        _apply_yaml_overlay(cfg, yaml_path)
        _apply_settings(cfg, station_config)
        _apply_settings(cfg, os.environ)
        return cfg


def _load_station_config() -> dict[str, str]:
    config_path = _station_config_path()
    if not config_path.exists():
        if os.getenv(_STATION_CONFIG_ENV) or os.getenv(_LEGACY_ROBOT_CONFIG_ENV):
            logger.warning("%s=%s not found; using YAML/env defaults",
                           _STATION_CONFIG_ENV, config_path)
        return {}

    settings: dict[str, str] = {}
    try:
        with open(config_path, "r", encoding="utf-8") as fh:
            for line_number, raw_line in enumerate(fh, start=1):
                line = raw_line.strip()
                if not line or line.startswith("#"):
                    continue
                if line.startswith("export "):
                    line = line[len("export "):].strip()
                if "=" not in line:
                    logger.warning(
                        "Ignoring invalid station_config line %d: %s",
                        line_number, raw_line.rstrip(),
                    )
                    continue
                key, value = line.split("=", 1)
                key = key.strip()
                value = _strip_optional_quotes(value.strip())
                if key:
                    settings[key] = value
    except Exception:
        logger.exception("Failed to read station_config=%s; using YAML/env defaults",
                         config_path)
        return {}
    return settings


def _station_config_path() -> Path:
    explicit = os.getenv(_STATION_CONFIG_ENV) or os.getenv(_LEGACY_ROBOT_CONFIG_ENV)
    if explicit:
        return Path(explicit).expanduser()

    candidates: list[Path] = []
    workspace_root = os.getenv("DOBOT_PICKN_PLACE_ROOT")
    if workspace_root:
        candidates.append(Path(workspace_root) / "station_config")
        candidates.append(Path(workspace_root) / "robot_config")
    candidates.append(Path.cwd() / "station_config")
    candidates.append(Path.cwd() / "robot_config")
    candidates.append(Path(__file__).resolve().parents[3] / "station_config")
    candidates.append(Path(__file__).resolve().parents[3] / "robot_config")

    for candidate in candidates:
        if candidate.exists():
            return candidate
    return candidates[0]


def _strip_optional_quotes(value: str) -> str:
    if len(value) >= 2 and value[0] == value[-1] and value[0] in ("'", '"'):
        return value[1:-1]
    return value


def _apply_yaml_overlay(cfg: Config, yaml_path: str | None) -> None:
    if not yaml_path or yaml is None:
        return
    try:
        with open(yaml_path, "r", encoding="utf-8") as fh:
            data = yaml.safe_load(fh) or {}
    except FileNotFoundError:
        logger.warning("CELL_BRIDGE_CONFIG_PATH=%s not found; using env defaults", yaml_path)
        return
    except Exception:
        logger.exception(
            "Failed to read CELL_BRIDGE_CONFIG_PATH=%s; using env defaults",
            yaml_path,
        )
        return
    for key in _YAML_KEYS:
        if key in data and data[key] is not None:
            setattr(cfg, key, data[key])
    for legacy_key, new_key in _LEGACY_YAML_KEY_ALIASES.items():
        if new_key not in data and legacy_key in data and data[legacy_key] is not None:
            setattr(cfg, new_key, data[legacy_key])


def _apply_settings(cfg: Config, settings: Mapping[str, str]) -> None:
    for attr, env_vars in _ENV_STR_OVERRIDES.items():
        value = _first_value(env_vars, settings)
        if value:
            setattr(cfg, attr, value)
    _apply_typed(cfg, settings, "ARM_NUMBER", "arm_number", int)
    _apply_typed(cfg, settings, "HEARTBEAT_INTERVAL_S", "heartbeat_interval_s", float)
    _apply_typed(
        cfg,
        settings,
        ("CELL_BRIDGE_WARMUP_TIMEOUT_S", "EDGE_BRIDGE_WARMUP_TIMEOUT_S"),
        "bridge_warmup_timeout_s",
        float,
    )
    _apply_typed(
        cfg,
        settings,
        ("ROBOT_CELL_ORCHESTRATOR_PHASE_TIMEOUT_S", "PICK_CYCLE_PHASE_TIMEOUT_S"),
        "robot_cell_orchestrator_phase_timeout_s",
        float,
    )
    _apply_typed(
        cfg, settings, "CELL_BRIDGE_OFFLINE_DEBUG", "offline_debug", _to_bool,
    )


def _apply_typed(
    cfg: Config,
    settings: Mapping[str, str],
    env_vars: str | tuple[str, ...],
    attr: str,
    caster: Callable[[str], Any],
) -> None:
    env_var_tuple = (env_vars,) if isinstance(env_vars, str) else env_vars
    env_var = _first_set_name(env_var_tuple, settings)
    raw = _first_value(env_var_tuple, settings)
    if not raw:
        return
    try:
        setattr(cfg, attr, caster(raw))
    except ValueError:
        logger.warning(
            "Invalid %s=%r; keeping current value %r",
            env_var, raw, getattr(cfg, attr),
        )


def _first_value(env_vars: tuple[str, ...], *sources: Mapping[str, str]) -> str | None:
    for source in sources:
        for env_var in env_vars:
            value = source.get(env_var)
            if value:
                return str(value)
    return None


def _to_bool(raw: str) -> bool:
    normalized = str(raw).strip().lower()
    if normalized in ("1", "true", "yes", "y", "on"):
        return True
    if normalized in ("0", "false", "no", "n", "off"):
        return False
    raise ValueError(raw)


def _first_set_name(env_vars: tuple[str, ...], settings: Mapping[str, str]) -> str:
    for env_var in env_vars:
        if settings.get(env_var):
            return env_var
    return env_vars[0]


__all__ = ["Config"]
