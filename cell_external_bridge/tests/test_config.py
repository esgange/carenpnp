"""Configuration loading for Cell External Bridge."""

from __future__ import annotations

from pathlib import Path

from cell_external_bridge.config import Config


_ENV_KEYS = (
    "ARM_NUMBER",
    "BELT_ID",
    "CELL_BRIDGE_CONFIG_PATH",
    "CELL_BRIDGE_DATALOG_DIR",
    "CELL_BRIDGE_ID",
    "CELL_BRIDGE_OFFLINE_DEBUG",
    "CELL_BRIDGE_SIMULATE_MODE",
    "CELL_BRIDGE_WARMUP_TIMEOUT_S",
    "EDGE_BRIDGE_WARMUP_TIMEOUT_S",
    "EDGE_CONFIG_PATH",
    "EDGE_NODE_ID",
    "EDGE_SIMULATE_MODE",
    "HEARTBEAT_INTERVAL_S",
    "ROBOT_CELL_ORCHESTRATOR_EVENTS_TOPIC",
    "ROBOT_CELL_ORCHESTRATOR_LOAD_PROGRAM_SERVICE",
    "ROBOT_CELL_ORCHESTRATOR_PHASE_TIMEOUT_S",
    "ROBOT_CELL_ORCHESTRATOR_PLACE_SERVICE",
    "ROBOT_CELL_ORCHESTRATOR_START_SERVICE",
    "ROBOT_CELL_ORCHESTRATOR_VALIDATE_SERVICE",
    "RABBITMQ_EXCHANGE",
    "RABBITMQ_URL",
    "ROBOT_ARM_ID",
    "ROBOT_CONFIG_PATH",
    "RUNTIME_DIR",
    "STATION_CONFIG_PATH",
)


def _clear_config_env(monkeypatch) -> None:
    for env_key in _ENV_KEYS:
        monkeypatch.delenv(env_key, raising=False)


def test_station_config_overrides_yaml(tmp_path: Path, monkeypatch) -> None:
    _clear_config_env(monkeypatch)
    monkeypatch.chdir(tmp_path)
    config_path = tmp_path / "station.yaml"
    config_path.write_text(
        "\n".join(
            [
                "belt_id: yaml-belt",
                "edge_node_id: yaml_bridge",
                "robot_arm_id: yaml_arm",
                "arm_number: 7",
                "runtime_dir: /yaml/runtime",
            ]
        ),
        encoding="utf-8",
    )

    station_config = tmp_path / "station_config"
    station_config.write_text(
        "\n".join(
            [
                f"CELL_BRIDGE_CONFIG_PATH={config_path}",
                "CELL_BRIDGE_ID=cell_bridge_file",
                "ROBOT_ARM_ID=arm_file",
                "ARM_NUMBER=3",
                "CELL_BRIDGE_SIMULATE_MODE=success",
                "CELL_BRIDGE_WARMUP_TIMEOUT_S=12.5",
            ]
        ),
        encoding="utf-8",
    )

    cfg = Config.from_env()

    assert cfg.belt_id == "yaml-belt"
    assert cfg.edge_node_id == "cell_bridge_file"
    assert cfg.robot_arm_id == "arm_file"
    assert cfg.arm_number == 3
    assert cfg.runtime_dir == "/yaml/runtime"
    assert cfg.simulate_mode == "success"
    assert cfg.bridge_warmup_timeout_s == 12.5


def test_env_overrides_station_config(tmp_path: Path, monkeypatch) -> None:
    _clear_config_env(monkeypatch)
    monkeypatch.chdir(tmp_path)
    (tmp_path / "station_config").write_text(
        "\n".join(
            [
                "CELL_BRIDGE_ID=cell_bridge_file",
                "ROBOT_ARM_ID=arm_file",
                "ARM_NUMBER=3",
            ]
        ),
        encoding="utf-8",
    )
    monkeypatch.setenv("CELL_BRIDGE_ID", "cell_bridge_env")
    monkeypatch.setenv("ROBOT_ARM_ID", "arm_env")
    monkeypatch.setenv("ARM_NUMBER", "4")

    cfg = Config.from_env()

    assert cfg.edge_node_id == "cell_bridge_env"
    assert cfg.robot_arm_id == "arm_env"
    assert cfg.arm_number == 4


def test_legacy_edge_env_names_remain_fallbacks(tmp_path: Path, monkeypatch) -> None:
    _clear_config_env(monkeypatch)
    monkeypatch.chdir(tmp_path)
    monkeypatch.setenv("EDGE_NODE_ID", "legacy_bridge")
    monkeypatch.setenv("EDGE_SIMULATE_MODE", "always_fail")
    monkeypatch.setenv("EDGE_BRIDGE_WARMUP_TIMEOUT_S", "4.0")

    cfg = Config.from_env()

    assert cfg.edge_node_id == "legacy_bridge"
    assert cfg.simulate_mode == "always_fail"
    assert cfg.bridge_warmup_timeout_s == 4.0


def test_legacy_robot_config_path_still_works(tmp_path: Path, monkeypatch) -> None:
    _clear_config_env(monkeypatch)
    legacy_path = tmp_path / "robot_config"
    legacy_path.write_text("CELL_BRIDGE_ID=legacy_file\n", encoding="utf-8")
    monkeypatch.setenv("ROBOT_CONFIG_PATH", str(legacy_path))

    cfg = Config.from_env()

    assert cfg.edge_node_id == "legacy_file"


def test_offline_debug_config_from_station_file(tmp_path: Path, monkeypatch) -> None:
    _clear_config_env(monkeypatch)
    monkeypatch.chdir(tmp_path)
    (tmp_path / "station_config").write_text(
        "\n".join(
            [
                "CELL_BRIDGE_OFFLINE_DEBUG=1",
                "CELL_BRIDGE_DATALOG_DIR=/tmp/cell-bridge-debug",
            ]
        ),
        encoding="utf-8",
    )

    cfg = Config.from_env()

    assert cfg.offline_debug is True
    assert cfg.datalog_dir == "/tmp/cell-bridge-debug"
