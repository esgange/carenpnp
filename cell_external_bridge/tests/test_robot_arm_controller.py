"""Unit tests for the robotic arm controller."""

from __future__ import annotations

import sys
from pathlib import Path
from tempfile import mkdtemp

import pytest

from cell_external_bridge.robot_arm_controller import (
    Config,
    EVENT_CONVEYOR_STOP_REQUESTED,
    EVENT_ISSUE_RESOLVED,
    EVENT_LOAD_PROGRAM_FAILED,
    EVENT_LOAD_PROGRAM_OK,
    EVENT_MANUAL_INTERVENTION_REQUIRED,
    EVENT_PICK_FAILED,
    EVENT_PICK_OK,
    EVENT_PLACE_FAILED,
    EVENT_PLACE_OK,
    RobotArmController,
)


def _make_controller(
    *,
    simulate_mode: str = "success",
    robot_arm_id: str = "arm_01",
    runtime_dir: str | None = None,
    offline_debug: bool = False,
    datalog_dir: str | None = None,
) -> RobotArmController:
    if runtime_dir is None:
        runtime_dir = mkdtemp(prefix="cell-bridge-runtime-")
    if datalog_dir is None:
        datalog_dir = mkdtemp(prefix="cell-bridge-datalog-")
    cfg = Config(
        belt_id="belt-test",
        edge_node_id="cell_bridge_test",
        robot_arm_id=robot_arm_id,
        arm_number=1,
        rabbitmq_url="amqp://localhost/",
        runtime_dir=runtime_dir,
        simulate_mode=simulate_mode,
        offline_debug=offline_debug,
        datalog_dir=datalog_dir,
    )

    async def _record(routing_key: str, payload: dict) -> None:
        return None

    return RobotArmController(cfg, publish=_record)


def _program_payload(qqc_id: str = "water_bottle") -> dict:
    return {
        "qqc_id": qqc_id,
        "xml_version": 1,
        "xml_sha256": "metadata-only",
        "bin_teach_file": "bin_blue_bin_08052026.yaml",
        "item_teach_file": "item_paper_cutlery_bin_blue_bin_08052026.yaml",
        "tray_teach_file": "tray_blue_tray_06052026.yaml",
        "tray_x_mm": 25.0,
        "tray_y_mm": 30.0,
        "tray_rz_deg": 5.0,
    }


def _events(controller: RobotArmController) -> list[str]:
    """Status events only."""
    return [
        payload["event"]
        for routing_key, payload in controller.published_messages
        if routing_key.endswith(".status")
    ]


# ---------------------------------------------------------------------------
# load_program
# ---------------------------------------------------------------------------

@pytest.mark.asyncio
async def test_load_program_accepts_local_teach_selection_and_tray_pose(tmp_path: Path) -> None:
    ctrl = _make_controller(runtime_dir=str(tmp_path))

    result = await ctrl.load(_program_payload())

    assert result["status"] == "ok"
    assert result["files"] == [
        "bin_blue_bin_08052026.yaml",
        "item_paper_cutlery_bin_blue_bin_08052026.yaml",
        "tray_blue_tray_06052026.yaml",
    ]
    assert ctrl._current_qqc_id == "water_bottle"
    assert "water_bottle" in ctrl._active_qqc
    assert ctrl._active_qqc["water_bottle"]["tray_x_mm"] == 25.0
    assert ctrl._active_qqc["water_bottle"]["tray_y_mm"] == 30.0
    assert ctrl._active_qqc["water_bottle"]["tray_rz_deg"] == 5.0
    assert not any(tmp_path.iterdir())
    assert _events(ctrl) == [EVENT_LOAD_PROGRAM_OK]


@pytest.mark.asyncio
async def test_load_program_rejects_missing_qqc_id(tmp_path: Path) -> None:
    ctrl = _make_controller(runtime_dir=str(tmp_path))
    payload = _program_payload()
    payload.pop("qqc_id")

    result = await ctrl.load(payload)

    assert result["status"] == "error"
    assert result["reason"] == "missing_qqc_id"
    assert _events(ctrl) == [EVENT_LOAD_PROGRAM_FAILED]


@pytest.mark.asyncio
async def test_load_program_accepts_nested_teach_and_placement_maps(tmp_path: Path) -> None:
    ctrl = _make_controller(runtime_dir=str(tmp_path))
    payload = {
        "qqc_id": "nested-test",
        "teach_files": {
            "bin": "bin_blue_bin_08052026.yaml",
            "item": "item_paper_cutlery_bin_blue_bin_08052026.yaml",
            "tray": "tray_blue_tray_06052026.yaml",
        },
        "tray_placement": {"x": 11, "y": 22, "rz": -10},
    }

    result = await ctrl.load(payload)

    assert result["status"] == "ok"
    assert result["qqc_id"] == "nested-test"
    assert result["tray_x_mm"] == 11.0
    assert result["tray_y_mm"] == 22.0
    assert result["tray_rz_deg"] == -10.0


@pytest.mark.asyncio
async def test_load_program_rejects_missing_teach_file_names(tmp_path: Path) -> None:
    ctrl = _make_controller(runtime_dir=str(tmp_path))

    result = await ctrl.load({"qqc_id": "missing"})

    assert result["status"] == "error"
    assert result["reason"] == "missing_bin_teach_file_item_teach_file_tray_teach_file"
    assert _events(ctrl) == [EVENT_LOAD_PROGRAM_FAILED]


@pytest.mark.asyncio
async def test_load_program_rejects_missing_tray_position(tmp_path: Path) -> None:
    ctrl = _make_controller(runtime_dir=str(tmp_path))
    payload = _program_payload("missing-position")
    payload.pop("tray_rz_deg")
    result = await ctrl.load(payload)

    assert result["status"] == "error"
    assert result["reason"] == "missing_tray_rz_deg"


@pytest.mark.asyncio
async def test_offline_debug_load_program_does_not_write_runtime(tmp_path: Path) -> None:
    datalog_dir = tmp_path / "logs"
    ctrl = _make_controller(
        simulate_mode="",
        runtime_dir=str(tmp_path / "runtime"),
        offline_debug=True,
        datalog_dir=str(datalog_dir),
    )

    result = await ctrl.load({"qqc_id": "debug-program", "xml_version": 1})

    assert result["status"] == "ok"
    assert result["debug_mode"] is True
    assert ctrl._current_qqc_id == "debug-program"
    assert not (tmp_path / "runtime").exists()
    assert _events(ctrl) == [EVENT_LOAD_PROGRAM_OK]
    assert (datalog_dir / "arm_01_offline_debug.jsonl").exists()


# ---------------------------------------------------------------------------
# pick / place QQC preconditions
# ---------------------------------------------------------------------------

async def _load_default_program(
    ctrl: RobotArmController,
    qqc_id: str = "water_bottle",
) -> None:
    result = await ctrl.load(_program_payload(qqc_id))
    assert result["status"] == "ok"


@pytest.mark.asyncio
async def test_pick_rejects_when_no_qqc_loaded() -> None:
    ctrl = _make_controller(simulate_mode="success")
    result = await ctrl.pick({"qqc_id": "water_bottle", "tray_id": "T-1"})
    assert result["status"] == "error"
    assert result["reason"] == "no_qqc_loaded"
    assert ctrl.state == "idle"
    assert EVENT_PICK_FAILED in _events(ctrl)
    assert EVENT_MANUAL_INTERVENTION_REQUIRED not in _events(ctrl)


@pytest.mark.asyncio
async def test_pick_rejects_qqc_mismatch() -> None:
    ctrl = _make_controller(simulate_mode="success")
    await _load_default_program(ctrl, "water_bottle")
    result = await ctrl.pick({"qqc_id": "bread_roll", "tray_id": "T-1"})
    assert result["status"] == "error"
    assert result["reason"] == "qqc_mismatch"
    assert ctrl.state == "idle"
    events_after_load = _events(ctrl)[1:]
    assert EVENT_PICK_FAILED in events_after_load
    assert EVENT_MANUAL_INTERVENTION_REQUIRED not in events_after_load


# ---------------------------------------------------------------------------
# pick/place online acknowledgement flow
# ---------------------------------------------------------------------------

@pytest.mark.asyncio
async def test_pick_success_first_try() -> None:
    ctrl = _make_controller(simulate_mode="success")
    await _load_default_program(ctrl)
    result = await ctrl.pick({"qqc_id": "water_bottle", "tray_id": "T-1"})
    assert result["status"] == "ok"
    assert result["attempt"] == 1
    events = _events(ctrl)[1:]
    assert events == [EVENT_PICK_OK]
    assert EVENT_CONVEYOR_STOP_REQUESTED not in events


@pytest.mark.asyncio
async def test_pick_failure_does_not_escalate() -> None:
    ctrl = _make_controller(simulate_mode="fail_then_pass")
    await _load_default_program(ctrl)
    result = await ctrl.pick({"qqc_id": "water_bottle", "tray_id": "T-1"})
    assert result["status"] == "error"
    events = _events(ctrl)[1:]
    assert EVENT_PICK_FAILED in events
    assert EVENT_CONVEYOR_STOP_REQUESTED not in events
    assert EVENT_PICK_OK not in events
    assert EVENT_ISSUE_RESOLVED not in events


@pytest.mark.asyncio
async def test_pick_all_failures_stays_idle() -> None:
    ctrl = _make_controller(simulate_mode="always_fail")
    await _load_default_program(ctrl)
    result = await ctrl.pick({"qqc_id": "water_bottle", "tray_id": "T-1"})
    assert result["status"] == "error"
    assert ctrl.state == "idle"
    events = _events(ctrl)[1:]
    assert events.count(EVENT_PICK_FAILED) == 1
    assert EVENT_CONVEYOR_STOP_REQUESTED not in events
    assert EVENT_MANUAL_INTERVENTION_REQUIRED not in events
    assert EVENT_ISSUE_RESOLVED not in events


@pytest.mark.asyncio
async def test_place_failure_does_not_escalate() -> None:
    ctrl = _make_controller(simulate_mode="always_fail")
    await _load_default_program(ctrl)
    result = await ctrl.place({"qqc_id": "water_bottle", "tray_id": "T-1"})
    assert result["status"] == "error"
    assert ctrl.state == "idle"
    events = _events(ctrl)[1:]
    assert events.count(EVENT_PLACE_FAILED) == 1
    assert EVENT_MANUAL_INTERVENTION_REQUIRED not in events


@pytest.mark.asyncio
async def test_offline_debug_pick_place_fake_ok_without_loaded_program() -> None:
    ctrl = _make_controller(simulate_mode="", offline_debug=True)

    pick_result = await ctrl.pick({"qqc_id": "debug-program", "tray_id": "T-1"})
    place_result = await ctrl.place({"qqc_id": "debug-program", "tray_id": "T-1"})

    assert pick_result["status"] == "ok"
    assert place_result["status"] == "ok"
    events = _events(ctrl)
    assert events == [EVENT_PICK_OK, EVENT_PLACE_OK]
    assert EVENT_PICK_FAILED not in events
    assert EVENT_PLACE_FAILED not in events


# ---------------------------------------------------------------------------
# conveyor state broadcast handler
# ---------------------------------------------------------------------------

def test_conveyor_state_broadcast_updates_local_mirror() -> None:
    ctrl = _make_controller()
    ctrl._on_conveyor_state_changed("stopped", "arm_03")
    assert ctrl._conveyor_state == "stopped"
    ctrl._on_conveyor_state_changed("running", "all_resolved")
    assert ctrl._conveyor_state == "running"


# ---------------------------------------------------------------------------
# Bridge fallback outside simulate mode
# ---------------------------------------------------------------------------

@pytest.mark.asyncio
async def test_do_pick_returns_error_when_not_simulated(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.delitem(sys.modules, "cell_external_bridge.bridge", raising=False)
    real_import = __import__

    def _reject_bridge(name, globals=None, locals=None, fromlist=(), level=0):
        if name == "cell_external_bridge.bridge":
            raise ImportError("rclpy not on path")
        return real_import(name, globals, locals, fromlist, level)

    monkeypatch.setattr("builtins.__import__", _reject_bridge)
    ctrl = _make_controller(simulate_mode="")
    result = await ctrl._do_pick({})
    assert result["status"] == "error"
    assert result["reason"] == "bridge_unavailable"


@pytest.mark.asyncio
async def test_do_place_returns_error_when_not_simulated(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.delitem(sys.modules, "cell_external_bridge.bridge", raising=False)
    real_import = __import__

    def _reject_bridge(name, globals=None, locals=None, fromlist=(), level=0):
        if name == "cell_external_bridge.bridge":
            raise ImportError("rclpy not on path")
        return real_import(name, globals, locals, fromlist, level)

    monkeypatch.setattr("builtins.__import__", _reject_bridge)
    ctrl = _make_controller(simulate_mode="")
    result = await ctrl._do_place({})
    assert result["status"] == "error"
    assert result["reason"] == "bridge_unavailable"


# ---------------------------------------------------------------------------
# Heartbeat / availability telemetry
# ---------------------------------------------------------------------------

@pytest.mark.asyncio
async def test_heartbeat_publishes_expected_shape() -> None:
    ctrl = _make_controller()
    await ctrl._publish_telemetry()
    routing_key, payload = ctrl.published_messages[-1]
    assert routing_key == "belt.belt-test.arm.1.telemetry"
    assert payload["event"] == "arm.heartbeat"
    assert payload["arm_number"] == 1
    assert payload["robot_arm_id"] == "arm_01"
    assert payload["belt_id"] == "belt-test"
    assert payload["state"] == "idle"
    assert "timestamp" in payload


@pytest.mark.asyncio
async def test_heartbeat_loop_emits_multiple_then_stops() -> None:
    """The loop should keep publishing until ``stop()`` is signalled."""
    import asyncio

    ctrl = _make_controller()
    ctrl.config.heartbeat_interval_s = 0.05
    task = asyncio.create_task(ctrl._heartbeat_loop())
    await asyncio.sleep(0.16)
    await ctrl.stop()
    await task

    telemetry_msgs = [
        message for message in ctrl.published_messages
        if message[0].endswith(".telemetry")
    ]
    assert len(telemetry_msgs) >= 2


# ---------------------------------------------------------------------------
# State-change telemetry
# ---------------------------------------------------------------------------

def _telemetry_states(controller: RobotArmController) -> list[str]:
    return [
        payload["state"]
        for routing_key, payload in controller.published_messages
        if routing_key.endswith(".telemetry")
    ]


@pytest.mark.asyncio
async def test_state_transitions_published_during_pick_and_place() -> None:
    ctrl = _make_controller(simulate_mode="success")
    await _load_default_program(ctrl)

    pre_states = _telemetry_states(ctrl)
    assert pre_states == [], "load() does not change state"

    await ctrl.pick({"qqc_id": "water_bottle", "tray_id": "T-1"})
    await ctrl.place({"qqc_id": "water_bottle", "tray_id": "T-1"})

    post_states = _telemetry_states(ctrl)
    assert post_states == ["executing_pick", "idle", "executing_place", "idle"], (
        f"state-transition telemetry stream should show all four phases, "
        f"got {post_states!r}"
    )


@pytest.mark.asyncio
async def test_state_transitions_published_when_online_ack_fails() -> None:
    ctrl = _make_controller(simulate_mode="always_fail")
    await _load_default_program(ctrl)

    await ctrl.pick({"qqc_id": "water_bottle", "tray_id": "T-1"})

    states = _telemetry_states(ctrl)
    assert states[0] == "executing_pick"
    assert "retrying" not in states
    assert states[-1] == "idle"
