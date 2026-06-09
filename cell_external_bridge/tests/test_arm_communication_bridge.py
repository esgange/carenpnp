"""Coverage for the new ROS-bridge wiring in :mod:`cell_external_bridge.arm_communication`.

The production controller status path is exercised in
``test_robot_arm_controller.py``; this file pins the contract between
``ArmCommunication._do_pick`` / ``_do_place`` and
:mod:`cell_external_bridge.bridge`:

- ``CELL_BRIDGE_SIMULATE_MODE`` short-circuits to ``_simulate`` and never
  touches the bridge import (so CI without rclpy stays deterministic).
- A monkey-patched bridge module is awaited correctly and its dict
  flows back to the caller unchanged.
- A missing bridge module (e.g. plain pip install of the package on a
  host without ROS) surfaces as a non-retryable ``bridge_unavailable``
  failure rather than crashing the FSM.
"""

from __future__ import annotations

import sys
import types

import pytest

from cell_external_bridge.arm_communication import ArmCommunication, _run_bridge_op
from cell_external_bridge.config import Config


def _make_config(simulate_mode: str = "") -> Config:
    return Config(
        belt_id="belt-test",
        edge_node_id="cell_bridge_test",
        robot_arm_id="arm_01",
        arm_number=1,
        rabbitmq_url="amqp://localhost/",
        runtime_dir="/tmp/runtime",
        simulate_mode=simulate_mode,
    )


class _StubArm(ArmCommunication):
    """Concrete host class for the mixin under test."""

    def __init__(self, simulate_mode: str = "") -> None:
        self.config = _make_config(simulate_mode=simulate_mode)
        self._active_qqc: dict = {}
        self._current_qqc_id = None
        self._simulate_calls: list[tuple[str, dict]] = []

    async def _simulate(self, op_name: str, payload: dict) -> dict:
        self._simulate_calls.append((op_name, payload))
        return {"status": "ok", "via": "simulate"}


# ---------------------------------------------------------------------------
# Helpers for installing / restoring a fake `cell_external_bridge.bridge` module.
# ---------------------------------------------------------------------------

class _FakeClient:
    def __init__(self) -> None:
        self.run_pick_calls: list = []
        self.run_place_calls: list = []
        self.next_pick: dict = {"status": "ok"}
        self.next_place: dict = {"status": "ok"}

    async def run_pick(self, request):
        self.run_pick_calls.append(request)
        return self.next_pick

    async def run_place(self, request):
        self.run_place_calls.append(request)
        return self.next_place


def _install_fake_bridge(monkeypatch: pytest.MonkeyPatch) -> _FakeClient:
    """Inject a stub `cell_external_bridge.bridge` module into ``sys.modules``.

    The lazy import inside ``_run_bridge_op`` will resolve to this stub
    instead of the real rclpy-backed module.
    """
    fake = types.ModuleType("cell_external_bridge.bridge")
    fake.REASON_BRIDGE_UNAVAILABLE = "bridge_unavailable"

    class _CycleRequest:
        def __init__(self, payload):
            self.payload = payload

        @classmethod
        def from_payload(cls, payload):
            return cls(payload)

    client = _FakeClient()

    async def _get_ros_client(_cfg):
        return client

    fake.CycleRequest = _CycleRequest
    fake.get_ros_client = _get_ros_client
    monkeypatch.setitem(sys.modules, "cell_external_bridge.bridge", fake)
    return client


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

@pytest.mark.asyncio
async def test_simulate_mode_bypasses_bridge(monkeypatch: pytest.MonkeyPatch) -> None:
    arm = _StubArm(simulate_mode="success")

    # Prove no import attempt is made by stripping the real bridge from the
    # module cache and asserting nothing tries to load it.
    monkeypatch.delitem(sys.modules, "cell_external_bridge.bridge", raising=False)

    pick = await arm._do_pick({"qqc_id": "q", "tray_id": "t1", "task_id": "k1"})
    place = await arm._do_place({"qqc_id": "q", "tray_id": "t1", "task_id": "k1"})

    assert pick == {"status": "ok", "via": "simulate"}
    assert place == {"status": "ok", "via": "simulate"}
    assert [op for op, _ in arm._simulate_calls] == ["pick", "place"]
    assert "cell_external_bridge.bridge" not in sys.modules


@pytest.mark.asyncio
async def test_real_motion_routes_through_bridge(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    arm = _StubArm(simulate_mode="")
    client = _install_fake_bridge(monkeypatch)
    client.next_pick = {"status": "ok"}
    client.next_place = {
        "status": "error", "reason": "tcp_feedback_stale", "retryable": True,
    }

    pick = await arm._do_pick({"qqc_id": "q", "tray_id": "t1", "task_id": "k1"})
    place = await arm._do_place({"qqc_id": "q", "tray_id": "t1", "task_id": "k1"})

    assert pick == {"status": "ok"}
    assert place == {
        "status": "error", "reason": "tcp_feedback_stale", "retryable": True,
    }
    assert len(client.run_pick_calls) == 1
    assert len(client.run_place_calls) == 1
    assert arm._simulate_calls == []


@pytest.mark.asyncio
async def test_missing_bridge_returns_non_retryable_unavailable(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    cfg = _make_config(simulate_mode="")

    # Force the lazy `from cell_external_bridge.bridge import ...` to raise.
    monkeypatch.delitem(sys.modules, "cell_external_bridge.bridge", raising=False)

    real_import = __import__

    def _reject_bridge(name, globals=None, locals=None, fromlist=(), level=0):
        if name == "cell_external_bridge.bridge":
            raise ImportError("rclpy not on path")
        return real_import(name, globals, locals, fromlist, level)

    monkeypatch.setattr("builtins.__import__", _reject_bridge)

    result = await _run_bridge_op("pick", {"qqc_id": "q"}, cfg)

    assert result["status"] == "error"
    assert result["reason"] == "bridge_unavailable"
    assert result["retryable"] is False
    assert "bridge import failed" in result["message"]
