"""Robot-arm motion entrypoints owned by the robot-arm team.

``_do_pick`` and ``_do_place`` are the only methods the surrounding
controller calls when a ``cmd.pick`` / ``cmd.place`` arrives. They
delegate to :mod:`cell_external_bridge.bridge`, which owns one rclpy client node
and waits on the Robot Cell Orchestrator online services/phase events.

CATARM RabbitMQ publishing, conveyor coordination, and
HMAC stay in :mod:`cell_external_bridge.robot_arm_controller`. Nothing in this
module knows about RabbitMQ.

The bridge import is intentionally lazy: pure-Python CI runs (and the
``CELL_BRIDGE_SIMULATE_MODE`` test path) must not require ``rclpy`` or the ROS
overlay to be importable.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any, Optional

if TYPE_CHECKING:
    from cell_external_bridge.config import Config


# Stable string returned when the bridge module itself
# (rclpy, robot_cell_orchestrator, ...) cannot be imported. Mirrors the value exported
# by cell_external_bridge.bridge so callers can rely on a single taxonomy
# regardless of whether the import succeeded.
_REASON_BRIDGE_UNAVAILABLE = "bridge_unavailable"


class ArmCommunication:
    """Mixin holding the pick/place entrypoints used by the controller.

    Compose onto a class that provides:

    - ``self.config`` (:class:`~cell_external_bridge.config.Config`)
    - ``self._active_qqc`` and ``self._current_qqc_id`` (cycle state)
    - ``self._simulate(op_name, payload)`` (deterministic simulation hook)

    Real motion goes through the rclpy-backed bridge; ``_simulate``
    remains available for ``CELL_BRIDGE_SIMULATE_MODE`` deterministic outcomes
    used by tests and CI.
    """

    if TYPE_CHECKING:
        config: "Config"
        _active_qqc: dict[str, dict[str, Any]]
        _current_qqc_id: Optional[str]

        async def _simulate(self, op_name: str, payload: dict) -> dict: ...

    async def _do_pick(self, payload: dict) -> dict:
        """Execute the physical pick motion through the ROS bridge."""
        if getattr(self.config, "simulate_mode", ""):
            return await self._simulate("pick", payload)
        return await _run_bridge_op("pick", payload, self.config)

    async def _do_place(self, payload: dict) -> dict:
        """Execute the physical place motion through the ROS bridge."""
        if getattr(self.config, "simulate_mode", ""):
            return await self._simulate("place", payload)
        return await _run_bridge_op("place", payload, self.config)


async def _run_bridge_op(op: str, payload: dict, config: "Config") -> dict:
    """Translate one CATARM payload into a bridge call + retry-shaped dict.

    The ``cell_external_bridge.bridge`` import is performed lazily so that
    importing this module never requires ``rclpy``. When the ROS overlay
    is missing (e.g. unit tests on a stock Python install) the failure
    surfaces as a non-retryable ``bridge_unavailable`` so the controller
    publishes the matching ``pick_failed`` / ``place_failed`` event.
    """
    try:
        from cell_external_bridge.bridge import (  # noqa: WPS433 (lazy by design)
            CycleRequest,
            REASON_BRIDGE_UNAVAILABLE,
            get_ros_client,
        )
    except ImportError as exc:
        return {
            "status": "error",
            "reason": _REASON_BRIDGE_UNAVAILABLE,
            "retryable": False,
            "message": f"bridge import failed: {exc}",
        }

    try:
        client = await get_ros_client(config)
    except Exception as exc:
        return {
            "status": "error",
            "reason": REASON_BRIDGE_UNAVAILABLE,
            "retryable": True,
            "message": f"failed to start ROS bridge: {exc}",
        }

    request = CycleRequest.from_payload(payload)
    if op == "pick":
        return await client.run_pick(request)
    if op == "place":
        return await client.run_place(request)
    return {
        "status": "error",
        "reason": "unknown_op",
        "retryable": False,
        "message": f"unsupported bridge op {op!r}",
    }


__all__ = ["ArmCommunication"]
