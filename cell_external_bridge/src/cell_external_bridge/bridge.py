"""ROS-side client for the Robot Cell Orchestrator online API.

The cell-external-bridge controller talks to the factory side over RabbitMQ and
to ROS through this module only. It no longer embeds ``RobotCellOrchestratorNode``;
instead it calls the running Robot Cell Orchestrator services and waits for phase
events published by that node.
"""

from __future__ import annotations

import asyncio
import json
import logging
import threading
import time
from dataclasses import dataclass
from typing import Any, Optional

logger = logging.getLogger("cell_external_bridge.bridge")


REASON_SERVICE_UNAVAILABLE = "service_unavailable"
REASON_ROBOT_CELL_ORCHESTRATOR_REJECTED = "robot_cell_orchestrator_rejected"
REASON_ROBOT_CELL_ORCHESTRATOR_TIMEOUT = "robot_cell_orchestrator_timeout"
REASON_CANCELLED = "cancelled"
REASON_BRIDGE_BUSY = "bridge_busy"
REASON_BRIDGE_UNAVAILABLE = "bridge_unavailable"
REASON_UNKNOWN = "ros_robot_cell_orchestrator_failed"


@dataclass(frozen=True)
class ProgramLoadRequest:
    """Program selection carried by external cmd.load_program."""

    qqc_id: str
    bin_teach_file: str
    item_teach_file: str
    tray_teach_file: str
    tray_x_mm: float
    tray_y_mm: float
    tray_rz_deg: float


@dataclass(frozen=True)
class CycleRequest:
    """Per-call metadata carried by the external cmd.pick / cmd.place."""

    qqc_id: Optional[str] = None
    tray_id: Optional[str] = None
    task_id: Optional[str] = None

    @classmethod
    def from_payload(cls, payload: dict[str, Any]) -> "CycleRequest":
        return cls(
            qqc_id=payload.get("qqc_id"),
            tray_id=payload.get("tray_id"),
            task_id=payload.get("task_id"),
        )


@dataclass(frozen=True)
class _PhaseEvent:
    event: str
    phase_id: int
    cycle_index: Optional[int]
    message: str


@dataclass(frozen=True)
class _BridgeResult:
    status: str
    reason: Optional[str] = None
    retryable: Optional[bool] = None
    message: Optional[str] = None

    @property
    def is_error(self) -> bool:
        return self.status != "ok"

    @classmethod
    def ok(cls, message: Optional[str] = None, **fields: Any) -> "_BridgeResult":
        del fields
        return cls(status="ok", message=message)

    @classmethod
    def error(cls, reason: str, message: str, *, retryable: bool) -> "_BridgeResult":
        return cls(status="error", reason=reason, retryable=retryable, message=message)

    def to_dict(self, **fields: Any) -> dict[str, Any]:
        if self.status == "ok":
            payload: dict[str, Any] = {"status": "ok"}
        else:
            payload = {
                "status": "error",
                "reason": self.reason or REASON_UNKNOWN,
                "retryable": bool(self.retryable),
            }
        if self.message:
            payload["message"] = self.message
        payload.update(fields)
        return payload


class RosCycleClient:
    """Long-lived ROS client for Robot Cell Orchestrator online control."""

    def __init__(
        self,
        *,
        load_program_service: str = "robot_cell_orchestrator/load_online_program",
        validate_service: str = "robot_cell_orchestrator/validate_online_program",
        start_service: str = "robot_cell_orchestrator/start_online",
        place_service: str = "robot_cell_orchestrator/place_online",
        events_topic: str = "robot_cell_orchestrator/events",
        phase_timeout_sec: float = 120.0,
        service_timeout_sec: float = 5.5,
    ) -> None:
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from dobot_msgs_v4.srv import LoadOnlineProgram
        from std_msgs.msg import String
        from std_srvs.srv import Trigger

        self._rclpy = rclpy
        self._Trigger = Trigger
        self._LoadOnlineProgram = LoadOnlineProgram
        self._String = String
        self._load_program_service = load_program_service
        self._validate_service = validate_service
        self._start_service = start_service
        self._place_service = place_service
        self._events_topic = events_topic
        self._phase_timeout_sec = max(0.1, float(phase_timeout_sec))
        self._service_timeout_sec = max(0.1, float(service_timeout_sec))
        self._load_program_timeout_sec = max(20.0, self._service_timeout_sec)

        self._owns_rclpy = not rclpy.ok()
        if self._owns_rclpy:
            rclpy.init()

        self._node = rclpy.create_node("cell_external_bridge_robot_cell_orchestrator_client")
        self._load_program_client = self._node.create_client(
            LoadOnlineProgram,
            self._load_program_service,
        )
        self._validate_client = self._node.create_client(Trigger, self._validate_service)
        self._start_client = self._node.create_client(Trigger, self._start_service)
        self._place_client = self._node.create_client(Trigger, self._place_service)
        self._subscription = self._node.create_subscription(
            String,
            self._events_topic,
            self._on_phase_event,
            10,
        )
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)

        self._closed = False
        self._spin_stop = threading.Event()
        self._events_condition = threading.Condition(threading.RLock())
        self._events: list[_PhaseEvent] = []
        self._last_robot_cell_orchestrator_index: Optional[int] = None
        self._last_pick_phase_id = 0
        self._last_place_phase_id = 0

        self._spin_thread = threading.Thread(
            target=self._spin_loop,
            name="cell-external-bridge-robot-cell-orchestrator-spin",
            daemon=True,
        )
        self._spin_thread.start()

    def _spin_loop(self) -> None:
        try:
            while not self._spin_stop.is_set() and self._rclpy.ok():
                self._executor.spin_once(timeout_sec=0.1)
        except Exception:
            logger.exception("Robot Cell Orchestrator ROS client spin thread crashed")

    def _on_phase_event(self, msg: Any) -> None:
        try:
            payload = json.loads(str(msg.data))
            event = str(payload.get("event", "")).strip()
            phase_id = int(payload.get("phase_id", 0) or 0)
            cycle_value = payload.get("cycle_index")
            cycle_index = int(cycle_value) if cycle_value is not None else None
            message = str(payload.get("message", ""))
        except Exception:
            logger.warning("Ignoring malformed Robot Cell Orchestrator event: %r", getattr(msg, "data", ""))
            return
        if not event or phase_id <= 0:
            return
        phase_event = _PhaseEvent(event, phase_id, cycle_index, message)
        with self._events_condition:
            self._events.append(phase_event)
            self._events = self._events[-200:]
            self._events_condition.notify_all()
        logger.debug(
            "Robot Cell Orchestrator phase event event=%s phase_id=%s cycle_index=%s message=%s",
            event,
            phase_id,
            cycle_index,
            message,
        )

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._spin_stop.set()
        try:
            self._spin_thread.join(timeout=2.0)
        except Exception:
            pass
        try:
            self._executor.remove_node(self._node)
        except Exception:
            pass
        try:
            self._node.destroy_node()
        except Exception:
            pass
        if self._owns_rclpy and self._rclpy.ok():
            try:
                self._rclpy.shutdown()
            except Exception:
                pass

    async def wait_ready(self) -> dict[str, Any]:
        result = await asyncio.to_thread(self._check_services_ready)
        return result.to_dict()

    async def load_online_program(self, request: ProgramLoadRequest) -> dict[str, Any]:
        result = await asyncio.to_thread(self._call_load_program, request)
        return result

    async def validate_online_program(self) -> dict[str, Any]:
        result = await asyncio.to_thread(
            self._call_trigger,
            self._validate_client,
            self._validate_service,
            self._service_timeout_sec,
        )
        return result.to_dict()

    async def run_pick(self, request: CycleRequest) -> dict[str, Any]:
        del request
        return await asyncio.to_thread(self._run_pick_blocking)

    async def run_place(self, request: CycleRequest) -> dict[str, Any]:
        del request
        return await asyncio.to_thread(self._run_place_blocking)

    def cancel_in_flight(self, reason: str = "external_cancel") -> bool:
        del reason
        return False

    def _check_services_ready(self) -> _BridgeResult:
        missing = []
        if not self._load_program_client.wait_for_service(timeout_sec=self._service_timeout_sec):
            missing.append(self._load_program_service)
        if not self._validate_client.wait_for_service(timeout_sec=self._service_timeout_sec):
            missing.append(self._validate_service)
        if not self._start_client.wait_for_service(timeout_sec=self._service_timeout_sec):
            missing.append(self._start_service)
        if not self._place_client.wait_for_service(timeout_sec=self._service_timeout_sec):
            missing.append(self._place_service)
        if missing:
            return _BridgeResult.error(
                REASON_SERVICE_UNAVAILABLE,
                "Required Robot Cell Orchestrator services unavailable: " + ", ".join(missing),
                retryable=True,
            )
        return _BridgeResult.ok("Robot Cell Orchestrator services ready")

    def _call_load_program(self, request: ProgramLoadRequest) -> dict[str, Any]:
        if self._closed:
            return _BridgeResult.error(
                REASON_BRIDGE_UNAVAILABLE,
                "Bridge is closed",
                retryable=False,
            ).to_dict()
        if not self._load_program_client.wait_for_service(timeout_sec=self._service_timeout_sec):
            return _BridgeResult.error(
                REASON_SERVICE_UNAVAILABLE,
                f"Service unavailable: {self._load_program_service}",
                retryable=True,
            ).to_dict()

        ros_request = self._LoadOnlineProgram.Request()
        ros_request.qqc_id = request.qqc_id
        ros_request.bin_teach_file = request.bin_teach_file
        ros_request.item_teach_file = request.item_teach_file
        ros_request.tray_teach_file = request.tray_teach_file
        ros_request.tray_x_mm = float(request.tray_x_mm)
        ros_request.tray_y_mm = float(request.tray_y_mm)
        ros_request.tray_rz_deg = float(request.tray_rz_deg)
        try:
            future = self._load_program_client.call_async(ros_request)
        except Exception as exc:
            return _BridgeResult.error(
                REASON_BRIDGE_UNAVAILABLE,
                f"Failed to call {self._load_program_service}: {exc}",
                retryable=True,
            ).to_dict()

        deadline = time.monotonic() + self._load_program_timeout_sec
        while self._rclpy.ok() and time.monotonic() < deadline:
            if future.done():
                try:
                    response = future.result()
                except Exception as exc:
                    return _BridgeResult.error(
                        REASON_ROBOT_CELL_ORCHESTRATOR_REJECTED,
                        f"{self._load_program_service} response failed: {exc}",
                        retryable=True,
                    ).to_dict()
                if bool(response.success):
                    return _BridgeResult.ok(str(response.message)).to_dict(
                        files=list(response.runtime_files),
                    )
                return _BridgeResult.error(
                    REASON_ROBOT_CELL_ORCHESTRATOR_REJECTED,
                    str(response.message),
                    retryable=False,
                ).to_dict()
            time.sleep(0.02)
        return _BridgeResult.error(
            REASON_ROBOT_CELL_ORCHESTRATOR_TIMEOUT,
            f"Timed out waiting for {self._load_program_service} response",
            retryable=False,
        ).to_dict()

    def _call_trigger(self, client: Any, service_name: str, timeout_sec: float) -> _BridgeResult:
        if self._closed:
            return _BridgeResult.error(REASON_BRIDGE_UNAVAILABLE, "Bridge is closed", retryable=False)
        if not client.wait_for_service(timeout_sec=timeout_sec):
            return _BridgeResult.error(
                REASON_SERVICE_UNAVAILABLE,
                f"Service unavailable: {service_name}",
                retryable=True,
            )
        try:
            future = client.call_async(self._Trigger.Request())
        except Exception as exc:
            return _BridgeResult.error(
                REASON_BRIDGE_UNAVAILABLE,
                f"Failed to call {service_name}: {exc}",
                retryable=True,
            )

        deadline = time.monotonic() + timeout_sec
        while self._rclpy.ok() and time.monotonic() < deadline:
            if future.done():
                try:
                    response = future.result()
                except Exception as exc:
                    return _BridgeResult.error(
                        REASON_ROBOT_CELL_ORCHESTRATOR_REJECTED,
                        f"{service_name} response failed: {exc}",
                        retryable=True,
                    )
                if bool(response.success):
                    return _BridgeResult.ok(str(response.message))
                return _BridgeResult.error(
                    REASON_ROBOT_CELL_ORCHESTRATOR_REJECTED,
                    str(response.message),
                    retryable=False,
                )
            time.sleep(0.02)
        return _BridgeResult.error(
            REASON_ROBOT_CELL_ORCHESTRATOR_TIMEOUT,
            f"Timed out waiting for {service_name} response",
            retryable=False,
        )

    def _run_pick_blocking(self) -> dict[str, Any]:
        snapshot = self._last_phase_id()
        start_result = self._call_trigger(
            self._start_client,
            self._start_service,
            self._service_timeout_sec,
        )
        if start_result.is_error:
            return start_result.to_dict()
        event = self._wait_for_phase("moving_to_tray", after_phase_id=snapshot)
        if isinstance(event, _BridgeResult):
            return event.to_dict()
        self._last_robot_cell_orchestrator_index = event.cycle_index
        self._last_pick_phase_id = event.phase_id
        return _BridgeResult.ok(event.message).to_dict(
            cycle_index=event.cycle_index,
            phase_id=event.phase_id,
        )

    def _run_place_blocking(self) -> dict[str, Any]:
        min_cycle_index = self._last_robot_cell_orchestrator_index
        if min_cycle_index is None:
            return _BridgeResult.error(
                REASON_ROBOT_CELL_ORCHESTRATOR_REJECTED,
                "cmd.place received before cmd.pick",
                retryable=False,
            ).to_dict()
        snapshot = self._last_phase_id()
        place_result = self._call_trigger(
            self._place_client,
            self._place_service,
            self._service_timeout_sec,
        )
        if place_result.is_error:
            return place_result.to_dict()
        event = self._wait_for_phase(
            "moving_to_bin",
            after_phase_id=max(snapshot, self._last_place_phase_id),
            min_cycle_index=min_cycle_index,
        )
        if isinstance(event, _BridgeResult):
            return event.to_dict()
        self._last_place_phase_id = event.phase_id
        return _BridgeResult.ok(event.message).to_dict(
            cycle_index=event.cycle_index,
            phase_id=event.phase_id,
        )

    def _last_phase_id(self) -> int:
        with self._events_condition:
            if not self._events:
                return 0
            return self._events[-1].phase_id

    def _find_cached_phase(
        self,
        event_name: str,
        *,
        after_phase_id: int,
        min_cycle_index: Optional[int] = None,
    ) -> Optional[_PhaseEvent]:
        with self._events_condition:
            candidates = [
                event
                for event in self._events
                if event.event == event_name
                and event.phase_id > after_phase_id
                and (
                    min_cycle_index is None
                    or event.cycle_index is None
                    or event.cycle_index >= min_cycle_index
                )
            ]
        if not candidates:
            return None
        return candidates[-1]

    def _wait_for_phase(
        self,
        event_name: str,
        *,
        after_phase_id: int,
        min_cycle_index: Optional[int] = None,
    ) -> _PhaseEvent | _BridgeResult:
        deadline = time.monotonic() + self._phase_timeout_sec
        with self._events_condition:
            while self._rclpy.ok() and not self._closed:
                cached = self._find_cached_phase(
                    event_name,
                    after_phase_id=after_phase_id,
                    min_cycle_index=min_cycle_index,
                )
                if cached is not None:
                    return cached
                timeout_event = self._find_cached_phase(
                    "timeout",
                    after_phase_id=after_phase_id,
                    min_cycle_index=min_cycle_index,
                )
                if timeout_event is not None:
                    return _BridgeResult.error(
                        REASON_ROBOT_CELL_ORCHESTRATOR_TIMEOUT,
                        timeout_event.message or f"Robot Cell Orchestrator reported timeout before {event_name}",
                        retryable=False,
                    )
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return _BridgeResult.error(
                        REASON_ROBOT_CELL_ORCHESTRATOR_TIMEOUT,
                        f"Timed out waiting for Robot Cell Orchestrator phase {event_name}",
                        retryable=False,
                    )
                self._events_condition.wait(timeout=min(0.25, remaining))
        return _BridgeResult.error(
            REASON_BRIDGE_UNAVAILABLE,
            f"ROS shutdown while waiting for Robot Cell Orchestrator phase {event_name}",
            retryable=True,
        )


_client_lock = asyncio.Lock()
_client: Optional[RosCycleClient] = None


async def get_ros_client(cfg: Any) -> RosCycleClient:
    global _client
    if _client is not None:
        return _client
    async with _client_lock:
        if _client is None:
            try:
                _client = await asyncio.to_thread(
                    RosCycleClient,
                    load_program_service=str(getattr(cfg, "robot_cell_orchestrator_load_program_service", "robot_cell_orchestrator/load_online_program")),
                    validate_service=str(getattr(cfg, "robot_cell_orchestrator_validate_service", "robot_cell_orchestrator/validate_online_program")),
                    start_service=str(getattr(cfg, "robot_cell_orchestrator_start_service", "robot_cell_orchestrator/start_online")),
                    place_service=str(getattr(cfg, "robot_cell_orchestrator_place_service", "robot_cell_orchestrator/place_online")),
                    events_topic=str(getattr(cfg, "robot_cell_orchestrator_events_topic", "robot_cell_orchestrator/events")),
                    phase_timeout_sec=float(getattr(cfg, "robot_cell_orchestrator_phase_timeout_s", 120.0)),
                )
            except Exception:
                logger.exception("Failed to construct Robot Cell Orchestrator ROS client")
                raise
    return _client


def cancel_ros_client_in_flight(reason: str = "external_cancel") -> bool:
    client = _client
    if client is None:
        return False
    try:
        return client.cancel_in_flight(reason)
    except Exception:
        logger.exception("cancel_ros_client_in_flight failed")
        return False


async def close_ros_client() -> None:
    global _client
    if _client is None:
        return
    client = _client
    _client = None
    await asyncio.to_thread(client.close)


__all__ = [
    "CycleRequest",
    "ProgramLoadRequest",
    "RosCycleClient",
    "REASON_BRIDGE_BUSY",
    "REASON_BRIDGE_UNAVAILABLE",
    "REASON_CANCELLED",
    "REASON_ROBOT_CELL_ORCHESTRATOR_REJECTED",
    "REASON_ROBOT_CELL_ORCHESTRATOR_TIMEOUT",
    "REASON_SERVICE_UNAVAILABLE",
    "REASON_UNKNOWN",
    "cancel_ros_client_in_flight",
    "close_ros_client",
    "get_ros_client",
]
