"""Robot Arm Controller -- single Cell External Bridge runtime.

Owns one robotic arm and talks to the Conveyor Server exclusively over
RabbitMQ. The three operations ``load_program`` / ``pick`` / ``place`` are public
coroutines on :class:`RobotArmController` and are also driven by command
messages. Placeholder motion methods ``_do_pick`` / ``_do_place`` live in
:mod:`cell_external_bridge.arm_communication` (the only file the robot-arm team
edits). See ``README.md`` and ``docs/robot_arm_team_handoff.md``.
"""

from __future__ import annotations

import asyncio
import json
import logging
import os
from collections import defaultdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Awaitable, Callable, Optional

import aio_pika
from aio_pika import ExchangeType
from aio_pika.abc import (
    AbstractIncomingMessage,
    AbstractRobustChannel,
    AbstractRobustConnection,
    AbstractRobustExchange,
    AbstractRobustQueue,
)

from cell_external_bridge.arm_communication import ArmCommunication
from cell_external_bridge.config import Config
from cell_external_bridge.messaging import (
    PublishCallback,
    _hmac_sign,
    _hmac_verify,
    envelope,
    publish as _publish_message,
    redact_url,
    verify_message,
)
from cell_external_bridge.runtime_program import (
    RuntimeProgramSelection,
    parse_runtime_program_selection,
)

logger = logging.getLogger("cell_external_bridge.robot_arm_controller")


# ---------------------------------------------------------------------------
# Public event names -- single source of truth so tests and the conveyor
# server's ArmCoordinationService import constants instead of literals.
# ---------------------------------------------------------------------------
EVENT_LOAD_PROGRAM_OK = "load_program_ok"
EVENT_LOAD_PROGRAM_FAILED = "load_program_failed"
EVENT_PICK_OK = "pick_ok"
EVENT_PICK_FAILED = "pick_failed"
EVENT_PLACE_OK = "place_ok"
EVENT_PLACE_FAILED = "place_failed"
EVENT_CONVEYOR_STOP_REQUESTED = "conveyor_stop_requested"
EVENT_ISSUE_RESOLVED = "issue_resolved"
EVENT_MANUAL_INTERVENTION_REQUIRED = "manual_intervention_required"


# ---------------------------------------------------------------------------
# RobotArmController
# ---------------------------------------------------------------------------

class RobotArmController(ArmCommunication):
    """Single robotic arm controller.

    Owns one RabbitMQ connection, a per-arm command queue, a
    state-broadcast queue, and the in-memory QQC cache.
    """

    def __init__(
        self,
        config: Config,
        *,
        publish: Optional[PublishCallback] = None,
    ) -> None:
        self.config = config
        self.state: str = "idle"
        self.attempt: int = 0
        self._active_qqc: dict[str, dict[str, Any]] = {}
        self._current_qqc_id: Optional[str] = None
        self._conveyor_state: str = "unknown"

        self._connection: Optional[AbstractRobustConnection] = None
        self._channel: Optional[AbstractRobustChannel] = None
        self._exchange: Optional[AbstractRobustExchange] = None
        self._cmd_queue: Optional[AbstractRobustQueue] = None
        self._state_queue: Optional[AbstractRobustQueue] = None

        self._inject_publish = publish
        self._stopping = asyncio.Event()
        self._published_messages: list[tuple[str, dict]] = []
        self._heartbeat_task: Optional[asyncio.Task] = None
        self._sim_counts: dict[str, int] = defaultdict(int)

    @property
    def published_messages(self) -> list[tuple[str, dict]]:
        """Test hook: every (routing_key, payload) ever published."""
        return self._published_messages

    # ------------------------------------------------------------------
    # Connection lifecycle
    # ------------------------------------------------------------------

    async def connect(self) -> None:
        """Open a robust connection and declare the per-arm topology."""
        cfg = self.config
        logger.info(
            "[belt=%s arm=%s] Connecting to RabbitMQ at %s",
            cfg.belt_id, cfg.robot_arm_id, redact_url(cfg.rabbitmq_url),
        )

        self._connection = await aio_pika.connect_robust(cfg.rabbitmq_url)
        self._channel = await self._connection.channel()
        await self._channel.set_qos(prefetch_count=10)

        self._exchange = await self._channel.declare_exchange(
            cfg.exchange_name, ExchangeType.TOPIC, durable=True,
        )

        self._cmd_queue = await self._channel.declare_queue(
            cfg.cmd_queue_name, durable=True,
        )
        await self._cmd_queue.bind(
            self._exchange, routing_key=f"{cfg.cmd_routing_key_prefix}.*",
        )

        self._state_queue = await self._channel.declare_queue(
            cfg.state_queue_name, durable=True,
        )
        await self._state_queue.bind(
            self._exchange, routing_key=cfg.state_changed_routing_key,
        )

        logger.info(
            "[belt=%s arm=%s] Bound %s <- %s.* and %s <- %s",
            cfg.belt_id, cfg.robot_arm_id,
            cfg.cmd_queue_name, cfg.cmd_routing_key_prefix,
            cfg.state_queue_name, cfg.state_changed_routing_key,
        )

    async def disconnect(self) -> None:
        if self._connection and not self._connection.is_closed:
            await self._connection.close()
        self._connection = None
        self._channel = None
        self._exchange = None
        self._cmd_queue = None
        self._state_queue = None

    # ------------------------------------------------------------------
    # Public API: load / pick / place
    # ------------------------------------------------------------------

    async def load_program(self, payload: dict) -> dict:
        """Ask Robot Cell Orchestrator to load local teach files into runtime.

        Always returns a status dict; never raises. Publishes
        ``load_program_ok`` on success, ``load_program_failed`` with a
        short ``reason`` otherwise.
        """
        qqc_id = str(payload.get("qqc_id") or "")
        xml_version = payload.get("xml_version")
        xml_sha256 = payload.get("xml_sha256")

        if self.config.offline_debug:
            return await self._debug_load_program(
                payload,
                qqc_id=qqc_id,
                xml_version=xml_version,
                xml_sha256=xml_sha256,
            )

        selection, parse_error = parse_runtime_program_selection(payload)
        if selection is None:
            return await self._fail_load_program(
                qqc_id or None,
                xml_version,
                parse_error or "invalid_load_program",
            )

        load_result = await self._load_robot_cell_orchestrator_online_program(selection)
        if load_result.get("status") != "ok":
            return await self._fail_load_program(
                selection.qqc_id,
                xml_version,
                load_result.get("reason", "robot_cell_orchestrator_load_program_failed"),
            )

        runtime_files = list(load_result.get("files", []) or selection.requested_files)
        self._active_qqc[selection.qqc_id] = {
            "xml_version": xml_version,
            "xml_sha256": xml_sha256,
            "runtime_files": runtime_files,
            "bin_teach_file": selection.bin_teach_file,
            "item_teach_file": selection.item_teach_file,
            "tray_teach_file": selection.tray_teach_file,
            "tray_x_mm": selection.tray_x_mm,
            "tray_y_mm": selection.tray_y_mm,
            "tray_rz_deg": selection.tray_rz_deg,
        }
        self._current_qqc_id = selection.qqc_id

        logger.info(
            "[arm=%s] Loaded runtime program %s v%s (%d files, tray x=%.1f y=%.1f rz=%.1f)",
            self.config.robot_arm_id,
            selection.qqc_id,
            xml_version,
            len(runtime_files),
            selection.tray_x_mm,
            selection.tray_y_mm,
            selection.tray_rz_deg,
        )
        await self._publish_status(
            EVENT_LOAD_PROGRAM_OK,
            qqc_id=selection.qqc_id,
            xml_version=xml_version,
            files=runtime_files,
            bin_teach_file=selection.bin_teach_file,
            item_teach_file=selection.item_teach_file,
            tray_teach_file=selection.tray_teach_file,
            tray_x_mm=selection.tray_x_mm,
            tray_y_mm=selection.tray_y_mm,
            tray_rz_deg=selection.tray_rz_deg,
        )
        return {
            "status": "ok",
            "qqc_id": selection.qqc_id,
            "xml_version": xml_version,
            "files": runtime_files,
            "bin_teach_file": selection.bin_teach_file,
            "item_teach_file": selection.item_teach_file,
            "tray_teach_file": selection.tray_teach_file,
            "tray_x_mm": selection.tray_x_mm,
            "tray_y_mm": selection.tray_y_mm,
            "tray_rz_deg": selection.tray_rz_deg,
        }

    async def load(self, payload: dict) -> dict:
        """Backward-compatible internal alias for old callers."""
        return await self.load_program(payload)

    async def _validate_robot_cell_orchestrator_online_program(self) -> dict:
        if self.config.offline_debug:
            return {"status": "ok", "message": "offline debug skips Robot Cell Orchestrator validation"}
        if self.config.simulate_mode:
            return {"status": "ok", "message": "simulate mode skips Robot Cell Orchestrator validation"}
        try:
            from cell_external_bridge.bridge import get_ros_client
        except ImportError as exc:
            return {
                "status": "error",
                "reason": "bridge_unavailable",
                "message": f"bridge import failed: {exc}",
            }
        try:
            client = await get_ros_client(self.config)
            return await client.validate_online_program()
        except Exception as exc:
            return {
                "status": "error",
                "reason": "robot_cell_orchestrator_validation_failed",
                "message": f"failed to validate Robot Cell Orchestrator online program: {exc}",
            }

    async def _load_robot_cell_orchestrator_online_program(self, selection: RuntimeProgramSelection) -> dict:
        if self.config.simulate_mode:
            return {
                "status": "ok",
                "message": "simulate mode skips Robot Cell Orchestrator program load",
                "files": selection.requested_files,
            }
        try:
            from cell_external_bridge.bridge import ProgramLoadRequest, get_ros_client
        except ImportError as exc:
            return {
                "status": "error",
                "reason": "bridge_unavailable",
                "message": f"bridge import failed: {exc}",
            }
        try:
            client = await get_ros_client(self.config)
            return await client.load_online_program(ProgramLoadRequest(
                qqc_id=selection.qqc_id,
                bin_teach_file=selection.bin_teach_file,
                item_teach_file=selection.item_teach_file,
                tray_teach_file=selection.tray_teach_file,
                tray_x_mm=selection.tray_x_mm,
                tray_y_mm=selection.tray_y_mm,
                tray_rz_deg=selection.tray_rz_deg,
            ))
        except Exception as exc:
            return {
                "status": "error",
                "reason": "robot_cell_orchestrator_load_program_failed",
                "message": f"failed to load Robot Cell Orchestrator online program: {exc}",
            }

    async def pick(self, payload: dict) -> dict:
        """Wait for the Robot Cell Orchestrator online moving-to-tray milestone."""
        if self.config.offline_debug:
            return await self._debug_ack_operation("pick", payload)
        precheck = await self._check_qqc("pick", payload)
        if precheck is not None:
            return precheck
        return await self._run_without_escalation("pick", self._do_pick, payload)

    async def place(self, payload: dict) -> dict:
        """Wait for the Robot Cell Orchestrator online moving-to-bin milestone."""
        if self.config.offline_debug:
            return await self._debug_ack_operation("place", payload)
        precheck = await self._check_qqc("place", payload)
        if precheck is not None:
            return precheck
        return await self._run_without_escalation("place", self._do_place, payload)

    # ------------------------------------------------------------------
    # Deterministic simulation hook (test/dev infrastructure).
    #
    # Real motion lives in arm_communication; the placeholder bodies
    # delegate here so status handling can be exercised without hardware.
    # ------------------------------------------------------------------

    async def _simulate(self, op_name: str, payload: dict) -> dict:
        """CELL_BRIDGE_SIMULATE_MODE-driven outcomes; raises if unset.

        Modes: ``success``, ``always_fail``, ``always_fail_hard``,
        ``fail_then_pass``, ``fail_twice_then_pass``.
        """
        mode = self.config.simulate_mode
        if not mode:
            raise NotImplementedError(
                f"_do_{op_name} is not wired "
                "(set CELL_BRIDGE_SIMULATE_MODE for tests/sim)"
            )
        if mode == "success":
            return {"status": "ok"}
        if mode == "always_fail":
            return {"status": "error", "reason": "sim_always_fail", "retryable": True}
        if mode == "always_fail_hard":
            return {"status": "error", "reason": "sim_always_fail_hard", "retryable": False}
        if mode in ("fail_then_pass", "fail_twice_then_pass"):
            counter = self._sim_counts[op_name]
            self._sim_counts[op_name] = counter + 1
            threshold = 1 if mode == "fail_then_pass" else 2
            if counter < threshold:
                reason = (
                    "sim_first_attempt_fail" if mode == "fail_then_pass"
                    else f"sim_attempt_{counter+1}_fail"
                )
                return {"status": "error", "reason": reason, "retryable": True}
            return {"status": "ok"}
        raise NotImplementedError(f"Unknown CELL_BRIDGE_SIMULATE_MODE={mode!r}")

    # ------------------------------------------------------------------
    # Online phase acknowledgements
    # ------------------------------------------------------------------

    async def _transition_state(self, new_state: str) -> None:
        """Update :attr:`state` and immediately publish telemetry.

        Cycle phases finish well inside the heartbeat interval, so a
        subscriber relying purely on the heartbeat aliases on whichever
        phase falls in the sample window. Publishing on every transition
        gives the dashboard sub-second resolution while the periodic
        heartbeat stays in place as a liveness signal.
        """
        if self.state == new_state:
            return
        self.state = new_state
        try:
            await self._publish_telemetry()
        except Exception:
            logger.exception(
                "[arm=%s] state-transition telemetry publish failed",
                self.config.robot_arm_id,
            )

    async def _check_qqc(self, op_name: str, payload: dict) -> Optional[dict]:
        """Reject pick/place if no QQC is loaded or the id mismatches.

        Mismatches indicate a coordination bug; the online v1 flow reports the
        operation failure directly without retry or conveyor escalation.
        """
        requested = payload.get("qqc_id")
        if self._current_qqc_id is None or self._current_qqc_id not in self._active_qqc:
            reason = "no_qqc_loaded"
        elif requested is not None and requested != self._current_qqc_id:
            reason = "qqc_mismatch"
        else:
            return None

        logger.error(
            "[arm=%s op=%s] Coordination error: %s (requested=%r loaded=%r)",
            self.config.robot_arm_id, op_name, reason,
            requested, self._current_qqc_id,
        )
        cycle = self._cycle_fields(payload)
        await self._publish_status(
            f"{op_name}_failed",
            reason=reason,
            requested_qqc_id=requested,
            loaded_qqc_id=self._current_qqc_id,
            **cycle,
        )
        await self._transition_state("idle")
        return {"status": "error", "reason": reason}

    async def _run_without_escalation(
        self,
        op_name: str,
        fn: Callable[[dict], Awaitable[dict]],
        payload: dict,
    ) -> dict:
        """Run one online phase acknowledgement without conveyor escalation.

        Online Robot Cell Orchestrator v1 has no sensor-backed failure path. A timeout
        reports only ``pick_failed`` / ``place_failed``; it does not request
        conveyor stop, issue resolution, or manual intervention.
        """
        cycle = self._cycle_fields(payload)
        self.attempt = 1
        await self._transition_state(f"executing_{op_name}")
        try:
            result = await fn(payload)
        except NotImplementedError:
            raise
        except Exception as exc:
            logger.exception(
                "[arm=%s op=%s] Unhandled exception",
                self.config.robot_arm_id, op_name,
            )
            result = {"status": "error", "reason": str(exc), "retryable": False}

        if result.get("status", "error") == "ok":
            await self._publish_status(
                f"{op_name}_ok",
                attempt=1,
                cycle_index=result.get("cycle_index"),
                phase_id=result.get("phase_id"),
                **cycle,
            )
            self.attempt = 0
            await self._transition_state("idle")
            return {"status": "ok", "attempt": 1}

        reason = result.get("reason", "unknown_error")
        await self._publish_status(
            f"{op_name}_failed",
            attempt=1,
            reason=reason,
            retryable=False,
            **cycle,
        )
        self.attempt = 0
        await self._transition_state("idle")
        return {"status": "error", "reason": reason}

    async def _debug_load_program(
        self,
        payload: dict,
        *,
        qqc_id: str,
        xml_version: Any,
        xml_sha256: Any,
    ) -> dict:
        """Offline GUI debug path: record command, skip runtime/ROS, ACK load."""
        selection, _parse_error = parse_runtime_program_selection(payload)
        if selection is not None:
            qqc_id = selection.qqc_id
            files = selection.requested_files
            tray_fields = {
                "tray_x_mm": selection.tray_x_mm,
                "tray_y_mm": selection.tray_y_mm,
                "tray_rz_deg": selection.tray_rz_deg,
            }
        else:
            qqc_id = qqc_id or "debug_program"
            files = self._debug_payload_filenames(payload)
            tray_fields = {}
        self._active_qqc[qqc_id] = {
            "xml_version": xml_version,
            "xml_sha256": xml_sha256,
            "runtime_files": files,
            "offline_debug": True,
            **tray_fields,
        }
        self._current_qqc_id = qqc_id
        logger.info(
            "[arm=%s] Offline debug accepted load_program qqc=%s v=%s (%d declared files)",
            self.config.robot_arm_id,
            qqc_id,
            xml_version,
            len(files),
        )
        await self._publish_status(
            EVENT_LOAD_PROGRAM_OK,
            qqc_id=qqc_id,
            xml_version=xml_version,
            files=files,
            debug_mode=True,
            message="offline debug accepted without writing runtime or calling Robot Cell Orchestrator",
            **tray_fields,
        )
        return {
            "status": "ok",
            "qqc_id": qqc_id,
            "xml_version": xml_version,
            "files": files,
            "debug_mode": True,
        }

    async def _debug_ack_operation(self, op_name: str, payload: dict) -> dict:
        """Offline GUI debug path: publish immediate pick/place success."""
        qqc_id = str(payload.get("qqc_id") or self._current_qqc_id or "debug_program")
        if qqc_id not in self._active_qqc:
            self._active_qqc[qqc_id] = {"offline_debug": True, "runtime_files": []}
        self._current_qqc_id = qqc_id
        cycle = self._cycle_fields(payload)
        await self._transition_state(f"debug_{op_name}")
        await self._publish_status(
            f"{op_name}_ok",
            attempt=1,
            qqc_id=qqc_id,
            debug_mode=True,
            message="offline debug fake success response",
            **cycle,
        )
        await self._transition_state("idle")
        return {"status": "ok", "attempt": 1, "qqc_id": qqc_id, "debug_mode": True}

    @staticmethod
    def _debug_payload_filenames(payload: dict) -> list[str]:
        for field_name in ("runtime_files", "program_files", "teach_files"):
            files = payload.get(field_name)
            if isinstance(files, dict):
                return [str(name) for name in files.keys()]
            if isinstance(files, list):
                names: list[str] = []
                for item in files:
                    if isinstance(item, dict) and item.get("filename"):
                        names.append(str(item["filename"]))
                return names
        return []

    @staticmethod
    def _cycle_fields(payload: dict) -> dict:
        return {
            "tray_id": payload.get("tray_id"),
            "task_id": payload.get("task_id"),
        }

    # ------------------------------------------------------------------
    # Conveyor state callback
    # ------------------------------------------------------------------

    def _on_conveyor_state_changed(self, new_state: str, triggered_by: str) -> None:
        """Mirror the broadcast conveyor state locally.

        For *new* commands the controller pauses implicitly: the
        conveyor server stops emitting ``cmd.pick`` / ``cmd.place``
        once the belt is stopped, so ``_consume_commands`` simply
        idles until the belt resumes.

        For an *in-flight* cycle however we need an explicit cancel:
        without it ``robot_cell_orchestrator`` happily polls ``item_detect/seek_status``
        forever (the seek loop has no upper bound) so a Stop pressed
        mid-pick leaves the arm armed and the bridge effectively
        deadlocked until something kills the process. Signal the
        bridge so the worker thread unwinds at its next polling
        boundary; the failure surfaces as ``REASON_CANCELLED`` and
        is reported back to the conveyor as a non-retryable failure.
        """
        previous_state = self._conveyor_state
        self._conveyor_state = new_state
        logger.info(
            "[arm=%s] Conveyor state -> %s (triggered_by=%s)",
            self.config.robot_arm_id, new_state, triggered_by,
        )

        if self.config.offline_debug:
            return

        if str(new_state).lower() == "stopped":
            try:
                from cell_external_bridge.bridge import cancel_ros_client_in_flight
            except ImportError:
                # Bridge module unavailable (sim/unit-test context); the
                # controller cannot have an in-flight ROS cycle either,
                # so treat this as a no-op.
                return
            try:
                signalled = cancel_ros_client_in_flight(
                    reason=f"conveyor_stopped(triggered_by={triggered_by})",
                )
            except Exception:
                logger.exception(
                    "[arm=%s] failed to cancel in-flight cycle on conveyor stop",
                    self.config.robot_arm_id,
                )
                return
            if signalled:
                logger.warning(
                    "[arm=%s] In-flight cycle cancelled (conveyor %s -> %s, by=%s)",
                    self.config.robot_arm_id,
                    previous_state, new_state, triggered_by,
                )

    # ------------------------------------------------------------------
    # Publish helpers
    # ------------------------------------------------------------------

    async def _publish_status(self, event: str, **fields: Any) -> None:
        """Publish on ``belt.{belt}.arm.{n}.status`` and log every send."""
        cfg = self.config
        payload = envelope(cfg, event, **fields)
        self._datalog_record("sent_status", cfg.status_routing_key, payload)
        sent = await _publish_message(
            self._exchange, cfg.status_routing_key, payload,
            inject=self._inject_publish, recorded=self._published_messages,
        )
        if not sent:
            logger.warning(
                "[arm=%s] No RabbitMQ exchange; dropping %s",
                cfg.robot_arm_id, event,
            )
            return
        logger.info(
            "[arm=%s] -> %s event=%s",
            cfg.robot_arm_id, cfg.status_routing_key, event,
        )

    async def _publish_telemetry(self) -> None:
        """Publish one heartbeat snapshot on the telemetry routing key."""
        cfg = self.config
        payload = envelope(
            cfg, "arm.heartbeat",
            state=self.state,
            current_qqc_id=self._current_qqc_id,
            attempt=self.attempt,
        )
        # ``attempt`` is intentionally kept at zero (envelope drops Nones,
        # not zeros) so dashboards see retry counters even between cycles.
        await _publish_message(
            self._exchange, cfg.telemetry_routing_key, payload,
            inject=self._inject_publish, recorded=self._published_messages,
        )

    async def _warmup_bridge(self) -> None:
        """Block briefly until the ROS bridge's startup gate passes.

        Skipped entirely in simulate mode (the bridge is never used) and
        when ``bridge_warmup_timeout_s`` is non-positive (legacy lazy
        behavior). On import failure -- which happens in environments
        without rclpy / robot_cell_orchestrator, e.g. CI -- we log once and proceed,
        because the per-call dispatch will surface the same error with
        a clear ``bridge_unavailable`` reason.

        On startup-gate timeout we also proceed and let the next online
        command report the current ROS readiness status.
        """
        cfg = self.config
        if cfg.offline_debug:
            return
        if cfg.simulate_mode:
            return
        timeout = float(getattr(cfg, "bridge_warmup_timeout_s", 0.0) or 0.0)
        if timeout <= 0:
            return

        try:
            from cell_external_bridge.bridge import get_ros_client
        except ImportError as exc:
            logger.warning(
                "[arm=%s] ROS bridge module unavailable; skipping warmup (%s)",
                cfg.robot_arm_id, exc,
            )
            return

        deadline = asyncio.get_event_loop().time() + timeout
        attempt = 0
        last_reason: Optional[str] = None
        while True:
            attempt += 1
            remaining = deadline - asyncio.get_event_loop().time()
            if remaining <= 0:
                logger.warning(
                    "[arm=%s] ROS bridge not ready after %.1fs (last_reason=%s); "
                    "starting cmd consumer anyway",
                    cfg.robot_arm_id, timeout, last_reason or "unknown",
                )
                return
            try:
                client = await get_ros_client(cfg)
            except Exception as exc:
                last_reason = f"client_init_failed: {exc}"
                logger.info(
                    "[arm=%s] bridge warmup attempt %d: %s; retrying",
                    cfg.robot_arm_id, attempt, last_reason,
                )
                await asyncio.sleep(min(2.0, max(remaining, 0.1)))
                continue

            result = await client.wait_ready()
            if result.get("status") == "ok":
                used = timeout - max(
                    deadline - asyncio.get_event_loop().time(), 0,
                )
                logger.info(
                    "[arm=%s] ROS bridge ready (attempt %d, %.1fs of warmup budget used)",
                    cfg.robot_arm_id, attempt, used,
                )
                return
            last_reason = result.get("reason", "unknown")
            # The internal startup gate can return an error in
            # microseconds when the ROS graph hasn't even published the
            # required services yet (it short-circuits without waiting
            # the full timeout). Without an explicit sleep here the
            # warmup would spin thousands of times per second until the
            # outer deadline is reached. Pause for a steady 2 s between
            # probes -- long enough for ROS discovery to make progress,
            # short enough that a real recovery is picked up promptly.
            now = asyncio.get_event_loop().time()
            remaining = deadline - now
            sleep_for = min(2.0, max(remaining, 0.1))
            logger.info(
                "[arm=%s] bridge warmup attempt %d: %s; sleeping %.1fs "
                "(%.1fs remaining)",
                cfg.robot_arm_id, attempt, last_reason,
                sleep_for, max(remaining, 0.0),
            )
            await asyncio.sleep(sleep_for)

    async def _heartbeat_loop(self) -> None:
        """Publish telemetry on a fixed interval until ``stop()`` fires."""
        cfg = self.config
        if cfg.heartbeat_interval_s <= 0:
            return
        try:
            while not self._stopping.is_set():
                try:
                    await self._publish_telemetry()
                except Exception:
                    logger.exception(
                        "[arm=%s] heartbeat publish failed", cfg.robot_arm_id,
                    )
                try:
                    await asyncio.wait_for(
                        self._stopping.wait(),
                        timeout=cfg.heartbeat_interval_s,
                    )
                except asyncio.TimeoutError:
                    continue
        except asyncio.CancelledError:
            raise

    async def _fail_load_program(
        self, qqc_id: Optional[str], xml_version: Any, reason: str,
    ) -> dict:
        logger.error(
            "[arm=%s] load failed (qqc=%s v=%s): %s",
            self.config.robot_arm_id, qqc_id, xml_version, reason,
        )
        await self._publish_status(
            EVENT_LOAD_PROGRAM_FAILED,
            qqc_id=qqc_id, xml_version=xml_version, reason=reason,
        )
        return {"status": "error", "reason": reason}

    # ------------------------------------------------------------------
    # Consumer loops
    # ------------------------------------------------------------------

    async def _run_consumer(
        self,
        queue: Optional[AbstractRobustQueue],
        handler: Callable[[AbstractIncomingMessage], Awaitable[None]],
        what: str,
    ) -> None:
        if queue is None:
            raise RuntimeError(f"connect() must be called before {what}")
        async with queue.iterator() as queue_iter:
            async for message in queue_iter:
                async with message.process():
                    await handler(message)

    async def _consume_commands(self) -> None:
        await self._run_consumer(
            self._cmd_queue, self._handle_command, "_consume_commands",
        )

    async def _consume_state_broadcasts(self) -> None:
        await self._run_consumer(
            self._state_queue, self._handle_state_broadcast,
            "_consume_state_broadcasts",
        )

    async def _handle_command(self, message: AbstractIncomingMessage) -> None:
        """Dispatch one command message to load_program / pick / place.

        Malformed messages are logged and discarded; raising would tear
        down the consumer iterator and silence the arm.
        """
        cfg = self.config
        try:
            if not verify_message(message):
                logger.warning(
                    "[arm=%s] Rejected cmd -- bad HMAC", cfg.robot_arm_id,
                )
                return

            try:
                payload = json.loads(message.body.decode())
            except json.JSONDecodeError:
                logger.exception("[arm=%s] Bad JSON in cmd", cfg.robot_arm_id)
                return

            routing_key = message.routing_key or ""
            self._datalog_record("received_command", routing_key, payload)
            handlers: dict[str, tuple[Callable[[dict], Awaitable[dict]], str]] = {
                ".load_program": (
                    self.load_program,
                    f"qqc={payload.get('qqc_id')} v={payload.get('xml_version')}",
                ),
                ".pick": (self.pick, f"tray={payload.get('tray_id')}"),
                ".place": (self.place, f"tray={payload.get('tray_id')}"),
            }
            for suffix, (handler, summary) in handlers.items():
                if routing_key.endswith(suffix):
                    logger.info(
                        "[arm=%s] <- %s %s",
                        cfg.robot_arm_id, routing_key, summary,
                    )
                    await handler(payload)
                    return

            logger.warning(
                "[arm=%s] Unknown command routing key: %s",
                cfg.robot_arm_id, routing_key,
            )
        except Exception:
            logger.exception(
                "[arm=%s] Error handling command", cfg.robot_arm_id,
            )

    async def _handle_state_broadcast(self, message: AbstractIncomingMessage) -> None:
        cfg = self.config
        try:
            if not verify_message(message):
                logger.warning(
                    "[arm=%s] Rejected state broadcast -- bad HMAC",
                    cfg.robot_arm_id,
                )
                return
            body = json.loads(message.body.decode())
            self._datalog_record(
                "received_state_broadcast",
                message.routing_key or "",
                body,
            )
            self._on_conveyor_state_changed(
                body.get("state", "unknown"),
                body.get("triggered_by", "unknown"),
            )
        except Exception:
            logger.exception(
                "[arm=%s] Error handling state broadcast", cfg.robot_arm_id,
            )

    # ------------------------------------------------------------------
    # Top-level run loop with reconnect
    # ------------------------------------------------------------------

    async def run(self) -> None:
        """Connect + run consumer loops; reconnect on connection loss.

        ``aio_pika.connect_robust`` handles channel-level blips. The
        outer ``while`` is a safety net for full-broker restarts.
        """
        backoff = 1.0
        while not self._stopping.is_set():
            heartbeat_task: Optional[asyncio.Task] = None
            try:
                await self.connect()
                backoff = 1.0
                heartbeat_task = asyncio.create_task(self._heartbeat_loop())
                self._heartbeat_task = heartbeat_task
                # Eagerly stand up the ROS bridge before we start pulling
                # commands. Without this gate, a cmd.pick that was queued
                # by the conveyor while this arm was offline gets handed
                # to ``_do_pick`` the instant the queue is bound -- often
                # less than a second after the background ros2 launches
                # fire and well before trigger services have registered,
                # so the very first cycle after a restart fails with
                # ``service_unavailable``. The warmup is best-effort: on
                # timeout we proceed anyway and the command reports the
                # current service state.
                await self._warmup_bridge()
                await asyncio.gather(
                    self._consume_commands(),
                    self._consume_state_broadcasts(),
                )
            except asyncio.CancelledError:
                raise
            except Exception:
                logger.exception(
                    "[arm=%s] run loop crashed; reconnecting in %.1fs",
                    self.config.robot_arm_id, backoff,
                )
            finally:
                if heartbeat_task is not None and not heartbeat_task.done():
                    heartbeat_task.cancel()
                    try:
                        await heartbeat_task
                    except (asyncio.CancelledError, Exception):
                        pass
                self._heartbeat_task = None
                await self.disconnect()

            if self._stopping.is_set():
                break
            try:
                await asyncio.wait_for(self._stopping.wait(), timeout=backoff)
            except asyncio.TimeoutError:
                pass
            backoff = min(backoff * 2, 30.0)

        await self.disconnect()

    async def stop(self) -> None:
        self._stopping.set()

    def _datalog_record(self, direction: str, routing_key: str, payload: dict) -> None:
        """Append one offline-debug JSONL record without affecting command flow."""
        if not self.config.offline_debug:
            return
        record = {
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "direction": direction,
            "routing_key": routing_key,
            "belt_id": self.config.belt_id,
            "robot_arm_id": self.config.robot_arm_id,
            "arm_number": self.config.arm_number,
            "payload": payload,
        }
        try:
            log_dir = Path(self.config.datalog_dir).expanduser()
            if not log_dir.is_absolute():
                log_dir = Path.cwd() / log_dir
            log_dir.mkdir(parents=True, exist_ok=True)
            log_path = log_dir / f"{self.config.robot_arm_id}_offline_debug.jsonl"
            with log_path.open("a", encoding="utf-8") as outfile:
                outfile.write(json.dumps(record, sort_keys=True, default=str) + "\n")
        except Exception:
            logger.exception(
                "[arm=%s] failed to write offline debug datalog",
                self.config.robot_arm_id,
            )


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def setup_logging() -> None:
    """Configure root logging from ``LOG_LEVEL`` (default INFO)."""
    logging.basicConfig(
        level=os.getenv("LOG_LEVEL", "INFO").upper(),
        format="%(asctime)s %(levelname)s %(name)s :: %(message)s",
    )


# Backwards-compatible alias for any caller still importing the private name.
_setup_logging = setup_logging


def main() -> None:
    setup_logging()
    cfg = Config.from_env()
    logger.info(
        "Starting RobotArmController belt=%s arm=%s simulate=%r offline_debug=%s",
        cfg.belt_id, cfg.robot_arm_id, cfg.simulate_mode or "off", cfg.offline_debug,
    )
    controller = RobotArmController(cfg)
    try:
        asyncio.run(controller.run())
    except KeyboardInterrupt:
        logger.info("Interrupted -- shutting down")


if __name__ == "__main__":
    main()


__all__ = [
    "Config",
    "RobotArmController",
    "EVENT_LOAD_PROGRAM_OK",
    "EVENT_LOAD_PROGRAM_FAILED",
    "EVENT_PICK_OK",
    "EVENT_PICK_FAILED",
    "EVENT_PLACE_OK",
    "EVENT_PLACE_FAILED",
    "EVENT_CONVEYOR_STOP_REQUESTED",
    "EVENT_ISSUE_RESOLVED",
    "EVENT_MANUAL_INTERVENTION_REQUIRED",
    "_hmac_sign",
    "_hmac_verify",
    "main",
    "setup_logging",
]
