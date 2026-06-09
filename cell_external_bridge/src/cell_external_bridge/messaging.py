"""RabbitMQ envelope, publish, and HMAC helpers shared by the controller
and tests.

This module keeps "build envelope, append to recorded, branch on inject
vs exchange, sign + publish" in one implementation so production and
tests stay byte-identical on the wire.
"""

from __future__ import annotations

import json
import logging
import os
import sys
from datetime import datetime, timezone
from typing import TYPE_CHECKING, Any, Awaitable, Callable, Optional
from urllib.parse import urlparse, urlunparse

import aio_pika

if TYPE_CHECKING:
    from aio_pika.abc import AbstractIncomingMessage, AbstractRobustExchange

    from cell_external_bridge.config import Config

logger = logging.getLogger("cell_external_bridge.messaging")


# ---------------------------------------------------------------------------
# HMAC signing -- best-effort import from packages/shared-contracts. The bridge
# runs unsigned only when MESSAGE_SIGNING_KEY is unset; in production both the
# conveyor/master and bridge share the secret.
# ---------------------------------------------------------------------------
_CONTRACTS_PATH = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "..", "..", "..",
                 "packages", "shared-contracts")
)
if _CONTRACTS_PATH not in sys.path:
    sys.path.insert(0, _CONTRACTS_PATH)

try:
    from hmac_signing import sign as _hmac_sign, verify as _hmac_verify  # noqa: E402
except Exception:  # pragma: no cover - shared-contracts not on path
    _hmac_sign = None  # type: ignore[assignment]
    _hmac_verify = None  # type: ignore[assignment]


PublishCallback = Callable[[str, dict], Awaitable[None]]


def envelope(cfg: "Config", event: str, **fields: Any) -> dict:
    """Build the standard outbound message envelope.

    None-valued fields are dropped so receivers see the same shape they
    saw before this helper existed.
    """
    payload: dict[str, Any] = {
        "event": event,
        "belt_id": cfg.belt_id,
        "edge_node_id": cfg.edge_node_id,
        "robot_arm_id": cfg.robot_arm_id,
        "arm_number": cfg.arm_number,
        "timestamp": datetime.now(timezone.utc).isoformat(),
    }
    for key, value in fields.items():
        if value is not None:
            payload[key] = value
    return payload


def sign_or_empty(body: bytes) -> str:
    """Return an HMAC for ``body`` or empty string when signing is disabled."""
    if _hmac_sign is None:
        return ""
    try:
        return _hmac_sign(body)
    except RuntimeError:
        return ""


def verify_message(message: "AbstractIncomingMessage") -> bool:
    """True when no signing key is configured or signature checks out."""
    if _hmac_verify is None:
        return True
    signature = (message.headers or {}).get("x-signature", "")
    return bool(signature) and bool(_hmac_verify(message.body, str(signature)))


async def publish(
    exchange: Optional["AbstractRobustExchange"],
    routing_key: str,
    payload: dict,
    *,
    inject: Optional[PublishCallback] = None,
    recorded: Optional[list[tuple[str, dict]]] = None,
) -> bool:
    """Record + dispatch one message; returns True iff it left the process.

    - Always appends to ``recorded`` for the test hook.
    - Prefers the test injection if provided.
    - Drops the message (returns False) if there is no exchange and no
      injection -- callers log the drop with their own arm-scoped context.
    """
    if recorded is not None:
        recorded.append((routing_key, payload))

    if inject is not None:
        await inject(routing_key, payload)
        return True

    if exchange is None:
        return False

    body = json.dumps(payload).encode()
    await exchange.publish(
        aio_pika.Message(
            body=body,
            content_type="application/json",
            delivery_mode=aio_pika.DeliveryMode.PERSISTENT,
            headers={"x-signature": sign_or_empty(body)},
        ),
        routing_key=routing_key,
    )
    return True


def redact_url(url: str) -> str:
    """Return ``url`` with the password (if any) replaced by ``***``."""
    parsed = urlparse(url)
    if not parsed.password:
        return url
    return urlunparse(parsed._replace(
        netloc=f"{parsed.username}:***@{parsed.hostname}:{parsed.port}"
    ))


__all__ = [
    "PublishCallback",
    "envelope",
    "publish",
    "redact_url",
    "sign_or_empty",
    "verify_message",
    "_hmac_sign",
    "_hmac_verify",
]
