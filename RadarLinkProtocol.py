"""Vanguard X radar/UI TCP framing and JSON safety helpers.

The protocol deliberately carries operator commands and display products only.
Raw IQ never crosses this link.  Messages use a four-byte network-order length
followed by UTF-8 JSON so normal TCP packet boundaries are irrelevant.
"""

from __future__ import annotations

import json
import math
import struct
from typing import Any, Dict, Iterable, List


PROTOCOL_NAME = "VANGUARD_X_RADAR_LINK"
PROTOCOL_VERSION = 1
MAX_MESSAGE_BYTES = 4 * 1024 * 1024
_HEADER = struct.Struct("!I")


class ProtocolError(ValueError):
    """Raised when a peer sends an invalid or unsupported frame."""


def JsonSafe(value: Any) -> Any:
    """Convert common radar/numpy values into strict JSON-compatible values."""

    if value is None or isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {str(key): JsonSafe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [JsonSafe(item) for item in value]

    # numpy scalar/array support without making numpy a protocol dependency.
    item_method = getattr(value, "item", None)
    if callable(item_method):
        try:
            return JsonSafe(item_method())
        except Exception:
            pass
    list_method = getattr(value, "tolist", None)
    if callable(list_method):
        try:
            return JsonSafe(list_method())
        except Exception:
            pass

    return str(value)


def MakeMessage(message_type: str, payload: Any = None, sequence: int = 0) -> Dict[str, Any]:
    return {
        "protocol": PROTOCOL_NAME,
        "version": PROTOCOL_VERSION,
        "type": str(message_type),
        "sequence": int(sequence),
        "payload": JsonSafe({} if payload is None else payload),
    }


def ValidateMessage(message: Any) -> Dict[str, Any]:
    if not isinstance(message, dict):
        raise ProtocolError("message is not an object")
    if message.get("protocol") != PROTOCOL_NAME:
        raise ProtocolError("unexpected protocol")
    if int(message.get("version", -1)) != PROTOCOL_VERSION:
        raise ProtocolError("unsupported protocol version")
    if not isinstance(message.get("type"), str):
        raise ProtocolError("message type is missing")
    if not isinstance(message.get("payload", {}), dict):
        raise ProtocolError("message payload is not an object")
    return message


def EncodeMessage(message: Dict[str, Any]) -> bytes:
    ValidateMessage(message)
    payload = json.dumps(
        JsonSafe(message),
        separators=(",", ":"),
        ensure_ascii=False,
        allow_nan=False,
    ).encode("utf-8")
    if len(payload) > MAX_MESSAGE_BYTES:
        raise ProtocolError(f"message exceeds {MAX_MESSAGE_BYTES} bytes")
    return _HEADER.pack(len(payload)) + payload


class FrameDecoder:
    """Incremental framed-message decoder for non-blocking sockets."""

    def __init__(self, maximum_message_bytes: int = MAX_MESSAGE_BYTES):
        self.MaximumMessageBytes = int(maximum_message_bytes)
        self.Buffer = bytearray()

    def Feed(self, data: bytes) -> List[Dict[str, Any]]:
        if data:
            self.Buffer.extend(data)
        messages: List[Dict[str, Any]] = []
        while len(self.Buffer) >= _HEADER.size:
            message_length = _HEADER.unpack(self.Buffer[: _HEADER.size])[0]
            if message_length <= 0 or message_length > self.MaximumMessageBytes:
                raise ProtocolError(f"invalid frame length {message_length}")
            frame_length = _HEADER.size + message_length
            if len(self.Buffer) < frame_length:
                break
            raw = bytes(self.Buffer[_HEADER.size:frame_length])
            del self.Buffer[:frame_length]
            try:
                message = json.loads(raw.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError) as error:
                raise ProtocolError(f"invalid JSON frame: {error}") from error
            messages.append(ValidateMessage(message))
        return messages


def FilterPublicConfig(config: Dict[str, Any], allowed_keys: Iterable[str]) -> Dict[str, Any]:
    """Return only display-relevant configuration fields."""

    return {
        key: JsonSafe(config[key])
        for key in allowed_keys
        if key in config
    }
