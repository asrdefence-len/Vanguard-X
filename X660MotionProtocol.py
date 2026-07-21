"""Pure X6-60 motion-command payload encoding for Vanguard X Stage 5A.

This module only constructs and validates eight-byte CAN data fields.  It has
no CAN interface, bus object, transmit function, brake command, or hardware
access.  Real X6-60 motion therefore remains locked by
``X660ReadOnlyController`` and ``CreateX660Controller``.

Documented commands implemented from Motor Motion Protocol V4.2:

* 0x81 - closed-loop motor stop (zero speed; holding remains active)
* 0xA2 - signed speed closed-loop control
* 0xA4 - absolute multi-turn position with a speed limit
* 0xA8 - incremental multi-turn position with a speed limit
"""

from __future__ import annotations

from decimal import Decimal, InvalidOperation
import math
import struct


MOTOR_STOP_COMMAND = 0x81
SPEED_CONTROL_COMMAND = 0xA2
ABSOLUTE_POSITION_COMMAND = 0xA4
INCREMENTAL_POSITION_COMMAND = 0xA8
CAN_DLC = 8

MOTION_COMMANDS = frozenset(
    (
        MOTOR_STOP_COMMAND,
        SPEED_CONTROL_COMMAND,
        ABSOLUTE_POSITION_COMMAND,
        INCREMENTAL_POSITION_COMMAND,
    )
)


def _RequireScaledInteger(Value, Scale: int, Name: str) -> int:
    """Return exactly scaled finite numeric input without silent rounding."""

    if isinstance(Value, bool):
        raise ValueError(f"{Name} must be numeric, not bool")
    try:
        Numeric = float(Value)
        ScaledDecimal = Decimal(str(Value)) * Decimal(int(Scale))
    except (InvalidOperation, TypeError, ValueError, OverflowError) as Error:
        raise ValueError(f"{Name} must be a finite numeric value") from Error
    if not math.isfinite(Numeric) or not ScaledDecimal.is_finite():
        raise ValueError(f"{Name} must be finite")
    Integral = ScaledDecimal.to_integral_value()
    if ScaledDecimal != Integral:
        Resolution = 1.0 / float(Scale)
        raise ValueError(
            f"{Name} must be an exact multiple of {Resolution:g}"
        )
    return int(Integral)


def _RequireUint16(Value, Name: str) -> int:
    if isinstance(Value, bool):
        raise ValueError(f"{Name} must be an integer, not bool")
    try:
        Integer = int(Value)
    except (TypeError, ValueError, OverflowError) as Error:
        raise ValueError(f"{Name} must be an integer") from Error
    if Integer != Value:
        raise ValueError(f"{Name} must be an integer")
    if not 1 <= Integer <= 0xFFFF:
        raise ValueError(f"{Name} must be between 1 and 65535 deg/s")
    return Integer


def _RequireInt32(Value: int, Name: str) -> int:
    if not -(2**31) <= int(Value) <= (2**31 - 1):
        raise ValueError(f"{Name} exceeds the signed 32-bit protocol range")
    return int(Value)


def BuildMotorStopRequest() -> bytes:
    """Build documented 0x81 zero-speed stop; closed-loop holding stays active."""

    return bytes([MOTOR_STOP_COMMAND, 0, 0, 0, 0, 0, 0, 0])


def BuildSpeedControlRequest(SpeedDegPerSec: float) -> bytes:
    """Build 0xA2 with signed output-shaft speed at 0.01 deg/s per LSB."""

    SpeedHundredths = _RequireInt32(
        _RequireScaledInteger(SpeedDegPerSec, 100, "SpeedDegPerSec"),
        "SpeedDegPerSec",
    )
    return bytes([SPEED_CONTROL_COMMAND, 0, 0, 0]) + struct.pack(
        "<i", SpeedHundredths
    )


def BuildAbsolutePositionRequest(
    TargetRawAngleDeg: float,
    MaxSpeedDegPerSec: int,
) -> bytes:
    """Build 0xA4 absolute multi-turn position and nonzero speed limit."""

    MaxSpeed = _RequireUint16(MaxSpeedDegPerSec, "MaxSpeedDegPerSec")
    TargetHundredths = _RequireInt32(
        _RequireScaledInteger(TargetRawAngleDeg, 100, "TargetRawAngleDeg"),
        "TargetRawAngleDeg",
    )
    return bytes([ABSOLUTE_POSITION_COMMAND, 0]) + struct.pack(
        "<Hi", MaxSpeed, TargetHundredths
    )


def BuildIncrementalPositionRequest(
    IncrementDeg: float,
    MaxSpeedDegPerSec: int,
) -> bytes:
    """Build 0xA8 relative multi-turn position and nonzero speed limit."""

    MaxSpeed = _RequireUint16(MaxSpeedDegPerSec, "MaxSpeedDegPerSec")
    IncrementHundredths = _RequireInt32(
        _RequireScaledInteger(IncrementDeg, 100, "IncrementDeg"),
        "IncrementDeg",
    )
    return bytes([INCREMENTAL_POSITION_COMMAND, 0]) + struct.pack(
        "<Hi", MaxSpeed, IncrementHundredths
    )


def ValidateMotionReply(ExpectedCommand: int, Data) -> bytes:
    """Validate the DLC and command byte of one implemented motion reply."""

    Command = int(ExpectedCommand)
    if Command not in MOTION_COMMANDS:
        raise ValueError("reply command is outside the Stage 5A allow-list")
    Payload = bytes(Data)
    if len(Payload) != CAN_DLC:
        raise ValueError(
            f"X6-60 reply DLC must be 8; received {len(Payload)}"
        )
    if Payload[0] != Command:
        raise ValueError(
            f"Expected reply command 0x{Command:02X}; "
            f"received 0x{Payload[0]:02X}"
        )
    return Payload
