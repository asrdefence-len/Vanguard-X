"""Narrow CAN protocol support for X6-60 direction calibration.

Only the two state-changing frames needed by the Stage 4C one-degree
calibration harness are constructible here:

* 0xA4 - absolute multi-turn position with a speed limit
* 0x81 - motor stop, used only as an abnormal-run safeguard

All telemetry requests and reply validation remain in X660CanProtocol.py.
"""

from __future__ import annotations

import struct


ABSOLUTE_POSITION_COMMAND = 0xA4
MOTOR_STOP_COMMAND = 0x81
CAN_DLC = 8


def BuildAbsolutePositionRequest(TargetRawAngleDeg: float, MaxSpeedDegPerSec: int) -> bytes:
    """Build one documented 0xA4 absolute multi-turn position command."""

    Speed = int(MaxSpeedDegPerSec)
    if Speed != MaxSpeedDegPerSec or not 1 <= Speed <= 2:
        raise ValueError("Stage 4C speed limit must be 1 or 2 deg/s")

    TargetHundredths = round(float(TargetRawAngleDeg) * 100.0)
    if not -(2**31) <= TargetHundredths <= (2**31 - 1):
        raise ValueError("target raw angle exceeds signed 32-bit protocol range")

    return bytes([ABSOLUTE_POSITION_COMMAND, 0]) + struct.pack(
        "<Hi", Speed, TargetHundredths
    )


def BuildMotorStopRequest() -> bytes:
    """Build the documented 0x81 stop command."""

    return bytes([MOTOR_STOP_COMMAND, 0, 0, 0, 0, 0, 0, 0])


def ValidateMotionReply(ExpectedCommand: int, Data) -> bytes:
    """Validate the command byte and DLC of an 0xA4 or 0x81 reply."""

    Command = int(ExpectedCommand)
    if Command not in (ABSOLUTE_POSITION_COMMAND, MOTOR_STOP_COMMAND):
        raise ValueError("reply command is outside the Stage 4C allow-list")
    Data = bytes(Data)
    if len(Data) != CAN_DLC:
        raise ValueError(f"X6-60 reply DLC must be 8; received {len(Data)}")
    if Data[0] != Command:
        raise ValueError(
            f"Expected reply command 0x{Command:02X}; received 0x{Data[0]:02X}"
        )
    return Data
