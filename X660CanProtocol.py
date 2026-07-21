"""Read-only CAN protocol support for the X6-60 motor/positioning unit.

This module intentionally implements no command that can move, stop, brake,
configure, zero, or otherwise alter the X6-60.  Stage 4A is limited to protocol
identification and passive state verification through read requests.
"""

from __future__ import annotations

from dataclasses import dataclass
import struct
from typing import Tuple


CAN_REQUEST_BASE_ID = 0x140
CAN_REPLY_BASE_ID = 0x240
MINIMUM_NODE_ID = 1
MAXIMUM_NODE_ID = 32
CAN_DLC = 8

READ_SOFTWARE_VERSION = 0xB2
READ_MULTI_TURN_ANGLE = 0x92
READ_STATUS_1 = 0x9A
READ_STATUS_2 = 0x9C

READ_ONLY_COMMANDS = frozenset({
    READ_SOFTWARE_VERSION,
    READ_MULTI_TURN_ANGLE,
    READ_STATUS_1,
    READ_STATUS_2,
})

ERROR_FLAG_NAMES = {
    0x0002: "motor stall",
    0x0004: "low voltage",
    0x0008: "over voltage",
    0x0010: "over current",
    0x0040: "power overrun",
    0x0080: "calibration parameter writing error",
    0x0100: "speeding",
    0x1000: "motor over temperature",
    0x2000: "encoder calibration error",
}


@dataclass(frozen=True)
class X660Status1:
    TemperatureC: int
    BrakeReleased: bool
    BusVoltageV: float
    ErrorFlags: int
    ErrorNames: Tuple[str, ...]


@dataclass(frozen=True)
class X660Status2:
    TemperatureC: int
    TorqueCurrentA: float
    SpeedDegPerSec: int
    ShaftAngleDeg: int


def ValidateNodeId(NodeId: int) -> int:
    NodeId = int(NodeId)
    if not MINIMUM_NODE_ID <= NodeId <= MAXIMUM_NODE_ID:
        raise ValueError("X6-60 CAN node ID must be between 1 and 32")
    return NodeId


def RequestArbitrationId(NodeId: int) -> int:
    return CAN_REQUEST_BASE_ID + ValidateNodeId(NodeId)


def ReplyArbitrationId(NodeId: int) -> int:
    return CAN_REPLY_BASE_ID + ValidateNodeId(NodeId)


def BuildReadRequest(Command: int) -> bytes:
    Command = int(Command)
    if Command not in READ_ONLY_COMMANDS:
        raise ValueError(
            f"Command 0x{Command:02X} is not permitted by the read-only harness"
        )
    return bytes([Command, 0, 0, 0, 0, 0, 0, 0])


def ValidateReply(
    NodeId: int,
    Command: int,
    ArbitrationId: int,
    Data,
    IsExtendedId: bool = False,
) -> bytes:
    """Validate one protocol reply and return an immutable eight-byte value."""

    ExpectedCommand = int(Command)
    if ExpectedCommand not in READ_ONLY_COMMANDS:
        raise ValueError("Expected command is not a read-only command")
    if bool(IsExtendedId):
        raise ValueError("X6-60 reply must use a standard 11-bit CAN frame")
    ExpectedId = ReplyArbitrationId(NodeId)
    if int(ArbitrationId) != ExpectedId:
        raise ValueError(
            f"Expected X6-60 reply ID 0x{ExpectedId:03X}; "
            f"received 0x{int(ArbitrationId):03X}"
        )
    Data = bytes(Data)
    if len(Data) != CAN_DLC:
        raise ValueError(f"X6-60 reply DLC must be 8; received {len(Data)}")
    if Data[0] != ExpectedCommand:
        raise ValueError(
            f"Expected reply command 0x{ExpectedCommand:02X}; "
            f"received 0x{Data[0]:02X}"
        )
    return Data


def DecodeSoftwareVersion(Data) -> int:
    Data = bytes(Data)
    if len(Data) != CAN_DLC or Data[0] != READ_SOFTWARE_VERSION:
        raise ValueError("Invalid X6-60 software-version reply")
    return int(struct.unpack_from("<I", Data, 4)[0])


def DecodeMultiTurnAngleDeg(Data) -> float:
    Data = bytes(Data)
    if len(Data) != CAN_DLC or Data[0] != READ_MULTI_TURN_ANGLE:
        raise ValueError("Invalid X6-60 multi-turn-angle reply")
    AngleHundredthsDeg = struct.unpack_from("<i", Data, 4)[0]
    return float(AngleHundredthsDeg) * 0.01


def DecodeStatus1(Data) -> X660Status1:
    Data = bytes(Data)
    if len(Data) != CAN_DLC or Data[0] != READ_STATUS_1:
        raise ValueError("Invalid X6-60 status-1 reply")
    TemperatureC = int(struct.unpack_from("<b", Data, 1)[0])
    BrakeReleased = bool(Data[3])
    BusVoltageV = float(struct.unpack_from("<H", Data, 4)[0]) * 0.1
    ErrorFlags = int(struct.unpack_from("<H", Data, 6)[0])
    KnownMask = 0
    ErrorNames = []
    for Flag, Name in ERROR_FLAG_NAMES.items():
        KnownMask |= Flag
        if ErrorFlags & Flag:
            ErrorNames.append(Name)
    UnknownFlags = ErrorFlags & ~KnownMask
    if UnknownFlags:
        ErrorNames.append(f"unknown flags 0x{UnknownFlags:04X}")
    return X660Status1(
        TemperatureC=TemperatureC,
        BrakeReleased=BrakeReleased,
        BusVoltageV=BusVoltageV,
        ErrorFlags=ErrorFlags,
        ErrorNames=tuple(ErrorNames),
    )


def DecodeStatus2(Data) -> X660Status2:
    Data = bytes(Data)
    if len(Data) != CAN_DLC or Data[0] != READ_STATUS_2:
        raise ValueError("Invalid X6-60 status-2 reply")
    return X660Status2(
        TemperatureC=int(struct.unpack_from("<b", Data, 1)[0]),
        TorqueCurrentA=float(struct.unpack_from("<h", Data, 2)[0]) * 0.01,
        SpeedDegPerSec=int(struct.unpack_from("<h", Data, 4)[0]),
        ShaftAngleDeg=int(struct.unpack_from("<h", Data, 6)[0]),
    )
