"""X6-60 motion-planner configuration protocol.

This module implements only the documented planner-parameter read and write
commands used during guarded Vanguard X motor initialisation:

* 0x42 - read one motion-planner parameter
* 0x43 - write one motion-planner parameter to RAM and ROM

Hardware characterisation established that a stored value of 1140 produces
approximately 60 deg/s^2 at the output shaft of the 19:1 X6-60 unit.
"""

from __future__ import annotations

import struct


CAN_DLC = 8
READ_PLANNER_PARAMETER = 0x42
WRITE_PLANNER_PARAMETER = 0x43

POSITION_ACCELERATION_INDEX = 0x00
POSITION_DECELERATION_INDEX = 0x01
SPEED_ACCELERATION_INDEX = 0x02
SPEED_DECELERATION_INDEX = 0x03

PLANNER_PARAMETER_INDICES = (
    POSITION_ACCELERATION_INDEX,
    POSITION_DECELERATION_INDEX,
    SPEED_ACCELERATION_INDEX,
    SPEED_DECELERATION_INDEX,
)

MINIMUM_PLANNER_VALUE = 100
MAXIMUM_PLANNER_VALUE = 60000
VANGUARD_X_REQUIRED_PLANNER_VALUE = 1140
VANGUARD_X_REQUIRED_VALUES = (
    VANGUARD_X_REQUIRED_PLANNER_VALUE,
    VANGUARD_X_REQUIRED_PLANNER_VALUE,
    VANGUARD_X_REQUIRED_PLANNER_VALUE,
    VANGUARD_X_REQUIRED_PLANNER_VALUE,
)


def ValidatePlannerIndex(Index: int) -> int:
    if isinstance(Index, bool):
        raise ValueError("planner index must be an integer, not bool")
    try:
        Value = int(Index)
    except (TypeError, ValueError, OverflowError) as Error:
        raise ValueError("planner index must be an integer") from Error
    if Value != Index or Value not in PLANNER_PARAMETER_INDICES:
        raise ValueError("planner index must be one of 0x00, 0x01, 0x02, 0x03")
    return Value


def ValidatePlannerValue(Value: int) -> int:
    if isinstance(Value, bool):
        raise ValueError("planner value must be an integer, not bool")
    try:
        Integer = int(Value)
    except (TypeError, ValueError, OverflowError) as Error:
        raise ValueError("planner value must be an integer") from Error
    if Integer != Value:
        raise ValueError("planner value must be an integer")
    if not MINIMUM_PLANNER_VALUE <= Integer <= MAXIMUM_PLANNER_VALUE:
        raise ValueError(
            "planner value must be between 100 and 60000 internal deg/s^2"
        )
    return Integer


def BuildPlannerReadRequest(Index: int) -> bytes:
    ParameterIndex = ValidatePlannerIndex(Index)
    return bytes(
        [READ_PLANNER_PARAMETER, ParameterIndex, 0, 0, 0, 0, 0, 0]
    )


def BuildPlannerWriteRequest(Index: int, Value: int) -> bytes:
    ParameterIndex = ValidatePlannerIndex(Index)
    ParameterValue = ValidatePlannerValue(Value)
    return bytes([WRITE_PLANNER_PARAMETER, ParameterIndex, 0, 0]) + struct.pack(
        "<I",
        ParameterValue,
    )


def _ValidateReplyHeader(ExpectedCommand: int, Index: int, Data) -> bytes:
    ParameterIndex = ValidatePlannerIndex(Index)
    Payload = bytes(Data)
    if len(Payload) != CAN_DLC:
        raise ValueError(
            f"X6-60 planner reply DLC must be 8; received {len(Payload)}"
        )
    if Payload[0] != int(ExpectedCommand):
        raise ValueError(
            f"Expected X6-60 planner reply 0x{int(ExpectedCommand):02X}; "
            f"received 0x{Payload[0]:02X}"
        )
    if Payload[1] != ParameterIndex:
        raise ValueError(
            f"Expected X6-60 planner index 0x{ParameterIndex:02X}; "
            f"received 0x{Payload[1]:02X}"
        )
    if Payload[2:4] != bytes(2):
        raise ValueError("X6-60 planner reply contains nonzero reserved bytes")
    return Payload


def DecodePlannerReadReply(Index: int, Data) -> int:
    Payload = _ValidateReplyHeader(READ_PLANNER_PARAMETER, Index, Data)
    return ValidatePlannerValue(struct.unpack_from("<I", Payload, 4)[0])


def ValidatePlannerWriteReply(Index: int, Value: int, Data) -> bytes:
    Expected = BuildPlannerWriteRequest(Index, Value)
    Payload = _ValidateReplyHeader(WRITE_PLANNER_PARAMETER, Index, Data)
    if Payload != Expected:
        ReceivedValue = struct.unpack_from("<I", Payload, 4)[0]
        raise ValueError(
            f"X6-60 planner write echo mismatch for index 0x{int(Index):02X}: "
            f"expected {int(Value)}, received {ReceivedValue}"
        )
    return Payload
