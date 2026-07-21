"""Guarded, injected-bus X6-60 motion transport for Vanguard X Stage 5B.

This module is dry-run by default and cannot open a CAN interface.  A caller
must inject both a bus and, when needed, a CAN-message factory.  Live
transmission additionally requires two explicit safety acknowledgements.

Only the four motion payloads implemented by :mod:`X660MotionProtocol` are
accepted.  Shutdown, brake, reset, configuration, and telemetry commands are
outside this transport's allow-list.
"""

from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Any, Callable, Optional

import X660MotionProtocol as MotionProtocol


MINIMUM_NODE_ID = 1
MAXIMUM_NODE_ID = 32
DEFAULT_TIMEOUT_SEC = 0.25


def RequestArbitrationId(NodeId: int) -> int:
    """Return the documented host-to-motor standard CAN identifier."""

    Node = _ValidateNodeId(NodeId)
    return 0x140 + Node


def ReplyArbitrationId(NodeId: int) -> int:
    """Return the documented motor-to-host standard CAN identifier."""

    Node = _ValidateNodeId(NodeId)
    return 0x240 + Node


def _ValidateNodeId(NodeId: int) -> int:
    if isinstance(NodeId, bool):
        raise ValueError("NodeId must be an integer, not bool")
    try:
        Node = int(NodeId)
    except (TypeError, ValueError, OverflowError) as Error:
        raise ValueError("NodeId must be an integer") from Error
    if Node != NodeId or not MINIMUM_NODE_ID <= Node <= MAXIMUM_NODE_ID:
        raise ValueError("NodeId must be between 1 and 32")
    return Node


def _RequireBoolean(Value, Name: str) -> bool:
    if not isinstance(Value, bool):
        raise ValueError(f"{Name} must be exactly True or False")
    return Value


def ValidateMotionRequest(Payload) -> bytes:
    """Validate one canonical eight-byte Stage 5A motion payload."""

    try:
        Data = bytes(Payload)
    except (TypeError, ValueError) as Error:
        raise ValueError("motion payload must be byte-convertible") from Error
    if len(Data) != MotionProtocol.CAN_DLC:
        raise ValueError(
            f"X6-60 motion request DLC must be 8; received {len(Data)}"
        )

    Command = Data[0]
    if Command not in MotionProtocol.MOTION_COMMANDS:
        raise ValueError(
            f"command 0x{Command:02X} is outside the Stage 5B motion allow-list"
        )

    if Command == MotionProtocol.MOTOR_STOP_COMMAND:
        if Data != MotionProtocol.BuildMotorStopRequest():
            raise ValueError("0x81 stop request contains nonzero reserved bytes")
    elif Command == MotionProtocol.SPEED_CONTROL_COMMAND:
        if Data[1:4] != bytes(3):
            raise ValueError("0xA2 request contains nonzero reserved bytes")
    else:
        if Data[1] != 0:
            raise ValueError(
                f"0x{Command:02X} request contains a nonzero reserved byte"
            )
        if int.from_bytes(Data[2:4], "little") == 0:
            raise ValueError(
                f"0x{Command:02X} position speed limit must be nonzero"
            )
    return Data


@dataclass(frozen=True)
class CanFrame:
    """Minimal CAN-frame value used for dry runs and fake-bus tests."""

    arbitration_id: int
    data: bytes
    is_extended_id: bool = False

    def __post_init__(self):
        object.__setattr__(self, "arbitration_id", int(self.arbitration_id))
        object.__setattr__(self, "data", bytes(self.data))
        object.__setattr__(self, "is_extended_id", bool(self.is_extended_id))


@dataclass(frozen=True)
class MotionTransaction:
    """Result of a dry-run preview or one completed fake/live transaction."""

    Command: int
    Request: Any
    DryRun: bool
    Transmitted: bool
    ReplyData: Optional[bytes] = None


class X660GuardedMotionTransport:
    """Dry-run-first motion transport with no CAN-interface opener."""

    def __init__(
        self,
        NodeId: int = 1,
        Bus=None,
        MessageFactory: Optional[Callable[..., Any]] = None,
        TimeoutSec: float = DEFAULT_TIMEOUT_SEC,
        DryRun: bool = True,
        IUnderstandMotionWillOccur: bool = False,
        IConfirmMotionAreaIsClear: bool = False,
    ):
        self.NodeId = _ValidateNodeId(NodeId)
        self.TimeoutSec = float(TimeoutSec)
        if not 0.01 <= self.TimeoutSec <= 5.0:
            raise ValueError("TimeoutSec must be between 0.01 and 5.0")

        self.DryRun = _RequireBoolean(DryRun, "DryRun")
        self.Bus = Bus
        self.MessageFactory = (
            CanFrame if MessageFactory is None else MessageFactory
        )
        if not callable(self.MessageFactory):
            raise ValueError("MessageFactory must be callable")

        if not self.DryRun:
            if self.Bus is None:
                raise RuntimeError("live transmission requires an injected bus")
            if not callable(getattr(self.Bus, "send", None)):
                raise RuntimeError("injected live bus must provide send()")
            if not callable(getattr(self.Bus, "recv", None)):
                raise RuntimeError("injected live bus must provide recv()")
            if IUnderstandMotionWillOccur is not True:
                raise RuntimeError(
                    "live transmission requires IUnderstandMotionWillOccur"
                )
            if IConfirmMotionAreaIsClear is not True:
                raise RuntimeError(
                    "live transmission requires IConfirmMotionAreaIsClear"
                )

    def _BuildRequestFrame(self, Payload: bytes):
        Frame = self.MessageFactory(
            arbitration_id=RequestArbitrationId(self.NodeId),
            data=Payload,
            is_extended_id=False,
        )
        if int(Frame.arbitration_id) != RequestArbitrationId(self.NodeId):
            raise RuntimeError("message factory changed the request arbitration ID")
        if bool(Frame.is_extended_id):
            raise RuntimeError("message factory produced an extended CAN frame")
        if bytes(Frame.data) != Payload:
            raise RuntimeError("message factory changed the motion payload")
        return Frame

    def Transact(self, Payload) -> MotionTransaction:
        """Preview or transmit one allow-listed motion command."""

        Data = ValidateMotionRequest(Payload)
        Command = Data[0]
        Request = self._BuildRequestFrame(Data)
        if self.DryRun:
            return MotionTransaction(
                Command=Command,
                Request=Request,
                DryRun=True,
                Transmitted=False,
            )

        self.Bus.send(Request, timeout=self.TimeoutSec)
        Deadline = time.monotonic() + self.TimeoutSec
        while True:
            Remaining = Deadline - time.monotonic()
            if Remaining <= 0.0:
                raise TimeoutError(
                    f"timed out waiting for X6-60 motion reply 0x{Command:02X}"
                )
            Reply = self.Bus.recv(timeout=Remaining)
            if Reply is None:
                continue
            if (
                int(Reply.arbitration_id) != ReplyArbitrationId(self.NodeId)
                or bool(Reply.is_extended_id)
            ):
                continue
            ReplyData = bytes(Reply.data)
            if not ReplyData or ReplyData[0] != Command:
                continue
            ValidatedReply = MotionProtocol.ValidateMotionReply(
                Command,
                ReplyData,
            )
            return MotionTransaction(
                Command=Command,
                Request=Request,
                DryRun=False,
                Transmitted=True,
                ReplyData=ValidatedReply,
            )
