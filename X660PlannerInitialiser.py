"""Fail-safe X6-60 motion-planner initialisation for Vanguard X."""

from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Any, Callable, Tuple

import X660PlannerProtocol as Protocol


CAN_REQUEST_BASE_ID = 0x140
CAN_REPLY_BASE_ID = 0x240
MINIMUM_NODE_ID = 1
MAXIMUM_NODE_ID = 32


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


def _ValidateRequiredValues(Values) -> Tuple[int, int, int, int]:
    try:
        Required = tuple(Values)
    except TypeError as Error:
        raise ValueError("RequiredValues must contain four planner values") from Error
    if len(Required) != len(Protocol.PLANNER_PARAMETER_INDICES):
        raise ValueError("RequiredValues must contain four planner values")
    return tuple(Protocol.ValidatePlannerValue(Value) for Value in Required)


@dataclass(frozen=True)
class X660PlannerInitialisationResult:
    Values: Tuple[int, int, int, int]
    CorrectedIndices: Tuple[int, ...]
    WriteCount: int


class X660PlannerInitialiser:
    """Read, correct only mismatches, and verify all four planner parameters."""

    def __init__(
        self,
        NodeId: int,
        Bus,
        MessageFactory: Callable[..., Any],
        TimeoutSec: float = 0.25,
        RequiredValues=Protocol.VANGUARD_X_REQUIRED_VALUES,
    ):
        self.NodeId = _ValidateNodeId(NodeId)
        self.Bus = Bus
        self.MessageFactory = MessageFactory
        self.TimeoutSec = float(TimeoutSec)
        self.RequiredValues = _ValidateRequiredValues(RequiredValues)

        if self.Bus is None:
            raise RuntimeError("planner initialisation requires an injected CAN bus")
        if not callable(getattr(self.Bus, "send", None)):
            raise RuntimeError("planner initialisation bus must provide send()")
        if not callable(getattr(self.Bus, "recv", None)):
            raise RuntimeError("planner initialisation bus must provide recv()")
        if not callable(self.MessageFactory):
            raise ValueError("MessageFactory must be callable")
        if not 0.01 <= self.TimeoutSec <= 5.0:
            raise ValueError("TimeoutSec must be between 0.01 and 5.0")

    @property
    def RequestArbitrationId(self) -> int:
        return CAN_REQUEST_BASE_ID + self.NodeId

    @property
    def ReplyArbitrationId(self) -> int:
        return CAN_REPLY_BASE_ID + self.NodeId

    def _BuildRequestFrame(self, Payload: bytes):
        Request = self.MessageFactory(
            arbitration_id=self.RequestArbitrationId,
            data=Payload,
            is_extended_id=False,
        )
        if int(Request.arbitration_id) != self.RequestArbitrationId:
            raise RuntimeError("message factory changed planner request CAN ID")
        if bool(Request.is_extended_id):
            raise RuntimeError("message factory produced an extended CAN frame")
        if bytes(Request.data) != Payload:
            raise RuntimeError("message factory changed planner request payload")
        return Request

    def _Transact(self, Payload: bytes) -> bytes:
        Command = int(Payload[0])
        Index = int(Payload[1])
        Request = self._BuildRequestFrame(Payload)
        self.Bus.send(Request, timeout=self.TimeoutSec)
        Deadline = time.monotonic() + self.TimeoutSec

        while True:
            Remaining = Deadline - time.monotonic()
            if Remaining <= 0.0:
                raise TimeoutError(
                    "timed out waiting for X6-60 planner "
                    f"reply 0x{Command:02X}, index 0x{Index:02X}"
                )
            Reply = self.Bus.recv(timeout=Remaining)
            if Reply is None:
                continue
            if (
                int(Reply.arbitration_id) != self.ReplyArbitrationId
                or bool(Reply.is_extended_id)
            ):
                continue
            Data = bytes(Reply.data)
            if len(Data) < 2 or Data[0] != Command or Data[1] != Index:
                continue
            return Data

    def _Read(self, Index: int) -> int:
        Reply = self._Transact(Protocol.BuildPlannerReadRequest(Index))
        return Protocol.DecodePlannerReadReply(Index, Reply)

    def _Write(self, Index: int, Value: int) -> None:
        Reply = self._Transact(Protocol.BuildPlannerWriteRequest(Index, Value))
        Protocol.ValidatePlannerWriteReply(Index, Value, Reply)

    def Initialise(self) -> X660PlannerInitialisationResult:
        """Enforce the required values without unnecessary non-volatile writes."""

        InitialValues = tuple(
            self._Read(Index) for Index in Protocol.PLANNER_PARAMETER_INDICES
        )
        Corrected = []

        for Index, Actual, Required in zip(
            Protocol.PLANNER_PARAMETER_INDICES,
            InitialValues,
            self.RequiredValues,
        ):
            if Actual == Required:
                continue
            self._Write(Index, Required)
            CorrectedValue = self._Read(Index)
            if CorrectedValue != Required:
                raise RuntimeError(
                    f"X6-60 planner index 0x{Index:02X} failed immediate "
                    f"verification: expected {Required}, read {CorrectedValue}"
                )
            Corrected.append(Index)

        FinalValues = tuple(
            self._Read(Index) for Index in Protocol.PLANNER_PARAMETER_INDICES
        )
        if FinalValues != self.RequiredValues:
            Differences = ", ".join(
                f"0x{Index:02X}={Actual} (required {Required})"
                for Index, Actual, Required in zip(
                    Protocol.PLANNER_PARAMETER_INDICES,
                    FinalValues,
                    self.RequiredValues,
                )
                if Actual != Required
            )
            raise RuntimeError(
                f"X6-60 planner final verification failed: {Differences}"
            )

        return X660PlannerInitialisationResult(
            Values=FinalValues,
            CorrectedIndices=tuple(Corrected),
            WriteCount=len(Corrected),
        )
