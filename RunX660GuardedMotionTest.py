#!/usr/bin/env python3
"""Standalone, bounded X6-60 motion harness for Vanguard X Stage 5C.

The harness is dry-run by default.  Live operation is intentionally limited to
one small incremental-position command, followed by a closed-loop stop and the
proven motor-output shutdown command.  It is not connected to the Vanguard X
controller, factory, pointing manager, or scheduler.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from decimal import Decimal, InvalidOperation
import math
import time
from typing import Any, Callable, Optional

import RunX660MotorShutdown as Shutdown
import X660GuardedMotionTransport as GuardedTransport
import X660MotionProtocol as MotionProtocol


DEFAULT_INCREMENT_DEG = 1.0
DEFAULT_MAX_SPEED_DEG_PER_SEC = 2
MAXIMUM_ABSOLUTE_INCREMENT_DEG = 2.0
MAXIMUM_SPEED_DEG_PER_SEC = 5
MOTION_COMPLETION_MARGIN_SEC = 0.5


@dataclass(frozen=True)
class Stage5CPlan:
    MotionRequest: Any
    StopRequest: Any
    ShutdownRequest: Any
    EstimatedMotionTimeSec: float


@dataclass(frozen=True)
class Stage5CResult:
    MotionTransaction: Any
    StopTransaction: Any
    ShutdownReply: bytes
    EstimatedMotionTimeSec: float


def _RequireExactHundredth(Value, Name: str) -> float:
    if isinstance(Value, bool):
        raise ValueError(f"{Name} must be numeric, not bool")
    try:
        Numeric = float(Value)
        Hundredths = Decimal(str(Value)) * Decimal(100)
    except (InvalidOperation, TypeError, ValueError, OverflowError) as Error:
        raise ValueError(f"{Name} must be a finite numeric value") from Error
    if not math.isfinite(Numeric) or not Hundredths.is_finite():
        raise ValueError(f"{Name} must be finite")
    if Hundredths != Hundredths.to_integral_value():
        raise ValueError(f"{Name} must be an exact multiple of 0.01")
    return Numeric


def ValidateMotionLimits(IncrementDeg, MaxSpeedDegPerSec) -> tuple[float, int]:
    """Validate the deliberately narrow Stage 5C jog envelope."""

    Increment = _RequireExactHundredth(IncrementDeg, "IncrementDeg")
    if not 0.01 <= abs(Increment) <= MAXIMUM_ABSOLUTE_INCREMENT_DEG:
        raise ValueError(
            "IncrementDeg magnitude must be between 0.01 and 2.00 degrees"
        )
    if isinstance(MaxSpeedDegPerSec, bool):
        raise ValueError("MaxSpeedDegPerSec must be an integer, not bool")
    try:
        Speed = int(MaxSpeedDegPerSec)
    except (TypeError, ValueError, OverflowError) as Error:
        raise ValueError("MaxSpeedDegPerSec must be an integer") from Error
    if Speed != MaxSpeedDegPerSec or not 1 <= Speed <= MAXIMUM_SPEED_DEG_PER_SEC:
        raise ValueError("MaxSpeedDegPerSec must be between 1 and 5 deg/s")
    return Increment, Speed


def EstimatedMotionTimeSec(IncrementDeg: float, MaxSpeedDegPerSec: int) -> float:
    return abs(float(IncrementDeg)) / float(MaxSpeedDegPerSec) + (
        MOTION_COMPLETION_MARGIN_SEC
    )


def _RequireLiveAcknowledgements(Args) -> None:
    Required = (
        ("IUnderstandMotionWillOccur", "--i-understand-motion-will-occur"),
        ("IConfirmMotionAreaIsClear", "--i-confirm-motion-area-is-clear"),
        ("IConfirmAxisIsSupported", "--i-confirm-axis-is-supported"),
        ("IConfirmEmergencyStopIsReady", "--i-confirm-emergency-stop-is-ready"),
    )
    Missing = [Flag for Attribute, Flag in Required if getattr(Args, Attribute) is not True]
    if Missing:
        raise RuntimeError(
            "live motion requires all safety acknowledgements: "
            + ", ".join(Missing)
        )


def _ValidateCommonArguments(Args) -> tuple[float, int]:
    if not 1 <= int(Args.NodeId) <= 32:
        raise ValueError("NodeId must be between 1 and 32")
    if not 0.01 <= float(Args.TimeoutSec) <= 5.0:
        raise ValueError("TimeoutSec must be between 0.01 and 5.0")
    return ValidateMotionLimits(Args.IncrementDeg, Args.MaxSpeedDegPerSec)


def BuildDryRunPlan(Args) -> Stage5CPlan:
    """Build the exact three-command plan without touching hardware."""

    Increment, Speed = _ValidateCommonArguments(Args)
    Transport = GuardedTransport.X660GuardedMotionTransport(
        NodeId=Args.NodeId,
        TimeoutSec=Args.TimeoutSec,
    )
    Motion = Transport.Transact(
        MotionProtocol.BuildIncrementalPositionRequest(Increment, Speed)
    )
    Stop = Transport.Transact(MotionProtocol.BuildMotorStopRequest())
    ShutdownFrame = GuardedTransport.CanFrame(
        arbitration_id=GuardedTransport.RequestArbitrationId(Args.NodeId),
        data=Shutdown.BuildEmptyCommand(Shutdown.MOTOR_SHUTDOWN_COMMAND),
        is_extended_id=False,
    )
    return Stage5CPlan(
        MotionRequest=Motion.Request,
        StopRequest=Stop.Request,
        ShutdownRequest=ShutdownFrame,
        EstimatedMotionTimeSec=EstimatedMotionTimeSec(Increment, Speed),
    )


def ExecuteLive(
    Args,
    *,
    CanModule=None,
    BusFactory: Optional[Callable[..., Any]] = None,
    InterfaceChecker: Callable[[str], None] = Shutdown.CheckCanInterface,
    Sleep: Callable[[float], None] = time.sleep,
) -> Stage5CResult:
    """Execute one bounded jog using an injected/testable hardware boundary."""

    if Args.LiveMotion is not True:
        raise RuntimeError("ExecuteLive requires explicit live-motion mode")
    Increment, Speed = _ValidateCommonArguments(Args)
    _RequireLiveAcknowledgements(Args)
    InterfaceChecker(Args.Interface)

    if CanModule is None:
        try:
            import can as ImportedCan
        except Exception as Error:
            raise RuntimeError("python-can is required for live motion") from Error
        CanModule = ImportedCan
    if BusFactory is None:
        BusFactory = CanModule.interface.Bus

    Bus = BusFactory(channel=Args.Interface, interface="socketcan")
    MotionResult = None
    StopResult = None
    ShutdownReply = None
    PrimaryError = None
    StopError = None
    ShutdownError = None
    MotionAttempted = False
    try:
        Shutdown.DrainReceiveQueue(Bus)
        Transport = GuardedTransport.X660GuardedMotionTransport(
            NodeId=Args.NodeId,
            Bus=Bus,
            MessageFactory=CanModule.Message,
            TimeoutSec=Args.TimeoutSec,
            DryRun=False,
            IUnderstandMotionWillOccur=True,
            IConfirmMotionAreaIsClear=True,
        )
        try:
            MotionAttempted = True
            MotionResult = Transport.Transact(
                MotionProtocol.BuildIncrementalPositionRequest(Increment, Speed)
            )
            Sleep(EstimatedMotionTimeSec(Increment, Speed))
        except BaseException as Error:
            PrimaryError = Error
        finally:
            if MotionAttempted:
                try:
                    StopResult = Transport.Transact(
                        MotionProtocol.BuildMotorStopRequest()
                    )
                except BaseException as Error:
                    StopError = Error
                try:
                    ShutdownReply = Shutdown.Transact(
                        Bus,
                        CanModule,
                        Args.NodeId,
                        Shutdown.BuildEmptyCommand(
                            Shutdown.MOTOR_SHUTDOWN_COMMAND
                        ),
                        Args.TimeoutSec,
                    )
                    if ShutdownReply != Shutdown.BuildEmptyCommand(
                        Shutdown.MOTOR_SHUTDOWN_COMMAND
                    ):
                        raise RuntimeError("0x80 shutdown reply did not echo request")
                except BaseException as Error:
                    ShutdownError = Error
    finally:
        Bus.shutdown()

    if ShutdownError is not None:
        raise RuntimeError(
            "FAIL-SAFE ERROR: X6-60 motor shutdown was not confirmed"
        ) from ShutdownError
    if PrimaryError is not None:
        raise RuntimeError(
            "bounded motion transaction failed; motor shutdown was confirmed"
        ) from PrimaryError
    if StopError is not None:
        raise RuntimeError(
            "closed-loop stop failed; motor shutdown was confirmed"
        ) from StopError
    return Stage5CResult(
        MotionTransaction=MotionResult,
        StopTransaction=StopResult,
        ShutdownReply=ShutdownReply,
        EstimatedMotionTimeSec=EstimatedMotionTimeSec(Increment, Speed),
    )


def BuildArgumentParser() -> argparse.ArgumentParser:
    Parser = argparse.ArgumentParser(description=__doc__)
    Parser.add_argument("--interface", dest="Interface", default="can0")
    Parser.add_argument("--node-id", dest="NodeId", type=int, default=1)
    Parser.add_argument(
        "--increment-deg",
        dest="IncrementDeg",
        type=float,
        default=DEFAULT_INCREMENT_DEG,
    )
    Parser.add_argument(
        "--max-speed-deg-s",
        dest="MaxSpeedDegPerSec",
        type=int,
        default=DEFAULT_MAX_SPEED_DEG_PER_SEC,
    )
    Parser.add_argument(
        "--timeout-sec", dest="TimeoutSec", type=float, default=0.25
    )
    Parser.add_argument(
        "--live-motion",
        dest="LiveMotion",
        action="store_true",
        help="perform the bounded jog; omitted means dry-run preview only",
    )
    Parser.add_argument(
        "--i-understand-motion-will-occur",
        dest="IUnderstandMotionWillOccur",
        action="store_true",
    )
    Parser.add_argument(
        "--i-confirm-motion-area-is-clear",
        dest="IConfirmMotionAreaIsClear",
        action="store_true",
    )
    Parser.add_argument(
        "--i-confirm-axis-is-supported",
        dest="IConfirmAxisIsSupported",
        action="store_true",
    )
    Parser.add_argument(
        "--i-confirm-emergency-stop-is-ready",
        dest="IConfirmEmergencyStopIsReady",
        action="store_true",
    )
    return Parser


def Main(Arguments=None) -> int:
    Args = BuildArgumentParser().parse_args(Arguments)
    if not Args.LiveMotion:
        Plan = BuildDryRunPlan(Args)
        print("DRY RUN ONLY - no CAN interface opened and no frame transmitted")
        print(
            f"Motion: id=0x{Plan.MotionRequest.arbitration_id:03X} "
            f"data={bytes(Plan.MotionRequest.data).hex(' ').upper()}"
        )
        print(
            f"Stop:   id=0x{Plan.StopRequest.arbitration_id:03X} "
            f"data={bytes(Plan.StopRequest.data).hex(' ').upper()}"
        )
        print(
            f"Off:    id=0x{Plan.ShutdownRequest.arbitration_id:03X} "
            f"data={bytes(Plan.ShutdownRequest.data).hex(' ').upper()}"
        )
        print(f"Estimated bounded motion window: {Plan.EstimatedMotionTimeSec:.2f} s")
        return 0

    Result = ExecuteLive(Args)
    print("Bounded X6-60 incremental motion transaction completed")
    print(
        "Closed-loop stop acknowledged; motor output shutdown confirmed "
        f"after {Result.EstimatedMotionTimeSec:.2f} s motion window"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(Main())
