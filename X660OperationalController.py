"""Operational SocketCAN controller for the Vanguard X X6-60 azimuth axis.

This adapter joins the proven read-only telemetry controller and the guarded
Stage 5B motion transport behind the interface already used by
``PointingManager``. Public azimuth and rate are clockwise-positive. Raw
multi-turn commands are mapped with the calibrated ``DirectionSign``.

Selecting this controller is deliberately explicit. It will not open unless
motion is enabled and both installation-level acknowledgements are exactly
``True``. Once open, the acknowledgements apply for that controller session;
normal scan, track, nudge and stop commands do not require repeated prompts.
"""

from __future__ import annotations

from decimal import Decimal, ROUND_HALF_UP
import math
from typing import Any, Callable, Optional

import RunX660MotorShutdown as Shutdown
import X660GuardedMotionTransport as GuardedTransport
import X660MotionProtocol as MotionProtocol
import X660PlannerInitialiser as PlannerInitialiser
import X660PlannerProtocol as PlannerProtocol
from X660ReadOnlyController import X660ReadOnlyController


def Wrap360(AngleDeg: float) -> float:
    return float(AngleDeg) % 360.0


def SignedAngleDeltaDeg(TargetDeg: float, CurrentDeg: float) -> float:
    Delta = Wrap360(TargetDeg) - Wrap360(CurrentDeg)
    if Delta > 180.0:
        Delta -= 360.0
    elif Delta < -180.0:
        Delta += 360.0
    return Delta


def QuantizeHundredth(Value: float, Name: str) -> float:
    if isinstance(Value, bool):
        raise ValueError(f"{Name} must be numeric, not bool")
    try:
        Numeric = float(Value)
    except (TypeError, ValueError, OverflowError) as Error:
        raise ValueError(f"{Name} must be finite") from Error
    if not math.isfinite(Numeric):
        raise ValueError(f"{Name} must be finite")
    return float(
        Decimal(str(Numeric)).quantize(Decimal("0.01"), rounding=ROUND_HALF_UP)
    )


class X660OperationalController:
    """Real X6-60 azimuth controller used by PointingManager and the scheduler."""

    MotionCommandsEnabled = True
    UnlimitedAzimuth = True
    SupportsContinuousRotation = True
    TelemetryWhileStopped = True
    SupportsElevation = False

    def __init__(
        self,
        Interface: str = "can0",
        NodeId: int = 1,
        TimeoutSec: float = 0.25,
        QueryIntervalSec: float = 0.10,
        NorthRawAngleDeg: float = -361.53,
        DirectionSign: int = +1,
        MaxPanRateDegPerSec: float = 14.0,
        PositionCommandSpeedDegPerSec: int = 14,
        MaximumNudgeDeg: float = 10.0,
        PositionToleranceDeg: float = 0.75,
        MotionEnabled: bool = False,
        IUnderstandMotionWillOccur: bool = False,
        IConfirmMotionAreaIsClear: bool = False,
        Debug: bool = False,
        TelemetryController=None,
        MotionTransportFactory: Callable[..., Any] = (
            GuardedTransport.X660GuardedMotionTransport
        ),
        PlannerInitialiserFactory: Callable[..., Any] = (
            PlannerInitialiser.X660PlannerInitialiser
        ),
        RequiredPlannerValues=PlannerProtocol.VANGUARD_X_REQUIRED_VALUES,
        ShutdownTransaction: Callable[..., bytes] = Shutdown.Transact,
    ):
        self.Interface = str(Interface)
        self.NodeId = int(NodeId)
        self.TimeoutSec = float(TimeoutSec)
        self.NorthRawAngleDeg = float(NorthRawAngleDeg)
        self.DirectionSign = int(DirectionSign)
        self.MaxPanRateDegPerSec = abs(float(MaxPanRateDegPerSec))
        self.PositionCommandSpeedDegPerSec = int(PositionCommandSpeedDegPerSec)
        self.MaximumNudgeDeg = abs(float(MaximumNudgeDeg))
        self.PositionToleranceDeg = abs(float(PositionToleranceDeg))
        self.MotionEnabled = MotionEnabled
        self.IUnderstandMotionWillOccur = IUnderstandMotionWillOccur
        self.IConfirmMotionAreaIsClear = IConfirmMotionAreaIsClear
        self.Debug = bool(Debug)
        self.MotionTransportFactory = MotionTransportFactory
        self.PlannerInitialiserFactory = PlannerInitialiserFactory
        self.RequiredPlannerValues = tuple(
            PlannerProtocol.ValidatePlannerValue(Value)
            for Value in RequiredPlannerValues
        )
        self.ShutdownTransaction = ShutdownTransaction

        if self.DirectionSign not in (-1, 1):
            raise ValueError(
                "operational X6-60 requires calibrated DirectionSign +1 or -1"
            )
        if not 0.01 <= self.MaxPanRateDegPerSec <= 360.0:
            raise ValueError("MaxPanRateDegPerSec must be between 0.01 and 360")
        if not 1 <= self.PositionCommandSpeedDegPerSec <= int(
            self.MaxPanRateDegPerSec
        ):
            raise ValueError(
                "PositionCommandSpeedDegPerSec must be between 1 and the "
                "operational pan-rate limit"
            )
        if not 0.01 <= self.MaximumNudgeDeg <= 180.0:
            raise ValueError("MaximumNudgeDeg must be between 0.01 and 180")
        if not 0.01 <= self.PositionToleranceDeg <= 10.0:
            raise ValueError("PositionToleranceDeg must be between 0.01 and 10")
        if not callable(self.MotionTransportFactory):
            raise ValueError("MotionTransportFactory must be callable")
        if not callable(self.PlannerInitialiserFactory):
            raise ValueError("PlannerInitialiserFactory must be callable")
        if len(self.RequiredPlannerValues) != 4:
            raise ValueError("RequiredPlannerValues must contain four values")
        if not callable(self.ShutdownTransaction):
            raise ValueError("ShutdownTransaction must be callable")

        self.Telemetry = TelemetryController or X660ReadOnlyController(
            Interface=self.Interface,
            NodeId=self.NodeId,
            TimeoutSec=self.TimeoutSec,
            QueryIntervalSec=QueryIntervalSec,
            NorthRawAngleDeg=self.NorthRawAngleDeg,
            DirectionSign=self.DirectionSign,
            Debug=self.Debug,
        )
        self.Transport = None
        self.PlannerInitialisationResult = None
        self.MotorReady = False
        self.IsOpen = False
        self.MotionFaulted = False
        self.MotionMode = "closed"
        self.TargetAzimuthDeg: Optional[float] = None
        self.LastCommandName: Optional[str] = None
        self.State = self.Telemetry.GetLastKnownState()

    def _RequireSessionAuthorisation(self) -> None:
        if self.MotionEnabled is not True:
            raise RuntimeError("operational X6-60 requires MotionEnabled=True")
        if self.IUnderstandMotionWillOccur is not True:
            raise RuntimeError(
                "operational X6-60 requires IUnderstandMotionWillOccur=True"
            )
        if self.IConfirmMotionAreaIsClear is not True:
            raise RuntimeError(
                "operational X6-60 requires IConfirmMotionAreaIsClear=True"
            )

    def _RequireOpen(self) -> None:
        if not self.IsOpen or self.Transport is None:
            raise RuntimeError("operational X6-60 controller is not open")

    def _RequireMotionReady(self) -> None:
        self._RequireOpen()
        if not self.MotorReady:
            raise RuntimeError(
                "X6-60 motion is inhibited because planner initialisation "
                "has not been verified"
            )
        if self.MotionFaulted:
            raise RuntimeError(
                "X6-60 motion is fault-latched; close and reopen after inspection"
            )

    def Open(self):
        if self.IsOpen:
            return
        self._RequireSessionAuthorisation()
        self.Telemetry.Open()
        try:
            if not bool(getattr(self.Telemetry, "Calibrated", False)):
                raise RuntimeError("X6-60 telemetry direction is not calibrated")
            CanModule = self.Telemetry.CanModule
            Initialiser = self.PlannerInitialiserFactory(
                NodeId=self.NodeId,
                Bus=self.Telemetry.Bus,
                MessageFactory=CanModule.Message,
                TimeoutSec=self.TimeoutSec,
                RequiredValues=self.RequiredPlannerValues,
            )
            self.PlannerInitialisationResult = Initialiser.Initialise()
            self.Transport = self.MotionTransportFactory(
                NodeId=self.NodeId,
                Bus=self.Telemetry.Bus,
                MessageFactory=CanModule.Message,
                TimeoutSec=self.TimeoutSec,
                DryRun=False,
                IUnderstandMotionWillOccur=True,
                IConfirmMotionAreaIsClear=True,
            )
            self.IsOpen = True
            self.MotionFaulted = False
            self.MotionMode = "hold"
            self.State = self.Telemetry.Update()
            self.MotorReady = True
        except Exception:
            self.MotorReady = False
            self.PlannerInitialisationResult = None
            self.Transport = None
            self.Telemetry.Close()
            raise
        Corrected = self.PlannerInitialisationResult.CorrectedIndices
        CorrectionText = (
            "no writes required"
            if not Corrected
            else "corrected indexes "
            + ", ".join(f"0x{Index:02X}" for Index in Corrected)
        )
        print(
            "X6-60 operational motion enabled "
            f"on {self.Interface}, node {self.NodeId} "
            f"(clockwise-positive, limit={self.MaxPanRateDegPerSec:.2f} deg/s; "
            f"planner={self.RequiredPlannerValues[0]} verified, {CorrectionText})"
        )

    def _DirectBestEffortStop(self) -> None:
        if self.Transport is None:
            return
        try:
            self.Transport.Transact(MotionProtocol.BuildMotorStopRequest())
        except BaseException:
            pass

    def _TransactMotion(self, Payload: bytes, CommandName: str):
        self._RequireMotionReady()
        try:
            Result = self.Transport.Transact(Payload)
        except BaseException as Error:
            self.MotionFaulted = True
            self.MotionMode = "fault"
            self._DirectBestEffortStop()
            raise RuntimeError(
                f"X6-60 {CommandName} failed; stop attempted and motion fault latched"
            ) from Error
        self.LastCommandName = str(CommandName)
        return Result

    def _ShutdownMotorOutput(self) -> bytes:
        self._RequireOpen()
        Payload = Shutdown.BuildEmptyCommand(Shutdown.MOTOR_SHUTDOWN_COMMAND)
        Reply = self.ShutdownTransaction(
            self.Telemetry.Bus,
            self.Telemetry.CanModule,
            self.NodeId,
            Payload,
            self.TimeoutSec,
        )
        if bytes(Reply) != Payload:
            raise RuntimeError("X6-60 0x80 shutdown reply did not echo request")
        return bytes(Reply)

    def Close(self):
        if not self.IsOpen:
            self.Telemetry.Close()
            return
        StopError = None
        ShutdownError = None
        try:
            try:
                self.Transport.Transact(MotionProtocol.BuildMotorStopRequest())
            except BaseException as Error:
                StopError = Error
            try:
                self._ShutdownMotorOutput()
            except BaseException as Error:
                ShutdownError = Error
        finally:
            self.MotorReady = False
            self.IsOpen = False
            self.Transport = None
            self.MotionMode = "closed"
            self.Telemetry.Close()
        if ShutdownError is not None:
            raise RuntimeError(
                "FAIL-SAFE ERROR: X6-60 motor-output shutdown was not confirmed"
            ) from ShutdownError
        if StopError is not None:
            raise RuntimeError(
                "X6-60 closed-loop stop failed; motor-output shutdown was confirmed"
            ) from StopError

    def Stop(self):
        if not self.IsOpen:
            return None
        try:
            Result = self.Transport.Transact(MotionProtocol.BuildMotorStopRequest())
        except BaseException as Error:
            self.MotionFaulted = True
            self.MotionMode = "fault"
            raise RuntimeError("X6-60 closed-loop stop failed") from Error
        self.LastCommandName = "Stop"
        self.MotionMode = "hold"
        self.TargetAzimuthDeg = float(self.State.AzimuthDeg)
        self.State.AtTarget = True
        return Result

    def StopAndHold(self):
        return self.Stop()

    def CommandSlew(
        self,
        PanRateDegPerSec: float,
        TiltRateDegPerSec: float = 0.0,
    ):
        if abs(float(TiltRateDegPerSec)) > 0.0:
            raise ValueError("X6-60 operational controller has no elevation axis")
        Rate = QuantizeHundredth(PanRateDegPerSec, "PanRateDegPerSec")
        if abs(Rate) > self.MaxPanRateDegPerSec:
            raise ValueError(
                f"PanRateDegPerSec exceeds {self.MaxPanRateDegPerSec:g} deg/s limit"
            )
        if Rate == 0.0:
            return self.Stop()
        RawRate = QuantizeHundredth(
            self.DirectionSign * Rate,
            "RawRateDegPerSec",
        )
        Result = self._TransactMotion(
            MotionProtocol.BuildSpeedControlRequest(RawRate),
            "CommandSlew",
        )
        self.MotionMode = "slew"
        self.TargetAzimuthDeg = None
        self.State.AtTarget = False
        return Result

    def SetPanPositionNative(self, PanDeg: float):
        TargetBearing = Wrap360(PanDeg)
        State = self.Update()
        if State.RawAngleDeg is None:
            raise RuntimeError("X6-60 raw multi-turn angle is unavailable")
        DeltaBearing = SignedAngleDeltaDeg(TargetBearing, State.AzimuthDeg)
        TargetRawAngle = QuantizeHundredth(
            float(State.RawAngleDeg) + self.DirectionSign * DeltaBearing,
            "TargetRawAngleDeg",
        )
        Result = self._TransactMotion(
            MotionProtocol.BuildAbsolutePositionRequest(
                TargetRawAngle,
                self.PositionCommandSpeedDegPerSec,
            ),
            "SetPanPositionNative",
        )
        self.MotionMode = "goto"
        self.TargetAzimuthDeg = TargetBearing
        self.State.AtTarget = False
        return Result

    def NudgePanPositionNative(self, IncrementDeg: float):
        Increment = QuantizeHundredth(IncrementDeg, "IncrementDeg")
        if abs(Increment) <= 0.0 or abs(Increment) > self.MaximumNudgeDeg:
            raise ValueError(
                f"IncrementDeg magnitude must be greater than zero and no more "
                f"than {self.MaximumNudgeDeg:g} degrees"
            )
        State = self.Update()
        RawIncrement = QuantizeHundredth(
            self.DirectionSign * Increment,
            "RawIncrementDeg",
        )
        Result = self._TransactMotion(
            MotionProtocol.BuildIncrementalPositionRequest(
                RawIncrement,
                self.PositionCommandSpeedDegPerSec,
            ),
            "NudgePanPositionNative",
        )
        self.MotionMode = "goto"
        self.TargetAzimuthDeg = Wrap360(State.AzimuthDeg + Increment)
        self.State.AtTarget = False
        return Result

    def SetTiltPositionNative(self, TiltDeg: float):
        if self.Debug:
            print(
                "X6-60 elevation command ignored: Vanguard X X6-60 is azimuth-only"
            )
        return None

    def CommandPosition(self, AzimuthDeg: float, ElevationDeg: float = 0.0):
        return self.SetPanPositionNative(AzimuthDeg)

    def Update(self):
        State = self.Telemetry.Update()
        self.State = State
        if int(getattr(State, "ErrorFlags", 0)) != 0:
            if self.IsOpen and not self.MotionFaulted:
                self._DirectBestEffortStop()
            self.MotionFaulted = True
            self.MotionMode = "fault"
            State.Valid = False
            State.AtTarget = False
            raise RuntimeError(
                f"X6-60 reported error flags 0x{int(State.ErrorFlags):04X}; "
                "stop attempted and motion fault latched"
            )

        if self.MotionMode == "goto" and self.TargetAzimuthDeg is not None:
            ErrorDeg = abs(
                SignedAngleDeltaDeg(self.TargetAzimuthDeg, State.AzimuthDeg)
            )
            State.AtTarget = bool(
                ErrorDeg <= self.PositionToleranceDeg
                and abs(float(State.PanRateDegPerSec)) <= 0.25
            )
            if State.AtTarget:
                self.MotionMode = "hold"
        elif self.MotionMode == "slew":
            State.AtTarget = False
        elif self.MotionMode == "hold":
            State.AtTarget = True
        else:
            State.AtTarget = False
        return State

    def GetState(self):
        return self.State

    def GetLastKnownState(self):
        return self.State

    def QueryPanPosition(self) -> float:
        return float(self.Update().AzimuthDeg)

    def QueryAzEl(self):
        State = self.Update()
        return float(State.AzimuthDeg), float(State.ElevationDeg)
