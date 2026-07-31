#!/usr/bin/env python3
"""X6-60 controller factory and software simulator for Vanguard X.

The simulator models the X6-60 as an unlimited multi-turn azimuth axis.  Its
public azimuth is always a clockwise-from-North bearing in the range
0 <= azimuth < 360 degrees, while
``RawAngleDeg`` preserves the continuous multi-turn position.
"""

from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Optional, Tuple


def Wrap360(AngleDeg: float) -> float:
    return float(AngleDeg) % 360.0


def SignedAngleDeltaDeg(TargetDeg: float, CurrentDeg: float) -> float:
    Delta = Wrap360(TargetDeg) - Wrap360(CurrentDeg)
    if Delta > 180.0:
        Delta -= 360.0
    elif Delta < -180.0:
        Delta += 360.0
    return Delta


def Clamp(Value: float, Minimum: float, Maximum: float) -> float:
    return max(float(Minimum), min(float(Maximum), float(Value)))


@dataclass
class X660State:
    AzimuthDeg: float = 0.0
    ElevationDeg: float = 0.0
    CommandedAzimuthDeg: float = 0.0
    CommandedElevationDeg: float = 0.0
    PanRateDegPerSec: float = 0.0
    Direction: str = "stopped"
    AtTarget: bool = False
    Valid: bool = False
    Source: str = "X660_NOT_READY"
    TimestampSec: float = 0.0
    RawAngleDeg: Optional[float] = None


class SimulatedX660Controller:
    """Unlimited multi-turn X6-60 simulator used by software regressions."""

    MotionCommandsEnabled = True
    UnlimitedAzimuth = True
    SupportsContinuousRotation = True
    TelemetryWhileStopped = True

    def __init__(
        self,
        InitialAzimuthDeg: float = 0.0,
        InitialElevationDeg: float = 0.0,
        MaxPanRateDegPerSec: float = 14.0,
        PositionToleranceDeg: float = 0.5,
        NorthRawAngleDeg: float = -361.53,
        DirectionSign: int = 1,
        Debug: bool = False,
    ):
        self.MaxPanRateDegPerSec = abs(float(MaxPanRateDegPerSec))
        self.PositionToleranceDeg = abs(float(PositionToleranceDeg))
        self.NorthRawAngleDeg = float(NorthRawAngleDeg)
        self.DirectionSign = int(DirectionSign)
        self.Debug = bool(Debug)

        if self.MaxPanRateDegPerSec <= 0.0:
            raise ValueError("MaxPanRateDegPerSec must be greater than zero")
        if self.DirectionSign not in (-1, 1):
            raise ValueError("DirectionSign must be +1 or -1")

        InitialAzimuth = Wrap360(InitialAzimuthDeg)
        self.UnwrappedAzimuthDeg = float(InitialAzimuth)
        self.TargetUnwrappedAzimuthDeg = float(InitialAzimuth)
        self.CommandedRateDegPerSec = 0.0
        self.GotoRateDegPerSec = self.MaxPanRateDegPerSec
        self.Mode = "stopped"
        self.LastUpdateSec = time.time()

        self.State = X660State(
            AzimuthDeg=InitialAzimuth,
            ElevationDeg=Clamp(InitialElevationDeg, -90.0, 90.0),
            CommandedAzimuthDeg=InitialAzimuth,
            CommandedElevationDeg=Clamp(InitialElevationDeg, -90.0, 90.0),
            AtTarget=True,
            Valid=True,
            Source="X660_SIMULATED",
            TimestampSec=self.LastUpdateSec,
        )
        self._UpdateRawAngle()

    def _UpdateRawAngle(self) -> None:
        self.State.RawAngleDeg = (
            self.NorthRawAngleDeg
            + self.DirectionSign * self.UnwrappedAzimuthDeg
        )

    def Open(self):
        print(
            "Simulated X6-60 opened "
            f"(initial={self.State.AzimuthDeg:.2f} deg, "
            f"rate={self.MaxPanRateDegPerSec:.2f} deg/s, unlimited azimuth)"
        )

    def Close(self):
        self.Stop()
        print("Simulated X6-60 closed")

    def Stop(self):
        self.Mode = "stopped"
        self.CommandedRateDegPerSec = 0.0
        self.State.PanRateDegPerSec = 0.0
        self.State.Direction = "stopped"
        self.State.AtTarget = True

    def StopAndHold(self):
        self.Stop()

    def SetPanPositionNative(self, PanDeg: float):
        return self.SetPanPositionNativeAtRate(
            PanDeg,
            self.MaxPanRateDegPerSec,
        )

    def SetPanPositionNativeAtRate(
        self,
        PanDeg: float,
        PanRateDegPerSec: float,
    ):
        TargetBearing = Wrap360(PanDeg)
        Delta = SignedAngleDeltaDeg(TargetBearing, self.State.AzimuthDeg)
        self.TargetUnwrappedAzimuthDeg = self.UnwrappedAzimuthDeg + Delta
        self.GotoRateDegPerSec = Clamp(
            abs(float(PanRateDegPerSec)),
            0.01,
            self.MaxPanRateDegPerSec,
        )
        self.State.CommandedAzimuthDeg = TargetBearing
        self.State.AtTarget = False
        self.Mode = "goto"

        if self.Debug:
            print(f"X6-60 SIM goto {TargetBearing:.2f} deg")

    def NudgePanPositionNative(self, IncrementDeg: float):
        self.TargetUnwrappedAzimuthDeg = (
            self.UnwrappedAzimuthDeg + float(IncrementDeg)
        )
        self.State.CommandedAzimuthDeg = Wrap360(
            self.TargetUnwrappedAzimuthDeg
        )
        self.State.AtTarget = False
        self.Mode = "goto"
        return True

    def SetTiltPositionNative(self, TiltDeg: float):
        Target = Clamp(TiltDeg, -90.0, 90.0)
        self.State.ElevationDeg = Target
        self.State.CommandedElevationDeg = Target

    def CommandPosition(self, AzimuthDeg: float, ElevationDeg: float = 0.0):
        self.SetPanPositionNative(AzimuthDeg)
        self.SetTiltPositionNative(ElevationDeg)

    def CommandSlew(
        self,
        PanRateDegPerSec: float,
        TiltRateDegPerSec: float = 0.0,
    ):
        Rate = Clamp(
            PanRateDegPerSec,
            -self.MaxPanRateDegPerSec,
            self.MaxPanRateDegPerSec,
        )
        if abs(Rate) <= 0.0:
            self.Stop()
            return

        self.CommandedRateDegPerSec = Rate
        self.Mode = "slew"
        self.State.AtTarget = False
        self.State.PanRateDegPerSec = Rate
        self.State.Direction = "right" if Rate > 0.0 else "left"

    def QueryAzEl(self) -> Tuple[float, float]:
        State = self.Update()
        return float(State.AzimuthDeg), float(State.ElevationDeg)

    def QueryPanPosition(self) -> float:
        return float(self.Update().AzimuthDeg)

    def Update(self) -> X660State:
        Now = time.time()
        Dt = max(0.0, min(Now - self.LastUpdateSec, 0.25))
        self.LastUpdateSec = Now

        if self.Mode == "slew":
            self.UnwrappedAzimuthDeg += self.CommandedRateDegPerSec * Dt
            self.State.PanRateDegPerSec = self.CommandedRateDegPerSec
            self.State.Direction = (
                "right" if self.CommandedRateDegPerSec > 0.0 else "left"
            )
            self.State.AtTarget = False

        elif self.Mode == "goto":
            Error = self.TargetUnwrappedAzimuthDeg - self.UnwrappedAzimuthDeg
            if abs(Error) <= self.PositionToleranceDeg:
                self.UnwrappedAzimuthDeg = self.TargetUnwrappedAzimuthDeg
                self.Stop()
            else:
                Direction = 1.0 if Error > 0.0 else -1.0
                Step = Direction * self.GotoRateDegPerSec * Dt
                if abs(Step) > abs(Error):
                    Step = Error
                self.UnwrappedAzimuthDeg += Step
                self.State.PanRateDegPerSec = (
                    Direction * self.GotoRateDegPerSec
                )
                self.State.Direction = "right" if Direction > 0.0 else "left"

        else:
            self.State.PanRateDegPerSec = 0.0
            self.State.Direction = "stopped"

        self.State.AzimuthDeg = Wrap360(self.UnwrappedAzimuthDeg)
        self.State.Valid = True
        self.State.Source = "X660_SIMULATED"
        self.State.TimestampSec = Now
        self._UpdateRawAngle()
        return self.State

    def GetState(self) -> X660State:
        return self.State

    def GetLastKnownState(self) -> X660State:
        return self.State


def CreateX660Controller(Config):
    """Create the selected X6-60 implementation."""

    Mode = str(Config.get("X660Mode", "x660-read-only")).lower()

    if Mode in ("sim", "x660-sim", "x6-60-sim", "simulated"):
        return SimulatedX660Controller(
            InitialAzimuthDeg=float(Config.get("InitialBeamAngleDeg", 0.0)),
            InitialElevationDeg=float(
                Config.get("X660StartupElevationDeg", 60.0)
            ),
            MaxPanRateDegPerSec=float(
                Config.get("X660SimMaxRateDegPerSec", 14.0)
            ),
            PositionToleranceDeg=float(
                Config.get("X660PositionToleranceDeg", 0.75)
            ),
            NorthRawAngleDeg=float(
                Config.get("X660NorthRawAngleDeg", -361.53)
            ),
            DirectionSign=int(Config.get("X660DirectionSign", 1)),
            Debug=bool(Config.get("X660Debug", False)),
        )

    if Mode in (
        "x660-read-only",
        "x6-60-read-only",
        "x660-telemetry",
        "x6-60-telemetry",
    ):
        from X660ReadOnlyController import X660ReadOnlyController

        return X660ReadOnlyController(
            Interface=str(Config.get("X660CanInterface", "can0")),
            NodeId=int(Config.get("X660NodeId", 1)),
            TimeoutSec=float(Config.get("X660CanTimeoutSec", 0.25)),
            QueryIntervalSec=float(
                Config.get("X660TelemetryIntervalSec", 0.10)
            ),
            NorthRawAngleDeg=float(
                Config.get("X660NorthRawAngleDeg", -361.53)
            ),
            DirectionSign=int(Config.get("X660DirectionSign", 0)),
            Debug=bool(Config.get("X660Debug", False)),
        )

    if Mode in (
        "x660-operational",
        "x6-60-operational",
        "x660-live",
        "x6-60-live",
    ):
        from X660OperationalController import X660OperationalController

        return X660OperationalController(
            Interface=str(Config.get("X660CanInterface", "can0")),
            NodeId=int(Config.get("X660NodeId", 1)),
            TimeoutSec=float(Config.get("X660CanTimeoutSec", 0.25)),
            QueryIntervalSec=float(
                Config.get("X660TelemetryIntervalSec", 0.10)
            ),
            NorthRawAngleDeg=float(
                Config.get("X660NorthRawAngleDeg", -361.53)
            ),
            DirectionSign=int(Config.get("X660DirectionSign", 0)),
            MaxPanRateDegPerSec=float(
                Config.get("X660OperationalMaxRateDegPerSec", 14.0)
            ),
            PositionCommandSpeedDegPerSec=int(
                Config.get("X660PositionCommandSpeedDegPerSec", 14)
            ),
            MaximumNudgeDeg=float(
                Config.get("X660MaximumNudgeDeg", 10.0)
            ),
            PositionToleranceDeg=float(
                Config.get("X660PositionToleranceDeg", 0.75)
            ),
            RequiredPlannerValues=(int(
                Config.get("X660PlannerInternalAccelerationDegPerSec2", 1140)
            ),) * 4,
            MotionEnabled=Config.get("X660MotionEnabled", False),
            IUnderstandMotionWillOccur=Config.get(
                "X660IUnderstandMotionWillOccur",
                False,
            ),
            IConfirmMotionAreaIsClear=Config.get(
                "X660IConfirmMotionAreaIsClear",
                False,
            ),
            Debug=bool(Config.get("X660Debug", False)),
        )

    raise ValueError(
        f"Unknown X660Mode: {Mode!r}. Expected 'x660-read-only', "
        "'x660-operational', or 'x660-sim'."
    )
