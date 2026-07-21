"""Read-only SocketCAN adapter for the Vanguard X X6-60.

This controller deliberately implements no CAN command capable of motion,
brake control, configuration, shutdown, or zero-offset changes.  It adapts the
proven Stage 4A telemetry reads to the public interface used by
``PointingManager`` so the full scheduler can be launched against the real
X6-60 without authorising movement.
"""

from __future__ import annotations

from dataclasses import dataclass
import subprocess
import time
from typing import Optional


@dataclass
class X660ReadOnlyState:
    AzimuthDeg: float = 0.0
    ElevationDeg: float = 0.0
    Valid: bool = False
    PanRateDegPerSec: float = 0.0
    Direction: str = "stopped"
    AtTarget: bool = False
    Source: str = "X660_CAN_READ_ONLY_NOT_OPEN"
    TimestampSec: float = 0.0
    RawAngleDeg: Optional[float] = None
    TorqueCurrentA: Optional[float] = None
    ErrorFlags: int = 0


def RawAngleToAzimuthDeg(
    RawAngleDeg: float,
    NorthRawAngleDeg: float,
    DirectionSign: int,
) -> float:
    """Convert raw multi-turn angle to clockwise-from-North azimuth.

    ``DirectionSign`` is +1 when increasing raw angle is clockwise and -1
    when decreasing raw angle is clockwise.  Stage 4B must establish this sign
    before the conversion is treated as calibrated.
    """

    Sign = int(DirectionSign)
    if Sign not in (-1, 1):
        raise ValueError("DirectionSign must be +1 or -1")
    return (Sign * (float(RawAngleDeg) - float(NorthRawAngleDeg))) % 360.0


class X660ReadOnlyController:
    """Telemetry-only X6-60 controller compatible with PointingManager."""

    MotionCommandsEnabled = False
    UnlimitedAzimuth = True
    SupportsContinuousRotation = True
    TelemetryWhileStopped = True

    def __init__(
        self,
        Interface: str = "can0",
        NodeId: int = 1,
        TimeoutSec: float = 0.25,
        QueryIntervalSec: float = 0.10,
        NorthRawAngleDeg: float = -361.53,
        DirectionSign: int = 0,
        Debug: bool = False,
    ):
        self.Interface = str(Interface)
        self.NodeId = int(NodeId)
        self.TimeoutSec = float(TimeoutSec)
        self.QueryIntervalSec = max(0.0, float(QueryIntervalSec))
        self.NorthRawAngleDeg = float(NorthRawAngleDeg)
        self.DirectionSign = int(DirectionSign)
        self.Debug = bool(Debug)

        if not self.Interface or any(Char.isspace() for Char in self.Interface):
            raise ValueError("Interface must be one non-empty device name")
        if not 1 <= self.NodeId <= 32:
            raise ValueError("NodeId must be between 1 and 32")
        if not 0.01 <= self.TimeoutSec <= 5.0:
            raise ValueError("TimeoutSec must be between 0.01 and 5.0")
        if self.DirectionSign not in (-1, 0, 1):
            raise ValueError("DirectionSign must be -1, 0 (uncalibrated), or +1")

        self.Bus = None
        self.CanModule = None
        self.Protocol = None
        self.IsOpen = False
        self.Calibrated = self.DirectionSign in (-1, 1)
        self.LastQueryMonotonicSec = 0.0
        self.LastStatus1MonotonicSec = 0.0
        self.FirmwareDate = None
        self.BusVoltageV = None
        self.TemperatureC = None
        self.BrakeReleased = None
        self.State = X660ReadOnlyState()
        self._RefusalPrinted = set()

    def _CheckCanInterface(self) -> None:
        Result = subprocess.run(
            ["ip", "-details", "link", "show", "dev", self.Interface],
            capture_output=True,
            text=True,
            check=False,
        )
        if Result.returncode != 0:
            raise RuntimeError(
                f"CAN interface {self.Interface!r} was not found: "
                f"{Result.stderr.strip()}"
            )
        if "state UP" not in Result.stdout:
            raise RuntimeError(f"CAN interface {self.Interface!r} is not UP")
        if "bitrate 1000000" not in Result.stdout:
            raise RuntimeError(
                f"CAN interface {self.Interface!r} is not reporting 1 Mbit/s"
            )

    def Open(self):
        if self.IsOpen:
            return

        try:
            import can
            import X660CanProtocol as Protocol
        except Exception as Error:
            raise RuntimeError(
                "X6-60 read-only controller requires python-can and "
                "X660CanProtocol.py from Stage 4A"
            ) from Error

        Protocol.ValidateNodeId(self.NodeId)
        self._CheckCanInterface()
        self.CanModule = can
        self.Protocol = Protocol
        self.Bus = can.interface.Bus(
            channel=self.Interface,
            interface="socketcan",
        )

        try:
            self._DrainReceiveQueue()
            VersionData = self._TransactRead(Protocol.READ_SOFTWARE_VERSION)
            Status1Data = self._TransactRead(Protocol.READ_STATUS_1)
            self.FirmwareDate = Protocol.DecodeSoftwareVersion(VersionData)
            self._ApplyStatus1(Protocol.DecodeStatus1(Status1Data))
            self.IsOpen = True
            self._RefreshTelemetry(force=True)
        except Exception:
            if self.Bus is not None:
                self.Bus.shutdown()
            self.Bus = None
            self.IsOpen = False
            raise

        CalibrationText = "calibrated" if self.Calibrated else "UNCALIBRATED"
        print(
            "X6-60 SocketCAN telemetry opened "
            f"on {self.Interface}, node {self.NodeId} ({CalibrationText}; motion locked)"
        )

    def Close(self):
        if self.Bus is not None:
            self.Bus.shutdown()
        self.Bus = None
        self.IsOpen = False
        self.State.Valid = False
        self.State.Source = "X660_CAN_READ_ONLY_CLOSED"

    def _DrainReceiveQueue(self, MaximumFrames: int = 1000) -> int:
        Count = 0
        while Count < int(MaximumFrames):
            Message = self.Bus.recv(timeout=0.0)
            if Message is None:
                break
            Count += 1
        if Count >= int(MaximumFrames):
            raise RuntimeError("CAN receive queue did not drain")
        return Count

    def _TransactRead(self, Command: int) -> bytes:
        Protocol = self.Protocol
        Request = self.CanModule.Message(
            arbitration_id=Protocol.RequestArbitrationId(self.NodeId),
            data=Protocol.BuildReadRequest(Command),
            is_extended_id=False,
        )
        self.Bus.send(Request, timeout=self.TimeoutSec)
        Deadline = time.monotonic() + self.TimeoutSec

        while True:
            Remaining = Deadline - time.monotonic()
            if Remaining <= 0.0:
                raise TimeoutError(
                    f"Timed out waiting for X6-60 command 0x{Command:02X}"
                )
            Reply = self.Bus.recv(timeout=Remaining)
            if Reply is None:
                continue
            if (
                int(Reply.arbitration_id)
                != Protocol.ReplyArbitrationId(self.NodeId)
                or bool(Reply.is_extended_id)
            ):
                continue
            return Protocol.ValidateReply(
                self.NodeId,
                Command,
                Reply.arbitration_id,
                Reply.data,
                Reply.is_extended_id,
            )

    def _ApplyStatus1(self, Status1) -> None:
        self.BusVoltageV = float(Status1.BusVoltageV)
        self.TemperatureC = int(Status1.TemperatureC)
        self.BrakeReleased = bool(Status1.BrakeReleased)
        self.State.ErrorFlags = int(Status1.ErrorFlags)

    def _RefreshTelemetry(self, force: bool = False) -> X660ReadOnlyState:
        if not self.IsOpen:
            raise RuntimeError("X6-60 read-only controller is not open")

        Now = time.monotonic()
        if (
            not force
            and self.LastQueryMonotonicSec > 0.0
            and Now - self.LastQueryMonotonicSec < self.QueryIntervalSec
        ):
            return self.State

        Protocol = self.Protocol
        RawAngle = Protocol.DecodeMultiTurnAngleDeg(
            self._TransactRead(Protocol.READ_MULTI_TURN_ANGLE)
        )
        Status2 = Protocol.DecodeStatus2(
            self._TransactRead(Protocol.READ_STATUS_2)
        )

        if Now - self.LastStatus1MonotonicSec >= 1.0:
            self._ApplyStatus1(Protocol.DecodeStatus1(
                self._TransactRead(Protocol.READ_STATUS_1)
            ))
            self.LastStatus1MonotonicSec = Now

        MappingSign = self.DirectionSign if self.Calibrated else 1
        Azimuth = RawAngleToAzimuthDeg(
            RawAngle,
            self.NorthRawAngleDeg,
            MappingSign,
        )
        RawRate = float(Status2.SpeedDegPerSec)
        PanRate = MappingSign * RawRate
        if PanRate > 0.01:
            Direction = "right"
        elif PanRate < -0.01:
            Direction = "left"
        else:
            Direction = "stopped"

        self.State.AzimuthDeg = Azimuth
        self.State.PanRateDegPerSec = PanRate
        self.State.RawAngleDeg = float(RawAngle)
        self.State.TorqueCurrentA = float(Status2.TorqueCurrentA)
        self.State.TimestampSec = time.time()
        self.State.Direction = Direction
        self.State.AtTarget = False
        self.State.Valid = bool(
            self.Calibrated and self.State.ErrorFlags == 0
        )
        self.State.Source = (
            "X660_CAN_READ_ONLY"
            if self.Calibrated
            else "X660_CAN_READ_ONLY_UNCALIBRATED"
        )
        self.LastQueryMonotonicSec = Now

        if self.Debug:
            print(
                "X6-60 telemetry: "
                f"raw={RawAngle:+.2f} deg, az={Azimuth:.2f} deg, "
                f"speed={PanRate:+.2f} deg/s, valid={self.State.Valid}"
            )
        return self.State

    def Update(self) -> X660ReadOnlyState:
        return self._RefreshTelemetry()

    def GetState(self) -> X660ReadOnlyState:
        return self.State

    def GetLastKnownState(self) -> X660ReadOnlyState:
        return self.State

    def QueryPanPosition(self) -> float:
        return float(self.Update().AzimuthDeg)

    def QueryAzEl(self):
        State = self.Update()
        return float(State.AzimuthDeg), float(State.ElevationDeg)

    def _RefuseMotion(self, CommandName: str) -> None:
        self.State.Valid = False
        self.State.AtTarget = False
        self.State.Source = "X660_CAN_READ_ONLY_MOTION_LOCKED"
        if CommandName not in self._RefusalPrinted:
            print(
                f"X6-60 {CommandName} refused: Stage 4B controller is telemetry-only"
            )
            self._RefusalPrinted.add(CommandName)

    def Stop(self):
        # Deliberately no CAN stop/shutdown/brake command.  Stage 4A established
        # that an unsolicited stop may change holding behaviour.
        return None

    def StopAndHold(self):
        return self.Stop()

    def CommandSlew(self, PanRateDegPerSec: float, TiltRateDegPerSec: float = 0.0):
        self._RefuseMotion("CommandSlew")

    def SetPanPositionNative(self, PanDeg: float):
        self._RefuseMotion("SetPanPositionNative")

    def NudgePanPositionNative(self, IncrementDeg: float):
        self._RefuseMotion("NudgePanPositionNative")

    def SetTiltPositionNative(self, TiltDeg: float):
        self._RefuseMotion("SetTiltPositionNative")

    def CommandPosition(self, AzimuthDeg: float, ElevationDeg: float = 0.0):
        self._RefuseMotion("CommandPosition")
