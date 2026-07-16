#!/usr/bin/env python3
"""
PTZController.py

Vanguard X PTZ controller.

This version is deliberately simple for hardware bring-up:

- Real Pelco-D native pan position command is supported.
- Query pan position is supported.
- Manual nudge uses native position command, not timed slew.
- Scan mode should command one absolute target and wait for the PTZ to reach it.
- No software servo/hunting loop is used.

Confirmed on hardware:
    Query pan:  00 51 -> response 00 59
    Set pan:    00 4B Data1 Data2
    Units:      raw = degrees * 100
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional, Tuple

try:
    import serial
except Exception:
    serial = None


def Wrap360(AngleDeg: float) -> float:
    return float(AngleDeg % 360.0)


def SignedAngleDeltaDeg(TargetDeg: float, CurrentDeg: float) -> float:
    Delta = float(TargetDeg) - float(CurrentDeg)

    while Delta > 180.0:
        Delta -= 360.0

    while Delta < -180.0:
        Delta += 360.0

    return Delta


def Clamp(Value: float, MinValue: float, MaxValue: float) -> float:
    return max(float(MinValue), min(float(MaxValue), float(Value)))


def ClampToSoftwareWindow(AzDeg: float, LeftLimitDeg: float, RightLimitDeg: float) -> float:
    """
    Vanguard current PTZ software window is non-wrapping:
        10 deg <= Az <= 300 deg
    """
    AzDeg = Wrap360(AzDeg)
    return Clamp(AzDeg, LeftLimitDeg, RightLimitDeg)


@dataclass
class PTZState:
    AzimuthDeg: float = 0.0
    ElevationDeg: float = 0.0
    CommandedAzimuthDeg: float = 0.0
    CommandedElevationDeg: float = 0.0
    PanRateDegPerSec: float = 0.0
    Direction: str = "stopped"
    AtLeftLimit: bool = False
    AtRightLimit: bool = False
    Valid: bool = False
    Source: str = "PTZ_NOT_READY"
    TimestampSec: float = 0.0


class SimulatedPTZController:
    """
    Software PTZ / drone-yaw simulator.

    This class implements the same public interface as PelcoDPTZController:

        Open()
        Close()
        Stop()
        CommandSlew(rate_deg_per_sec)
        SetPanPositionNative(target_deg)
        CommandPosition(target_deg)
        Update()
        GetState()

    Modes:
        - slew mode: CommandSlew() moves continuously at the commanded rate.
        - goto mode: SetPanPositionNative() slews toward a target and stops.
        - stopped: no motion.

    This lets Vanguard X run without the real PTZ attached, and it also gives
    us a future place to emulate drone yaw scanning.
    """

    def __init__(
        self,
        LeftLimitDeg=10.0,
        RightLimitDeg=300.0,
        InitialAzimuthDeg=230.0,
        InitialElevationDeg=0.0,
        MaxPanRateDegPerSec=14.0,
        PositionToleranceDeg=0.5,
        QueryIntervalSec=0.10,
        WrapMode=False,
        Debug=False,
        **kwargs,
    ):
        self.LeftLimitDeg = float(LeftLimitDeg)
        self.RightLimitDeg = float(RightLimitDeg)
        self.MaxPanRateDegPerSec = abs(float(MaxPanRateDegPerSec))
        self.PositionToleranceDeg = float(PositionToleranceDeg)
        self.QueryIntervalSec = float(QueryIntervalSec)
        self.WrapMode = bool(WrapMode)
        self.Debug = bool(Debug)
        InitialElevationDeg = Clamp(float(InitialElevationDeg), -90.0, 90.0)

        InitialAzimuthDeg = ClampToSoftwareWindow(
            InitialAzimuthDeg,
            self.LeftLimitDeg,
            self.RightLimitDeg,
        )

        self.State = PTZState(
            AzimuthDeg=InitialAzimuthDeg,
            ElevationDeg=InitialElevationDeg,
            CommandedAzimuthDeg=InitialAzimuthDeg,
            CommandedElevationDeg=InitialElevationDeg,
            Valid=True,
            Source="SIMULATED_PTZ",
            TimestampSec=time.time(),
        )

        self.Mode = "stopped"      # stopped, slew, goto
        self.TargetAzimuthDeg = InitialAzimuthDeg
        self.CommandedRateDegPerSec = 0.0
        self.LastUpdateSec = time.time()
        self.LastQueryTimeSec = 0.0

    def Open(self):
        print(
            "Simulated PTZ opened "
            f"(initial={self.State.AzimuthDeg:.2f} deg, "
            f"rate={self.MaxPanRateDegPerSec:.2f} deg/s)"
        )

    def Close(self):
        self.Stop()
        print("Simulated PTZ closed")

    def Stop(self):
        self.Mode = "stopped"
        self.CommandedRateDegPerSec = 0.0
        self.State.PanRateDegPerSec = 0.0
        self.State.Direction = "stopped"

    def _ClampOrWrap(self, AngleDeg):
        if self.WrapMode:
            return Wrap360(AngleDeg)

        return ClampToSoftwareWindow(
            AngleDeg,
            self.LeftLimitDeg,
            self.RightLimitDeg,
        )

    def _ApplyLimits(self):
        if self.WrapMode:
            self.State.AtLeftLimit = False
            self.State.AtRightLimit = False
            return

        if self.State.AzimuthDeg <= self.LeftLimitDeg:
            self.State.AzimuthDeg = self.LeftLimitDeg
            self.State.AtLeftLimit = True
            if self.CommandedRateDegPerSec < 0.0:
                self.Stop()
        else:
            self.State.AtLeftLimit = False

        if self.State.AzimuthDeg >= self.RightLimitDeg:
            self.State.AzimuthDeg = self.RightLimitDeg
            self.State.AtRightLimit = True
            if self.CommandedRateDegPerSec > 0.0:
                self.Stop()
        else:
            self.State.AtRightLimit = False

    def SetPanPositionNative(self, PanDeg: float):
        Target = self._ClampOrWrap(PanDeg)
        self.TargetAzimuthDeg = Target
        self.State.CommandedAzimuthDeg = Target
        self.Mode = "goto"

        if self.Debug:
            print(f"SIM PTZ goto {Target:.2f} deg")

    def ScanToEndpoint(self, EndpointDeg: float):
        self.SetPanPositionNative(EndpointDeg)

    def NudgePanPositionNative(self, IncrementDeg: float):
        self.SetPanPositionNative(self.State.AzimuthDeg + float(IncrementDeg))
        return True

    def SetTiltPositionNative(self, TiltDeg: float):
        Target = Clamp(float(TiltDeg), -90.0, 90.0)
        self.State.ElevationDeg = Target
        self.State.CommandedElevationDeg = Target

        if self.Debug:
            print(f"SIM PTZ tilt {Target:.2f} deg")

    def CommandPosition(self, AzimuthDeg: float, ElevationDeg: float = 0.0):
        self.SetPanPositionNative(AzimuthDeg)
        self.SetTiltPositionNative(ElevationDeg)

    def CommandSlew(self, PanRateDegPerSec: float, TiltRateDegPerSec: float = 0.0):
        Rate = float(PanRateDegPerSec)

        if Rate > 0.0:
            Rate = min(abs(Rate), self.MaxPanRateDegPerSec)
        elif Rate < 0.0:
            Rate = -min(abs(Rate), self.MaxPanRateDegPerSec)
        else:
            self.Stop()
            return

        self.CommandedRateDegPerSec = Rate
        self.Mode = "slew"
        self.State.PanRateDegPerSec = Rate
        self.State.Direction = "right" if Rate > 0.0 else "left"

        if self.Debug:
            print(f"SIM PTZ slew {self.State.Direction} at {abs(Rate):.2f} deg/s")

    def QueryAzEl(self) -> Tuple[Optional[float], Optional[float]]:
        return self.State.AzimuthDeg, self.State.ElevationDeg

    def QueryPanPosition(self) -> Optional[float]:
        self.Update()
        return self.State.AzimuthDeg

    def Update(self) -> PTZState:
        Now = time.time()
        Dt = max(0.0, min(Now - self.LastUpdateSec, 0.25))
        self.LastUpdateSec = Now

        if self.Mode == "slew":
            self.State.AzimuthDeg = self._ClampOrWrap(
                self.State.AzimuthDeg + self.CommandedRateDegPerSec * Dt
            )
            self.State.PanRateDegPerSec = self.CommandedRateDegPerSec
            self.State.Direction = "right" if self.CommandedRateDegPerSec > 0.0 else "left"
            self._ApplyLimits()

        elif self.Mode == "goto":
            Error = SignedAngleDeltaDeg(self.TargetAzimuthDeg, self.State.AzimuthDeg)

            # In non-wrap mode use direct physical-axis difference. This matches
            # the real PTZ software window, e.g. 10..300 deg.
            if not self.WrapMode:
                Error = self.TargetAzimuthDeg - self.State.AzimuthDeg

            if abs(Error) <= self.PositionToleranceDeg:
                self.State.AzimuthDeg = self.TargetAzimuthDeg
                self.Stop()
            else:
                Direction = 1.0 if Error > 0.0 else -1.0
                Step = Direction * self.MaxPanRateDegPerSec * Dt

                if abs(Step) > abs(Error):
                    Step = Error

                self.State.AzimuthDeg = self._ClampOrWrap(self.State.AzimuthDeg + Step)
                self.CommandedRateDegPerSec = Direction * self.MaxPanRateDegPerSec
                self.State.PanRateDegPerSec = self.CommandedRateDegPerSec
                self.State.Direction = "right" if Direction > 0.0 else "left"
                self._ApplyLimits()

        else:
            self.State.PanRateDegPerSec = 0.0
            self.State.Direction = "stopped"

        self.State.Valid = True
        self.State.Source = "SIMULATED_PTZ"
        self.State.TimestampSec = Now
        return self.State

    def GetLastKnownState(self):
        return self.State

    def GetState(self):
        return self.State


class PelcoDPTZController:
    def __init__(
        self,
        Port="/dev/ttyACM0",
        BaudRate=2400,
        Address=1,
        PanSpeed=0x5F,
        TiltSpeed=0x3F,
        LeftLimitDeg=10.0,
        RightLimitDeg=300.0,
        LimitMarginDeg=1.0,
        TimeoutSec=0.3,
        Debug=False,
        **kwargs,
    ):
        self.Port = str(Port)
        self.BaudRate = int(BaudRate)
        self.Address = int(Address)
        self.PanSpeed = int(PanSpeed)
        self.TiltSpeed = int(TiltSpeed)
        self.LeftLimitDeg = float(LeftLimitDeg)
        self.RightLimitDeg = float(RightLimitDeg)
        self.LimitMarginDeg = float(LimitMarginDeg)
        self.TimeoutSec = float(TimeoutSec)
        self.Debug = bool(Debug)

        self.SerialPort = None
        self.State = PTZState(
            AzimuthDeg=0.0,
            ElevationDeg=0.0,
            CommandedAzimuthDeg=0.0,
            CommandedElevationDeg=0.0,
            Valid=False,
            Source="PELCO_D_NOT_OPEN",
            TimestampSec=time.time(),
        )

        self.LastNativeTargetDeg = None
        self.LastNativeTiltTargetDeg = None
        self.LastSlewCommand = None
        self.LastNativeCommandTimeSec = 0.0
        self.QueryIntervalSec = float(kwargs.get('QueryIntervalSec', 0.10))
        self.QueryTiltInUpdate = bool(kwargs.get('QueryTiltInUpdate', False))
        self.LastQueryTimeSec = 0.0

    def Open(self):
        if serial is None:
            raise RuntimeError("pyserial is not installed. Run: python3 -m pip install pyserial")

        if self.SerialPort is None or not self.SerialPort.is_open:
            self.SerialPort = serial.Serial(
                self.Port,
                self.BaudRate,
                bytesize=8,
                parity="N",
                stopbits=1,
                timeout=self.TimeoutSec,
            )
            time.sleep(0.2)
            print(f"Pelco-D PTZ opened on {self.Port} at {self.BaudRate} baud")

            # Prime state with one query.
            self.Update()

    def Close(self):
        try:
            self.Stop()
        finally:
            if self.SerialPort is not None and self.SerialPort.is_open:
                self.SerialPort.close()
                print("Pelco-D PTZ closed")

    def _BuildPacket(self, Command1, Command2, Data1, Data2):
        Checksum = (self.Address + Command1 + Command2 + Data1 + Data2) & 0xFF
        return bytes([
            0xFF,
            self.Address & 0xFF,
            Command1 & 0xFF,
            Command2 & 0xFF,
            Data1 & 0xFF,
            Data2 & 0xFF,
            Checksum,
        ])

    def SendPelcoD(self, Command1, Command2, Data1, Data2, ReadReply=False, ReadLength=16):
        self.Open()

        Packet = self._BuildPacket(Command1, Command2, Data1, Data2)

        if ReadReply:
            try:
                self.SerialPort.reset_input_buffer()
            except Exception:
                pass

        try:
            self.SerialPort.write(Packet)
            self.SerialPort.flush()
        except Exception as Error:
            if self.Debug:
                print(f"PTZ serial write failed: {Error}")
            return b""

        if self.Debug:
            print("PTZ TX:", Packet.hex(" "))

        if not ReadReply:
            return b""

        # Position replies are 7-byte Pelco-D frames.  Do not call
        # read(16) directly, because pyserial will wait for all 16 bytes or
        # timeout even when the valid 7-byte frame has already arrived.
        # Instead, collect whatever arrives during a short reply window.
        Reply = bytearray()
        DeadlineSec = time.time() + float(getattr(self, "ReplyWaitSec", 0.08))

        try:
            while time.time() < DeadlineSec and len(Reply) < int(ReadLength):
                Waiting = int(getattr(self.SerialPort, "in_waiting", 0))

                if Waiting > 0:
                    Need = int(ReadLength) - len(Reply)
                    Reply.extend(self.SerialPort.read(min(Waiting, Need)))

                    # A normal Pelco-D reply frame is complete at 7 bytes.
                    # Keep scanning/parsing code tolerant of leading bytes, but
                    # do not wait for unused bytes once a minimum frame exists.
                    if len(Reply) >= 7:
                        break
                else:
                    time.sleep(0.002)
        except Exception as Error:
            if self.Debug:
                print(f"PTZ serial read failed: {Error}")
            return b""

        Reply = bytes(Reply)

        if self.Debug:
            if Reply:
                print("PTZ RX:", Reply.hex(" "))
            else:
                print("PTZ RX: no reply")

        return Reply

    def Stop(self):
        """
        Hard motor stop.

        Pelco-D units often need repeated stop packets, especially if they are
        executing a native goto-position command.
        """
        if self.SerialPort is not None:
            try:
                for _ in range(4):
                    self.SendPelcoD(0x00, 0x00, 0x00, 0x00, ReadReply=False)
                    time.sleep(0.04)
                try:
                    self.SerialPort.reset_output_buffer()
                    self.SerialPort.reset_input_buffer()
                except Exception:
                    pass
            except Exception:
                pass

        self.State.PanRateDegPerSec = 0.0
        self.State.Direction = "stopped"

        # IMPORTANT:
        # A stop command physically stops the PTZ motor, but the old software
        # remembered the last commanded slew direction.  If scan is restarted
        # and the required direction is the same as before, CommandSlew() would
        # think the command had already been sent and would not send a fresh
        # Pelco-D left/right packet.  That leaves the PTZ stopped near an edge.
        self.LastSlewCommand = None

        self.LastNativeTargetDeg = None

    def StopAndHold(self):
        """
        Hard stop and remain idle. This does not send any position-hold command.
        """
        self.Stop()
        self.State.Direction = "stopped"
        self.LastSlewCommand = None
        self.LastNativeTargetDeg = None

    def QueryPanPosition(self) -> Optional[float]:
        Reply = self.SendPelcoD(0x00, 0x51, 0x00, 0x00, ReadReply=True, ReadLength=16)

        if len(Reply) < 7:
            return None

        for Start in range(0, len(Reply) - 6):
            Frame = Reply[Start:Start + 7]

            if Frame[0] != 0xFF:
                continue
            if Frame[1] != (self.Address & 0xFF):
                continue
            if Frame[3] != 0x59:
                continue

            Checksum = sum(Frame[1:6]) & 0xFF
            if Frame[6] != Checksum:
                continue

            Raw = (Frame[4] << 8) | Frame[5]
            return Wrap360(Raw / 100.0)

        return None

    def QueryTiltPosition(self) -> Optional[float]:
        """
        Native absolute tilt query.

        Common Pelco-D extended command:
            Query tilt: 0x00 0x53 -> response command2 0x5B
            raw = deg * 100

        Some PTZ heads report tilt as a signed or offset value. This function
        returns the raw decoded degrees first; clamp/offset can be adjusted once
        the hardware response is confirmed.
        """
        Reply = self.SendPelcoD(0x00, 0x53, 0x00, 0x00, ReadReply=True, ReadLength=16)

        if len(Reply) < 7:
            return None

        for Start in range(0, len(Reply) - 6):
            Frame = Reply[Start:Start + 7]

            if Frame[0] != 0xFF:
                continue
            if Frame[1] != (self.Address & 0xFF):
                continue
            if Frame[3] != 0x5B:
                continue

            Checksum = sum(Frame[1:6]) & 0xFF
            if Frame[6] != Checksum:
                continue

            Raw = (Frame[4] << 8) | Frame[5]
            return float(Raw) / 100.0

        return None

    def SetPanPositionNative(self, PanDeg: float):
        """
        Native absolute pan goto.

        Confirmed hardware command:
            0x00 0x4B Data1 Data2
            raw = deg * 100
        """
        Target = ClampToSoftwareWindow(
            PanDeg,
            self.LeftLimitDeg,
            self.RightLimitDeg,
        )

        # Avoid spamming identical commands every loop.
        Now = time.time()
        if self.LastNativeTargetDeg is not None:
            if abs(SignedAngleDeltaDeg(Target, self.LastNativeTargetDeg)) < 0.05:
                if Now - self.LastNativeCommandTimeSec < 0.5:
                    return

        Raw = int(round(Target * 100.0)) & 0xFFFF
        Data1 = (Raw >> 8) & 0xFF
        Data2 = Raw & 0xFF

        if self.Debug:
            print(f"PTZ SET PAN {Target:.2f} deg raw={Raw}")

        self.State.CommandedAzimuthDeg = Target
        self.LastNativeTargetDeg = Target
        self.LastNativeCommandTimeSec = Now

        self.SendPelcoD(0x00, 0x4B, Data1, Data2, ReadReply=False)

    def NudgePanPositionNative(self, IncrementDeg: float):
        Current = self.QueryPanPosition()

        if Current is None:
            self.State.Valid = False
            self.State.Source = "PELCO_D_NO_POSITION"
            return False

        self.SetPanPositionNative(Current + float(IncrementDeg))
        return True

    def SetTiltPositionNative(self, TiltDeg: float):
        """
        Native absolute tilt goto.

        Common Pelco-D extended command:
            0x00 0x4D Data1 Data2
            raw = deg * 100
        """
        Target = Clamp(float(TiltDeg), -90.0, 90.0)

        Raw = int(round(Target * 100.0)) & 0xFFFF
        Data1 = (Raw >> 8) & 0xFF
        Data2 = Raw & 0xFF

        if self.Debug:
            print(f"PTZ SET TILT {Target:.2f} deg raw={Raw}")

        self.State.ElevationDeg = Target
        self.State.CommandedElevationDeg = Target
        self.LastNativeTiltTargetDeg = Target

        self.SendPelcoD(0x00, 0x4D, Data1, Data2, ReadReply=False)

    def CommandPosition(self, AzimuthDeg: float, ElevationDeg: float = 0.0):
        self.SetPanPositionNative(AzimuthDeg)
        self.SetTiltPositionNative(ElevationDeg)

    def CommandSlew(self, PanRateDegPerSec: float, TiltRateDegPerSec: float = 0.0):
        """
        Low-level continuous slew. Send only when direction changes.
        """
        self.LastNativeTargetDeg = None

        if PanRateDegPerSec > 0:
            if self.LastSlewCommand != "right":
                self.SendPelcoD(0x00, 0x02, self.PanSpeed, 0x00, ReadReply=False)
                self.LastSlewCommand = "right"
            self.State.Direction = "right"

        elif PanRateDegPerSec < 0:
            if self.LastSlewCommand != "left":
                self.SendPelcoD(0x00, 0x04, self.PanSpeed, 0x00, ReadReply=False)
                self.LastSlewCommand = "left"
            self.State.Direction = "left"

        else:
            self.Stop()

    def QueryAzEl(self):
        Az = self.QueryPanPosition()
        Tilt = self.QueryTiltPosition()
        if Tilt is not None:
            self.State.ElevationDeg = Tilt
        return Az, self.State.ElevationDeg

    def Update(self) -> PTZState:
        Now = time.time()

        # The Pelco-D link is slow. Query at a controlled rate and return the
        # last known state between query instants.
        if (
            self.LastQueryTimeSec > 0.0
            and (Now - self.LastQueryTimeSec) < self.QueryIntervalSec
        ):
            return self.State

        self.LastQueryTimeSec = Now
        Pan = self.QueryPanPosition()

        if Pan is None:
            # During native movement, the PTZ sometimes does not reply.
            # Keep the last known position usable for this dwell. If we have
            # ever had a valid position, keep Valid=True so the display/radar
            # continues to use the last measured boresight rather than falling
            # back to a simulated/commanded angle.
            if self.State.Source in ["PELCO_D", "PELCO_D_LAST_KNOWN_NO_REPLY"]:
                self.State.Valid = True
            self.State.Source = "PELCO_D_LAST_KNOWN_NO_REPLY"
            self.State.TimestampSec = Now
            return self.State

        OldAz = self.State.AzimuthDeg
        OldTime = self.State.TimestampSec

        self.State.AzimuthDeg = Pan

        if self.QueryTiltInUpdate:
            Tilt = self.QueryTiltPosition()
            if Tilt is not None:
                self.State.ElevationDeg = Tilt

        self.State.Valid = True
        self.State.Source = "PELCO_D"
        self.State.TimestampSec = Now

        if OldTime > 0.0 and Now > OldTime:
            self.State.PanRateDegPerSec = SignedAngleDeltaDeg(Pan, OldAz) / (Now - OldTime)

        self.State.AtLeftLimit = Pan <= (self.LeftLimitDeg + self.LimitMarginDeg)
        self.State.AtRightLimit = Pan >= (self.RightLimitDeg - self.LimitMarginDeg)

        return self.State

    def GetState(self):
        return self.State


def CreatePTZController(Config):
    Mode = str(Config.get("PTZMode", "pelco")).lower()

    Common = {
        "LeftLimitDeg": float(Config.get("PTZLeftLimitDeg", 10.0)),
        "RightLimitDeg": float(Config.get("PTZRightLimitDeg", 300.0)),
        "LimitMarginDeg": float(Config.get("PTZLimitMarginDeg", 1.0)),
    }

    if Mode in ["pelco", "pelcod", "pelco-d", "real", "device", "hardware"]:
        return PelcoDPTZController(
            Port=str(Config.get("PTZPort", "/dev/ttyACM0")),
            BaudRate=int(Config.get("PTZBaudRate", 2400)),
            Address=int(Config.get("PTZAddress", 1)),
            PanSpeed=int(Config.get("PTZPanSpeed", 0x5F)),
            TiltSpeed=int(Config.get("PTZTiltSpeed", 0x3F)),
            TimeoutSec=float(Config.get("PTZTimeoutSec", 0.3)),
            QueryIntervalSec=float(Config.get("PTZQueryIntervalSec", 0.10)),
            QueryTiltInUpdate=bool(Config.get("PTZQueryTiltInUpdate", False)),
            Debug=bool(Config.get("PTZDebug", False)),
            **Common,
        )

    if Mode in ["sim", "simulated", "simulated_ptz", "drone", "sim-drone"]:
        return SimulatedPTZController(
            InitialAzimuthDeg=float(Config.get("InitialBeamAngleDeg", Config.get("PTZStartupAzimuthDeg", 200.0))),
            InitialElevationDeg=float(Config.get("PTZStartupElevationDeg", 90.0)),
            MaxPanRateDegPerSec=float(Config.get("PTZSimPanRateDegPerSec", Config.get("PTZScanSlewRateDegPerSec", 14.0))),
            PositionToleranceDeg=float(Config.get("PTZPositionToleranceDeg", 0.5)),
            QueryIntervalSec=float(Config.get("PTZQueryIntervalSec", 0.10)),
            WrapMode=bool(Config.get("PTZSimWrapMode", False)),
            Debug=bool(Config.get("PTZDebug", False)),
            **Common,
        )

    raise ValueError(f"Unknown PTZMode: {Mode}")


if __name__ == "__main__":
    Config = {
        "PTZMode": "pelco",
        "PTZPort": "/dev/ttyACM0",
        "PTZBaudRate": 2400,
        "PTZAddress": 1,
        "PTZLeftLimitDeg": 10.0,
        "PTZRightLimitDeg": 300.0,
        "PTZDebug": True,
    }

    Ptz = CreatePTZController(Config)
    Ptz.Open()

    try:
        while True:
            Cmd = input("PTZ> ").strip().lower()

            if Cmd == "q":
                break
            elif Cmd == "p":
                print(Ptz.Update())
            elif Cmd.startswith("g "):
                Target = float(Cmd.split()[1])
                Ptz.SetPanPositionNative(Target)
            elif Cmd == "+":
                Ptz.NudgePanPositionNative(+1.0)
            elif Cmd == "-":
                Ptz.NudgePanPositionNative(-1.0)
            elif Cmd == "s":
                Ptz.Stop()
            else:
                print("Commands: p=query, g <deg>=goto, +=+1deg, -=-1deg, s=stop, q=quit")
    finally:
        Ptz.Close()
