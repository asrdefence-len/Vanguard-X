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
    def __init__(
        self,
        LeftLimitDeg=10.0,
        RightLimitDeg=300.0,
        InitialAzimuthDeg=230.0,
        MaxPanRateDegPerSec=60.0,
        PositionToleranceDeg=0.5,
        **kwargs,
    ):
        self.LeftLimitDeg = float(LeftLimitDeg)
        self.RightLimitDeg = float(RightLimitDeg)
        self.MaxPanRateDegPerSec = float(MaxPanRateDegPerSec)
        self.PositionToleranceDeg = float(PositionToleranceDeg)

        InitialAzimuthDeg = ClampToSoftwareWindow(
            InitialAzimuthDeg,
            self.LeftLimitDeg,
            self.RightLimitDeg,
        )

        self.State = PTZState(
            AzimuthDeg=InitialAzimuthDeg,
            CommandedAzimuthDeg=InitialAzimuthDeg,
            Valid=True,
            Source="SIMULATED_PTZ",
            TimestampSec=time.time(),
        )
        self.TargetAzimuthDeg = InitialAzimuthDeg
        self.LastUpdateSec = time.time()

    def Open(self):
        print("Simulated PTZ opened")

    def Close(self):
        self.Stop()

    def Stop(self):
        self.State.PanRateDegPerSec = 0.0
        self.State.Direction = "stopped"

    def SetPanPositionNative(self, PanDeg: float):
        Target = ClampToSoftwareWindow(PanDeg, self.LeftLimitDeg, self.RightLimitDeg)
        self.TargetAzimuthDeg = Target
        self.State.CommandedAzimuthDeg = Target
        self.LastSlewCommand = None

    def ScanToEndpoint(self, EndpointDeg: float):
        """
        Command a continuous scan by asking the PTZ to go to the far endpoint.

        The radar should keep collecting dwells while the PTZ moves and should
        use Query/Update position as the boresight angle.
        """
        self.SetPanPositionNative(EndpointDeg)

    def NudgePanPositionNative(self, IncrementDeg: float):
        self.SetPanPositionNative(self.State.AzimuthDeg + float(IncrementDeg))
        return True

    def CommandPosition(self, AzimuthDeg: float, ElevationDeg: float = 0.0):
        self.SetPanPositionNative(AzimuthDeg)

    def CommandSlew(self, PanRateDegPerSec: float, TiltRateDegPerSec: float = 0.0):
        self.State.PanRateDegPerSec = float(PanRateDegPerSec)
        self.State.Direction = "right" if PanRateDegPerSec > 0 else "left" if PanRateDegPerSec < 0 else "stopped"

    def QueryAzEl(self) -> Tuple[Optional[float], Optional[float]]:
        return self.State.AzimuthDeg, self.State.ElevationDeg

    def Update(self) -> PTZState:
        Now = time.time()
        Dt = max(0.0, min(Now - self.LastUpdateSec, 0.2))
        self.LastUpdateSec = Now

        Error = SignedAngleDeltaDeg(self.TargetAzimuthDeg, self.State.AzimuthDeg)

        if abs(Error) <= self.PositionToleranceDeg:
            self.State.AzimuthDeg = self.TargetAzimuthDeg
            self.State.PanRateDegPerSec = 0.0
            self.State.Direction = "stopped"
        else:
            Direction = 1.0 if Error > 0.0 else -1.0
            Step = Direction * self.MaxPanRateDegPerSec * Dt
            if abs(Step) > abs(Error):
                Step = Error
            self.State.AzimuthDeg = Wrap360(self.State.AzimuthDeg + Step)
            self.State.PanRateDegPerSec = Direction * self.MaxPanRateDegPerSec
            self.State.Direction = "right" if Direction > 0 else "left"

        self.State.AzimuthDeg = ClampToSoftwareWindow(
            self.State.AzimuthDeg,
            self.LeftLimitDeg,
            self.RightLimitDeg,
        )
        self.State.AtLeftLimit = self.State.AzimuthDeg <= self.LeftLimitDeg + 0.5
        self.State.AtRightLimit = self.State.AzimuthDeg >= self.RightLimitDeg - 0.5
        self.State.Valid = True
        self.State.Source = "SIMULATED_PTZ"
        self.State.TimestampSec = Now
        return self.State

    def GetLastKnownState(self):
        """
        Return the last known PTZ state without sending a serial query.
        Useful for display updates when the PTZ is busy executing a goto.
        """
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
            CommandedAzimuthDeg=0.0,
            Valid=False,
            Source="PELCO_D_NOT_OPEN",
            TimestampSec=time.time(),
        )

        self.LastNativeTargetDeg = None
        self.LastSlewCommand = None
        self.LastNativeCommandTimeSec = 0.0

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

    def SendPelcoD(self, Command1, Command2, Data1, Data2, ReadReply=False):
        self.Open()

        Packet = self._BuildPacket(Command1, Command2, Data1, Data2)

        if ReadReply:
            try:
                self.SerialPort.reset_input_buffer()
            except Exception:
                pass

        self.SerialPort.write(Packet)
        self.SerialPort.flush()

        if self.Debug:
            print("PTZ TX:", Packet.hex(" "))

        if not ReadReply:
            return b""

        time.sleep(0.03)
        Reply = self.SerialPort.read(64)

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
        self.LastNativeTargetDeg = None

    def StopAndHold(self):
        """
        Hard stop and remain idle. This does not send any position-hold command.
        """
        self.Stop()
        self.State.Direction = "stopped"
        self.LastNativeTargetDeg = None

    def QueryPanPosition(self) -> Optional[float]:
        Reply = self.SendPelcoD(0x00, 0x51, 0x00, 0x00, ReadReply=True)

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

    def CommandPosition(self, AzimuthDeg: float, ElevationDeg: float = 0.0):
        self.SetPanPositionNative(AzimuthDeg)

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
        return Az, self.State.ElevationDeg

    def Update(self) -> PTZState:
        Now = time.time()
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

    if Mode in ["pelco", "pelcod", "pelco-d", "real", "device"]:
        return PelcoDPTZController(
            Port=str(Config.get("PTZPort", "/dev/ttyACM0")),
            BaudRate=int(Config.get("PTZBaudRate", 2400)),
            Address=int(Config.get("PTZAddress", 1)),
            PanSpeed=int(Config.get("PTZPanSpeed", 0x5F)),
            TiltSpeed=int(Config.get("PTZTiltSpeed", 0x3F)),
            TimeoutSec=float(Config.get("PTZTimeoutSec", 0.3)),
            Debug=bool(Config.get("PTZDebug", False)),
            **Common,
        )

    return SimulatedPTZController(
        InitialAzimuthDeg=float(Config.get("InitialBeamAngleDeg", 230.0)),
        MaxPanRateDegPerSec=float(Config.get("PTZSimPanRateDegPerSec", 60.0)),
        **Common,
    )


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
