#!/usr/bin/env python3
"""
ReadIMU.py

Dummy / placeholder IMU interface for Vanguard antenna attitude.

Purpose
-------
Provides a simple function/class interface that returns antenna:
    - AzimuthDeg  (AZ)
    - ElevationDeg (EL)

This is written so Vanguard can be integrated now, before the real BLE parser for
WitMotion WT901BLECL / MPU9250 is connected.

Later, replace _read_real_imu_ble() with the actual WitMotion BLE data decode.

Typical use from Vanguard main loop
-----------------------------------
    from ReadIMU import IMUReader

    Imu = IMUReader(dummy=True)
    Imu.SetSimulatedAntennaPosition(CommandedBoresightDeg, 0.0)
    AzDeg, ElDeg = Imu.ReadAzEl()

or:
    from ReadIMU import ReadIMU
    ImuData = ReadIMU()
    AzDeg = ImuData["AzimuthDeg"]
    ElDeg = ImuData["ElevationDeg"]
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, asdict
from typing import Dict, Optional, Tuple


IMU_VERSION = "dummy-follow-commanded-az-el-v2"


@dataclass
class IMUData:
    """Container for antenna attitude data."""

    AzimuthDeg: float = 0.0
    ElevationDeg: float = 0.0
    RollDeg: float = 0.0
    PitchDeg: float = 0.0
    YawDeg: float = 0.0
    TimestampSec: float = 0.0
    Valid: bool = True
    Source: str = "DUMMY"

    def ToDict(self) -> Dict[str, float]:
        return asdict(self)


class IMUReader:
    """
    Vanguard IMU reader abstraction.

    For now this defaults to dummy mode. In dummy mode it does not invent its own
    scan. It acts like a simulated antenna position sensor: Vanguard tells it
    the current commanded/simulated antenna AZ/EL using SetSimulatedAntennaPosition(),
    and Read() returns that value as the measured antenna attitude.

    Parameters
    ----------
    dummy:
        If True, produce simulated AZ/EL values.
        If False, call the placeholder real BLE routine.
    az_offset_deg:
        Calibration offset added to yaw/azimuth.
    el_offset_deg:
        Calibration offset added to pitch/elevation.
    invert_az:
        If True, reverses azimuth sign before offset/wrap.
    invert_el:
        If True, reverses elevation sign before offset.
    """

    def __init__(
        self,
        dummy: bool = True,
        az_offset_deg: float = 0.0,
        el_offset_deg: float = 0.0,
        invert_az: bool = False,
        invert_el: bool = False,
    ) -> None:
        self.Dummy = bool(dummy)
        self.AzOffsetDeg = float(az_offset_deg)
        self.ElOffsetDeg = float(el_offset_deg)
        self.InvertAz = bool(invert_az)
        self.InvertEl = bool(invert_el)
        self.StartTimeSec = time.time()

        # In dummy mode, these are the simulated antenna angles supplied by
        # Vanguard's scan/motor logic. The IMU simply reports them back as if
        # they were measured antenna attitude.
        self.SimulatedAzimuthDeg = 0.0
        self.SimulatedElevationDeg = 0.0
        self.SimulatedRollDeg = 0.0

        self.LastData = IMUData(TimestampSec=self.StartTimeSec, Source="DUMMY_FOLLOW_COMMAND")

    @staticmethod
    def _wrap_azimuth_deg(angle_deg: float) -> float:
        """Wrap angle to 0..360 degrees."""
        return float(angle_deg % 360.0)

    @staticmethod
    def _clip_elevation_deg(angle_deg: float) -> float:
        """Clip elevation to a sensible antenna range."""
        return float(max(-90.0, min(90.0, angle_deg)))

    def _apply_calibration(self, yaw_deg: float, pitch_deg: float, roll_deg: float = 0.0) -> IMUData:
        """Convert IMU yaw/pitch/roll into calibrated antenna AZ/EL."""

        az = -yaw_deg if self.InvertAz else yaw_deg
        el = -pitch_deg if self.InvertEl else pitch_deg

        az = self._wrap_azimuth_deg(az + self.AzOffsetDeg)
        el = self._clip_elevation_deg(el + self.ElOffsetDeg)

        return IMUData(
            AzimuthDeg=az,
            ElevationDeg=el,
            RollDeg=float(roll_deg),
            PitchDeg=float(pitch_deg),
            YawDeg=float(yaw_deg),
            TimestampSec=time.time(),
            Valid=True,
            Source="DUMMY" if self.Dummy else "WT901BLECL",
        )

    def SetSimulatedAntennaPosition(
        self,
        AzimuthDeg: float,
        ElevationDeg: float = 0.0,
        RollDeg: float = 0.0,
    ) -> None:
        """
        Set the simulated antenna position used by dummy mode.

        Vanguard should call this once per dwell after it has calculated the
        current commanded/simulated boresight angle. The dummy IMU then reports
        this same position back through Read(), allowing the rest of the code to
        use the IMU interface before real hardware is connected.
        """

        self.SimulatedAzimuthDeg = self._wrap_azimuth_deg(float(AzimuthDeg))
        self.SimulatedElevationDeg = self._clip_elevation_deg(float(ElevationDeg))
        self.SimulatedRollDeg = float(RollDeg)

    # Backwards-compatible alias in case the main loop uses shorter names.
    def SetSimulatedAzEl(self, AzimuthDeg: float, ElevationDeg: float = 0.0) -> None:
        self.SetSimulatedAntennaPosition(AzimuthDeg, ElevationDeg)

    def _read_dummy_imu(self) -> IMUData:
        """
        Dummy IMU pass-through.

        It returns the simulated antenna AZ/EL that Vanguard last supplied via
        SetSimulatedAntennaPosition(). It does not free-run, sweep, or exceed the
        scan limits by itself.
        """

        data = self._apply_calibration(
            yaw_deg=self.SimulatedAzimuthDeg,
            pitch_deg=self.SimulatedElevationDeg,
            roll_deg=self.SimulatedRollDeg,
        )
        data.Source = "DUMMY_FOLLOW_COMMAND"
        return data

    def _read_real_imu_ble(self) -> IMUData:
        """
        Placeholder for real WitMotion WT901BLECL BLE read.

        The WT901BLECL BLE implementation should eventually:
            1. Connect to the BLE device using bleak or another BLE library.
            2. Subscribe to the notify characteristic.
            3. Decode the WitMotion angle packet.
            4. Extract yaw, pitch, roll.
            5. Return calibrated AzimuthDeg and ElevationDeg.

        For now, this returns the last valid value and marks it invalid if no
        real BLE code has been implemented.
        """

        data = self.LastData
        data.Valid = False
        data.Source = "WT901BLECL_NOT_CONNECTED"
        data.TimestampSec = time.time()
        return data

    def Read(self) -> IMUData:
        """Read the current antenna attitude."""

        if self.Dummy:
            data = self._read_dummy_imu()
        else:
            data = self._read_real_imu_ble()

        self.LastData = data
        return data

    def ReadAzEl(self) -> Tuple[float, float]:
        """Return (AzimuthDeg, ElevationDeg)."""

        data = self.Read()
        return data.AzimuthDeg, data.ElevationDeg

    def ReadDict(self) -> Dict[str, float]:
        """Return data as a dictionary for compatibility with existing code."""

        return self.Read().ToDict()


# -----------------------------------------------------------------------------
# Simple module-level convenience function
# -----------------------------------------------------------------------------
_DefaultReader: Optional[IMUReader] = None


def ReadIMU() -> Dict[str, float]:
    """
    Convenience function returning the latest IMU reading as a dictionary.

    This keeps calling code very simple:
        Imu = ReadIMU()
        AzDeg = Imu["AzimuthDeg"]
        ElDeg = Imu["ElevationDeg"]
    """

    global _DefaultReader
    if _DefaultReader is None:
        _DefaultReader = IMUReader(dummy=True)

    return _DefaultReader.ReadDict()


if __name__ == "__main__":
    print(f"ReadIMU.py version: {IMU_VERSION}")
    reader = IMUReader(dummy=True)

    try:
        while True:
            imu = reader.Read()
            print(
                f"AZ={imu.AzimuthDeg:7.2f} deg  "
                f"EL={imu.ElevationDeg:7.2f} deg  "
                f"Yaw={imu.YawDeg:7.2f}  "
                f"Pitch={imu.PitchDeg:7.2f}  "
                f"Roll={imu.RollDeg:7.2f}  "
                f"Valid={imu.Valid}  "
                f"Source={imu.Source}"
            )
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("\nIMU dummy read stopped.")
