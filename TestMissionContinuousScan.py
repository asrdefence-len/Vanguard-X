import unittest

from NavigationState import PlatformAttitude
from PointingManager import PointingManager
from RadarTasks import (
    AngleFrame,
    MakeSearchTask,
    SearchPattern,
)


class _State:
    def __init__(self, azimuth_deg=0.0, rate_deg_sec=0.0):
        self.AzimuthDeg = float(azimuth_deg)
        self.ElevationDeg = 0.0
        self.PanRateDegPerSec = float(rate_deg_sec)
        self.Valid = True
        self.Source = "TEST_X660"


class _UnlimitedX660:
    UnlimitedAzimuth = True

    def __init__(self):
        self.State = _State()
        self.CommandedRateDegSec = 0.0
        self.StopCount = 0

    def CommandSlew(self, rate_deg_per_sec, tilt_rate_deg_per_sec=0.0):
        self.CommandedRateDegSec = float(rate_deg_per_sec)
        self.State.PanRateDegPerSec = float(rate_deg_per_sec)

    def SetPanPositionNative(self, target_deg):
        self.State.AzimuthDeg = float(target_deg) % 360.0

    def Stop(self):
        self.StopCount += 1
        self.CommandedRateDegSec = 0.0
        self.State.PanRateDegPerSec = 0.0

    def Update(self):
        return self.State

    def GetState(self):
        return self.State


def _Attitude():
    return PlatformAttitude(
        TimestampSec=0.0,
        HeadingTrueDeg=0.0,
        PitchDeg=0.0,
        RollDeg=0.0,
        Valid=True,
    )


class MissionContinuousScanTests(unittest.TestCase):
    def _RunCrossing(self, pattern, angles, expected_rate):
        x660 = _UnlimitedX660()
        pointing = PointingManager(x660=x660)
        task = MakeSearchTask(
            TaskId=1,
            SectorStartDeg=0.0,
            SectorStopDeg=359.999,
            ScanRateDegPerSec=20.0,
            SectorFrame=AngleFrame.PLATFORM,
            Pattern=pattern,
        )
        pointing.ActivateTask(task, _Attitude())
        self.assertEqual(x660.CommandedRateDegSec, expected_rate)

        initial_cycle = task.Sector.ScanCycle
        for index, angle in enumerate(angles):
            x660.State.AzimuthDeg = float(angle)
            pointing.Update(_Attitude(), current_time_sec=float(index))

        self.assertEqual(task.Sector.ScanCycle, initial_cycle + 1)
        self.assertEqual(x660.CommandedRateDegSec, expected_rate)
        self.assertEqual(x660.StopCount, 0)

    def test_continuous_cw_crosses_north_without_reversing(self):
        self._RunCrossing(
            SearchPattern.CONTINUOUS_CW,
            (358.0, 359.5, 0.5, 2.0),
            20.0,
        )

    def test_continuous_ccw_crosses_north_without_reversing(self):
        self._RunCrossing(
            SearchPattern.CONTINUOUS_CCW,
            (2.0, 0.5, 359.5, 358.0),
            -20.0,
        )


if __name__ == "__main__":
    unittest.main()
