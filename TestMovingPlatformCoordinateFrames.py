"""Moving-platform true/relative angle regressions for Vanguard X."""

from types import SimpleNamespace
import unittest

from CoordinateFrames import angle_in_frame_to_true_bearing
from PointingManager import PointingManager
from RadarTasks import AngleFrame, MakeSearchTask


class MovingPlatformCoordinateFrameTests(unittest.TestCase):
    def setUp(self):
        self.TrueSector = MakeSearchTask(
            TaskId=1,
            SectorStartDeg=40.0,
            SectorStopDeg=90.0,
            ScanRateDegPerSec=20.0,
            SectorFrame=AngleFrame.TRUE,
        )

    def test_true_sector_limits_convert_against_live_heading(self):
        self.assertAlmostEqual(
            PointingManager.TrueToRelativeAzimuth(40.0, 90.0),
            310.0,
        )
        self.assertAlmostEqual(
            PointingManager.TrueToRelativeAzimuth(90.0, 90.0),
            0.0,
        )
        self.assertAlmostEqual(
            PointingManager.RelativeToTrueBearing(310.0, 90.0),
            40.0,
        )

        # Ten degrees of vessel turn moves both X6-60-relative endpoints by
        # ten degrees while the geographic sector remains exactly 040..090.
        self.assertAlmostEqual(
            PointingManager.TrueToRelativeAzimuth(40.0, 100.0),
            300.0,
        )
        self.assertAlmostEqual(
            PointingManager.TrueToRelativeAzimuth(90.0, 100.0),
            350.0,
        )

    def test_true_scan_rate_removes_clockwise_vessel_yaw(self):
        navigation = SimpleNamespace(YawRateDegPerSec=0.2864789)

        clockwise_relative_rate = (
            PointingManager._search_relative_slew_rate(
                self.TrueSector,
                navigation,
                +1.0,
            )
        )
        counter_clockwise_relative_rate = (
            PointingManager._search_relative_slew_rate(
                self.TrueSector,
                navigation,
                -1.0,
            )
        )

        self.assertAlmostEqual(
            clockwise_relative_rate + navigation.YawRateDegPerSec,
            20.0,
        )
        self.assertAlmostEqual(
            counter_clockwise_relative_rate
            + navigation.YawRateDegPerSec,
            -20.0,
        )

    def test_platform_scan_rate_retains_vessel_relative_semantics(self):
        platform_sector = MakeSearchTask(
            TaskId=2,
            SectorStartDeg=40.0,
            SectorStopDeg=90.0,
            ScanRateDegPerSec=20.0,
            SectorFrame=AngleFrame.PLATFORM,
        )
        navigation = SimpleNamespace(YawRateDegPerSec=5.0)

        self.assertAlmostEqual(
            PointingManager._search_relative_slew_rate(
                platform_sector,
                navigation,
                +1.0,
            ),
            20.0,
        )

    def test_ppi_sector_limits_are_converted_without_changing_task_values(self):
        self.assertEqual(
            angle_in_frame_to_true_bearing(40.0, "TRUE", 135.0),
            40.0,
        )
        self.assertEqual(
            angle_in_frame_to_true_bearing(90.0, "TRUE", 135.0),
            90.0,
        )
        self.assertEqual(
            angle_in_frame_to_true_bearing(40.0, "PLATFORM", 90.0),
            130.0,
        )
        self.assertEqual(
            angle_in_frame_to_true_bearing(90.0, "PLATFORM", 90.0),
            180.0,
        )


if __name__ == "__main__":
    unittest.main()
