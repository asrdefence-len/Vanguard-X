"""Dashboard true-bearing scan-frame regression for Vanguard X."""

from pathlib import Path
from types import SimpleNamespace
import unittest

from PointingManager import PointingManager
from RadarTasks import AngleFrame, MakeSearchTask


ROOT = Path(__file__).resolve().parent


class DashboardEarthReferencedScanTests(unittest.TestCase):
    def test_dashboard_sector_defaults_to_true_bearing(self):
        source = (ROOT / "VanguardxMain_scheduler.py").read_text(
            encoding="utf-8"
        )
        self.assertIn('"X660ScanFrame": "TRUE"', source)
        self.assertNotIn(
            'Config.get("X660ScanFrame", "PLATFORM")',
            source,
        )

    def test_dashboard_labels_distinguish_true_and_relative_frames(self):
        source = (ROOT / "RadarDisplayQt5.py").read_text(
            encoding="utf-8"
        )
        for expected in (
            'QLabel("Start °T")',
            'QLabel("Stop °T")',
            "TRUE BRG",
            "Ship hdg:",
            "deg rel",
        ):
            self.assertIn(expected, source)

    def test_ppi_beam_boundary_requires_true_bearing(self):
        source = (ROOT / "RadarDisplayQt5.py").read_text(
            encoding="utf-8"
        )
        self.assertIn(
            "def SetMeasuredBeamAngle(self, BearingTrueDeg):",
            source,
        )
        self.assertIn(
            "caller must convert X6-60 vessel-relative encoder telemetry",
            source,
        )

    def test_dashboard_true_sector_converts_only_at_x660_boundary(self):
        task = MakeSearchTask(
            TaskId=1,
            SectorStartDeg=40.0,
            SectorStopDeg=90.0,
            ScanRateDegPerSec=20.0,
            SectorFrame=AngleFrame.TRUE,
        )
        navigation = SimpleNamespace(
            HeadingTrueDeg=135.0,
            YawRateDegPerSec=0.0,
        )
        pointing = PointingManager(
            x660=SimpleNamespace(UnlimitedAzimuth=True)
        )

        # The operator/PPI values remain the entered Earth bearings.
        self.assertEqual(task.Sector.StartDeg, 40.0)
        self.assertEqual(task.Sector.StopDeg, 90.0)

        # Only the motor-side boundary is ship adjusted.
        self.assertAlmostEqual(
            pointing._search_endpoint_relative(task, navigation),
            315.0,
        )
        task.Sector.Reverse()
        self.assertAlmostEqual(
            pointing._search_endpoint_relative(task, navigation),
            265.0,
        )


if __name__ == "__main__":
    unittest.main(verbosity=2)
