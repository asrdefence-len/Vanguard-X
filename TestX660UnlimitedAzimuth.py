#!/usr/bin/env python3
"""Stage 4D regressions for the X6-60-only unlimited-azimuth architecture."""

from __future__ import annotations

from pathlib import Path
from types import ModuleType, SimpleNamespace
import sys
import time
import unittest


# Allow this focused test to run beside the delivered files without requiring
# the rest of the Vanguard repository.  In the real repository the actual
# NavigationState module is used.
try:
    import NavigationState  # noqa: F401
except ImportError:
    NavigationState = ModuleType("NavigationState")

    class PlatformAttitude:
        def __init__(
            self,
            HeadingTrueDeg=0.0,
            PitchDeg=0.0,
            Valid=True,
        ):
            self.HeadingTrueDeg = float(HeadingTrueDeg)
            self.PitchDeg = float(PitchDeg)
            self.Valid = bool(Valid)

    NavigationState.PlatformAttitude = PlatformAttitude
    sys.modules["NavigationState"] = NavigationState


from NavigationState import PlatformAttitude
from PointingManager import PointingManager
from RadarTasks import AngleFrame, MakeSearchTask, SearchPattern
from X660Controller import CreateX660Controller, SimulatedX660Controller
from X660ReadOnlyController import (
    RawAngleToAzimuthDeg,
    X660ReadOnlyController,
)


class FakeUnlimitedX660:
    MotionCommandsEnabled = True
    UnlimitedAzimuth = True
    SupportsContinuousRotation = True
    TelemetryWhileStopped = True

    def __init__(self, AzimuthDeg=0.0):
        self.State = SimpleNamespace(
            AzimuthDeg=float(AzimuthDeg) % 360.0,
            ElevationDeg=0.0,
            PanRateDegPerSec=0.0,
            Valid=True,
            Source="TEST_X660",
        )
        self.SlewCommands = []
        self.PositionCommands = []
        self.StopCount = 0

    def Update(self):
        return self.State

    def GetState(self):
        return self.State

    def CommandSlew(self, RateDegPerSec, TiltRateDegPerSec=0.0):
        self.SlewCommands.append(float(RateDegPerSec))
        self.State.PanRateDegPerSec = float(RateDegPerSec)

    def SetPanPositionNative(self, TargetDeg):
        self.PositionCommands.append(float(TargetDeg) % 360.0)

    def Stop(self):
        self.StopCount += 1
        self.State.PanRateDegPerSec = 0.0


class TestX660UnlimitedAzimuth(unittest.TestCase):
    def setUp(self):
        self.Navigation = PlatformAttitude(
            HeadingTrueDeg=0.0,
            PitchDeg=0.0,
            Valid=True,
        )

    def test_north_mapping_is_zero_and_clockwise_is_positive(self):
        self.assertAlmostEqual(
            RawAngleToAzimuthDeg(-361.53, -361.53, +1),
            0.0,
        )
        self.assertAlmostEqual(
            RawAngleToAzimuthDeg(-350.00, -361.53, +1),
            11.53,
        )
        self.assertAlmostEqual(
            RawAngleToAzimuthDeg(10.00, -361.53, +1),
            11.53,
        )

    def test_read_only_controller_is_unlimited_and_motion_locked(self):
        self.assertTrue(X660ReadOnlyController.UnlimitedAzimuth)
        self.assertTrue(X660ReadOnlyController.SupportsContinuousRotation)
        self.assertTrue(X660ReadOnlyController.TelemetryWhileStopped)
        self.assertFalse(X660ReadOnlyController.MotionCommandsEnabled)
        controller = X660ReadOnlyController(DirectionSign=+1)
        self.assertFalse(hasattr(controller, "LeftLimitDeg"))
        self.assertFalse(hasattr(controller, "RightLimitDeg"))

    def test_factory_has_only_x660_modes(self):
        simulated = CreateX660Controller({"X660Mode": "x660-sim"})
        self.assertIsInstance(simulated, SimulatedX660Controller)
        with self.assertRaises(ValueError):
            CreateX660Controller({"X660Mode": "pelco"})

    def test_simulator_preserves_multiturn_raw_angle_across_north(self):
        controller = SimulatedX660Controller(
            InitialAzimuthDeg=359.5,
            MaxPanRateDegPerSec=10.0,
            NorthRawAngleDeg=-361.53,
            DirectionSign=+1,
        )
        controller.CommandSlew(+10.0)
        controller.LastUpdateSec = time.time() - 0.20
        state = controller.Update()
        self.assertGreater(state.RawAngleDeg, -1.53)
        self.assertGreaterEqual(state.AzimuthDeg, 0.0)
        self.assertLess(state.AzimuthDeg, 5.0)

    def test_all_bearings_are_reachable_and_nudge_wraps(self):
        x660 = FakeUnlimitedX660(359.5)
        pointing = PointingManager(x660=x660)
        for bearing in (-720.0, -1.0, 0.0, 301.0, 720.0):
            self.assertTrue(pointing.IsRelativeAzimuthReachable(bearing))
        self.assertAlmostEqual(pointing.ClampRelativeAzimuth(361.0), 1.0)
        self.assertAlmostEqual(pointing.Nudge(+1.0), 0.5)
        self.assertEqual(x660.PositionCommands, [0.5])

    def test_sector_scan_can_cross_north_and_still_reverse(self):
        x660 = FakeUnlimitedX660(350.0)
        pointing = PointingManager(x660=x660, endpoint_margin_deg=0.25)
        task = MakeSearchTask(
            TaskId=1,
            SectorStartDeg=350.0,
            SectorStopDeg=10.0,
            ScanRateDegPerSec=5.0,
            SectorFrame=AngleFrame.PLATFORM,
            Pattern=SearchPattern.SECTOR,
        )
        pointing.ActivateTask(task, self.Navigation)
        self.assertEqual(x660.SlewCommands[-1], +5.0)

        pointing.Update(self.Navigation)
        x660.State.AzimuthDeg = 10.1
        x660.State.PanRateDegPerSec = +5.0
        pointing.Update(self.Navigation)

        self.assertEqual(task.Sector.ActiveEndpoint, "START")
        self.assertEqual(task.Sector.ScanCycle, 2)
        self.assertEqual(x660.SlewCommands[-1], -5.0)

    def test_continuous_cw_scan_never_reverses_at_north(self):
        x660 = FakeUnlimitedX660(359.0)
        pointing = PointingManager(x660=x660, endpoint_margin_deg=0.25)
        task = MakeSearchTask(
            TaskId=1,
            SectorStartDeg=0.0,
            SectorStopDeg=0.0,
            ScanRateDegPerSec=6.0,
            SectorFrame=AngleFrame.PLATFORM,
            Pattern=SearchPattern.CONTINUOUS_CW,
        )
        pointing.ActivateTask(task, self.Navigation)
        pointing.Update(self.Navigation)

        x660.State.AzimuthDeg = 0.5
        x660.State.PanRateDegPerSec = +6.0
        state = pointing.Update(self.Navigation)

        self.assertEqual(x660.SlewCommands, [+6.0])
        self.assertEqual(task.Sector.ScanCycle, 2)
        self.assertEqual(state.SearchActiveEndpoint, "CONTINUOUS_CW")

    def test_active_sources_contain_no_old_controller_configuration(self):
        root = Path(__file__).resolve().parent
        sources = [
            root / "VanguardxMain_scheduler.py",
            root / "PointingManager.py",
            root / "RadarExecutor.py",
            root / "RadarTasks.py",
            root / "RadarScheduler.py",
            root / "RadarDisplayQt5.py",
            root / "X660Controller.py",
            root / "X660ReadOnlyController.py",
        ]
        forbidden = (
            "PTZLeftLimitDeg",
            "PTZRightLimitDeg",
            "PTZMode",
            "PelcoDPTZController",
            "SimulatedPTZController",
        )
        combined = "\n".join(path.read_text() for path in sources)
        for token in forbidden:
            self.assertNotIn(token, combined)

    def test_stopped_display_refresh_path_is_present(self):
        root = Path(__file__).resolve().parent
        main_source = (root / "VanguardxMain_scheduler.py").read_text()
        display_source = (root / "RadarDisplayQt5.py").read_text()
        self.assertIn("RefreshX660MeasuredBeam", main_source)
        self.assertIn('DisplayMode == "STOP"', main_source)
        self.assertIn("def SetMeasuredBeamAngle", display_source)


if __name__ == "__main__":
    unittest.main()
