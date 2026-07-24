#!/usr/bin/env python3
"""Regression tests for X6-60 sector-scan braking anticipation."""

from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace
import unittest

from NavigationState import PlatformAttitude
from PointingManager import PointingManager
from RadarTasks import AngleFrame, MakeSearchTask, SearchPattern


class FakeX660:
    UnlimitedAzimuth = True

    def __init__(self, azimuth_deg: float):
        self.State = SimpleNamespace(
            AzimuthDeg=float(azimuth_deg) % 360.0,
            ElevationDeg=0.0,
            PanRateDegPerSec=0.0,
            Valid=True,
            Source="TEST_X660",
        )
        self.SlewCommands = []

    def Update(self):
        return self.State

    def GetState(self):
        return self.State

    def CommandSlew(self, rate_deg_per_sec, tilt_rate_deg_per_sec=0.0):
        rate = float(rate_deg_per_sec)
        self.SlewCommands.append(rate)
        self.State.PanRateDegPerSec = rate

    def Stop(self):
        self.State.PanRateDegPerSec = 0.0


def MakeSector(start_deg=10.0, stop_deg=120.0, rate_deg_per_sec=40.0):
    return MakeSearchTask(
        TaskId=1,
        SectorStartDeg=start_deg,
        SectorStopDeg=stop_deg,
        ScanRateDegPerSec=rate_deg_per_sec,
        SectorFrame=AngleFrame.PLATFORM,
        Pattern=SearchPattern.SECTOR,
    )


class ScanBrakingTests(unittest.TestCase):
    def setUp(self):
        self.Navigation = PlatformAttitude(
            HeadingTrueDeg=0.0,
            PitchDeg=0.0,
            Valid=True,
        )

    def MakePointing(self, x660, **overrides):
        settings = dict(
            endpoint_margin_deg=0.25,
            scan_braking_enabled=True,
            scan_deceleration_deg_per_sec2=60.0,
            scan_command_latency_sec=0.05,
        )
        settings.update(overrides)
        return PointingManager(x660=x660, **settings)

    def test_40_deg_per_sec_reverses_about_15_3_deg_before_boundary(self):
        x660 = FakeX660(90.0)
        pointing = self.MakePointing(x660)
        task = MakeSector()
        pointing.ActivateTask(task, self.Navigation)

        # v^2/(2a) + v*t = 1600/120 + 2 = 15.333 degrees.
        x660.State.AzimuthDeg = 104.5  # 15.5 degrees remain: keep scanning.
        x660.State.PanRateDegPerSec = +40.0
        pointing.Update(self.Navigation)
        self.assertEqual(task.Sector.ActiveEndpoint, "STOP")
        self.assertEqual(x660.SlewCommands[-1], +40.0)

        x660.State.AzimuthDeg = 105.0  # 15.0 degrees remain: reverse now.
        x660.State.PanRateDegPerSec = +40.0
        pointing.Update(self.Navigation)
        self.assertEqual(task.Sector.ActiveEndpoint, "START")
        self.assertEqual(task.Sector.ScanCycle, 2)
        self.assertEqual(x660.SlewCommands[-1], -40.0)

    def test_advance_uses_actual_speed_not_selected_scan_rate(self):
        x660 = FakeX660(112.0)
        pointing = self.MakePointing(x660)
        task = MakeSector(rate_deg_per_sec=40.0)
        pointing.ActivateTask(task, self.Navigation)

        # Actual speed is only 20 deg/s while ramping.  Its advance is
        # 400/120 + 1 = 4.333 degrees, so 8 degrees remaining is too early.
        x660.State.PanRateDegPerSec = +20.0
        pointing.Update(self.Navigation)
        self.assertEqual(task.Sector.ActiveEndpoint, "STOP")

        x660.State.AzimuthDeg = 116.0
        x660.State.PanRateDegPerSec = +20.0
        pointing.Update(self.Navigation)
        self.assertEqual(task.Sector.ActiveEndpoint, "START")

    def test_does_not_reverse_while_still_moving_away_from_new_endpoint(self):
        x660 = FakeX660(105.0)
        pointing = self.MakePointing(x660)
        task = MakeSector()
        pointing.ActivateTask(task, self.Navigation)

        x660.State.PanRateDegPerSec = +40.0
        pointing.Update(self.Navigation)
        self.assertEqual(task.Sector.ActiveEndpoint, "START")

        # The reverse command has been sent, but telemetry can still report
        # positive speed during deceleration.  START is behind the antenna, so
        # no second reversal is allowed.
        x660.State.AzimuthDeg = 112.0
        x660.State.PanRateDegPerSec = +20.0
        pointing.Update(self.Navigation)
        self.assertEqual(task.Sector.ActiveEndpoint, "START")
        self.assertEqual(task.Sector.ScanCycle, 2)

    def test_braking_can_be_disabled_for_legacy_endpoint_crossing(self):
        x660 = FakeX660(105.0)
        pointing = self.MakePointing(
            x660,
            scan_braking_enabled=False,
        )
        task = MakeSector()
        pointing.ActivateTask(task, self.Navigation)

        x660.State.PanRateDegPerSec = +40.0
        pointing.Update(self.Navigation)
        self.assertEqual(task.Sector.ActiveEndpoint, "STOP")

        x660.State.AzimuthDeg = 120.1
        x660.State.PanRateDegPerSec = +40.0
        pointing.Update(self.Navigation)
        self.assertEqual(task.Sector.ActiveEndpoint, "START")

    def test_scheduler_enables_compensation_only_for_operational_x660(self):
        source = (
            Path(__file__).resolve().parent / "VanguardxMain_scheduler.py"
        ).read_text(encoding="utf-8")
        self.assertIn('"X660ScanBrakingEnabled": True', source)
        self.assertIn('"X660ScanDecelerationDegPerSec2": 60.0', source)
        self.assertIn('"X660ScanCommandLatencySec": 0.05', source)
        self.assertIn(
            'str(Config.get("X660Mode", "")).lower() in (',
            source,
        )

    def test_invalid_braking_parameters_are_rejected(self):
        x660 = FakeX660(90.0)
        with self.assertRaises(ValueError):
            self.MakePointing(
                x660,
                scan_deceleration_deg_per_sec2=0.0,
            )
        with self.assertRaises(ValueError):
            self.MakePointing(
                x660,
                scan_deceleration_deg_per_sec2=-60.0,
            )
        with self.assertRaises(ValueError):
            self.MakePointing(
                x660,
                scan_command_latency_sec=-0.01,
            )


if __name__ == "__main__":
    unittest.main()
