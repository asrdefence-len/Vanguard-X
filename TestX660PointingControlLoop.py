#!/usr/bin/env python3
"""Regression tests for the independent 20 ms X6-60 control clock."""

from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace
import unittest

from X660PointingControlLoop import X660PointingControlLoop


class FakeX660:
    def __init__(self):
        self.State = SimpleNamespace(
            AzimuthDeg=42.5,
            PanRateDegPerSec=40.0,
            Valid=True,
            Source="TEST_X660",
            AtTarget=False,
            RawAngleDeg=-319.03,
        )

    def GetState(self):
        return self.State


class FakePointing:
    def __init__(self, x660):
        self.Positioner = x660
        self.UpdateCount = 0

    def Update(self, navigation):
        self.UpdateCount += 1
        return SimpleNamespace(
            AntennaAzimuthRelativeDeg=self.Positioner.State.AzimuthDeg,
            PanRateDegPerSec=self.Positioner.State.PanRateDegPerSec,
            Valid=True,
            Source="TEST_POINTING",
            Ready=False,
        )


class FakeDisplay:
    def __init__(self):
        self.Angles = []

    def SetMeasuredBeamAngle(self, angle_deg):
        self.Angles.append(float(angle_deg))


class PointingControlLoopTests(unittest.TestCase):
    def test_runs_at_20_ms_and_not_at_radar_dwell_rate(self):
        x660 = FakeX660()
        pointing = FakePointing(x660)
        control = X660PointingControlLoop(interval_sec=0.020)

        self.assertIsNotNone(
            control.TickIfDue(pointing, object(), x660, now_monotonic_sec=1.0)
        )
        self.assertIsNone(
            control.TickIfDue(
                pointing,
                object(),
                x660,
                now_monotonic_sec=1.019,
            )
        )
        self.assertIsNotNone(
            control.TickIfDue(
                pointing,
                object(),
                x660,
                now_monotonic_sec=1.020,
            )
        )
        self.assertEqual(pointing.UpdateCount, 2)

    def test_skips_missed_ticks_instead_of_replaying_can_updates(self):
        x660 = FakeX660()
        pointing = FakePointing(x660)
        control = X660PointingControlLoop(interval_sec=0.020)

        control.TickIfDue(pointing, object(), x660, now_monotonic_sec=2.0)
        control.TickIfDue(pointing, object(), x660, now_monotonic_sec=2.105)
        self.assertGreaterEqual(control.MissedTickCount, 4)

        self.assertIsNone(
            control.TickIfDue(
                pointing,
                object(),
                x660,
                now_monotonic_sec=2.106,
            )
        )
        self.assertEqual(pointing.UpdateCount, 2)

    def test_publishes_fresh_motor_state_and_timing_diagnostics(self):
        x660 = FakeX660()
        pointing = FakePointing(x660)
        display = FakeDisplay()
        config = {}
        control = X660PointingControlLoop(interval_sec=0.020)

        control.TickIfDue(
            pointing,
            object(),
            x660,
            display,
            config,
            now_monotonic_sec=3.0,
        )
        x660.State.AzimuthDeg = 43.3
        control.TickIfDue(
            pointing,
            object(),
            x660,
            display,
            config,
            now_monotonic_sec=3.021,
        )

        self.assertEqual(display.Angles, [42.5, 43.3])
        self.assertAlmostEqual(config["X660AzDeg"], 43.3)
        self.assertAlmostEqual(config["X660RateDegPerSec"], 40.0)
        self.assertAlmostEqual(
            config["X660PointingControlActualIntervalSec"],
            0.021,
        )
        self.assertAlmostEqual(
            control.GetTimingDiagnostics()["MaximumAbsoluteJitterSec"],
            0.001,
        )

    def test_scheduler_has_independent_20_ms_control_and_telemetry_clocks(self):
        source = (
            Path(__file__).resolve().parent / "VanguardxMain_scheduler.py"
        ).read_text(encoding="utf-8")
        self.assertIn('"X660PointingControlIntervalSec": 0.020', source)
        self.assertIn('"X660TelemetryIntervalSec": 0.020', source)
        self.assertIn('"RadarDwellIntervalSec": 0.10', source)
        self.assertIn("X660PointingControlLoop(", source)
        self.assertIn("PointingControl.TickIfDue(", source)

    def test_rejects_invalid_intervals(self):
        for interval_sec in (0.0, -0.02, float("inf"), float("nan")):
            with self.assertRaises(ValueError):
                X660PointingControlLoop(interval_sec=interval_sec)


if __name__ == "__main__":
    unittest.main()
