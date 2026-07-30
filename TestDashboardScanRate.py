#!/usr/bin/env python3
"""Regression tests for the front-dashboard X6-60 scan-rate control."""

from pathlib import Path
import unittest

from RadarRemoteDisplay import (
    CONTROL_KEYS,
    PUBLIC_CONFIG_KEYS,
    _InitialControlState,
)


ROOT = Path(__file__).parent


class DashboardScanRateTests(unittest.TestCase):
    def test_remote_control_state_carries_dashboard_scan_rate(self):
        config = {
            "X660ScanSlewRateDegPerSec": 37.5,
            "X660OperationalMaxRateDegPerSec": 60.0,
        }
        state = _InitialControlState(config)
        self.assertEqual(state["ScanRateDegPerSec"], 37.5)
        self.assertIn("ScanRateDegPerSec", CONTROL_KEYS)
        self.assertIn("X660ScanSlewRateDegPerSec", PUBLIC_CONFIG_KEYS)
        self.assertIn("X660OperationalMaxRateDegPerSec", PUBLIC_CONFIG_KEYS)

    def test_remote_control_state_carries_manual_control_edge(self):
        state = _InitialControlState({})
        self.assertEqual(state["ManualControlCommandId"], 0)
        self.assertIn("ManualControlCommandId", CONTROL_KEYS)

    def test_dashboard_places_scan_rate_on_start_stop_pulses_row(self):
        source = (ROOT / "RadarDisplayQt5.py").read_text(encoding="utf-8")
        self.assertIn('ScanRateLabel = QtWidgets.QLabel("Scan rate")', source)
        self.assertIn(
            'Layout.addWidget(ScanRateLabel, 2, 6)',
            source,
        )
        self.assertIn(
            'Layout.addWidget(self.ControlWidgets["ScanRate"], 2, 7)',
            source,
        )
        self.assertIn('"ScanRateDegPerSec": self.ScanRateDegPerSec', source)
        self.assertIn(
            '"ManualControlCommandId": self.ManualControlCommandId',
            source,
        )

    def test_scheduler_uses_dashboard_rate_below_independent_ceiling(self):
        source = (ROOT / "VanguardxMain_scheduler.py").read_text(
            encoding="utf-8"
        )
        self.assertIn('"X660ScanSlewRateDegPerSec": 20.0', source)
        self.assertIn('"X660OperationalMaxRateDegPerSec": 60.0', source)
        self.assertIn('"X660SimMaxRateDegPerSec": 60.0', source)
        self.assertIn('"ScanRateDegPerSec" in ControlState', source)
        self.assertIn('"MissionScanRateDegSec" in ControlState', source)


if __name__ == "__main__":
    unittest.main()
