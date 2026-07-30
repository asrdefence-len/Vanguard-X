"""Tests for the moving-platform PPI operational acceptance gate."""

from pathlib import Path
import unittest

from MovingPlatformPpiOperationalValidation import (
    RunMovingPlatformPpiValidation,
)


class MovingPlatformPpiOperationalValidationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.Result = RunMovingPlatformPpiValidation(frame_count=73)

    def test_complete_gate_passes(self):
        self.assertTrue(self.Result.Passed)

    def test_route_is_two_kilometres_in_diameter(self):
        self.assertEqual(self.Result.RouteDiameterM, 2000.0)

    def test_true_sector_is_numerically_fixed(self):
        self.assertLessEqual(
            self.Result.MaximumSectorBearingErrorDeg,
            1.0e-9,
        )

    def test_true_scan_rate_is_yaw_compensated(self):
        self.assertLessEqual(
            self.Result.MaximumTrueScanRateErrorDegPerSec,
            1.0e-9,
        )

    def test_trajectory_always_ends_at_radar_centre(self):
        self.assertLessEqual(
            self.Result.MaximumTrailEndpointErrorM,
            1.0e-9,
        )

    def test_normal_simulator_defaults_to_circular_route(self):
        main_source = Path("VanguardxMain_scheduler.py").read_text(
            encoding="utf-8"
        )
        self.assertIn(
            '"SimulatedNavigationMode": "CIRCLE"',
            main_source,
        )


if __name__ == "__main__":
    unittest.main()
