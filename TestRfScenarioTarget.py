"""Tests for TargetScenario-to-single-RF-target selection."""

import unittest

from RfScenarioTarget import SelectStrongestScenarioTarget


class TestRfScenarioTarget(unittest.TestCase):
    def test_selects_strongest_parent_inside_two_degree_gate(self):
        returns = [
            {
                "name": "ShipA_S01", "parent_name": "ShipA",
                "range_m": 5900.0, "bearing_deg": 79.8,
                "radial_velocity_mps": -3.0, "amplitude": 2.0,
            },
            {
                "name": "ShipA_S02", "parent_name": "ShipA",
                "range_m": 6100.0, "bearing_deg": 80.2,
                "radial_velocity_mps": -3.0, "amplitude": 2.0,
            },
            {
                "name": "Buoy", "parent_name": "Buoy",
                "range_m": 5000.0, "bearing_deg": 80.0,
                "radial_velocity_mps": 0.0, "amplitude": 1.0,
            },
            {
                "name": "OutOfBeam", "parent_name": "OutOfBeam",
                "range_m": 4000.0, "bearing_deg": 82.01,
                "radial_velocity_mps": 0.0, "amplitude": 100.0,
            },
        ]

        target = SelectStrongestScenarioTarget(
            returns,
            BoresightDeg=80.0,
            AngleHalfWidthDeg=2.0,
        )

        self.assertEqual(target["Name"], "ShipA")
        self.assertAlmostEqual(target["RangeM"], 6000.0)
        self.assertAlmostEqual(target["BearingDeg"], 80.0, places=5)
        self.assertAlmostEqual(target["RadialVelocityMps"], -3.0)
        self.assertEqual(target["ConstituentReturnCount"], 2)

    def test_returns_none_when_no_target_is_in_gate(self):
        target = SelectStrongestScenarioTarget(
            [{
                "name": "Ship", "range_m": 6000.0,
                "bearing_deg": 83.0, "radial_velocity_mps": 0.0,
                "amplitude": 10.0,
            }],
            BoresightDeg=80.0,
            AngleHalfWidthDeg=2.0,
        )
        self.assertIsNone(target)

    def test_excludes_targets_outside_receive_range(self):
        target = SelectStrongestScenarioTarget(
            [{
                "name": "BeyondWindow", "range_m": 16000.0,
                "bearing_deg": 80.0, "radial_velocity_mps": 0.0,
                "amplitude": 10.0,
            }],
            BoresightDeg=80.0,
            MaximumRangeM=15000.0,
        )
        self.assertIsNone(target)


if __name__ == "__main__":
    unittest.main()
