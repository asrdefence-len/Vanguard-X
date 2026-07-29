"""Moving-platform regression tests for the Vanguard target simulator."""

import unittest

from TargetScenario import calculate_object_geometry


class TestTargetScenarioPlatformMotion(unittest.TestCase):
    def setUp(self):
        self.stationary_target_north = {
            "name": "Stationary_Buoy",
            # TargetScenario legacy axes: x=north, y=east.
            "x_m": 1000.0,
            "y_m": 0.0,
            "vx_mps": 0.0,
            "vy_mps": 0.0,
            "rcs": 10.0,
        }
        self.base_radar = {
            "carrier_frequency_hz": 9.4e9,
            "radar_x_m": 0.0,
            "radar_y_m": 0.0,
            "radar_vx_mps": 0.0,
            "radar_vy_mps": 0.0,
        }

    def test_stationary_radar_observes_stationary_target_at_zero_doppler(self):
        geometry = calculate_object_geometry(
            self.stationary_target_north,
            self.base_radar,
        )

        self.assertAlmostEqual(geometry["range_m"], 1000.0)
        self.assertAlmostEqual(geometry["bearing_deg"], 0.0)
        self.assertAlmostEqual(geometry["radial_velocity_mps"], 0.0)
        self.assertAlmostEqual(geometry["doppler_hz"], 0.0)

    def test_ownship_velocity_is_subtracted_before_doppler_projection(self):
        radar = dict(self.base_radar)
        radar["radar_vx_mps"] = 5.0

        geometry = calculate_object_geometry(
            self.stationary_target_north,
            radar,
        )

        # Positive radial velocity is outward/receding.  The radar moving
        # north toward this buoy therefore measures negative (closing) speed.
        self.assertAlmostEqual(geometry["radial_velocity_mps"], -5.0)
        expected_doppler_hz = (
            2.0 * -5.0
            / (3.0e8 / self.base_radar["carrier_frequency_hz"])
        )
        self.assertAlmostEqual(
            geometry["doppler_hz"],
            expected_doppler_hz,
        )
        self.assertAlmostEqual(geometry["radar_los_velocity_mps"], 5.0)
        self.assertAlmostEqual(geometry["target_los_velocity_mps"], 0.0)

    def test_cross_line_of_sight_ownship_motion_has_no_doppler(self):
        radar = dict(self.base_radar)
        radar["radar_vy_mps"] = 5.0

        geometry = calculate_object_geometry(
            self.stationary_target_north,
            radar,
        )

        self.assertAlmostEqual(geometry["radial_velocity_mps"], 0.0)
        self.assertAlmostEqual(geometry["doppler_hz"], 0.0)

    def test_target_and_ownship_velocity_are_combined_once(self):
        moving_target = dict(self.stationary_target_north)
        moving_target["vx_mps"] = 8.0
        radar = dict(self.base_radar)
        radar["radar_vx_mps"] = 5.0

        geometry = calculate_object_geometry(moving_target, radar)

        self.assertAlmostEqual(
            geometry["measured_relative_radial_velocity_mps"],
            3.0,
        )
        self.assertAlmostEqual(geometry["radial_velocity_mps"], 3.0)
        self.assertAlmostEqual(geometry["target_los_velocity_mps"], 8.0)
        self.assertAlmostEqual(geometry["radar_los_velocity_mps"], 5.0)

    def test_platform_position_changes_relative_range_and_bearing(self):
        first = calculate_object_geometry(
            self.stationary_target_north,
            self.base_radar,
        )
        moved_radar = dict(self.base_radar)
        moved_radar["radar_x_m"] = 100.0
        moved_radar["radar_y_m"] = 100.0
        second = calculate_object_geometry(
            self.stationary_target_north,
            moved_radar,
        )

        self.assertAlmostEqual(first["range_m"], 1000.0)
        self.assertNotAlmostEqual(second["range_m"], first["range_m"])
        self.assertNotAlmostEqual(
            second["bearing_deg"],
            first["bearing_deg"],
        )


if __name__ == "__main__":
    unittest.main()
