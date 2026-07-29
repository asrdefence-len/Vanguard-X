"""Regression tests for Vanguard X coordinate geometry."""

import math
import unittest

from CoordinateFrames import (
    EnuPosition,
    EnuVelocity,
    GeodeticPosition,
    LocalEnuFrame,
    detection_to_enu_position,
    enu_delta_to_relative_polar,
    line_of_sight_velocity_mps,
    measured_relative_radial_velocity_mps,
    normalise_bearing_degrees,
    relative_polar_to_enu_delta,
    signed_angle_difference_degrees,
    target_los_velocity_from_measured_mps,
    target_relative_to_radar,
    true_bearing_to_x660_encoder,
    x660_encoder_to_true_bearing,
)


class TestAngles(unittest.TestCase):
    def test_bearing_normalisation(self):
        self.assertEqual(normalise_bearing_degrees(360.0), 0.0)
        self.assertEqual(normalise_bearing_degrees(-90.0), 270.0)
        self.assertEqual(normalise_bearing_degrees(810.0), 90.0)

    def test_signed_angle_difference_crosses_north(self):
        self.assertAlmostEqual(
            signed_angle_difference_degrees(2.0, 358.0),
            4.0,
        )
        self.assertAlmostEqual(
            signed_angle_difference_degrees(358.0, 2.0),
            -4.0,
        )


class TestPolarAndEnu(unittest.TestCase):
    def test_cardinal_true_bearings(self):
        cases = (
            (0.0, 0.0, 1000.0),
            (90.0, 1000.0, 0.0),
            (180.0, 0.0, -1000.0),
            (270.0, -1000.0, 0.0),
        )
        for bearing_deg, expected_east_m, expected_north_m in cases:
            with self.subTest(bearing_deg=bearing_deg):
                position = relative_polar_to_enu_delta(1000.0, bearing_deg)
                self.assertAlmostEqual(position.east_m, expected_east_m)
                self.assertAlmostEqual(position.north_m, expected_north_m)

    def test_example_from_coordinate_specification(self):
        polar = enu_delta_to_relative_polar(500.0, 300.0)
        self.assertAlmostEqual(polar.range_m, math.sqrt(500.0**2 + 300.0**2))
        self.assertAlmostEqual(polar.true_bearing_deg, 59.0362434679)

    def test_detection_is_placed_in_mission_frame(self):
        radar = EnuPosition(east_m=20_000.0, north_m=0.0)
        target = detection_to_enu_position(
            radar,
            measured_range_m=math.sqrt(5_000.0**2 + 3_000.0**2),
            true_bearing_deg=math.degrees(math.atan2(5_000.0, 3_000.0)),
        )
        self.assertAlmostEqual(target.east_m, 25_000.0)
        self.assertAlmostEqual(target.north_m, 3_000.0)

    def test_relative_output_changes_when_radar_moves(self):
        target = EnuPosition(east_m=25_000.0, north_m=3_000.0)
        first_radar_position = EnuPosition(east_m=0.0, north_m=0.0)
        second_radar_position = EnuPosition(east_m=20_000.0, north_m=0.0)

        first_relative = target_relative_to_radar(
            target,
            first_radar_position,
        )
        second_relative = target_relative_to_radar(
            target,
            second_radar_position,
        )

        self.assertGreater(first_relative.range_m, second_relative.range_m)
        self.assertAlmostEqual(
            second_relative.range_m,
            math.sqrt(5_000.0**2 + 3_000.0**2),
        )
        self.assertAlmostEqual(second_relative.true_bearing_deg, 59.0362434679)


class TestWgs84EnuFrame(unittest.TestCase):
    def setUp(self):
        self.origin = GeodeticPosition(
            latitude_deg=-33.8688,
            longitude_deg=151.2093,
            altitude_m=12.0,
        )
        self.frame = LocalEnuFrame(self.origin)

    def test_origin_is_enu_zero(self):
        position = self.frame.to_enu(self.origin)
        self.assertAlmostEqual(position.east_m, 0.0)
        self.assertAlmostEqual(position.north_m, 0.0)
        self.assertAlmostEqual(position.up_m, 0.0)

    def test_latitude_and_longitude_have_expected_axis_signs(self):
        north = self.frame.to_enu(
            GeodeticPosition(-33.8678, 151.2093, 12.0)
        )
        east = self.frame.to_enu(
            GeodeticPosition(-33.8688, 151.2103, 12.0)
        )
        self.assertGreater(north.north_m, 0.0)
        self.assertGreater(east.east_m, 0.0)

    def test_40_kilometre_operating_extent_round_trip(self):
        # Represents 20 km vessel travel plus a further 20 km radar extent.
        source = EnuPosition(
            east_m=40_000.0,
            north_m=-40_000.0,
            up_m=-250.0,
        )
        geodetic = self.frame.to_geodetic(source)
        recovered = self.frame.to_enu(geodetic)

        self.assertAlmostEqual(recovered.east_m, source.east_m, places=5)
        self.assertAlmostEqual(recovered.north_m, source.north_m, places=5)
        self.assertAlmostEqual(recovered.up_m, source.up_m, places=5)


class TestX660Geometry(unittest.TestCase):
    def test_encoder_zero_points_aft(self):
        # Vessel heading true east; aft is true west.
        true_bearing = x660_encoder_to_true_bearing(
            encoder_angle_deg=0.0,
            vessel_true_heading_deg=90.0,
        )
        self.assertAlmostEqual(true_bearing, 270.0)

    def test_encoder_true_bearing_round_trip(self):
        for vessel_heading_deg in (0.0, 45.0, 359.0):
            for encoder_angle_deg in (0.0, 1.0, 90.0, 270.0, 359.0):
                with self.subTest(
                    vessel_heading_deg=vessel_heading_deg,
                    encoder_angle_deg=encoder_angle_deg,
                ):
                    true_bearing = x660_encoder_to_true_bearing(
                        encoder_angle_deg,
                        vessel_heading_deg,
                        installation_correction_deg=2.5,
                    )
                    recovered = true_bearing_to_x660_encoder(
                        true_bearing,
                        vessel_heading_deg,
                        installation_correction_deg=2.5,
                    )
                    self.assertAlmostEqual(recovered, encoder_angle_deg)

    def test_counterclockwise_encoder_option_round_trip(self):
        true_bearing = x660_encoder_to_true_bearing(
            encoder_angle_deg=30.0,
            vessel_true_heading_deg=10.0,
            encoder_clockwise_positive=False,
        )
        self.assertAlmostEqual(true_bearing, 160.0)
        self.assertAlmostEqual(
            true_bearing_to_x660_encoder(
                true_bearing,
                vessel_true_heading_deg=10.0,
                encoder_clockwise_positive=False,
            ),
            30.0,
        )


class TestVelocityProjection(unittest.TestCase):
    def test_ownship_velocity_projection_uses_line_of_sight_component(self):
        eastbound = EnuVelocity(east_mps=10.0, north_mps=0.0)
        self.assertAlmostEqual(
            line_of_sight_velocity_mps(eastbound, true_bearing_deg=90.0),
            10.0,
        )
        self.assertAlmostEqual(
            line_of_sight_velocity_mps(eastbound, true_bearing_deg=0.0),
            0.0,
        )
        self.assertAlmostEqual(
            line_of_sight_velocity_mps(eastbound, true_bearing_deg=270.0),
            -10.0,
        )

    def test_measured_doppler_combines_target_and_platform_motion_once(self):
        target_velocity = EnuVelocity(
            east_mps=0.0,
            north_mps=8.0,
        )
        radar_velocity = EnuVelocity(
            east_mps=0.0,
            north_mps=5.0,
        )

        measured = measured_relative_radial_velocity_mps(
            target_velocity,
            radar_velocity,
            true_bearing_deg=0.0,
        )

        self.assertAlmostEqual(measured, 3.0)
        self.assertAlmostEqual(
            target_los_velocity_from_measured_mps(
                measured,
                radar_velocity,
                true_bearing_deg=0.0,
            ),
            8.0,
        )

    def test_stationary_target_has_ownship_induced_measured_doppler(self):
        stationary_target = EnuVelocity(0.0, 0.0)
        radar_velocity = EnuVelocity(0.0, 5.0)

        measured = measured_relative_radial_velocity_mps(
            stationary_target,
            radar_velocity,
            true_bearing_deg=0.0,
        )

        self.assertAlmostEqual(measured, -5.0)
        self.assertAlmostEqual(
            target_los_velocity_from_measured_mps(
                measured,
                radar_velocity,
                true_bearing_deg=0.0,
            ),
            0.0,
        )


if __name__ == "__main__":
    unittest.main(verbosity=2)
