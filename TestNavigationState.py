"""Regression tests for Vanguard X timestamped navigation state."""

import math
import unittest

from CoordinateFrames import (
    EnuPosition,
    GeodeticPosition,
    target_relative_to_radar,
)
from NavigationState import (
    CircularRouteNavigationSource,
    NavigationPose,
    SimulatedNavigationSource,
)


class _ManualClock:
    def __init__(self, initial_time_sec: float):
        self.now = float(initial_time_sec)

    def __call__(self) -> float:
        return self.now

    def advance(self, delta_sec: float) -> None:
        self.now += float(delta_sec)


class TestNavigationPose(unittest.TestCase):
    def test_heading_is_normalised(self):
        source = SimulatedNavigationSource(initial_heading_deg=370.0)
        pose = source.get_pose()
        self.assertAlmostEqual(pose.HeadingTrueDeg, 10.0)

    def test_pose_freshness_and_validity(self):
        clock = _ManualClock(1000.0)
        source = SimulatedNavigationSource(time_source=clock)
        pose = source.get_pose()

        self.assertTrue(pose.Valid)
        self.assertTrue(pose.is_fresh(0.5, current_time_sec=1000.5))
        self.assertFalse(pose.is_fresh(0.5, current_time_sec=1000.5001))

    def test_invalid_heading_prevents_earth_reference(self):
        source = SimulatedNavigationSource()
        pose = source.get_pose()
        invalid = NavigationPose(
            TimestampSec=pose.TimestampSec,
            SequenceNumber=pose.SequenceNumber,
            Geodetic=pose.Geodetic,
            PositionEnu=pose.PositionEnu,
            VelocityEnu=pose.VelocityEnu,
            HeadingTrueDeg=pose.HeadingTrueDeg,
            PositionValid=True,
            HeadingValid=False,
            VelocityValid=True,
            Source="TEST",
        )
        self.assertFalse(invalid.Valid)


class TestSimulatedNavigationSource(unittest.TestCase):
    def setUp(self):
        self.clock = _ManualClock(1_000_000.0)
        self.origin = GeodeticPosition(
            latitude_deg=-34.4278,
            longitude_deg=150.8931,
            altitude_m=5.0,
        )

    def make_source(self, **kwargs):
        return SimulatedNavigationSource(
            mission_origin=self.origin,
            time_source=self.clock,
            **kwargs,
        )

    def test_initial_pose_is_at_mission_origin(self):
        pose = self.make_source(initial_heading_deg=90.0).get_pose()

        self.assertAlmostEqual(pose.PositionEnu.east_m, 0.0, places=6)
        self.assertAlmostEqual(pose.PositionEnu.north_m, 0.0, places=6)
        self.assertAlmostEqual(pose.Geodetic.latitude_deg, -34.4278, places=8)
        self.assertAlmostEqual(pose.Geodetic.longitude_deg, 150.8931, places=8)
        self.assertEqual(pose.Source, "SIMULATED_NAVIGATION")

    def test_constant_turn_is_deterministic_and_wraps_north(self):
        source = self.make_source(
            initial_heading_deg=350.0,
            turn_rate_deg_per_sec=20.0,
        )

        self.clock.advance(1.0)
        pose = source.get_pose()

        self.assertAlmostEqual(pose.HeadingTrueDeg, 10.0)
        self.assertAlmostEqual(pose.YawRateDegPerSec, 20.0)

    def test_constant_enu_velocity_moves_platform(self):
        source = self.make_source(
            velocity_east_mps=5.0,
            velocity_north_mps=2.0,
        )

        self.clock.advance(10.0)
        pose = source.get_pose()

        self.assertAlmostEqual(pose.PositionEnu.east_m, 50.0)
        self.assertAlmostEqual(pose.PositionEnu.north_m, 20.0)
        self.assertAlmostEqual(pose.VelocityEnu.east_mps, 5.0)
        self.assertAlmostEqual(pose.VelocityEnu.north_mps, 2.0)

        round_trip = source.Frame.to_enu(pose.Geodetic)
        self.assertAlmostEqual(round_trip.east_m, 50.0, places=5)
        self.assertAlmostEqual(round_trip.north_m, 20.0, places=5)

    def test_velocity_change_integrates_old_then_new_velocity(self):
        source = self.make_source(velocity_east_mps=4.0)

        self.clock.advance(5.0)
        source.set_velocity_enu(0.0, 3.0)
        self.clock.advance(2.0)
        pose = source.get_pose()

        self.assertAlmostEqual(pose.PositionEnu.east_m, 20.0)
        self.assertAlmostEqual(pose.PositionEnu.north_m, 6.0)

    def test_sequence_number_increases_per_published_pose(self):
        source = self.make_source()
        first = source.get_pose()
        second = source.get_pose()
        attitude = source.get_attitude()

        self.assertEqual(first.SequenceNumber, 1)
        self.assertEqual(second.SequenceNumber, 2)
        self.assertEqual(attitude.TimestampSec, self.clock.now)

    def test_legacy_attitude_interface_is_preserved(self):
        source = self.make_source(initial_heading_deg=30.0)

        attitude = source.get_attitude()
        self.assertAlmostEqual(attitude.HeadingTrueDeg, 30.0)
        self.assertTrue(attitude.Valid)

        source.set_heading(35.0)
        changed = source.get_attitude()
        self.assertAlmostEqual(changed.HeadingTrueDeg, 35.0)

    def test_time_cannot_move_backwards(self):
        source = self.make_source()
        with self.assertRaisesRegex(ValueError, "cannot move backwards"):
            source.get_pose(current_time_sec=self.clock.now - 1.0)


class TestCircularRouteNavigationSource(unittest.TestCase):
    def setUp(self):
        self.clock = _ManualClock(2_000_000.0)
        self.origin = GeodeticPosition(
            latitude_deg=-34.4278,
            longitude_deg=150.8931,
            altitude_m=0.0,
        )

    def make_source(self, **kwargs):
        return CircularRouteNavigationSource(
            mission_origin=self.origin,
            centre_east_m=5000.0,
            centre_north_m=8000.0,
            radius_m=1000.0,
            speed_mps=10.0,
            time_source=self.clock,
            **kwargs,
        )

    def test_clockwise_route_starts_north_and_heads_east(self):
        pose = self.make_source(
            clockwise=True,
            initial_radial_bearing_deg=0.0,
        ).get_pose()

        self.assertAlmostEqual(pose.PositionEnu.east_m, 5000.0)
        self.assertAlmostEqual(pose.PositionEnu.north_m, 9000.0)
        self.assertAlmostEqual(pose.VelocityEnu.east_mps, 10.0)
        self.assertAlmostEqual(pose.VelocityEnu.north_mps, 0.0)
        self.assertAlmostEqual(pose.HeadingTrueDeg, 90.0)
        self.assertGreater(pose.YawRateDegPerSec, 0.0)
        self.assertEqual(pose.Source, "SIMULATED_CIRCULAR_ROUTE")

    def test_quarter_orbit_position_velocity_and_heading_agree(self):
        source = self.make_source(clockwise=True)
        self.clock.advance(source.OrbitPeriodSec / 4.0)
        pose = source.get_pose()

        self.assertAlmostEqual(pose.PositionEnu.east_m, 6000.0, places=6)
        self.assertAlmostEqual(pose.PositionEnu.north_m, 8000.0, places=6)
        self.assertAlmostEqual(pose.VelocityEnu.east_mps, 0.0, places=6)
        self.assertAlmostEqual(pose.VelocityEnu.north_mps, -10.0, places=6)
        self.assertAlmostEqual(pose.HeadingTrueDeg, 180.0, places=6)

    def test_counterclockwise_route_has_negative_yaw_rate(self):
        pose = self.make_source(clockwise=False).get_pose()

        self.assertAlmostEqual(pose.VelocityEnu.east_mps, -10.0)
        self.assertAlmostEqual(pose.HeadingTrueDeg, 270.0)
        self.assertLess(pose.YawRateDegPerSec, 0.0)

    def test_full_orbit_returns_to_start_without_numeric_drift(self):
        source = self.make_source(initial_radial_bearing_deg=37.0)
        first = source.get_pose()
        self.clock.advance(source.OrbitPeriodSec)
        final = source.get_pose()

        self.assertAlmostEqual(
            final.PositionEnu.east_m,
            first.PositionEnu.east_m,
            places=6,
        )
        self.assertAlmostEqual(
            final.PositionEnu.north_m,
            first.PositionEnu.north_m,
            places=6,
        )
        self.assertAlmostEqual(
            final.HeadingTrueDeg,
            first.HeadingTrueDeg,
            places=6,
        )

    def test_stationary_earth_target_stays_fixed_as_relative_view_changes(self):
        source = self.make_source(clockwise=True)
        target_position = EnuPosition(
            east_m=7500.0,
            north_m=8000.0,
            up_m=0.0,
        )

        first_pose = source.get_pose()
        first_relative = target_relative_to_radar(
            target_position,
            first_pose.PositionEnu,
        )

        self.clock.advance(source.OrbitPeriodSec / 4.0)
        second_pose = source.get_pose()
        second_relative = target_relative_to_radar(
            target_position,
            second_pose.PositionEnu,
        )

        self.assertEqual(target_position.east_m, 7500.0)
        self.assertEqual(target_position.north_m, 8000.0)
        self.assertNotAlmostEqual(
            second_relative.range_m,
            first_relative.range_m,
        )
        self.assertNotAlmostEqual(
            second_relative.true_bearing_deg,
            first_relative.true_bearing_deg,
        )

    def test_rejects_invalid_route_parameters(self):
        with self.assertRaisesRegex(ValueError, "radius_m"):
            CircularRouteNavigationSource(
                mission_origin=self.origin,
                radius_m=0.0,
                time_source=self.clock,
            )
        with self.assertRaisesRegex(ValueError, "speed_mps"):
            CircularRouteNavigationSource(
                mission_origin=self.origin,
                radius_m=1000.0,
                speed_mps=-1.0,
                time_source=self.clock,
            )

    def test_orbit_period_matches_radius_and_speed(self):
        source = self.make_source()
        self.assertAlmostEqual(
            source.OrbitPeriodSec,
            2.0 * math.pi * 1000.0 / 10.0,
        )


if __name__ == "__main__":
    unittest.main()
