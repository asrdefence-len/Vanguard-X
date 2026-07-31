"""Regression tests for non-authoritative Earth-referenced detections."""

from pathlib import Path
from types import SimpleNamespace
import unittest

from CoordinateFrames import (
    EnuPosition,
    EnuVelocity,
    GeodeticPosition,
    measured_relative_radial_velocity_mps,
    target_relative_to_radar,
)
from EarthReferencedMeasurements import (
    AnnotateDetectionWithEarthReference,
    AnnotateDetectionsWithEarthReference,
    BuildEarthReferencedDetectionMeasurement,
)
from NavigationState import (
    CircularRouteNavigationSource,
    NavigationPose,
)


def _detection(range_m, bearing_deg, measured_velocity_mps):
    return SimpleNamespace(
        RangeM=float(range_m),
        AzimuthDeg=float(bearing_deg),
        VelocityMps=float(measured_velocity_mps),
    )


def _pose(
    *,
    east_m=0.0,
    north_m=0.0,
    velocity_east_mps=0.0,
    velocity_north_mps=0.0,
    position_valid=True,
    heading_valid=True,
    velocity_valid=True,
):
    return NavigationPose(
        TimestampSec=100.0,
        SequenceNumber=7,
        Geodetic=GeodeticPosition(-33.0, 151.0, 0.0),
        PositionEnu=EnuPosition(east_m, north_m, 0.0),
        VelocityEnu=EnuVelocity(
            velocity_east_mps,
            velocity_north_mps,
            0.0,
        ),
        HeadingTrueDeg=25.0,
        PositionValid=position_valid,
        HeadingValid=heading_valid,
        VelocityValid=velocity_valid,
        Source="TEST_NAVIGATION",
    )


class TestEarthReferencedDetectionMeasurement(unittest.TestCase):
    def test_places_detection_in_mission_east_north(self):
        measurement = BuildEarthReferencedDetectionMeasurement(
            _detection(500.0, 90.0, 0.0),
            _pose(east_m=100.0, north_m=200.0),
        )

        self.assertTrue(measurement.Valid)
        self.assertAlmostEqual(measurement.EastM, 600.0)
        self.assertAlmostEqual(measurement.NorthM, 200.0)
        self.assertAlmostEqual(measurement.RadarEastM, 100.0)
        self.assertAlmostEqual(measurement.RadarNorthM, 200.0)

    def test_ownship_velocity_is_compensated_exactly_once(self):
        measurement = BuildEarthReferencedDetectionMeasurement(
            _detection(1000.0, 0.0, 3.0),
            _pose(velocity_north_mps=5.0),
        )

        self.assertEqual(
            measurement.MeasuredRelativeRadialVelocityMps,
            3.0,
        )
        self.assertEqual(measurement.TargetLineOfSightVelocityMps, 8.0)

    def test_stationary_target_recovers_zero_earth_los_velocity(self):
        measurement = BuildEarthReferencedDetectionMeasurement(
            _detection(1000.0, 0.0, -5.0),
            _pose(velocity_north_mps=5.0),
        )

        self.assertEqual(measurement.TargetLineOfSightVelocityMps, 0.0)

    def test_invalid_pose_preserves_detection_as_explicitly_invalid(self):
        detection = _detection(1000.0, 45.0, 2.0)
        measurement = AnnotateDetectionWithEarthReference(
            detection,
            _pose(position_valid=False),
        )

        self.assertFalse(measurement.Valid)
        self.assertFalse(detection.EarthReferenceValid)
        self.assertIsNone(detection.EarthEastM)
        self.assertIsNone(detection.EarthNorthM)
        self.assertEqual(detection.RangeM, 1000.0)
        self.assertEqual(detection.AzimuthDeg, 45.0)
        self.assertEqual(detection.VelocityMps, 2.0)

    def test_invalid_velocity_still_allows_earth_position(self):
        measurement = BuildEarthReferencedDetectionMeasurement(
            _detection(1000.0, 180.0, 4.0),
            _pose(velocity_valid=False),
        )

        self.assertTrue(measurement.Valid)
        self.assertFalse(measurement.VelocityValid)
        self.assertIsNotNone(measurement.EastM)
        self.assertIsNotNone(measurement.NorthM)
        self.assertIsNone(measurement.TargetLineOfSightVelocityMps)

    def test_annotation_supports_detection_dictionaries(self):
        detection = {
            "RangeM": 250.0,
            "AzimuthDeg": 270.0,
            "VelocityMps": 0.0,
        }

        AnnotateDetectionWithEarthReference(detection, _pose())

        self.assertTrue(detection["EarthReferenceValid"])
        self.assertAlmostEqual(detection["EarthEastM"], -250.0)
        self.assertAlmostEqual(detection["EarthNorthM"], 0.0)

    def test_multiple_annotations_preserve_order_and_object_identity(self):
        detections = [
            _detection(100.0, 0.0, 0.0),
            _detection(200.0, 90.0, 0.0),
        ]

        measurements = AnnotateDetectionsWithEarthReference(
            detections,
            _pose(),
        )

        self.assertEqual(len(measurements), 2)
        self.assertIs(detections[0].EarthReference, measurements[0])
        self.assertIs(detections[1].EarthReference, measurements[1])

    def test_rejects_invalid_detection_measurements(self):
        for detection in (
            _detection(-1.0, 0.0, 0.0),
            _detection(float("nan"), 0.0, 0.0),
            _detection(100.0, float("inf"), 0.0),
            _detection(100.0, 0.0, float("nan")),
        ):
            with self.assertRaises(ValueError):
                BuildEarthReferencedDetectionMeasurement(
                    detection,
                    _pose(),
                )


class TestCircularPlatformEarthReference(unittest.TestCase):
    def test_stationary_target_remains_fixed_around_full_circle(self):
        clock = [1000.0]
        navigation = CircularRouteNavigationSource(
            mission_origin=GeodeticPosition(-33.0, 151.0, 0.0),
            centre_east_m=2000.0,
            centre_north_m=-1000.0,
            radius_m=500.0,
            speed_mps=10.0,
            clockwise=True,
            initial_radial_bearing_deg=0.0,
            time_source=lambda: clock[0],
        )
        target_position = EnuPosition(3500.0, 750.0, 0.0)
        target_velocity = EnuVelocity(0.0, 0.0, 0.0)

        measured_ranges = []
        measured_bearings = []
        for orbit_fraction in (0.0, 0.25, 0.5, 0.75, 1.0):
            clock[0] = 1000.0 + navigation.OrbitPeriodSec * orbit_fraction
            pose = navigation.get_pose()
            relative = target_relative_to_radar(
                target_position,
                pose.PositionEnu,
            )
            measured_velocity = measured_relative_radial_velocity_mps(
                target_velocity,
                pose.VelocityEnu,
                relative.true_bearing_deg,
            )
            detection = _detection(
                relative.range_m,
                relative.true_bearing_deg,
                measured_velocity,
            )

            measurement = AnnotateDetectionWithEarthReference(
                detection,
                pose,
            )
            measured_ranges.append(relative.range_m)
            measured_bearings.append(relative.true_bearing_deg)

            self.assertAlmostEqual(
                measurement.TargetLineOfSightVelocityMps,
                0.0,
                places=10,
            )
            self.assertAlmostEqual(
                measurement.EastM,
                target_position.east_m,
                places=9,
            )
            self.assertAlmostEqual(
                measurement.NorthM,
                target_position.north_m,
                places=9,
            )

        # Prove this is not a stationary-radar tautology: the raw radar view
        # changes materially even though every recovered Earth position agrees.
        self.assertGreater(max(measured_ranges) - min(measured_ranges), 500.0)
        self.assertGreater(
            max(measured_bearings) - min(measured_bearings),
            10.0,
        )


class TestSchedulerIntegrationBoundary(unittest.TestCase):
    def test_annotation_occurs_before_angular_processing_and_tracker_update(self):
        source = (
            Path(__file__).resolve().parent / "VanguardxMain_scheduler.py"
        ).read_text(encoding="utf-8")

        annotation_index = source.index(
            "AnnotateDetectionsWithEarthReference("
        )
        angular_index = source.index("AngularProcessor.Update(")
        tracker_index = source.index("Tracker.Update(")
        self.assertLess(annotation_index, angular_index)
        self.assertLess(angular_index, tracker_index)
        self.assertIn(
            "AnnotateDetectionsWithEarthReference(\n"
            "            AngularPlots,",
            source,
        )
        self.assertIn(
            '"EarthReferenceAuthoritative"] = False',
            source,
        )


if __name__ == "__main__":
    unittest.main(verbosity=2)
