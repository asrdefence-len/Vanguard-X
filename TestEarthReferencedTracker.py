"""Regression tests for the non-authoritative Stage 6 ENU tracker."""

from pathlib import Path
from types import SimpleNamespace
import math
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
)
from EarthReferencedTracker import (
    BuildParallelTrackerComparison,
    EarthReferencedTracker,
    _EarthPoint,
)
from NavigationState import (
    CircularRouteNavigationSource,
    NavigationPose,
)


def _earth_detection(east_m, north_m, valid=True, timestamp_sec=None):
    return SimpleNamespace(
        EarthReferenceValid=bool(valid),
        EarthEastM=float(east_m),
        EarthNorthM=float(north_m),
        NavigationPoseTimestampSec=timestamp_sec,
        AmplitudeDb=-25.0,
        SnrDb=18.0,
        DopplerHz=0.0,
    )


def _radar_detection(range_m, bearing_deg, velocity_mps):
    return SimpleNamespace(
        RangeM=float(range_m),
        AzimuthDeg=float(bearing_deg),
        VelocityMps=float(velocity_mps),
        AmplitudeDb=-25.0,
        SnrDb=18.0,
        DopplerHz=0.0,
    )


def _pose(east_m=0.0, north_m=0.0):
    return NavigationPose(
        TimestampSec=100.0,
        SequenceNumber=1,
        Geodetic=GeodeticPosition(-33.0, 151.0, 0.0),
        PositionEnu=EnuPosition(east_m, north_m, 0.0),
        VelocityEnu=EnuVelocity(0.0, 0.0, 0.0),
        HeadingTrueDeg=0.0,
        PositionValid=True,
        HeadingValid=True,
        VelocityValid=True,
        Source="TEST",
    )


def _update_scan(tracker, scan_id, detections):
    return tracker.Update(
        detections,
        ThisDwell={"ScanCycle": scan_id, "AzimuthDeg": 0.0},
    )


def _confirm_track(tracker, positions):
    for scan_id, (east_m, north_m) in enumerate(positions):
        _update_scan(
            tracker,
            scan_id,
            [_earth_detection(east_m, north_m)],
        )
    _update_scan(tracker, len(positions), [])
    confirmed = tracker.GetConfirmedTracks()
    if len(confirmed) != 1:
        raise AssertionError(
            f"expected one confirmed track, got {len(confirmed)}"
        )
    return confirmed[0]


class TestEarthReferencedTrackerInitiation(unittest.TestCase):
    def test_repeated_earth_position_confirms_two_of_three(self):
        tracker = EarthReferencedTracker(
            {
                "EarthInitiationWindow": 3,
                "EarthInitiationRequiredHits": 2,
            }
        )

        track = _confirm_track(
            tracker,
            [(1000.0, 2000.0)] * 3,
        )

        self.assertTrue(track.IsConfirmed)
        self.assertEqual(track.Status, "CONFIRMED")
        self.assertEqual(track.Hits, 3)
        self.assertEqual(track.Attempts, 3)
        self.assertAlmostEqual(track.EastM, 1000.0)
        self.assertAlmostEqual(track.NorthM, 2000.0)

    def test_unassociated_single_detection_dies_after_window(self):
        tracker = EarthReferencedTracker()
        _update_scan(tracker, 0, [_earth_detection(1000.0, 0.0)])
        _update_scan(tracker, 1, [])
        _update_scan(tracker, 2, [])
        _update_scan(tracker, 3, [])

        self.assertEqual(tracker.GetTracks(), [])

    def test_invalid_earth_reference_is_ignored(self):
        tracker = EarthReferencedTracker()
        _update_scan(
            tracker,
            0,
            [_earth_detection(1000.0, 0.0, valid=False)],
        )

        debug = tracker.GetDebugInfo()
        self.assertEqual(debug["ValidEarthDetectionsThisDwell"], 0)
        self.assertEqual(debug["RejectedEarthDetectionsThisDwell"], 1)
        self.assertEqual(debug["CurrentScanPoints"], 0)

    def test_dictionary_detection_is_supported(self):
        tracker = EarthReferencedTracker()
        detection = {
            "EarthReferenceValid": True,
            "EarthEastM": 500.0,
            "EarthNorthM": -250.0,
        }

        _update_scan(tracker, 0, [detection])

        self.assertEqual(tracker.GetDebugInfo()["CurrentScanPoints"], 1)

    def test_directed_nod_promotes_selected_earth_track_immediately(self):
        tracker = EarthReferencedTracker(
            {
                "EarthInitiationWindow": 3,
                "EarthInitiationRequiredHits": 2,
            }
        )
        _update_scan(
            tracker,
            0,
            [_earth_detection(1000.0, 2000.0, timestamp_sec=100.0)],
        )
        _update_scan(tracker, 1, [])
        initiating = tracker.GetTentativeTracks()[0]
        self.assertEqual((initiating.Hits, initiating.Attempts), (1, 1))

        result = tracker.ApplyDirectedResult(
            initiating.TrackId,
            [
                _earth_detection(
                    1005.0,
                    1995.0,
                    timestamp_sec=101.0,
                )
            ],
            coverage_valid=True,
            authoritative_hit=True,
            authoritative_promoted=True,
        )

        self.assertTrue(result["Applied"])
        self.assertTrue(result["Hit"])
        self.assertTrue(result["Promoted"])
        self.assertEqual(result["Outcome"], "EARTH_TENTATIVE_PROMOTED")
        confirmed = tracker.GetConfirmedTracks()
        self.assertEqual(len(confirmed), 1)
        self.assertEqual(
            (confirmed[0].Hits, confirmed[0].Attempts),
            (2, 2),
        )

    def test_directed_nod_is_one_earth_initiation_opportunity(self):
        tracker = EarthReferencedTracker()
        _update_scan(
            tracker,
            0,
            [_earth_detection(1000.0, 2000.0)],
        )
        _update_scan(tracker, 1, [])
        initiating = tracker.GetTentativeTracks()[0]

        result = tracker.ApplyDirectedResult(
            initiating.TrackId,
            [
                _earth_detection(1001.0, 2001.0),
                _earth_detection(1002.0, 2002.0),
                _earth_detection(1003.0, 2003.0),
            ],
            coverage_valid=True,
            authoritative_hit=True,
            authoritative_promoted=True,
        )

        self.assertEqual(result["Hits"], 2)
        self.assertEqual(result["Attempts"], 2)

    def test_incomplete_directed_nod_does_not_age_earth_track(self):
        tracker = EarthReferencedTracker()
        _update_scan(
            tracker,
            0,
            [_earth_detection(1000.0, 2000.0)],
        )
        _update_scan(tracker, 1, [])
        initiating = tracker.GetTentativeTracks()[0]

        result = tracker.ApplyDirectedResult(
            initiating.TrackId,
            [],
            coverage_valid=False,
            authoritative_hit=False,
        )

        self.assertEqual(result["Outcome"], "EARTH_INCOMPLETE_COVERAGE")
        self.assertEqual(
            (initiating.Hits, initiating.Attempts, initiating.Misses),
            (1, 1, 0),
        )


class TestEarthReferencedTrackerGeometry(unittest.TestCase):
    def test_clustering_uses_cartesian_distance(self):
        tracker = EarthReferencedTracker(
            {"EarthClusterDistanceM": 100.0}
        )
        blobs = tracker._cluster_points(
            [
                _EarthPoint(0.0, 0.0),
                _EarthPoint(60.0, 80.0),
                _EarthPoint(300.0, 0.0),
            ]
        )

        self.assertEqual(len(blobs), 2)
        self.assertEqual(
            sorted(blob.NumDetections for blob in blobs),
            [1, 2],
        )

    def test_constant_velocity_is_estimated_in_east_north(self):
        tracker = EarthReferencedTracker(
            {
                "EarthTrackDtSec": 1.0,
                "EarthInitiationGateM": 100.0,
            }
        )

        track = _confirm_track(
            tracker,
            [(0.0, 0.0), (10.0, -4.0), (20.0, -8.0)],
        )

        self.assertGreater(track.VelocityEastMps, 8.0)
        self.assertLess(track.VelocityNorthMps, -3.0)
        self.assertAlmostEqual(
            track.CourseTrueDeg,
            math.degrees(math.atan2(10.0, -4.0)) % 360.0,
            delta=1.0,
        )

    def test_navigation_timestamps_override_fixed_scan_interval(self):
        tracker = EarthReferencedTracker(
            {
                "EarthTrackDtSec": 1.0,
                "EarthInitiationGateM": 500.0,
            }
        )
        for scan_id, timestamp_sec, east_m in (
            (0, 100.0, 0.0),
            (1, 110.0, 20.0),
            (2, 120.0, 40.0),
        ):
            _update_scan(
                tracker,
                scan_id,
                [
                    _earth_detection(
                        east_m,
                        0.0,
                        timestamp_sec=timestamp_sec,
                    )
                ],
            )
        _update_scan(tracker, 3, [])

        track = tracker.GetConfirmedTracks()[0]
        self.assertAlmostEqual(track.VelocityEastMps, 1.955, places=3)
        self.assertAlmostEqual(track.VelocityNorthMps, 0.0)

    def test_confirmed_track_coasts_in_fixed_cartesian_state(self):
        tracker = EarthReferencedTracker(
            {
                "EarthTrackDtSec": 1.0,
                "EarthInitiationGateM": 100.0,
            }
        )
        track = _confirm_track(
            tracker,
            [(0.0, 0.0), (10.0, 0.0), (20.0, 0.0)],
        )
        east_before_coast = track.EastM

        _update_scan(tracker, 4, [])
        coasted = tracker.GetConfirmedTracks()[0]

        self.assertEqual(coasted.Misses, 1)
        self.assertGreater(coasted.EastM, east_before_coast)
        self.assertAlmostEqual(coasted.NorthM, 0.0)

    def test_two_targets_are_associated_one_to_one(self):
        tracker = EarthReferencedTracker(
            {
                "EarthClusterDistanceM": 50.0,
                "EarthInitiationGateM": 100.0,
            }
        )
        for scan_id in range(3):
            _update_scan(
                tracker,
                scan_id,
                [
                    _earth_detection(1000.0 + scan_id, 0.0),
                    _earth_detection(2000.0 + scan_id, 0.0),
                ],
            )
        _update_scan(tracker, 3, [])

        confirmed = tracker.GetConfirmedTracks()
        self.assertEqual(len(confirmed), 2)
        self.assertEqual(
            sorted(round(track.EastM) for track in confirmed),
            [1002, 2002],
        )


class TestCircularOwnshipEarthTracking(unittest.TestCase):
    def test_stationary_target_track_remains_fixed_around_full_orbit(self):
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
        tracker = EarthReferencedTracker(
            {"EarthAssociationGateM": 50.0}
        )
        raw_ranges = []
        raw_bearings = []

        fractions = (0.0, 0.25, 0.5)
        for scan_id, fraction in enumerate(fractions):
            clock[0] = (
                1000.0 + navigation.OrbitPeriodSec * fraction
            )
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
            detection = _radar_detection(
                relative.range_m,
                relative.true_bearing_deg,
                measured_velocity,
            )
            AnnotateDetectionWithEarthReference(detection, pose)
            _update_scan(tracker, scan_id, [detection])
            raw_ranges.append(relative.range_m)
            raw_bearings.append(relative.true_bearing_deg)

        _update_scan(tracker, len(fractions), [])
        track = tracker.GetConfirmedTracks()[0]

        self.assertAlmostEqual(track.EastM, target_position.east_m)
        self.assertAlmostEqual(track.NorthM, target_position.north_m)
        self.assertAlmostEqual(track.VelocityEastMps, 0.0, places=9)
        self.assertAlmostEqual(track.VelocityNorthMps, 0.0, places=9)
        self.assertGreater(max(raw_ranges) - min(raw_ranges), 500.0)
        self.assertGreater(
            max(raw_bearings) - min(raw_bearings),
            10.0,
        )


class TestParallelTrackerComparison(unittest.TestCase):
    def test_legacy_tracks_are_projected_then_nearest_matched(self):
        pose = _pose(east_m=100.0, north_m=200.0)
        earth_tracks = [
            SimpleNamespace(EastM=100.0, NorthM=1200.0),
            SimpleNamespace(EastM=600.0, NorthM=200.0),
        ]
        legacy_tracks = [
            SimpleNamespace(RangeM=500.0, AzimuthDeg=90.0),
            SimpleNamespace(RangeM=1000.0, AzimuthDeg=0.0),
        ]

        comparison = BuildParallelTrackerComparison(
            earth_tracks,
            legacy_tracks,
            pose,
        )

        self.assertTrue(comparison.Valid)
        self.assertEqual(comparison.MatchedTrackCount, 2)
        self.assertAlmostEqual(comparison.RmsPositionSeparationM, 0.0)
        self.assertAlmostEqual(comparison.MaximumPositionSeparationM, 0.0)

    def test_invalid_navigation_makes_comparison_explicitly_invalid(self):
        pose = NavigationPose(
            TimestampSec=100.0,
            SequenceNumber=1,
            Geodetic=GeodeticPosition(-33.0, 151.0, 0.0),
            PositionEnu=EnuPosition(0.0, 0.0, 0.0),
            VelocityEnu=EnuVelocity(0.0, 0.0, 0.0),
            HeadingTrueDeg=0.0,
            PositionValid=False,
            HeadingValid=True,
            VelocityValid=True,
            Source="TEST_INVALID_POSITION",
        )

        comparison = BuildParallelTrackerComparison([], [], pose)

        self.assertFalse(comparison.Valid)
        self.assertEqual(
            comparison.Reason,
            "INVALID_NAVIGATION_POSITION",
        )
        self.assertIsNone(comparison.RmsPositionSeparationM)


class TestSchedulerStage6Boundary(unittest.TestCase):
    def test_parallel_tracker_cannot_replace_legacy_tasking_tracks(self):
        source = (
            Path(__file__).resolve().parent / "VanguardxMain_scheduler.py"
        ).read_text(encoding="utf-8")

        legacy_update_index = source.index("Tracker.Update(")
        earth_update_index = source.index(
            "EarthTracker.Update("
        )
        earth_directed_update_index = source.index(
            "EarthTracker.ApplyDirectedResult("
        )
        selection_index = source.index(
            "DisplayTrackSelection = SelectDisplayTrackProducts("
        )

        self.assertLess(legacy_update_index, earth_update_index)
        self.assertLess(earth_update_index, selection_index)
        self.assertLess(earth_directed_update_index, selection_index)
        self.assertIn("TrackerMeasurements = AngularPlots", source)
        self.assertIn("TrackerMeasurements = Detections", source)
        self.assertIn(
            'Processed.Diagnostics["EarthTrackerAuthoritative"] = False',
            source,
        )
        self.assertIn(
            'Processed.Diagnostics["TaskingTrackSource"] = "LEGACY"',
            source,
        )
        self.assertIn(
            'Processed.Diagnostics["TrackUpdateSource"] = "LEGACY"',
            source,
        )
        self.assertIn(
            '"Tracks": Tracks',
            source,
        )


if __name__ == "__main__":
    unittest.main(verbosity=2)
