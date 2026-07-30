"""Stage 9 tests for auditable local/Ethernet Earth-track display."""

from pathlib import Path
from types import SimpleNamespace
import unittest

from DisplayTrackSource import SelectDisplayTrackProducts
from OperationalDisplayTrackValidation import (
    RunOperationalDisplayValidation,
    _CaptureDisplay,
    _earth_track,
    _legacy_track,
    _pose,
    _processed,
    _wire_delivery,
)
from RadarLinkProtocol import FilterPublicConfig
from RadarRemoteDisplay import PUBLIC_CONFIG_KEYS


class OperationalDisplayTrackValidationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.Result = RunOperationalDisplayValidation(frame_count=36)

    def test_complete_operational_gate_passes(self):
        self.assertTrue(self.Result.Passed)

    def test_all_valid_route_frames_use_earth_tracks(self):
        self.assertEqual(self.Result.EarthFrames, 36)

    def test_invalid_navigation_fallback_is_exercised(self):
        self.assertEqual(self.Result.FallbackFrames, 1)

    def test_local_and_ethernet_ranges_are_identical(self):
        self.assertLessEqual(self.Result.MaximumRangeDifferenceM, 1.0e-9)

    def test_local_and_ethernet_bearings_are_identical(self):
        self.assertLessEqual(
            self.Result.MaximumBearingDifferenceDeg,
            1.0e-9,
        )

    def test_local_and_ethernet_range_rates_are_identical(self):
        self.assertLessEqual(
            self.Result.MaximumRangeRateDifferenceMps,
            1.0e-9,
        )

    def test_earth_track_metadata_survives_wire_path(self):
        self.assertTrue(self.Result.TrackMetadataPreserved)

    def test_source_diagnostics_survive_wire_path(self):
        self.assertTrue(self.Result.DiagnosticsPreserved)

    def test_tasking_and_track_update_stay_legacy(self):
        self.assertTrue(self.Result.LegacyTaskingPreserved)

    def test_remote_config_publishes_requested_track_source(self):
        filtered = FilterPublicConfig(
            {
                "DisplayTrackSource": "EARTH",
                "PlatformTrajectoryEnabled": True,
                "PlatformTrajectoryColour": (255, 40, 40, 210),
                "MapPositionUpdateMinimumM": 25.0,
                "PrivateKey": "not public",
            },
            PUBLIC_CONFIG_KEYS,
        )
        self.assertEqual(filtered["DisplayTrackSource"], "EARTH")
        self.assertTrue(filtered["PlatformTrajectoryEnabled"])
        self.assertEqual(
            filtered["PlatformTrajectoryColour"],
            [255, 40, 40, 210],
        )
        self.assertEqual(filtered["MapPositionUpdateMinimumM"], 25.0)
        self.assertNotIn("PrivateKey", filtered)

    def test_navigation_diagnostics_survive_ethernet_snapshot(self):
        selection = SelectDisplayTrackProducts(
            {"DisplayTrackSource": "EARTH"},
            [_legacy_track()],
            [],
            [_earth_track(9, 1500.0, 2500.0, 0.0, 0.0)],
            [],
            _pose(0.25),
        )
        processed = _processed(selection)
        processed.Diagnostics.update({
            "NavigationPositionValid": True,
            "NavigationLatitudeDeg": -34.35,
            "NavigationLongitudeDeg": 150.98,
            "NavigationEastM": 5000.0,
            "NavigationNorthM": 1000.0,
            "NavigationHeadingTrueDeg": 90.0,
        })
        remote = _wire_delivery(
            {"RadarLinkMaxRangeProfilePoints": 1500},
            processed,
            selection.Tracks,
            selection.Plots,
        )

        self.assertEqual(
            remote.Processed.Diagnostics,
            processed.Diagnostics,
        )

    def test_one_snapshot_preserves_track_source_and_earth_coordinates(self):
        selection = SelectDisplayTrackProducts(
            {"DisplayTrackSource": "EARTH"},
            [_legacy_track()],
            [],
            [_earth_track(7, 1200.0, 2400.0, 2.0, -1.0)],
            [],
            _pose(0.5),
        )
        remote = _wire_delivery(
            {"RadarLinkMaxRangeProfilePoints": 1500},
            _processed(selection),
            selection.Tracks,
            selection.Plots,
        )
        track = remote.Tracks[0]
        self.assertEqual(track.TrackSource, "EARTH")
        self.assertEqual(track.EarthEastM, 1200.0)
        self.assertEqual(track.EarthNorthM, 2400.0)
        self.assertEqual(track.VelocityEastMps, 2.0)
        self.assertEqual(track.VelocityNorthMps, -1.0)

    def test_local_display_receives_the_selected_objects_directly(self):
        selection = SelectDisplayTrackProducts(
            {"DisplayTrackSource": "EARTH"},
            [_legacy_track()],
            [],
            [_earth_track(8, 1000.0, 3000.0, 0.0, 0.0)],
            [],
            _pose(1.0),
        )
        display = _CaptureDisplay()
        display.Update(
            _processed(selection),
            [],
            Tracks=selection.Tracks,
            Plots=selection.Plots,
        )
        self.assertIs(display.Tracks[0], selection.Tracks[0])

    def test_qt_status_panel_exposes_applied_source_and_fallback(self):
        source = Path("RadarDisplayQt5.py").read_text(encoding="utf-8")
        self.assertIn(
            'f"Track src: {self.DisplayTrackSourceApplied}"',
            source,
        )
        self.assertIn("Track fallback:", source)
        self.assertIn(
            'f"Source:     {TrackSource}\\n"',
            source,
        )

    def test_scheduler_authority_boundary_remains_legacy(self):
        source = Path("VanguardxMain_scheduler.py").read_text(
            encoding="utf-8"
        )
        self.assertIn(
            'Processed.Diagnostics["TaskingTrackSource"] = "LEGACY"',
            source,
        )
        self.assertIn(
            'Processed.Diagnostics["TrackUpdateSource"] = "LEGACY"',
            source,
        )
        self.assertIn('"Tracks": Tracks', source)
        self.assertIn("Tracks=DisplayTracks", source)


if __name__ == "__main__":
    unittest.main()
