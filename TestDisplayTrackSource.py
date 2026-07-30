"""Stage 8 regression tests for selectable display-track authority."""

from pathlib import Path
from types import SimpleNamespace
import math
import unittest

from DisplayTrackSource import (
    DISPLAY_TRACK_SOURCE_EARTH,
    DISPLAY_TRACK_SOURCE_LEGACY,
    NormaliseDisplayTrackSource,
    SelectDisplayTrackProducts,
)


def _pose(
    east_m=0.0,
    north_m=0.0,
    velocity_east_mps=0.0,
    velocity_north_mps=0.0,
    position_valid=True,
    velocity_valid=True,
):
    return SimpleNamespace(
        PositionValid=position_valid,
        VelocityValid=velocity_valid,
        PositionEnu=SimpleNamespace(
            east_m=east_m,
            north_m=north_m,
        ),
        VelocityEnu=SimpleNamespace(
            east_mps=velocity_east_mps,
            north_mps=velocity_north_mps,
        ),
    )


def _earth_track(
    track_id=7,
    east_m=0.0,
    north_m=1000.0,
    velocity_east_mps=0.0,
    velocity_north_mps=0.0,
    status="CONFIRMED",
):
    return SimpleNamespace(
        TrackId=track_id,
        EastM=east_m,
        NorthM=north_m,
        VelocityEastMps=velocity_east_mps,
        VelocityNorthMps=velocity_north_mps,
        Status=status,
        IsConfirmed=status == "CONFIRMED",
        Hits=4,
        Attempts=5,
        Misses=1,
        LastUpdateScan=12,
        SnrDb=18.0,
        AmplitudeDb=-42.0,
    )


class DisplayTrackSourceTests(unittest.TestCase):
    def test_source_name_is_case_insensitive(self):
        self.assertEqual(
            NormaliseDisplayTrackSource(" earth "),
            DISPLAY_TRACK_SOURCE_EARTH,
        )

    def test_invalid_source_is_rejected(self):
        with self.assertRaises(ValueError):
            NormaliseDisplayTrackSource("parallel")

    def test_legacy_is_the_default_source(self):
        legacy_track = object()
        legacy_plot = object()
        selected = SelectDisplayTrackProducts(
            {},
            [legacy_track],
            [legacy_plot],
            [],
            [],
            _pose(),
        )
        self.assertEqual(selected.AppliedSource, DISPLAY_TRACK_SOURCE_LEGACY)
        self.assertIs(selected.Tracks[0], legacy_track)
        self.assertIs(selected.Plots[0], legacy_plot)
        self.assertFalse(selected.FallbackUsed)

    def test_legacy_selection_does_not_require_navigation(self):
        selected = SelectDisplayTrackProducts(
            {"DisplayTrackSource": "LEGACY"},
            ["legacy"],
            [],
            [],
            [],
            None,
        )
        self.assertEqual(selected.Tracks, ["legacy"])
        self.assertEqual(selected.Reason, "REQUESTED_LEGACY")

    def test_earth_track_projects_to_true_north(self):
        selected = SelectDisplayTrackProducts(
            {"DisplayTrackSource": "EARTH"},
            [],
            [],
            [_earth_track(north_m=2000.0)],
            [],
            _pose(),
        )
        track = selected.Tracks[0]
        self.assertAlmostEqual(track.RangeM, 2000.0)
        self.assertAlmostEqual(track.AzimuthDeg, 0.0)

    def test_earth_track_projects_to_true_east(self):
        selected = SelectDisplayTrackProducts(
            {"DisplayTrackSource": "EARTH"},
            [],
            [],
            [_earth_track(east_m=1500.0, north_m=0.0)],
            [],
            _pose(),
        )
        track = selected.Tracks[0]
        self.assertAlmostEqual(track.RangeM, 1500.0)
        self.assertAlmostEqual(track.AzimuthDeg, 90.0)

    def test_projection_uses_current_radar_position(self):
        selected = SelectDisplayTrackProducts(
            {"DisplayTrackSource": "EARTH"},
            [],
            [],
            [_earth_track(east_m=1300.0, north_m=2400.0)],
            [],
            _pose(east_m=300.0, north_m=1400.0),
        )
        track = selected.Tracks[0]
        self.assertAlmostEqual(track.RangeM, math.sqrt(2.0) * 1000.0)
        self.assertAlmostEqual(track.AzimuthDeg, 45.0)

    def test_relative_range_rate_compensates_ownship_once(self):
        selected = SelectDisplayTrackProducts(
            {"DisplayTrackSource": "EARTH"},
            [],
            [],
            [_earth_track(north_m=1000.0, velocity_north_mps=3.0)],
            [],
            _pose(velocity_north_mps=8.0),
        )
        self.assertAlmostEqual(selected.Tracks[0].RangeRateMps, -5.0)

    def test_cross_line_of_sight_motion_produces_bearing_rate(self):
        selected = SelectDisplayTrackProducts(
            {"DisplayTrackSource": "EARTH"},
            [],
            [],
            [_earth_track(north_m=1000.0, velocity_east_mps=10.0)],
            [],
            _pose(),
        )
        track = selected.Tracks[0]
        self.assertAlmostEqual(track.RangeRateMps, 0.0)
        self.assertAlmostEqual(
            track.AzimuthRateDps,
            math.degrees(0.01),
        )

    def test_invalid_navigation_falls_back_to_legacy(self):
        selected = SelectDisplayTrackProducts(
            {"DisplayTrackSource": "EARTH"},
            ["legacy"],
            [],
            [_earth_track()],
            [],
            _pose(position_valid=False),
        )
        self.assertEqual(selected.AppliedSource, DISPLAY_TRACK_SOURCE_LEGACY)
        self.assertEqual(selected.Tracks, ["legacy"])
        self.assertTrue(selected.FallbackUsed)
        self.assertEqual(selected.Reason, "INVALID_NAVIGATION_POSITION")

    def test_disabled_earth_tracker_falls_back_to_legacy(self):
        selected = SelectDisplayTrackProducts(
            {
                "DisplayTrackSource": "EARTH",
                "EarthTrackerEnabled": False,
            },
            ["legacy"],
            [],
            [_earth_track()],
            [],
            _pose(),
        )
        self.assertEqual(selected.Tracks, ["legacy"])
        self.assertEqual(selected.Reason, "EARTH_TRACKER_DISABLED")

    def test_invalid_earth_track_is_skipped_without_fallback(self):
        invalid_track = SimpleNamespace(
            TrackId=9,
            EastM=float("nan"),
            NorthM=1000.0,
        )
        selected = SelectDisplayTrackProducts(
            {"DisplayTrackSource": "EARTH"},
            ["legacy"],
            [],
            [invalid_track],
            [],
            _pose(),
        )
        self.assertEqual(selected.AppliedSource, DISPLAY_TRACK_SOURCE_EARTH)
        self.assertEqual(selected.Tracks, [])
        self.assertFalse(selected.FallbackUsed)

    def test_track_identity_and_metadata_are_preserved(self):
        selected = SelectDisplayTrackProducts(
            {"DisplayTrackSource": "EARTH"},
            [],
            [],
            [_earth_track(track_id=23, status="TENTATIVE")],
            [],
            _pose(),
        )
        track = selected.Tracks[0]
        self.assertEqual(track.TrackId, 23)
        self.assertEqual(track.Status, "TENTATIVE")
        self.assertFalse(track.IsConfirmed)
        self.assertEqual(track.Hits, 4)
        self.assertEqual(track.Misses, 1)
        self.assertEqual(track.TrackSource, "EARTH")

    def test_earth_blobs_are_projected_for_tracker_plot_display(self):
        earth_plot = SimpleNamespace(
            EastM=-1000.0,
            NorthM=0.0,
            AmplitudeDb=-50.0,
            DopplerHz=120.0,
        )
        selected = SelectDisplayTrackProducts(
            {"DisplayTrackSource": "EARTH"},
            [],
            [],
            [],
            [earth_plot],
            _pose(),
        )
        plot = selected.Plots[0]
        self.assertAlmostEqual(plot.RangeM, 1000.0)
        self.assertAlmostEqual(plot.AzimuthDeg, 270.0)
        self.assertEqual(plot.TrackSource, "EARTH")

    def test_scheduler_passes_only_selected_products_to_display(self):
        scheduler = Path("VanguardxMain_scheduler.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("Tracks=DisplayTracks", scheduler)
        self.assertIn("Plots=DisplayPlots", scheduler)
        self.assertIn('"Tracks": Tracks', scheduler)
        self.assertIn('"Plots": Plots', scheduler)

    def test_scheduler_keeps_tasking_and_track_update_on_legacy(self):
        scheduler = Path("VanguardxMain_scheduler.py").read_text(
            encoding="utf-8"
        )
        self.assertIn(
            'Processed.Diagnostics["TaskingTrackSource"] = "LEGACY"',
            scheduler,
        )
        self.assertIn(
            'Processed.Diagnostics["TrackUpdateSource"] = "LEGACY"',
            scheduler,
        )
        self.assertIn("--display-track-source", scheduler)


if __name__ == "__main__":
    unittest.main()
