"""Regression tests for the Stage 7 complete-orbit tracker validation."""

from pathlib import Path
import json
import unittest

from ParallelTrackerOperationalValidation import (
    OperationalValidationConfig,
    RunOperationalCircularTrackerValidation,
    ValidationTarget,
)


class TestOperationalValidationConfiguration(unittest.TestCase):
    def test_orbit_and_scan_period_are_consistent(self):
        config = OperationalValidationConfig(
            OrbitRadiusM=500.0,
            OwnshipSpeedMps=10.0,
            ScansPerOrbit=72,
        )

        self.assertAlmostEqual(
            config.scan_period_sec * config.ScansPerOrbit,
            config.orbit_period_sec,
        )

    def test_rejects_undersampled_orbit(self):
        with self.assertRaisesRegex(
            ValueError,
            "ScansPerOrbit must be at least 12",
        ):
            OperationalValidationConfig(ScansPerOrbit=11)

    def test_rejects_duplicate_target_names(self):
        target = ValidationTarget("duplicate", 1.0, 2.0, 0.0, 0.0)

        with self.assertRaisesRegex(ValueError, "names must be unique"):
            RunOperationalCircularTrackerValidation(
                Targets=(target, target),
            )

    def test_rejects_empty_target_set(self):
        with self.assertRaisesRegex(
            ValueError,
            "at least one validation target",
        ):
            RunOperationalCircularTrackerValidation(Targets=())


class TestCompleteOrbitParallelTracking(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.report = RunOperationalCircularTrackerValidation()

    def test_complete_orbit_passes_quantitative_gate(self):
        self.assertTrue(
            self.report.Passed,
            self.report.format_text(),
        )
        self.assertEqual(self.report.ScansPerOrbit, 72)
        self.assertGreater(self.report.EvaluatedScans, 100)

    def test_stationary_and_moving_targets_are_both_evaluated(self):
        results = {
            result.Name: result
            for result in self.report.TargetResults
        }

        self.assertEqual(
            set(results),
            {"Stationary buoy", "Moving vessel"},
        )
        for result in results.values():
            self.assertGreaterEqual(result.CoverageFraction, 0.90)
            self.assertLess(result.EarthPositionRmsErrorM, 25.0)
            self.assertLess(result.EarthPositionMaximumErrorM, 75.0)

    def test_earth_velocity_and_identity_remain_stable(self):
        for result in self.report.TargetResults:
            self.assertLess(result.EarthVelocityRmsErrorMps, 1.5)
            self.assertEqual(result.EarthTrackIdSwitches, 0)
            self.assertTrue(result.ReacquiredSameTrackAfterMisses)
            self.assertEqual(len(result.EarthTrackIds), 1)

    def test_mission_enu_filter_outperforms_legacy_polar_filter(self):
        for result in self.report.TargetResults:
            self.assertIsNotNone(result.LegacyPositionRmsErrorM)
            self.assertLess(
                result.EarthPositionRmsErrorM,
                result.LegacyPositionRmsErrorM,
            )

    def test_no_duplicate_confirmed_earth_tracks(self):
        self.assertEqual(
            self.report.MaximumDuplicateEarthConfirmedTracks,
            0,
        )
        self.assertLessEqual(
            self.report.MaximumEarthConfirmedTracks,
            2,
        )

    def test_missed_scan_outside_shorter_run_is_not_a_failure(self):
        report = RunOperationalCircularTrackerValidation(
            OperationalValidationConfig(ScansPerOrbit=36)
        )

        self.assertTrue(report.Passed, report.format_text())

    def test_report_is_serialisable_and_repeatable(self):
        second = RunOperationalCircularTrackerValidation()

        self.assertEqual(self.report, second)
        decoded = json.loads(self.report.to_json())
        self.assertTrue(decoded["Passed"])
        self.assertEqual(
            decoded["Version"],
            "parallel-tracker-full-orbit-v1",
        )


class TestStage7AuthorityBoundary(unittest.TestCase):
    def test_validation_report_keeps_legacy_authoritative(self):
        report = RunOperationalCircularTrackerValidation()

        self.assertTrue(report.LegacyTrackerAuthoritative)
        self.assertFalse(report.EarthTrackerAuthoritative)

    def test_scheduler_still_displays_only_legacy_tracks(self):
        scheduler = (
            Path(__file__).resolve().parent / "VanguardxMain_scheduler.py"
        ).read_text(encoding="utf-8")

        self.assertIn(
            'Processed.Diagnostics["EarthTrackerAuthoritative"] = False',
            scheduler,
        )
        self.assertIn(
            "Display.Update(Processed, Detections, Tracks=Tracks",
            scheduler,
        )
        self.assertNotIn(
            "Display.Update(Processed, Detections, Tracks=EarthTracks",
            scheduler,
        )


if __name__ == "__main__":
    unittest.main(verbosity=2)
