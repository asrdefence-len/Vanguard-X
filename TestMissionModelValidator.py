import ast
from dataclasses import replace
from pathlib import Path
import tempfile
import unittest

from MissionController import MissionController
from MissionPersistence import (
    LoadLastKnownGoodMission,
    SaveLastKnownGoodMission,
)
from MissionProfile import (
    CircularSpanDeg,
    CloneMission,
    CreateDefaultMission,
    MissionProfileFromJson,
    MissionProfileToJson,
    ReconcileMissionWaveforms,
    Scan360Task,
    SectorScanTask,
    TouchMission,
)
from MissionValidator import MissionLimits, MissionValidator


WAVEFORMS = ("Frank10_20MHz", "Golay64_20MHz")


class MissionProfileTests(unittest.TestCase):
    def test_json_round_trip_and_circular_sector_span(self):
        profile = CreateDefaultMission()
        restored = MissionProfileFromJson(MissionProfileToJson(profile))
        self.assertEqual(restored, profile)
        self.assertAlmostEqual(CircularSpanDeg(350.0, 10.0, "CW"), 20.0)
        self.assertAlmostEqual(CircularSpanDeg(10.0, 350.0, "CCW"), 20.0)
        self.assertAlmostEqual(CircularSpanDeg(10.0, 10.0, "CW"), 0.0)

    def test_loaded_snapshot_is_isolated_from_later_draft_edits(self):
        validator = MissionValidator(MissionLimits(WAVEFORMS))
        controller = MissionController(validator, CreateDefaultMission())
        result = controller.ValidateDraft()
        self.assertTrue(result.IsValid)
        loaded = controller.LoadValidated()

        controller.SetDraft(TouchMission(
            controller.DraftProfile,
            Name="Changed draft",
            PrimaryTasks=(
                replace(
                    controller.DraftProfile.PrimaryTasks[0],
                    RotationRpm=3.0,
                ),
            ),
        ))
        self.assertEqual(loaded.Name, "Mission 1")
        self.assertEqual(controller.LoadedProfile.Name, "Mission 1")
        self.assertEqual(controller.DraftProfile.Name, "Changed draft")
        self.assertEqual(len(controller.LoadedProfile.PrimaryTasks), 2)

    def test_unavailable_task_waveforms_reconcile_to_advertised_frank(self):
        profile = CreateDefaultMission()
        legacy_tasks = (
            replace(profile.PrimaryTasks[0], WaveformId="Golay24_20MHz"),
            replace(profile.PrimaryTasks[1], WaveformId="Golay64_20MHz"),
        )
        profile = TouchMission(profile, PrimaryTasks=legacy_tasks)

        reconciled = ReconcileMissionWaveforms(
            profile,
            ("Frank10_20MHz",),
        )

        self.assertEqual(
            tuple(task.WaveformId for task in reconciled.PrimaryTasks),
            ("Frank10_20MHz", "Frank10_20MHz"),
        )
        self.assertEqual(
            reconciled.TrackUpdate.WaveformId,
            "Frank10_20MHz",
        )
        self.assertTrue(
            MissionValidator(
                MissionLimits(("Frank10_20MHz",))
            ).Validate(reconciled).IsValid
        )

    def test_last_known_good_is_revalidated_and_reconciled_on_restore(self):
        profile = CreateDefaultMission()
        legacy_sector = replace(
            profile.PrimaryTasks[1],
            WaveformId="Golay24_20MHz",
        )
        profile = TouchMission(
            profile,
            Name="Last known good",
            PrimaryTasks=(profile.PrimaryTasks[0], legacy_sector),
        )
        validator = MissionValidator(
            MissionLimits(("Frank10_20MHz",))
        )

        with tempfile.TemporaryDirectory() as directory:
            config = {
                "MissionLastKnownGoodPath": str(
                    Path(directory) / "mission.vxmission.json"
                ),
            }
            SaveLastKnownGoodMission(profile, config)
            restored = LoadLastKnownGoodMission(config, validator)

        self.assertIsNotNone(restored)
        self.assertEqual(restored.Name, "Last known good")
        self.assertEqual(
            tuple(task.WaveformId for task in restored.PrimaryTasks),
            ("Frank10_20MHz", "Frank10_20MHz"),
        )


class MissionValidatorTests(unittest.TestCase):
    def test_continuous_baseline_allows_periodic_sector_interrupt(self):
        profile = CreateDefaultMission()
        result = MissionValidator(MissionLimits(WAVEFORMS)).Validate(profile)
        codes = {issue.Code for issue in result.Issues}
        self.assertTrue(result.IsValid)
        self.assertNotIn("CONTINUOUS_TASK_NOT_LAST", codes)
        self.assertNotIn("PERIODIC_BASELINE_REQUIRED", codes)

    def test_continuous_baseline_rejects_unscheduled_later_task(self):
        profile = CreateDefaultMission()
        sector = replace(
            profile.PrimaryTasks[1],
            RepeatIntervalSec=None,
        )
        profile = TouchMission(
            profile,
            PrimaryTasks=(profile.PrimaryTasks[0], sector),
        )
        result = MissionValidator(MissionLimits(WAVEFORMS)).Validate(profile)
        self.assertFalse(result.IsValid)
        self.assertIn(
            "CONTINUOUS_TASK_NOT_LAST",
            {issue.Code for issue in result.Issues},
        )

    def test_periodic_interval_must_exceed_interrupt_duration(self):
        profile = CreateDefaultMission()
        sector = replace(
            profile.PrimaryTasks[1],
            DurationSec=60.0,
            RepeatIntervalSec=60.0,
        )
        profile = TouchMission(
            profile,
            PrimaryTasks=(profile.PrimaryTasks[0], sector),
        )
        result = MissionValidator(MissionLimits(WAVEFORMS)).Validate(profile)
        self.assertFalse(result.IsValid)
        self.assertIn(
            "TASK_REPEAT_INTERVAL_TOO_SHORT",
            {issue.Code for issue in result.Issues},
        )

    def test_valid_wrap_sector_derives_timing_and_revisit(self):
        profile = CreateDefaultMission()
        sector = SectorScanTask(
            TaskId="sector-wrap",
            DurationSec=None,
            StartDeg=350.0,
            StopDeg=10.0,
            InitialDirection="CW",
            ScanRateDegSec=10.0,
            EndpointMarginDeg=1.0,
        )
        profile = TouchMission(profile, PrimaryTasks=(sector,))
        result = MissionValidator(MissionLimits(WAVEFORMS)).Validate(profile)
        self.assertTrue(result.IsValid)
        derived = result.TaskDerived["sector-wrap"]
        self.assertAlmostEqual(derived["SectorWidthDeg"], 20.0)
        self.assertAlmostEqual(derived["TraverseSec"], 1.8)
        self.assertAlmostEqual(derived["ExpectedRevisitSec"], 3.6)
        self.assertAlmostEqual(derived["CpiMs"], 16.0)

    def test_duplicate_id_no_enabled_task_and_unknown_waveform_are_errors(self):
        profile = CreateDefaultMission()
        first = replace(
            profile.PrimaryTasks[0],
            Enabled=False,
            WaveformId="UNKNOWN",
        )
        second = replace(
            profile.PrimaryTasks[1],
            TaskId=first.TaskId,
            Enabled=False,
        )
        profile = TouchMission(profile, PrimaryTasks=(first, second))
        result = MissionValidator(MissionLimits(WAVEFORMS)).Validate(profile)
        codes = {issue.Code for issue in result.Issues}
        self.assertFalse(result.IsValid)
        self.assertIn("NO_ENABLED_TASKS", codes)
        self.assertIn("TASK_ID_DUPLICATE", codes)
        self.assertIn("WAVEFORM_UNKNOWN", codes)

    def test_characterised_beamwidth_detects_unusable_360_scan(self):
        profile = CreateDefaultMission()
        task = Scan360Task(
            TaskId="fast",
            DurationSec=None,
            RotationRpm=24.0,
        )
        profile = TouchMission(profile, PrimaryTasks=(task,))
        limits = MissionLimits(
            WAVEFORMS,
            UsableBeamwidthDeg=2.0,
            MaximumRotationRpm=30.0,
            MaximumScanRateDegSec=180.0,
        )
        result = MissionValidator(limits).Validate(profile)
        codes = {issue.Code for issue in result.Issues}
        self.assertFalse(result.IsValid)
        self.assertIn("CPI_DOES_NOT_FIT_BEAM", codes)
        self.assertIn("AZIMUTH_COVERAGE_GAP", codes)

    def test_track_update_timeout_is_derived_and_rejected(self):
        profile = CreateDefaultMission()
        policy = replace(
            profile.TrackUpdate,
            Enabled=True,
            MaximumUpdateDurationSec=1.0,
        )
        profile = TouchMission(profile, TrackUpdate=policy)
        result = MissionValidator(MissionLimits(WAVEFORMS)).Validate(profile)
        self.assertGreater(result.TrackUpdateDerived["SampleCount"], 1)
        self.assertGreater(
            result.TrackUpdateDerived["EstimatedInterruptionSec"],
            policy.MaximumUpdateDurationSec,
        )
        self.assertIn(
            "TRACK_UPDATE_TIMEOUT",
            {issue.Code for issue in result.Issues},
        )


class MissionUiBoundaryTests(unittest.TestCase):
    def test_mission_page_has_no_hardware_imports(self):
        source_path = Path(__file__).with_name("MissionPage.py")
        tree = ast.parse(source_path.read_text(encoding="utf-8"))
        imported = set()
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                imported.update(alias.name for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module:
                imported.add(node.module)
        forbidden = {
            "EttusRadarSource",
            "RadarScheduler",
            "RadarExecutor",
            "AntennaController",
            "PTZController",
            "TRMInterface",
        }
        self.assertTrue(forbidden.isdisjoint(imported))

    def test_radar_page_is_wrapped_not_replaced(self):
        display_source = (
            Path(__file__).with_name("RadarDisplayQt5.py")
            .read_text(encoding="utf-8")
        )
        self.assertIn('self.OperatorTabs.addTab(CentralWidget, "Radar")', display_source)
        self.assertIn('self.OperatorTabs.addTab(self.MissionPage, "Mission")', display_source)
        self.assertIn("CreatePersistentOperatorHeader", display_source)

    def test_mission_execution_controller_has_no_hardware_or_qt_imports(self):
        source_path = Path(__file__).with_name(
            "MissionExecutionController.py"
        )
        tree = ast.parse(source_path.read_text(encoding="utf-8"))
        imported = set()
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                imported.update(alias.name for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module:
                imported.add(node.module)
        forbidden = {
            "PyQt5",
            "EttusRadarSource",
            "RadarExecutor",
            "RadarScheduler",
            "PointingManager",
            "X660Controller",
            "TRMInterface",
        }
        self.assertTrue(forbidden.isdisjoint(imported))

    def test_main_consumes_mission_commands_before_timing_and_dwell(self):
        main_source = (
            Path(__file__).with_name("VanguardxMain_scheduler.py")
            .read_text(encoding="utf-8")
        )
        apply_index = main_source.index(
            "MissionExecution.ApplyControlState"
        )
        timing_index = main_source.index(
            "TimingApplication = ApplyTimingControlState",
            apply_index,
        )
        dwell_index = main_source.index(
            "DwellResult = ExecuteRadarDwell",
            timing_index,
        )
        self.assertLess(apply_index, timing_index)
        self.assertLess(timing_index, dwell_index)


if __name__ == "__main__":
    unittest.main()
