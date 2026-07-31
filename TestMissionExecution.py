from dataclasses import replace
import unittest

from MissionExecutionController import MissionExecutionController
from MissionProfile import (
    CreateDefaultMission,
    MissionProfileToDict,
    Scan360Task,
    SectorScanTask,
    TouchMission,
)


def _Config():
    return {
        "AvailableWaveformIds": (
            "Frank10_20MHz",
            "Golay64_20MHz",
        ),
        "MinPrfHz": 1000.0,
        "MaxPrfHz": 4000.0,
        "MinPulsesPerCpi": 8,
        "MaxPulsesPerCpi": 128,
        "MinSelectableRangeM": 1000.0,
        "MaxSelectableRangeM": 15000.0,
        "RadarDwellIntervalSec": 0.1,
    }


def _HardConfig():
    config = _Config()
    config.update({
        "SystemMode": "HARD",
        "RadarSource": "ETTUS",
        "EttusOperatingMode": "RECEIVE_ONLY",
        "EttusTimedTransmitEnabled": False,
        "EttusAtrGpioEnabled": False,
        "X660Mode": "x660-operational",
        "X660MotionEnabled": True,
    })
    return config


def _StoppedControl(revision, command, profile=None):
    return {
        "DisplayMode": "STOP",
        "ScanEnabled": False,
        "TransmitAvailable": True,
        "TransmitEnabled": False,
        "MissionCommandRevision": revision,
        "MissionCommand": command,
        "MissionProfile": profile,
    }


def _ShortMission(completion="STOP"):
    base = CreateDefaultMission()
    tasks = (
        Scan360Task(
            TaskId="scan",
            Name="Short 360",
            DurationSec=1.0,
            WaveformId="Frank10_20MHz",
            PrfHz=2000.0,
            PulsesPerCpi=32,
            RotationRpm=2.0,
        ),
        SectorScanTask(
            TaskId="sector",
            Name="Short sector",
            DurationSec=2.0,
            WaveformId="Golay64_20MHz",
            PrfHz=4000.0,
            PulsesPerCpi=64,
            StartDeg=350.0,
            StopDeg=10.0,
            ScanRateDegSec=5.0,
            EndpointMarginDeg=1.0,
        ),
    )
    return TouchMission(
        base,
        CompletionPolicy=completion,
        PrimaryTasks=tasks,
    )


def _PeriodicMission():
    base = CreateDefaultMission()
    tasks = (
        Scan360Task(
            TaskId="baseline",
            Name="Continuous 360",
            DurationSec=None,
            WaveformId="Frank10_20MHz",
            PrfHz=2000.0,
            PulsesPerCpi=32,
            RotationRpm=2.0,
        ),
        SectorScanTask(
            TaskId="periodic-sector",
            Name="Periodic sector",
            DurationSec=2.0,
            RepeatIntervalSec=5.0,
            WaveformId="Golay64_20MHz",
            PrfHz=4000.0,
            PulsesPerCpi=64,
            StartDeg=350.0,
            StopDeg=10.0,
            ScanRateDegSec=5.0,
            EndpointMarginDeg=1.0,
        ),
    )
    return TouchMission(base, PrimaryTasks=tasks)


class MissionExecutionTests(unittest.TestCase):
    def _LoadedRuntime(self, profile=None):
        runtime = MissionExecutionController(_Config())
        mission = profile or _ShortMission()
        result = runtime.ApplyControlState(
            _StoppedControl(1, "LOAD", MissionProfileToDict(mission)),
            system_mode="SIM",
            now_sec=0.0,
        )
        self.assertTrue(result.Applied, result.Message)
        self.assertEqual(runtime.State, "LOADED")
        return runtime

    def test_load_start_and_task_intent_are_applied_in_sim(self):
        runtime = self._LoadedRuntime()
        result = runtime.ApplyControlState(
            _StoppedControl(2, "START"),
            system_mode="SIM",
            now_sec=10.0,
        )
        self.assertTrue(result.Applied)
        self.assertEqual(runtime.State, "RUNNING_360")
        effective = runtime.BuildEffectiveControlState(
            {"TransmitAvailable": True}
        )
        self.assertEqual(effective["DisplayMode"], "SCAN")
        self.assertTrue(effective["ScanEnabled"])
        self.assertTrue(effective["TransmitEnabled"])
        self.assertEqual(effective["SelectedWaveformId"], "Frank10_20MHz")
        self.assertEqual(effective["SelectedPrfHz"], 2000.0)
        self.assertEqual(effective["ScanStartDeg"], 0.0)
        self.assertAlmostEqual(effective["MissionScanRateDegSec"], 12.0)
        self.assertEqual(
            effective["MissionScanPattern"],
            "CONTINUOUS_CW",
        )
        self.assertEqual(effective["MissionScanFrame"], "TRUE")
        status = runtime.GetStatus()
        self.assertEqual(status["ActiveWaveformId"], "Frank10_20MHz")
        self.assertEqual(status["ActivePrfHz"], 2000.0)
        self.assertEqual(status["ActivePulsesPerCpi"], 32)

    def test_receive_only_hard_mode_can_start_and_resume(self):
        runtime = MissionExecutionController(_HardConfig())
        loaded = runtime.ApplyControlState(
            _StoppedControl(
                1,
                "LOAD",
                MissionProfileToDict(_ShortMission()),
            ),
            system_mode="HARD",
            now_sec=0.0,
        )
        self.assertTrue(loaded.Applied, loaded.Message)
        started = runtime.ApplyControlState(
            _StoppedControl(2, "START"),
            system_mode="HARD",
            now_sec=10.0,
        )
        self.assertTrue(started.Applied, started.Message)
        self.assertEqual(runtime.State, "RUNNING_360")

        paused = runtime.ApplyControlState(
            {"MissionCommandRevision": 3, "MissionCommand": "PAUSE"},
            system_mode="HARD",
            now_sec=10.5,
        )
        self.assertTrue(paused.Applied, paused.Message)
        resumed = runtime.ApplyControlState(
            {"MissionCommandRevision": 4, "MissionCommand": "RESUME"},
            system_mode="HARD",
            now_sec=11.0,
        )
        self.assertTrue(resumed.Applied, resumed.Message)
        self.assertEqual(runtime.State, "RUNNING_360")

    def test_hard_mode_rejects_non_receive_only_configuration(self):
        unsafe_config = _HardConfig()
        unsafe_config["EttusTimedTransmitEnabled"] = True
        runtime = MissionExecutionController(unsafe_config)
        runtime.ApplyControlState(
            _StoppedControl(
                1,
                "LOAD",
                MissionProfileToDict(_ShortMission()),
            ),
            system_mode="HARD",
            now_sec=0.0,
        )
        rejected = runtime.ApplyControlState(
            _StoppedControl(2, "START"),
            system_mode="HARD",
            now_sec=10.0,
        )
        self.assertFalse(rejected.Applied)
        self.assertIn("timed transmit must be disabled", rejected.Message)
        self.assertEqual(runtime.State, "LOADED")

    def test_rf_loopback_mode_cannot_start_mission(self):
        runtime = self._LoadedRuntime()
        rejected = runtime.ApplyControlState(
            _StoppedControl(2, "START"),
            system_mode="RF LOOPBACK",
            now_sec=10.0,
        )
        self.assertFalse(rejected.Applied)
        self.assertIn("receive-only HARD", rejected.Message)
        self.assertEqual(runtime.State, "LOADED")

    def test_pause_resume_preserves_active_time_and_task(self):
        runtime = self._LoadedRuntime()
        runtime.ApplyControlState(
            _StoppedControl(2, "START"),
            system_mode="SIM",
            now_sec=10.0,
        )
        runtime.Advance(10.4)
        pause = runtime.ApplyControlState(
            {"MissionCommandRevision": 3, "MissionCommand": "PAUSE"},
            system_mode="SIM",
            now_sec=10.6,
        )
        self.assertTrue(pause.Applied)
        self.assertEqual(runtime.State, "PAUSED")
        self.assertAlmostEqual(runtime.ActiveTaskElapsedSec, 0.6)

        runtime.Advance(100.0)
        self.assertAlmostEqual(runtime.ActiveTaskElapsedSec, 0.6)
        resume = runtime.ApplyControlState(
            {"MissionCommandRevision": 4, "MissionCommand": "RESUME"},
            system_mode="SIM",
            now_sec=100.0,
        )
        self.assertTrue(resume.Applied)
        self.assertEqual(runtime.State, "RUNNING_360")
        runtime.Advance(100.4)
        self.assertEqual(runtime.State, "RUNNING_SECTOR")
        self.assertEqual(runtime.ActiveTask.TaskId, "sector")

    def test_sequence_completes_only_at_advance_boundary(self):
        runtime = self._LoadedRuntime()
        runtime.ApplyControlState(
            _StoppedControl(2, "START"),
            system_mode="SIM",
            now_sec=0.0,
        )
        runtime.Advance(0.99)
        self.assertEqual(runtime.ActiveTask.TaskId, "scan")
        first_activation = runtime.TaskActivationRevision
        runtime.Advance(1.0)
        self.assertEqual(runtime.ActiveTask.TaskId, "sector")
        self.assertGreater(runtime.TaskActivationRevision, first_activation)
        runtime.Advance(3.0)
        self.assertEqual(runtime.State, "COMPLETED")
        effective = runtime.BuildEffectiveControlState(
            {"DisplayMode": "SCAN", "TransmitEnabled": True}
        )
        self.assertEqual(effective["DisplayMode"], "STOP")
        self.assertFalse(effective["TransmitEnabled"])

    def test_repeat_policy_restarts_first_enabled_task(self):
        runtime = self._LoadedRuntime(_ShortMission(completion="REPEAT"))
        runtime.ApplyControlState(
            _StoppedControl(2, "START"),
            system_mode="SIM",
            now_sec=0.0,
        )
        runtime.Advance(1.0)
        runtime.Advance(3.0)
        self.assertEqual(runtime.State, "RUNNING_360")
        self.assertEqual(runtime.ActiveTask.TaskId, "scan")
        self.assertAlmostEqual(runtime.ActiveTaskElapsedSec, 0.0)

    def test_operator_stop_aborts_and_requires_reload(self):
        runtime = self._LoadedRuntime()
        runtime.ApplyControlState(
            _StoppedControl(2, "START"),
            system_mode="SIM",
            now_sec=0.0,
        )
        stopped = runtime.ApplyControlState(
            {"MissionCommandRevision": 3, "MissionCommand": "STOP"},
            system_mode="SIM",
            now_sec=0.5,
        )
        self.assertTrue(stopped.Applied)
        self.assertEqual(runtime.State, "ABORTED")
        self.assertIsNone(runtime.LoadedProfile)
        restart = runtime.ApplyControlState(
            _StoppedControl(4, "START"),
            system_mode="SIM",
            now_sec=1.0,
        )
        self.assertFalse(restart.Applied)

    def test_dashboard_start_reclaims_control_after_mission_abort(self):
        runtime = self._LoadedRuntime()
        runtime.ApplyControlState(
            {
                **_StoppedControl(2, "START"),
                "ManualControlCommandId": 4,
            },
            system_mode="SIM",
            now_sec=0.0,
        )
        runtime.ApplyControlState(
            {
                "MissionCommandRevision": 3,
                "MissionCommand": "STOP",
                "ManualControlCommandId": 4,
            },
            system_mode="SIM",
            now_sec=0.5,
        )

        terminal_stop = runtime.BuildEffectiveControlState({
            "DisplayMode": "SCAN",
            "ScanEnabled": True,
            "TransmitEnabled": True,
            "ManualControlCommandId": 4,
        })
        self.assertEqual(terminal_stop["DisplayMode"], "STOP")
        self.assertFalse(terminal_stop["ScanEnabled"])
        self.assertFalse(terminal_stop["TransmitEnabled"])

        dashboard_start = runtime.BuildEffectiveControlState({
            "DisplayMode": "SCAN",
            "ScanEnabled": True,
            "TransmitEnabled": True,
            "ManualControlCommandId": 5,
        })
        self.assertEqual(dashboard_start["DisplayMode"], "SCAN")
        self.assertTrue(dashboard_start["ScanEnabled"])
        self.assertTrue(dashboard_start["TransmitEnabled"])
        self.assertEqual(runtime.State, "ABORTED")

    def test_loaded_mission_cannot_be_bypassed_by_dashboard_start(self):
        runtime = self._LoadedRuntime()
        effective = runtime.BuildEffectiveControlState({
            "DisplayMode": "SCAN",
            "ScanEnabled": True,
            "TransmitEnabled": True,
            "ManualControlCommandId": 1,
        })
        self.assertEqual(effective["DisplayMode"], "STOP")
        self.assertFalse(effective["ScanEnabled"])
        self.assertFalse(effective["TransmitEnabled"])

    def test_loaded_snapshot_is_detached_from_command_payload(self):
        profile_dict = MissionProfileToDict(_ShortMission())
        runtime = MissionExecutionController(_Config())
        runtime.ApplyControlState(
            _StoppedControl(1, "LOAD", profile_dict),
            system_mode="SIM",
            now_sec=0.0,
        )
        profile_dict["Name"] = "Mutated"
        profile_dict["PrimaryTasks"][0]["Name"] = "Mutated task"
        self.assertEqual(runtime.LoadedProfile.Name, "Mission 1")
        self.assertEqual(runtime.LoadedProfile.PrimaryTasks[0].Name, "Short 360")

    def test_duplicate_revision_is_idempotent(self):
        runtime = self._LoadedRuntime()
        runtime.ApplyControlState(
            _StoppedControl(2, "START"),
            system_mode="SIM",
            now_sec=0.0,
        )
        activation = runtime.TaskActivationRevision
        duplicate = runtime.ApplyControlState(
            _StoppedControl(2, "START"),
            system_mode="SIM",
            now_sec=0.1,
        )
        self.assertTrue(duplicate.Applied)
        self.assertEqual(runtime.TaskActivationRevision, activation)
        self.assertEqual(runtime.State, "RUNNING_360")

    def test_invalid_live_profile_is_rejected(self):
        mission = _ShortMission()
        invalid = TouchMission(
            mission,
            PrimaryTasks=(
                replace(mission.PrimaryTasks[0], WaveformId="UNKNOWN"),
            ),
        )
        runtime = MissionExecutionController(_Config())
        result = runtime.ApplyControlState(
            _StoppedControl(1, "LOAD", MissionProfileToDict(invalid)),
            system_mode="SIM",
            now_sec=0.0,
        )
        self.assertFalse(result.Applied)
        self.assertEqual(runtime.State, "STOPPED")

    def test_periodic_sector_preempts_and_resumes_continuous_baseline(self):
        runtime = self._LoadedRuntime(_PeriodicMission())
        runtime.ApplyControlState(
            _StoppedControl(2, "START"),
            system_mode="SIM",
            now_sec=0.0,
        )
        runtime.Advance(4.9)
        self.assertEqual(runtime.ActiveTask.TaskId, "baseline")

        runtime.Advance(5.0)
        self.assertEqual(runtime.State, "RUNNING_SECTOR")
        self.assertEqual(runtime.ActiveTask.TaskId, "periodic-sector")
        sector_control = runtime.BuildEffectiveControlState({})
        self.assertEqual(sector_control["MissionScanPattern"], "SECTOR")
        self.assertEqual(sector_control["MissionScanFrame"], "TRUE")

        runtime.Advance(7.0)
        self.assertEqual(runtime.State, "RUNNING_360")
        self.assertEqual(runtime.ActiveTask.TaskId, "baseline")
        baseline_control = runtime.BuildEffectiveControlState({})
        self.assertEqual(
            baseline_control["MissionScanPattern"],
            "CONTINUOUS_CW",
        )
        self.assertEqual(baseline_control["MissionScanFrame"], "TRUE")
        self.assertAlmostEqual(runtime.ActiveTaskElapsedSec, 5.0)
        status = runtime.GetStatus()
        self.assertTrue(status["PeriodicScheduling"])
        self.assertEqual(status["NextPeriodicTaskId"], "periodic-sector")
        self.assertAlmostEqual(status["NextPeriodicDueInSec"], 3.0)

        runtime.Advance(9.9)
        self.assertEqual(runtime.ActiveTask.TaskId, "baseline")
        runtime.Advance(10.0)
        self.assertEqual(runtime.ActiveTask.TaskId, "periodic-sector")

    def test_pause_freezes_periodic_recurrence_clock(self):
        runtime = self._LoadedRuntime(_PeriodicMission())
        runtime.ApplyControlState(
            _StoppedControl(2, "START"),
            system_mode="SIM",
            now_sec=0.0,
        )
        runtime.Advance(4.0)
        runtime.ApplyControlState(
            {"MissionCommandRevision": 3, "MissionCommand": "PAUSE"},
            system_mode="SIM",
            now_sec=4.0,
        )
        runtime.ApplyControlState(
            {"MissionCommandRevision": 4, "MissionCommand": "RESUME"},
            system_mode="SIM",
            now_sec=100.0,
        )
        runtime.Advance(100.9)
        self.assertEqual(runtime.ActiveTask.TaskId, "baseline")
        runtime.Advance(101.0)
        self.assertEqual(runtime.ActiveTask.TaskId, "periodic-sector")

    def test_track_resource_hold_defers_periodic_sector_and_task_time(self):
        runtime = self._LoadedRuntime(_PeriodicMission())
        runtime.ApplyControlState(
            _StoppedControl(2, "START"),
            system_mode="SIM",
            now_sec=0.0,
        )
        runtime.Advance(4.9)
        elapsed_before_hold = runtime.ActiveTaskElapsedSec
        activation_before_hold = runtime.TaskActivationRevision

        runtime.Advance(5.5, resource_available=False)
        runtime.Advance(8.0, resource_available=False)
        self.assertEqual(runtime.ActiveTask.TaskId, "baseline")
        self.assertAlmostEqual(
            runtime.ActiveTaskElapsedSec,
            elapsed_before_hold,
        )
        self.assertEqual(
            runtime.TaskActivationRevision,
            activation_before_hold,
        )
        self.assertTrue(runtime.GetStatus()["RadarResourceHeld"])

        runtime.Advance(8.0, resource_available=True)
        self.assertEqual(runtime.ActiveTask.TaskId, "baseline")
        self.assertFalse(runtime.GetStatus()["RadarResourceHeld"])
        runtime.Advance(8.1, resource_available=True)
        self.assertEqual(runtime.ActiveTask.TaskId, "periodic-sector")


if __name__ == "__main__":
    unittest.main()
