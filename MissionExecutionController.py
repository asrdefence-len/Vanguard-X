"""Radar-side mission execution state machine for Vanguard X.

This module is deliberately independent of Qt, the radar source, the antenna
adapter, and UHD.  It accepts versioned operator requests, owns the immutable
loaded mission snapshot, accounts active task time, and publishes task intent
for the existing scheduler/main loop to apply at a safe dwell boundary.

Mission execution is permitted in SIM and in the guarded HARD profile.  HARD
must retain the operational X6-60 controller while the Ettus remains
receive-only with timed transmit and ATR disabled.
"""

from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Any, Dict, Optional

from MissionProfile import (
    CloneMission,
    MissionProfile,
    MissionProfileFromDict,
    Scan360Task,
    SectorScanTask,
)
from MissionValidator import MissionValidator


RUNNING_STATES = ("RUNNING_360", "RUNNING_SECTOR")
TERMINAL_STATES = ("COMPLETED", "ABORTED", "FAULTED")
MISSION_COMMANDS = ("LOAD", "START", "PAUSE", "RESUME", "STOP")
MISSION_TIMING_REVISION_BASE = 1_000_000


@dataclass(frozen=True)
class MissionCommandResult:
    Applied: bool
    Changed: bool
    Message: str


class MissionExecutionController:
    """Own and advance one validated mission snapshot."""

    def __init__(self, config: Dict[str, Any]):
        self.Config = dict(config)
        self.Validator = MissionValidator.FromConfig(self.Config)
        self.LoadedProfile: Optional[MissionProfile] = None
        self.State = "STOPPED"
        self.ActiveTaskIndex: Optional[int] = None
        self.ActiveTaskElapsedSec = 0.0
        self.MissionWallStartSec: Optional[float] = None
        self.MissionWallElapsedSec = 0.0
        self.LastCommandRevision = 0
        self.StatusRevision = 0
        self.TaskActivationRevision = 0
        self.TaskTimingRevision = 0
        self.LastMessage = "No mission loaded"
        self.FaultReason = ""
        self._last_accounting_sec: Optional[float] = None
        self._last_boundary_sec: Optional[float] = None
        self._periodic_next_due_sec: Dict[int, float] = {}
        self._suspended_task_index: Optional[int] = None
        self._suspended_task_elapsed_sec = 0.0
        self._pause_started_sec: Optional[float] = None
        self._last_manual_control_command_id = 0
        self._terminal_manual_control_command_id = 0

    @property
    def IsRunning(self) -> bool:
        return self.State in RUNNING_STATES

    @property
    def ActiveTask(self):
        if self.LoadedProfile is None or self.ActiveTaskIndex is None:
            return None
        tasks = self.LoadedProfile.PrimaryTasks
        if not 0 <= self.ActiveTaskIndex < len(tasks):
            return None
        return tasks[self.ActiveTaskIndex]

    def _Now(self, now_sec: Optional[float]) -> float:
        return time.monotonic() if now_sec is None else float(now_sec)

    def _SetStatus(self, state: str, message: str):
        new_state = str(state)
        state_changed = self.State != new_state
        changed = state_changed or self.LastMessage != str(message)
        if state_changed and new_state in TERMINAL_STATES:
            # A terminal Mission transition owns one safe STOP.  A later,
            # explicit Dashboard run/nudge command may then take control back
            # without requiring the Mission state or status history to be
            # discarded.
            self._terminal_manual_control_command_id = (
                self._last_manual_control_command_id
            )
        self.State = new_state
        self.LastMessage = str(message)
        if changed:
            self.StatusRevision += 1

    def _ExecutionModeError(self, system_mode: str) -> Optional[str]:
        """Return why Mission execution is unsafe in the selected mode."""

        mode = str(system_mode).upper()
        if mode == "SIM":
            return None
        if mode != "HARD":
            return (
                "Mission execution is enabled in SIM and receive-only HARD "
                "modes only"
            )

        hard_requirements = (
            (
                str(self.Config.get("SystemMode", "")).upper() == "HARD",
                "SystemMode must be HARD",
            ),
            (
                str(self.Config.get("RadarSource", "")).upper() == "ETTUS",
                "RadarSource must be ETTUS",
            ),
            (
                str(
                    self.Config.get("EttusOperatingMode", "")
                ).upper() == "RECEIVE_ONLY",
                "EttusOperatingMode must be RECEIVE_ONLY",
            ),
            (
                not bool(
                    self.Config.get("EttusTimedTransmitEnabled", False)
                ),
                "timed transmit must be disabled",
            ),
            (
                not bool(self.Config.get("EttusAtrGpioEnabled", False)),
                "ATR GPIO must be disabled",
            ),
            (
                str(self.Config.get("X660Mode", "")).lower()
                == "x660-operational",
                "X660Mode must be x660-operational",
            ),
            (
                bool(self.Config.get("X660MotionEnabled", False)),
                "X6-60 motion must be enabled",
            ),
        )
        failures = [
            message for requirement, message in hard_requirements
            if not requirement
        ]
        if failures:
            return "HARD Mission execution rejected: " + "; ".join(failures)
        return None

    def _AccountRunningTime(self, now_sec: float):
        if not self.IsRunning:
            self._last_accounting_sec = None
            return
        if self._last_accounting_sec is None:
            self._last_accounting_sec = now_sec
            return
        delta_sec = max(0.0, now_sec - self._last_accounting_sec)
        self.ActiveTaskElapsedSec += delta_sec
        self._last_accounting_sec = now_sec
        if self.MissionWallStartSec is not None:
            self.MissionWallElapsedSec = max(
                0.0, now_sec - self.MissionWallStartSec
            )

    def _EnabledTaskIndices(self):
        if self.LoadedProfile is None:
            return []
        return [
            index
            for index, task in enumerate(self.LoadedProfile.PrimaryTasks)
            if task.Enabled
        ]

    def _PeriodicTaskIndices(self):
        if self.LoadedProfile is None:
            return []
        return [
            index
            for index in self._EnabledTaskIndices()
            if self.LoadedProfile.PrimaryTasks[index].RepeatIntervalSec
            is not None
        ]

    def _UsesPeriodicInterrupts(self) -> bool:
        return bool(self._PeriodicTaskIndices())

    def _ContinuousBaselineIndex(self) -> Optional[int]:
        if self.LoadedProfile is None:
            return None
        for index in self._EnabledTaskIndices():
            if self.LoadedProfile.PrimaryTasks[index].DurationSec is None:
                return index
        return None

    def _InitialisePeriodicSchedule(self, now_sec: float):
        self._periodic_next_due_sec = {}
        if self.LoadedProfile is None:
            return
        for index in self._PeriodicTaskIndices():
            task = self.LoadedProfile.PrimaryTasks[index]
            self._periodic_next_due_sec[index] = (
                now_sec + float(task.RepeatIntervalSec)
            )

    def _NextDuePeriodicIndex(self, now_sec: float) -> Optional[int]:
        due = [
            (due_sec, index)
            for index, due_sec in self._periodic_next_due_sec.items()
            if due_sec <= now_sec
        ]
        return min(due)[1] if due else None

    def _ScheduleNextOccurrence(self, task_index: int, now_sec: float):
        if self.LoadedProfile is None:
            return
        task = self.LoadedProfile.PrimaryTasks[task_index]
        interval_sec = float(task.RepeatIntervalSec)
        next_due = self._periodic_next_due_sec[task_index] + interval_sec
        while next_due <= now_sec:
            next_due += interval_sec
        self._periodic_next_due_sec[task_index] = next_due

    def _RunningStateForTask(self, task) -> str:
        if isinstance(task, Scan360Task):
            return "RUNNING_360"
        if isinstance(task, SectorScanTask):
            return "RUNNING_SECTOR"
        raise TypeError(f"unsupported mission task {type(task).__name__}")

    def _ActivateTask(self, task_index: int, now_sec: float, reason: str):
        self.ActiveTaskIndex = int(task_index)
        self.ActiveTaskElapsedSec = 0.0
        self._last_accounting_sec = now_sec
        self.TaskActivationRevision += 1
        self.TaskTimingRevision += 1
        task = self.ActiveTask
        self._SetStatus(
            self._RunningStateForTask(task),
            f"{reason}: {task.Name}",
        )

    def _ResumeSuspendedBaseline(self, now_sec: float):
        task_index = self._suspended_task_index
        if task_index is None:
            task_index = self._ContinuousBaselineIndex()
        if task_index is None:
            self._CompleteOrRepeat(now_sec)
            return
        self.ActiveTaskIndex = int(task_index)
        self.ActiveTaskElapsedSec = float(self._suspended_task_elapsed_sec)
        self._suspended_task_index = None
        self._suspended_task_elapsed_sec = 0.0
        self._last_accounting_sec = now_sec
        self.TaskActivationRevision += 1
        self.TaskTimingRevision += 1
        task = self.ActiveTask
        self._SetStatus(
            self._RunningStateForTask(task),
            f"Baseline resumed: {task.Name}",
        )

    def _AdvancePeriodicMission(self, now_sec: float) -> bool:
        previous_revision = self.StatusRevision
        task = self.ActiveTask
        if not self.IsRunning or task is None:
            return False

        active_index = int(self.ActiveTaskIndex)
        is_periodic = active_index in self._periodic_next_due_sec
        if is_periodic:
            if (
                task.DurationSec is not None
                and self.ActiveTaskElapsedSec >= float(task.DurationSec)
            ):
                self._ScheduleNextOccurrence(active_index, now_sec)
                next_due_index = self._NextDuePeriodicIndex(now_sec)
                if next_due_index is not None:
                    self._ActivateTask(
                        next_due_index,
                        now_sec,
                        "Periodic task started",
                    )
                else:
                    self._ResumeSuspendedBaseline(now_sec)
            return self.StatusRevision != previous_revision

        due_index = self._NextDuePeriodicIndex(now_sec)
        if due_index is not None:
            self._suspended_task_index = active_index
            self._suspended_task_elapsed_sec = self.ActiveTaskElapsedSec
            self._ActivateTask(
                due_index,
                now_sec,
                "Periodic task started",
            )
        return self.StatusRevision != previous_revision

    def _FirstEnabledTaskIndex(self) -> Optional[int]:
        enabled = self._EnabledTaskIndices()
        return enabled[0] if enabled else None

    def _NextEnabledTaskIndex(self) -> Optional[int]:
        if self.ActiveTaskIndex is None:
            return self._FirstEnabledTaskIndex()
        for index in self._EnabledTaskIndices():
            if index > self.ActiveTaskIndex:
                return index
        return None

    def _CompleteOrRepeat(self, now_sec: float):
        profile = self.LoadedProfile
        if profile is not None and profile.CompletionPolicy == "REPEAT":
            first_index = self._FirstEnabledTaskIndex()
            if first_index is not None:
                self._ActivateTask(first_index, now_sec, "Mission repeated")
                return
        self.ActiveTaskIndex = None
        self.ActiveTaskElapsedSec = 0.0
        self._last_accounting_sec = None
        self._periodic_next_due_sec = {}
        self._suspended_task_index = None
        self._suspended_task_elapsed_sec = 0.0
        self._SetStatus("COMPLETED", "Mission completed")

    def Advance(self, now_sec: Optional[float] = None) -> bool:
        """Advance duration accounting at a safe scheduler boundary."""

        now = self._Now(now_sec)
        self._last_boundary_sec = now
        previous_revision = self.StatusRevision
        self._AccountRunningTime(now)
        task = self.ActiveTask
        if self._UsesPeriodicInterrupts():
            return self._AdvancePeriodicMission(now)
        if not self.IsRunning or task is None or task.DurationSec is None:
            return self.StatusRevision != previous_revision
        if self.ActiveTaskElapsedSec < float(task.DurationSec):
            return self.StatusRevision != previous_revision

        next_index = self._NextEnabledTaskIndex()
        if next_index is None:
            self._CompleteOrRepeat(now)
        else:
            self._ActivateTask(next_index, now, "Task started")
        return self.StatusRevision != previous_revision

    def ApplyControlState(
        self,
        control_state: Optional[Dict[str, Any]],
        *,
        system_mode: str,
        now_sec: Optional[float] = None,
    ) -> MissionCommandResult:
        """Apply at most one new operator command, then advance task time.

        Command revisions are edge identifiers, not counters that must increase.
        This allows a freshly restarted UI to reconnect to an existing radar
        process without its first command being discarded as numerically older.
        """

        now = self._Now(now_sec)
        self._last_boundary_sec = now
        control = dict(control_state or {})
        self._last_manual_control_command_id = int(
            control.get(
                "ManualControlCommandId",
                self._last_manual_control_command_id,
            )
        )
        command = str(control.get("MissionCommand", "")).upper().strip()
        revision = int(control.get("MissionCommandRevision", 0))

        if not command or revision == self.LastCommandRevision:
            changed = self.Advance(now)
            return MissionCommandResult(True, changed, self.LastMessage)

        self.LastCommandRevision = revision
        if command not in MISSION_COMMANDS:
            message = f"Rejected unsupported mission command {command!r}"
            self.LastMessage = message
            self.StatusRevision += 1
            return MissionCommandResult(False, True, message)

        if command == "LOAD":
            return self._Load(control, now)
        if command == "START":
            return self._Start(control, system_mode, now)
        if command == "PAUSE":
            return self._Pause(now)
        if command == "RESUME":
            return self._Resume(system_mode, now)
        return self._Stop(now)

    def _Load(
        self, control_state: Dict[str, Any], now_sec: float
    ) -> MissionCommandResult:
        is_stopped = bool(
            str(control_state.get("DisplayMode", "STOP")).upper() == "STOP"
            and not control_state.get("ScanEnabled", False)
            and not control_state.get("TransmitEnabled", False)
        )
        if self.IsRunning or self.State == "PAUSED" or not is_stopped:
            message = "Stop the radar before loading a mission"
            self.LastMessage = message
            self.StatusRevision += 1
            return MissionCommandResult(False, True, message)
        try:
            profile = MissionProfileFromDict(
                dict(control_state.get("MissionProfile") or {})
            )
            validation = self.Validator.Validate(profile)
            if not validation.IsValid:
                errors = [
                    issue.Message
                    for issue in validation.Issues
                    if issue.Severity == "ERROR"
                ]
                raise ValueError("; ".join(errors) or "mission validation failed")
        except Exception as error:
            message = f"Mission load rejected: {error}"
            self.LastMessage = message
            self.StatusRevision += 1
            return MissionCommandResult(False, True, message)

        self.LoadedProfile = CloneMission(profile)
        self.ActiveTaskIndex = None
        self.ActiveTaskElapsedSec = 0.0
        self.MissionWallStartSec = None
        self.MissionWallElapsedSec = 0.0
        self.FaultReason = ""
        self._last_accounting_sec = None
        self._periodic_next_due_sec = {}
        self._suspended_task_index = None
        self._suspended_task_elapsed_sec = 0.0
        self._pause_started_sec = None
        message = (
            f"Loaded {self.LoadedProfile.Name} "
            f"r{self.LoadedProfile.Revision}"
        )
        self._SetStatus("LOADED", message)
        return MissionCommandResult(True, True, message)

    def _Start(
        self,
        control_state: Dict[str, Any],
        system_mode: str,
        now_sec: float,
    ) -> MissionCommandResult:
        mode_error = self._ExecutionModeError(system_mode)
        if mode_error is not None:
            message = mode_error
            self.LastMessage = message
            self.StatusRevision += 1
            return MissionCommandResult(False, True, message)
        if self.State != "LOADED" or self.LoadedProfile is None:
            message = "Load a validated mission before starting"
            self.LastMessage = message
            self.StatusRevision += 1
            return MissionCommandResult(False, True, message)
        is_stopped = bool(
            str(control_state.get("DisplayMode", "STOP")).upper() == "STOP"
            and not control_state.get("ScanEnabled", False)
            and not control_state.get("TransmitEnabled", False)
        )
        if not is_stopped:
            message = "Stop the radar before starting a mission"
            self.LastMessage = message
            self.StatusRevision += 1
            return MissionCommandResult(False, True, message)
        if self._UsesPeriodicInterrupts():
            first_index = self._ContinuousBaselineIndex()
            self._InitialisePeriodicSchedule(now_sec)
        else:
            first_index = self._FirstEnabledTaskIndex()
        if first_index is None:
            message = "Loaded mission contains no enabled task"
            self.LastMessage = message
            self.StatusRevision += 1
            return MissionCommandResult(False, True, message)

        self.MissionWallStartSec = now_sec
        self.MissionWallElapsedSec = 0.0
        self._ActivateTask(first_index, now_sec, "Mission started")
        return MissionCommandResult(True, True, self.LastMessage)

    def _Pause(self, now_sec: float) -> MissionCommandResult:
        if not self.IsRunning:
            message = "Mission is not running"
            self.LastMessage = message
            self.StatusRevision += 1
            return MissionCommandResult(False, True, message)
        self._AccountRunningTime(now_sec)
        task = self.ActiveTask
        self._last_accounting_sec = None
        self._pause_started_sec = now_sec
        self._SetStatus(
            "PAUSED",
            f"Paused: {task.Name if task is not None else 'mission'}",
        )
        return MissionCommandResult(True, True, self.LastMessage)

    def _Resume(
        self, system_mode: str, now_sec: float
    ) -> MissionCommandResult:
        mode_error = self._ExecutionModeError(system_mode)
        if mode_error is not None:
            message = mode_error
            self.LastMessage = message
            self.StatusRevision += 1
            return MissionCommandResult(False, True, message)
        task = self.ActiveTask
        if self.State != "PAUSED" or task is None:
            message = "Mission is not paused"
            self.LastMessage = message
            self.StatusRevision += 1
            return MissionCommandResult(False, True, message)
        if self._pause_started_sec is not None:
            paused_sec = max(0.0, now_sec - self._pause_started_sec)
            self._periodic_next_due_sec = {
                index: due_sec + paused_sec
                for index, due_sec in self._periodic_next_due_sec.items()
            }
        self._pause_started_sec = None
        self._last_accounting_sec = now_sec
        self.TaskActivationRevision += 1
        self._SetStatus(
            self._RunningStateForTask(task),
            f"Resumed: {task.Name}",
        )
        return MissionCommandResult(True, True, self.LastMessage)

    def _Stop(self, now_sec: float) -> MissionCommandResult:
        was_active = self.IsRunning or self.State == "PAUSED"
        if self.IsRunning:
            self._AccountRunningTime(now_sec)
        self.ActiveTaskIndex = None
        self.ActiveTaskElapsedSec = 0.0
        self._last_accounting_sec = None
        self._periodic_next_due_sec = {}
        self._suspended_task_index = None
        self._suspended_task_elapsed_sec = 0.0
        self._pause_started_sec = None
        self.LoadedProfile = None
        self.FaultReason = ""
        state = "ABORTED" if was_active else "STOPPED"
        message = "Mission aborted by operator" if was_active else "Mission stopped"
        self._SetStatus(state, message)
        return MissionCommandResult(True, True, message)

    def Fault(self, reason: str, now_sec: Optional[float] = None):
        now = self._Now(now_sec)
        if self.IsRunning:
            self._AccountRunningTime(now)
        self.FaultReason = str(reason)
        self._last_accounting_sec = None
        self._pause_started_sec = None
        self._SetStatus("FAULTED", f"Mission fault: {self.FaultReason}")

    def BuildEffectiveControlState(
        self, control_state: Optional[Dict[str, Any]]
    ) -> Dict[str, Any]:
        """Overlay authoritative mission intent onto normal operator controls."""

        result = dict(control_state or {})
        task = self.ActiveTask
        if self.IsRunning and task is not None:
            result["DisplayMode"] = "SCAN"
            result["ScanEnabled"] = True
            result["TransmitEnabled"] = bool(
                result.get("TransmitAvailable", False)
            )
            result["SelectedWaveformId"] = str(task.WaveformId)
            result["SelectedPrfHz"] = float(task.PrfHz)
            result["SelectedPulsesPerCpi"] = int(task.PulsesPerCpi)
            result["SelectedMaximumRangeM"] = float(task.MaximumRangeM)
            result["TimingSelectionRevision"] = (
                MISSION_TIMING_REVISION_BASE + self.TaskTimingRevision
            )
            dwell_cadence = float(
                self.Config.get("RadarDwellIntervalSec", 0.100)
            )
            if isinstance(task, Scan360Task):
                # A 360 mission is continuous rotation, not a sector whose
                # endpoints happen to be close to 0/360.  PointingManager uses
                # this explicit pattern to avoid reversing at North.
                result["ScanStartDeg"] = 0.0
                result["ScanStopDeg"] = 359.999
                result["ScanStepDeg"] = max(
                    0.01, 6.0 * float(task.RotationRpm) * dwell_cadence
                )
                result["MissionScanRateDegSec"] = (
                    6.0 * float(task.RotationRpm)
                )
                result["MissionScanDirection"] = str(task.Direction)
                result["MissionScanPattern"] = (
                    "CONTINUOUS_CW"
                    if str(task.Direction).upper() == "CW"
                    else "CONTINUOUS_CCW"
                )
            else:
                start_deg = float(task.StartDeg)
                stop_deg = float(task.StopDeg)
                if str(task.InitialDirection).upper() == "CCW":
                    start_deg, stop_deg = stop_deg, start_deg
                result["ScanStartDeg"] = start_deg
                result["ScanStopDeg"] = stop_deg
                result["ScanStepDeg"] = max(
                    0.01, float(task.ScanRateDegSec) * dwell_cadence
                )
                result["MissionScanRateDegSec"] = float(task.ScanRateDegSec)
                result["MissionScanDirection"] = str(task.InitialDirection)
                result["MissionScanPattern"] = "SECTOR"
        elif self.State in ("LOADED", "PAUSED"):
            result["DisplayMode"] = "STOP"
            result["ScanEnabled"] = False
            result["TransmitEnabled"] = False
        elif self.State in TERMINAL_STATES:
            manual_control_command_id = int(
                result.get("ManualControlCommandId", 0)
            )
            if (
                manual_control_command_id
                == self._terminal_manual_control_command_id
            ):
                result["DisplayMode"] = "STOP"
                result["ScanEnabled"] = False
                result["TransmitEnabled"] = False
        return result

    def GetStatus(self) -> Dict[str, Any]:
        task = self.ActiveTask
        duration_sec = None if task is None else task.DurationSec
        remaining_sec = None
        if duration_sec is not None:
            remaining_sec = max(
                0.0, float(duration_sec) - self.ActiveTaskElapsedSec
            )
        profile = self.LoadedProfile
        next_periodic_index = None
        next_periodic_due_in_sec = None
        if self._periodic_next_due_sec:
            next_due_sec, next_periodic_index = min(
                (due_sec, index)
                for index, due_sec in self._periodic_next_due_sec.items()
            )
            if self._last_boundary_sec is not None:
                next_periodic_due_in_sec = max(
                    0.0, next_due_sec - self._last_boundary_sec
                )
        next_periodic_task = (
            None
            if profile is None or next_periodic_index is None
            else profile.PrimaryTasks[next_periodic_index]
        )
        return {
            "StatusRevision": int(self.StatusRevision),
            "State": str(self.State),
            "Message": str(self.LastMessage),
            "FaultReason": str(self.FaultReason),
            "MissionId": "" if profile is None else str(profile.MissionId),
            "MissionName": "" if profile is None else str(profile.Name),
            "MissionRevision": 0 if profile is None else int(profile.Revision),
            "CompletionPolicy": (
                "" if profile is None else str(profile.CompletionPolicy)
            ),
            "ActiveTaskIndex": self.ActiveTaskIndex,
            "ActiveTaskNumber": (
                None if self.ActiveTaskIndex is None
                else int(self.ActiveTaskIndex) + 1
            ),
            "TaskCount": (
                0 if profile is None else len(profile.PrimaryTasks)
            ),
            "ActiveTaskId": "" if task is None else str(task.TaskId),
            "ActiveTaskName": "" if task is None else str(task.Name),
            "ActiveTaskType": "" if task is None else str(task.TaskType),
            "ActiveWaveformId": (
                "" if task is None else str(task.WaveformId)
            ),
            "ActivePrfHz": (
                None if task is None else float(task.PrfHz)
            ),
            "ActivePulsesPerCpi": (
                None if task is None else int(task.PulsesPerCpi)
            ),
            "ActiveTaskElapsedSec": float(self.ActiveTaskElapsedSec),
            "ActiveTaskDurationSec": duration_sec,
            "ActiveTaskRemainingSec": remaining_sec,
            "ActiveTaskRepeatIntervalSec": (
                None if task is None else task.RepeatIntervalSec
            ),
            "PeriodicScheduling": bool(self._periodic_next_due_sec),
            "NextPeriodicTaskId": (
                "" if next_periodic_task is None
                else str(next_periodic_task.TaskId)
            ),
            "NextPeriodicTaskName": (
                "" if next_periodic_task is None
                else str(next_periodic_task.Name)
            ),
            "NextPeriodicDueInSec": next_periodic_due_in_sec,
            "MissionWallElapsedSec": float(self.MissionWallElapsedSec),
            "TaskActivationRevision": int(self.TaskActivationRevision),
            "TaskTimingRevision": int(self.TaskTimingRevision),
            "SimulationOnly": True,
        }
