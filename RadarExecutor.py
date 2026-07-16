"""
===============================================================================
Vanguard X Radar Executor
RadarExecutor.py
===============================================================================

Purpose
-------
Coordinates one scheduled radar task with the pointing layer and radar source.

The first implementation is simulator-first and hardware-independent. It:

    1. Activates the selected RadarTask in PointingManager.
    2. Waits until the pointing state is valid and executable.
    3. Builds a fixed-waveform, fixed-PRI DwellPlan.
    4. Executes the dwell through the configured RadarSource.
    5. Attaches task and measured-pointing metadata to RawDwellData.
    6. Returns both RawDwellData and DwellPlan.

The executor does not:
    - choose which task runs next
    - process IQ
    - detect targets
    - update tracks
    - directly control Ettus or TRM hardware

Those responsibilities remain with RadarScheduler, RadarProcessor, detector,
tracker and later hardware-specific source implementations.
===============================================================================
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional
import time

from RadarPlans import DwellPlan, make_uniform_dwell_plan
from RadarTasks import RadarTask, RadarTaskType, SearchTask, TrackTask
from PointingManager import PointingManager, PointingState
from NavigationState import PlatformAttitude


@dataclass
class RadarExecutionResult:
    """
    Result of one executor attempt.

    Executed:
        True when a dwell was actually run.

    WaitingForPointing:
        True when the task is active but the antenna is not ready yet.

    Raw:
        RawDwellData returned by the radar source when Executed=True.

    Dwell:
        DwellPlan used for execution when Executed=True.
    """

    Executed: bool
    WaitingForPointing: bool
    TaskId: int
    TaskType: str

    Pointing: PointingState
    Raw: object = None
    Dwell: Optional[DwellPlan] = None
    Reason: str = ""


class RadarExecutor:
    """
    Simulator-first task executor for Vanguard X.
    """

    def __init__(
        self,
        source,
        pointing_manager: PointingManager,
        config: Dict,
        debug: bool = False,
    ):
        self.Source = source
        self.PointingManager = pointing_manager
        self.Config = config
        self.Debug = bool(debug)

        self._active_task_id: Optional[int] = None
        self._next_dwell_id = int(config.get("InitialDwellId", 1))

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def ExecuteTaskStep(
        self,
        task: RadarTask,
        navigation: PlatformAttitude,
        current_time_sec: Optional[float] = None,
    ) -> RadarExecutionResult:
        """
        Advance one task by one executor step.

        Search:
            Continuous scanning remains active. A search dwell can execute as
            soon as the pointing state is valid.

        Track:
            The executor waits until PointingManager reports Ready=True before
            running the dedicated track dwell.
        """
        now = time.time() if current_time_sec is None else float(current_time_sec)

        if self._active_task_id != int(task.TaskId):
            self.PointingManager.ActivateTask(task, navigation)
            self._active_task_id = int(task.TaskId)

        pointing = self.PointingManager.Update(
            navigation=navigation,
            current_time_sec=now,
        )

        if not pointing.Valid:
            return RadarExecutionResult(
                Executed=False,
                WaitingForPointing=True,
                TaskId=int(task.TaskId),
                TaskType=task.TaskType.value,
                Pointing=pointing,
                Reason="Pointing state invalid",
            )

        if task.TaskType == RadarTaskType.TRACK and not pointing.Ready:
            return RadarExecutionResult(
                Executed=False,
                WaitingForPointing=True,
                TaskId=int(task.TaskId),
                TaskType=task.TaskType.value,
                Pointing=pointing,
                Reason="Track pointing not ready",
            )

        dwell = self.BuildDwellPlan(task, pointing)

        raw = self.Source.ExecuteDwell(dwell)
        self._attach_execution_metadata(
            raw=raw,
            dwell=dwell,
            task=task,
            pointing=pointing,
            navigation=navigation,
            execution_time_sec=now,
        )

        if self.Debug:
            print(
                "RadarExecutor executed "
                f"{task.TaskType.value} task={task.TaskId} "
                f"dwell={dwell.DwellId} "
                f"bearing_true={pointing.BeamBearingTrueDeg:.2f} deg"
            )

        return RadarExecutionResult(
            Executed=True,
            WaitingForPointing=False,
            TaskId=int(task.TaskId),
            TaskType=task.TaskType.value,
            Pointing=pointing,
            Raw=raw,
            Dwell=dwell,
            Reason="",
        )

    def BuildDwellPlan(
        self,
        task: RadarTask,
        pointing: PointingState,
    ) -> DwellPlan:
        """
        Build the current fixed-PRI DwellPlan for SEARCH or TRACK.

        This deliberately uses the existing uniform dwell helper so the new
        execution architecture remains compatible with RadarProcessor.
        """
        profile = self._profile_for_task(task)

        dwell_id = self._allocate_dwell_id()

        metadata = {
            "TaskId": int(task.TaskId),
            "TaskType": task.TaskType.value,
            "WaveformProfileId": str(task.WaveformProfileId),
            "PointingMode": pointing.Mode.value,
            "AntennaAzimuthRelativeDeg": float(
                pointing.AntennaAzimuthRelativeDeg
            ),
            "PlatformHeadingTrueDeg": float(
                pointing.PlatformHeadingTrueDeg
            ),
            "BeamBearingTrueDeg": float(
                pointing.BeamBearingTrueDeg
            ),
            "PointingReady": bool(pointing.Ready),
            "PointingReachable": bool(pointing.Reachable),
        }

        if isinstance(task, SearchTask):
            metadata.update({
                "SearchCycle": int(task.Sector.ScanCycle),
                "SearchActiveEndpoint": str(task.Sector.ActiveEndpoint),
                "SearchSectorFrame": task.Sector.Frame.value,
                "SearchSectorStartDeg": float(task.Sector.StartDeg),
                "SearchSectorStopDeg": float(task.Sector.StopDeg),
            })

        if isinstance(task, TrackTask):
            metadata.update({
                "TrackId": int(task.TrackId),
                "PredictedRangeM": (
                    None
                    if task.PredictedRangeM is None
                    else float(task.PredictedRangeM)
                ),
                "PredictedTrueBearingDeg": (
                    None
                    if task.PredictedTrueBearingDeg is None
                    else float(task.PredictedTrueBearingDeg)
                ),
                "RevisitIntervalSec": float(task.RevisitIntervalSec),
            })

        return make_uniform_dwell_plan(
            dwell_id=dwell_id,
            task_id=int(task.TaskId),
            task_type=task.TaskType.value,
            waveform_id=profile["WaveformId"],
            sample_rate=float(profile["SampleRate"]),
            num_samples=int(profile["NumSamples"]),
            num_pulses=int(profile["NumPulses"]),
            pri_sec=float(profile["PriSec"]),
            azimuth_deg=float(pointing.BeamBearingTrueDeg),
            elevation_deg=float(pointing.BeamElevationTrueDeg),
            rx_attenuation_db=float(profile["RxAttenuationDb"]),
            tx_attenuation_db=float(profile["TxAttenuationDb"]),
            metadata=metadata,
        )

    def ReleaseTask(self, task: Optional[RadarTask] = None) -> None:
        """
        Release executor ownership after the scheduler completes/fails a task.

        The PointingManager remains responsible for explicit search resume or
        stop commands.
        """
        if task is None or self._active_task_id == int(task.TaskId):
            self._active_task_id = None

    # ------------------------------------------------------------------
    # Profiles / configuration
    # ------------------------------------------------------------------

    def _profile_for_task(self, task: RadarTask) -> Dict:
        if task.TaskType == RadarTaskType.SEARCH:
            return {
                "WaveformId": self.Config.get(
                    "SearchWaveformId",
                    "Frank10",
                ),
                "SampleRate": self.Config["SampleRate"],
                "NumSamples": self.Config["NumSamples"],
                "NumPulses": self.Config["NumPulses"],
                "PriSec": self.Config["PRI"],
                "RxAttenuationDb": self.Config.get(
                    "TRMRxAttenuationDb",
                    0.0,
                ),
                "TxAttenuationDb": self.Config.get(
                    "TRMTxAttenuationDb",
                    31.5,
                ),
            }

        if task.TaskType == RadarTaskType.TRACK:
            return {
                "WaveformId": self.Config.get(
                    "TrackWaveformId",
                    self.Config.get("SearchWaveformId", "Barker13"),
                ),
                "SampleRate": self.Config.get(
                    "TrackSampleRate",
                    self.Config["SampleRate"],
                ),
                "NumSamples": self.Config.get(
                    "TrackNumSamples",
                    self.Config["NumSamples"],
                ),
                "NumPulses": self.Config.get(
                    "TrackNumPulses",
                    self.Config["NumPulses"],
                ),
                "PriSec": self.Config.get(
                    "TrackPRI",
                    self.Config["PRI"],
                ),
                "RxAttenuationDb": self.Config.get(
                    "TrackRxAttenuationDb",
                    self.Config.get("TRMRxAttenuationDb", 0.0),
                ),
                "TxAttenuationDb": self.Config.get(
                    "TrackTxAttenuationDb",
                    self.Config.get("TRMTxAttenuationDb", 31.5),
                ),
            }

        raise ValueError(
            f"No execution profile for task type {task.TaskType.value}"
        )

    def _allocate_dwell_id(self) -> int:
        dwell_id = self._next_dwell_id
        self._next_dwell_id += 1
        return dwell_id

    # ------------------------------------------------------------------
    # Metadata
    # ------------------------------------------------------------------

    @staticmethod
    def _attach_execution_metadata(
        raw,
        dwell: DwellPlan,
        task: RadarTask,
        pointing: PointingState,
        navigation: PlatformAttitude,
        execution_time_sec: float,
    ) -> None:
        diagnostics = getattr(raw, "Diagnostics", None)

        if diagnostics is None:
            diagnostics = {}
            raw.Diagnostics = diagnostics

        diagnostics.update({
            "TaskId": int(task.TaskId),
            "TaskType": task.TaskType.value,
            "DwellId": int(dwell.DwellId),
            "ExecutionTimeSec": float(execution_time_sec),

            "BoresightDeg": float(pointing.BeamBearingTrueDeg),
            "BeamBearingTrueDeg": float(pointing.BeamBearingTrueDeg),
            "BeamElevationTrueDeg": float(pointing.BeamElevationTrueDeg),

            "AntennaAzimuthRelativeDeg": float(
                pointing.AntennaAzimuthRelativeDeg
            ),
            "AntennaElevationRelativeDeg": float(
                pointing.AntennaElevationRelativeDeg
            ),

            "PlatformHeadingTrueDeg": float(
                navigation.HeadingTrueDeg
            ),
            "PlatformPitchDeg": float(navigation.PitchDeg),
            "PlatformRollDeg": float(navigation.RollDeg),

            "PointingMode": pointing.Mode.value,
            "PointingReady": bool(pointing.Ready),
            "PointingReachable": bool(pointing.Reachable),
            "PointingSource": str(pointing.Source),
        })

        if isinstance(task, SearchTask):
            diagnostics.update({
                "ScanCycle": int(task.Sector.ScanCycle),
                "SearchActiveEndpoint": str(task.Sector.ActiveEndpoint),
                "SearchSectorFrame": task.Sector.Frame.value,
            })

        if isinstance(task, TrackTask):
            diagnostics.update({
                "TrackId": int(task.TrackId),
                "PredictedRangeM": (
                    None
                    if task.PredictedRangeM is None
                    else float(task.PredictedRangeM)
                ),
                "PredictedTrueBearingDeg": (
                    None
                    if task.PredictedTrueBearingDeg is None
                    else float(task.PredictedTrueBearingDeg)
                ),
            })
