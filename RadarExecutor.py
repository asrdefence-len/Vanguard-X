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
from RadarTiming import CalculateRadarTiming, RadarTimingSolution


@dataclass(frozen=True)
class ExecutionProfile:
    """
    Hardware-independent parameters required to execute one radar dwell.

    The profile is selected from the scheduled task and translated into a
    DwellPlan. Hardware-specific source implementations may later use the
    SDRProfile field to select Ettus/TRM configuration without changing the
    scheduler or signal-processing layers.
    """

    Name: str
    WaveformId: str
    SampleRate: float
    NumSamples: int
    NumPulses: int
    PriSec: float
    RxStartDelaySec: float
    RxAttenuationDb: float
    TxAttenuationDb: float
    Timing: RadarTimingSolution
    SDRProfile: str = "Default"


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
        waveform_library=None,
    ):
        self.Source = source
        self.PointingManager = pointing_manager
        self.Config = config
        self.WaveformLibrary = (
            waveform_library
            if waveform_library is not None
            else getattr(source, "TheWaveformLibrary", None)
        )
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

        profile = self._profile_for_task(task)
        dwell = self.BuildDwellPlan(task, pointing, profile=profile)

        raw = self.Source.ExecuteDwell(dwell)
        self._attach_execution_metadata(
            raw=raw,
            dwell=dwell,
            task=task,
            pointing=pointing,
            navigation=navigation,
            profile=profile,
            execution_time_sec=now,
        )

        if self.Debug:
            print(
                "RadarExecutor executed "
                f"{task.TaskType.value} task={task.TaskId} "
                f"dwell={dwell.DwellId} "
                f"profile={profile.Name} "
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
        profile: Optional[ExecutionProfile] = None,
    ) -> DwellPlan:
        """
        Build the current fixed-PRI DwellPlan for SEARCH or TRACK.

        This deliberately uses the existing uniform dwell helper so the new
        execution architecture remains compatible with RadarProcessor.
        """
        if profile is None:
            profile = self._profile_for_task(task)

        dwell_id = self._allocate_dwell_id()

        metadata = {
            "TaskId": int(task.TaskId),
            "TaskType": task.TaskType.value,
            "WaveformProfileId": str(task.WaveformProfileId),
            "ExecutionProfileName": str(profile.Name),
            "ExecutionWaveformId": str(profile.WaveformId),
            "ExecutionSampleRate": float(profile.SampleRate),
            "ExecutionNumSamples": int(profile.NumSamples),
            "ExecutionNumPulses": int(profile.NumPulses),
            "ExecutionPriSec": float(profile.PriSec),
            "ExecutionPrfHz": float(profile.Timing.SelectedPrfHz),
            "ExecutionRxStartDelaySec": float(profile.RxStartDelaySec),
            "ExecutionCpiDurationSec": float(profile.Timing.CpiDurationSec),
            "ExecutionMaximumRangeM": float(profile.Timing.MaximumRangeM),
            "ExecutionRxAttenuationDb": float(profile.RxAttenuationDb),
            "ExecutionTxAttenuationDb": float(profile.TxAttenuationDb),
            "ExecutionSDRProfile": str(profile.SDRProfile),
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
            "RadarTiming": profile.Timing.ToMetadata(),
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
            waveform_id=profile.WaveformId,
            sample_rate=float(profile.SampleRate),
            num_samples=int(profile.NumSamples),
            num_pulses=int(profile.NumPulses),
            pri_sec=float(profile.PriSec),
            rx_start_delay_sec=float(profile.RxStartDelaySec),
            azimuth_deg=float(pointing.BeamBearingTrueDeg),
            elevation_deg=float(pointing.BeamElevationTrueDeg),
            rx_attenuation_db=float(profile.RxAttenuationDb),
            tx_attenuation_db=float(profile.TxAttenuationDb),
            metadata=metadata,
        )

    def GetExecutionProfile(self, task: RadarTask) -> ExecutionProfile:
        """Return the validated timing/execution profile for a task."""

        return self._profile_for_task(task)

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

    def _profile_for_task(self, task: RadarTask) -> ExecutionProfile:
        if task.TaskType == RadarTaskType.SEARCH:
            return self._make_execution_profile(
                Name="Search",
                WaveformId=self.Config.get(
                    "SearchWaveformId",
                    "Frank10_20MHz",
                ),
                RxAttenuationDb=float(self.Config.get(
                    "TRMRxAttenuationDb",
                    0.0,
                )),
                TxAttenuationDb=float(self.Config.get(
                    "TRMTxAttenuationDb",
                    31.5,
                )),
                SDRProfile=str(self.Config.get(
                    "SearchSDRProfile",
                    "Default",
                )),
            )

        if task.TaskType == RadarTaskType.TRACK:
            return self._make_execution_profile(
                Name="Track",
                WaveformId=self.Config.get(
                    "TrackWaveformId",
                    self.Config.get(
                        "SearchWaveformId",
                        "Barker13_20MHz",
                    ),
                ),
                RxAttenuationDb=float(self.Config.get(
                    "TrackRxAttenuationDb",
                    self.Config.get("TRMRxAttenuationDb", 0.0),
                )),
                TxAttenuationDb=float(self.Config.get(
                    "TrackTxAttenuationDb",
                    self.Config.get("TRMTxAttenuationDb", 31.5),
                )),
                SDRProfile=str(self.Config.get(
                    "TrackSDRProfile",
                    self.Config.get("SearchSDRProfile", "Default"),
                )),
            )

        raise ValueError(
            f"No execution profile for task type {task.TaskType.value}"
        )

    def _make_execution_profile(
        self,
        Name: str,
        WaveformId: str,
        RxAttenuationDb: float,
        TxAttenuationDb: float,
        SDRProfile: str,
    ) -> ExecutionProfile:
        """Build a profile entirely from one validated timing solution."""

        if self.WaveformLibrary is None:
            raise RuntimeError(
                "RadarExecutor requires WaveformLibrary to derive dwell timing"
            )

        Prefix = str(Name)
        IsSearch = Prefix.upper() == "SEARCH"
        WaveformMetadata = self.WaveformLibrary.GetMetadata(WaveformId)

        Timing = CalculateRadarTiming(
            WaveformMetadata,
            SelectedPrfHz=float(self.Config.get(
                f"{Prefix}PrfHz",
                self.Config.get("SelectedPrfHz", 2000.0),
            )),
            PulsesPerCpi=int(self.Config.get(
                f"{Prefix}PulsesPerCpi",
                self.Config.get("SelectedPulsesPerCpi", 32),
            )),
            MaximumRangeM=float(self.Config.get(
                f"{Prefix}MaximumRangeM",
                self.Config.get(
                    "InstrumentedMaxRangeM",
                    self.Config.get("MaxRangeM", 15000.0),
                ),
            )),
            ReceiverRecoveryTimeSec=float(self.Config.get(
                "ReceiverRecoveryTimeSec",
                1.0e-6,
            )),
            RxEndMarginSec=float(self.Config.get(
                "RxEndMarginSec",
                2.0e-6,
            )),
            NextTxGuardTimeSec=float(self.Config.get(
                "NextTxGuardTimeSec",
                2.0e-6,
            )),
            OperatorMinPrfHz=float(self.Config.get(
                "MinPrfHz",
                1000.0,
            )),
            OperatorMaxPrfHz=float(self.Config.get(
                "MaxPrfHz",
                4000.0,
            )),
            RfFrequencyHz=float(self.Config["RfFrequency"]),
            AntennaScanRateDegPerSec=float(
                self.Config.get("PTZScanSlewRateDegPerSec", 0.0)
                if IsSearch
                else self.Config.get("TrackAntennaRateDegPerSec", 0.0)
            ),
        )

        return ExecutionProfile(
            Name=Prefix,
            WaveformId=str(WaveformId),
            SampleRate=float(Timing.SampleRateHz),
            NumSamples=int(Timing.NumRxSamples),
            NumPulses=int(Timing.PulsesPerCpi),
            PriSec=float(Timing.PriSec),
            RxStartDelaySec=float(Timing.RxStartDelaySec),
            RxAttenuationDb=float(RxAttenuationDb),
            TxAttenuationDb=float(TxAttenuationDb),
            Timing=Timing,
            SDRProfile=str(SDRProfile),
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
        profile: ExecutionProfile,
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

            "ExecutionProfileName": str(profile.Name),
            "ExecutionWaveformId": str(profile.WaveformId),
            "ExecutionSampleRate": float(profile.SampleRate),
            "ExecutionNumSamples": int(profile.NumSamples),
            "ExecutionNumPulses": int(profile.NumPulses),
            "ExecutionPriSec": float(profile.PriSec),
            "ExecutionPrfHz": float(profile.Timing.SelectedPrfHz),
            "ExecutionRxStartDelaySec": float(profile.RxStartDelaySec),
            "ExecutionCpiDurationSec": float(profile.Timing.CpiDurationSec),
            "ExecutionMaximumRangeM": float(profile.Timing.MaximumRangeM),
            "ExecutionRxAttenuationDb": float(profile.RxAttenuationDb),
            "ExecutionTxAttenuationDb": float(profile.TxAttenuationDb),
            "ExecutionSDRProfile": str(profile.SDRProfile),
            "RadarTiming": profile.Timing.ToMetadata(),

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
