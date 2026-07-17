"""
===============================================================================
Vanguard X Pointing Manager
PointingManager.py
===============================================================================

Purpose
-------
Provides the software-only pointing layer between radar tasks and the PTZ.

The Pointing Manager:
    - converts TRUE bearings to PLATFORM-relative antenna angles
    - converts PLATFORM-relative encoder angles to TRUE bearings
    - manages continuous search scanning
    - manages goto-and-hold track pointing
    - reports pointing readiness and reachability
    - preserves search state while a track task temporarily interrupts search

It does not:
    - choose which task runs next
    - process IQ
    - control the Ettus or TRM
    - perform track filtering

The PTZ object must provide the public interface already used by Vanguard:
    CommandSlew(rate_deg_per_sec, tilt_rate_deg_per_sec=0.0)
    SetPanPositionNative(target_deg)
    Stop()
    Update() -> state with AzimuthDeg, ElevationDeg, Valid, PanRateDegPerSec
===============================================================================
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional
import time

from RadarTasks import (
    AngleFrame,
    PointingMode,
    RadarTask,
    RadarTaskType,
    SearchTask,
    TrackTask,
)
from NavigationState import PlatformAttitude


def wrap360(angle_deg: float) -> float:
    return float(angle_deg) % 360.0


def signed_angle_delta_deg(target_deg: float, current_deg: float) -> float:
    delta = wrap360(target_deg) - wrap360(current_deg)
    if delta > 180.0:
        delta -= 360.0
    return delta


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(float(minimum), min(float(maximum), float(value)))


@dataclass
class PointingState:
    TimestampSec: float
    Mode: PointingMode

    AntennaAzimuthRelativeDeg: float
    AntennaElevationRelativeDeg: float
    PlatformHeadingTrueDeg: float

    BeamBearingTrueDeg: float
    BeamElevationTrueDeg: float

    CommandedAzimuthRelativeDeg: Optional[float]
    CommandedBearingTrueDeg: Optional[float]

    PanRateDegPerSec: float
    Ready: bool
    Reachable: bool
    Valid: bool
    Source: str

    ActiveTaskId: Optional[int] = None
    ActiveTrackId: Optional[int] = None
    SearchCycle: Optional[int] = None
    SearchActiveEndpoint: Optional[str] = None


class PointingManager:
    """
    Software-only pointing manager for search and track tasks.
    """

    def __init__(
        self,
        ptz,
        left_limit_deg: float = 10.0,
        right_limit_deg: float = 300.0,
        endpoint_margin_deg: float = 1.0,
        position_tolerance_deg: float = 1.0,
        settle_rate_threshold_deg_per_sec: float = 0.25,
        default_scan_rate_deg_per_sec: float = 14.0,
        debug: bool = False,
    ):
        self.Ptz = ptz

        self.LeftLimitDeg = float(left_limit_deg)
        self.RightLimitDeg = float(right_limit_deg)
        self.EndpointMarginDeg = abs(float(endpoint_margin_deg))
        self.PositionToleranceDeg = abs(float(position_tolerance_deg))
        self.SettleRateThresholdDegPerSec = abs(
            float(settle_rate_threshold_deg_per_sec)
        )
        self.DefaultScanRateDegPerSec = abs(
            float(default_scan_rate_deg_per_sec)
        )
        self.Debug = bool(debug)

        if self.RightLimitDeg <= self.LeftLimitDeg:
            raise ValueError("right_limit_deg must be greater than left_limit_deg")

        self.ActiveTask: Optional[RadarTask] = None
        self.ActiveMode = PointingMode.HOLD_CURRENT

        self.LastCommandedRelativeDeg: Optional[float] = None
        self.LastCommandedTrueDeg: Optional[float] = None

        self.SearchInterrupted = False
        self.SearchTask: Optional[SearchTask] = None
        self.LastSearchRelativeDeg: Optional[float] = None

        self._goto_ready_since_sec: Optional[float] = None

    # ------------------------------------------------------------------
    # Coordinate conversion
    # ------------------------------------------------------------------

    @staticmethod
    def RelativeToTrueBearing(
        antenna_relative_deg: float,
        platform_heading_true_deg: float,
    ) -> float:
        return wrap360(
            float(platform_heading_true_deg) + float(antenna_relative_deg)
        )

    @staticmethod
    def TrueToRelativeAzimuth(
        true_bearing_deg: float,
        platform_heading_true_deg: float,
    ) -> float:
        return wrap360(
            float(true_bearing_deg) - float(platform_heading_true_deg)
        )

    def IsRelativeAzimuthReachable(self, relative_azimuth_deg: float) -> bool:
        relative = wrap360(relative_azimuth_deg)
        return self.LeftLimitDeg <= relative <= self.RightLimitDeg

    def ClampRelativeAzimuth(self, relative_azimuth_deg: float) -> float:
        return clamp(
            wrap360(relative_azimuth_deg),
            self.LeftLimitDeg,
            self.RightLimitDeg,
        )

    # ------------------------------------------------------------------
    # Task lifecycle
    # ------------------------------------------------------------------

    def ActivateTask(
        self,
        task: RadarTask,
        navigation: PlatformAttitude,
    ) -> None:
        """
        Activate a task and issue its initial PTZ command.

        Search:
            starts/resumes continuous slew toward the active sector endpoint.

        Track:
            converts the target true bearing to a platform-relative PTZ angle
            and commands goto-and-hold.
        """
        if task.TaskType == RadarTaskType.SEARCH:
            self._activate_search(task, navigation)
        elif task.TaskType == RadarTaskType.TRACK:
            self._activate_track(task, navigation)
        else:
            self.ActiveTask = task
            self.ActiveMode = task.Pointing.Mode
            if task.Pointing.Mode == PointingMode.HOLD_CURRENT:
                self.Ptz.Stop()

    def _activate_search(
        self,
        task: SearchTask,
        navigation: PlatformAttitude,
    ) -> None:
        self.ActiveTask = task
        self.SearchTask = task
        self.ActiveMode = PointingMode.CONTINUOUS_SCAN
        self.SearchInterrupted = False
        self._goto_ready_since_sec = None

        self._command_search_slew(task, navigation)

        if self.Debug:
            print(
                "PointingManager SEARCH active: "
                f"endpoint={task.Sector.ActiveEndpoint} "
                f"{task.Sector.ActiveEndpointDeg:.2f} deg "
                f"frame={task.Sector.Frame.value}"
            )

    def _activate_track(
        self,
        task: TrackTask,
        navigation: PlatformAttitude,
    ) -> None:
        if self.SearchTask is not None:
            try:
                state = self.Ptz.Update()
                self.LastSearchRelativeDeg = float(state.AzimuthDeg)
                self.SearchTask.Sector.InterruptedAzimuthDeg = (
                    self.LastSearchRelativeDeg
                )
            except Exception:
                pass
            self.SearchInterrupted = True

        self.ActiveTask = task
        self.ActiveMode = PointingMode.GOTO_AND_HOLD
        self._goto_ready_since_sec = None

        true_target = float(task.Pointing.TargetAzimuthDeg)
        relative_target = self.TrueToRelativeAzimuth(
            true_target,
            navigation.HeadingTrueDeg,
        )

        reachable = self.IsRelativeAzimuthReachable(relative_target)
        command_target = (
            relative_target
            if reachable
            else self.ClampRelativeAzimuth(relative_target)
        )

        self.LastCommandedTrueDeg = true_target
        self.LastCommandedRelativeDeg = command_target

        self.Ptz.SetPanPositionNative(command_target)

        if self.Debug:
            print(
                "PointingManager TRACK active: "
                f"track={task.TrackId} true={true_target:.2f} deg "
                f"relative={relative_target:.2f} deg "
                f"command={command_target:.2f} deg "
                f"reachable={reachable}"
            )

    def PauseSearchForTrack(self) -> None:
        if self.SearchTask is not None:
            self.SearchInterrupted = True
        self.Ptz.Stop()

    def ResumeSearch(self, navigation: PlatformAttitude) -> None:
        if self.SearchTask is None:
            return

        self.ActiveTask = self.SearchTask
        self.ActiveMode = PointingMode.CONTINUOUS_SCAN
        self.SearchInterrupted = False
        self._goto_ready_since_sec = None
        self._command_search_slew(self.SearchTask, navigation)

        if self.Debug:
            print(
                "PointingManager SEARCH resumed: "
                f"toward {self.SearchTask.Sector.ActiveEndpoint}"
            )

    def Nudge(self, delta_deg: float) -> float:
        """Move the antenna by a relative operator-requested increment.

        Manual positioning is owned here so callers never command the PTZ
        directly. The target is clamped to the configured mechanical limits.
        Any active search or track task is cleared; the scheduler may reactivate
        search when the operator returns to SCAN mode.

        Returns the commanded platform-relative azimuth in degrees.
        """
        delta = float(delta_deg)

        try:
            state = self.Ptz.Update()
            current_relative = float(state.AzimuthDeg)
        except Exception:
            if self.LastCommandedRelativeDeg is None:
                raise
            current_relative = float(self.LastCommandedRelativeDeg)

        target_relative = clamp(
            current_relative + delta,
            self.LeftLimitDeg,
            self.RightLimitDeg,
        )

        self.Ptz.Stop()
        self.Ptz.SetPanPositionNative(target_relative)

        self.ActiveTask = None
        self.ActiveMode = PointingMode.HOLD_CURRENT
        self.SearchInterrupted = self.SearchTask is not None
        self.LastCommandedRelativeDeg = target_relative
        self.LastCommandedTrueDeg = None
        self._goto_ready_since_sec = None

        if self.Debug:
            print(
                "PointingManager NUDGE: "
                f"current={current_relative:.2f} deg "
                f"delta={delta:+.2f} deg "
                f"target={target_relative:.2f} deg"
            )

        return target_relative

    def Stop(self) -> None:
        self.Ptz.Stop()
        self.ActiveTask = None
        self.ActiveMode = PointingMode.HOLD_CURRENT
        self._goto_ready_since_sec = None

    # ------------------------------------------------------------------
    # Update
    # ------------------------------------------------------------------

    def Update(
        self,
        navigation: PlatformAttitude,
        current_time_sec: Optional[float] = None,
    ) -> PointingState:
        now = time.time() if current_time_sec is None else float(current_time_sec)

        ptz_state = self.Ptz.Update()

        relative_az = float(ptz_state.AzimuthDeg)
        relative_el = float(getattr(ptz_state, "ElevationDeg", 0.0))
        rate = float(getattr(ptz_state, "PanRateDegPerSec", 0.0))
        ptz_valid = bool(getattr(ptz_state, "Valid", True))
        source = str(getattr(ptz_state, "Source", "PTZ"))

        beam_true = self.RelativeToTrueBearing(
            relative_az,
            navigation.HeadingTrueDeg,
        )

        reachable = True
        ready = False

        if (
            self.ActiveTask is not None
            and self.ActiveTask.TaskType == RadarTaskType.SEARCH
        ):
            search_task = self.ActiveTask
            self._update_search_endpoint(search_task, navigation, beam_true)
            ready = ptz_valid
            reachable = True

        elif (
            self.ActiveTask is not None
            and self.ActiveTask.TaskType == RadarTaskType.TRACK
        ):
            track_task = self.ActiveTask

            # Recalculate the relative target while the platform heading changes.
            true_target = float(track_task.Pointing.TargetAzimuthDeg)
            relative_target = self.TrueToRelativeAzimuth(
                true_target,
                navigation.HeadingTrueDeg,
            )

            reachable = self.IsRelativeAzimuthReachable(relative_target)
            command_target = (
                relative_target
                if reachable
                else self.ClampRelativeAzimuth(relative_target)
            )

            if (
                self.LastCommandedRelativeDeg is None
                or abs(
                    signed_angle_delta_deg(
                        command_target,
                        self.LastCommandedRelativeDeg,
                    )
                ) > 0.05
            ):
                self.LastCommandedRelativeDeg = command_target
                self.LastCommandedTrueDeg = true_target
                self.Ptz.SetPanPositionNative(command_target)

            error_deg = abs(
                signed_angle_delta_deg(command_target, relative_az)
            )
            on_angle = error_deg <= float(
                track_task.Pointing.PositionToleranceDeg
                or self.PositionToleranceDeg
            )
            low_rate = abs(rate) <= self.SettleRateThresholdDegPerSec

            if on_angle and low_rate and reachable:
                if self._goto_ready_since_sec is None:
                    self._goto_ready_since_sec = now

                settle_time = max(
                    0.0,
                    float(track_task.Pointing.SettleTimeSec),
                )
                ready = (now - self._goto_ready_since_sec) >= settle_time
            else:
                self._goto_ready_since_sec = None
                ready = False

        else:
            ready = ptz_valid and abs(rate) <= self.SettleRateThresholdDegPerSec

        active_task_id = (
            None if self.ActiveTask is None else int(self.ActiveTask.TaskId)
        )

        active_track_id = None
        search_cycle = None
        search_endpoint = None

        if isinstance(self.ActiveTask, TrackTask):
            active_track_id = int(self.ActiveTask.TrackId)

        if self.SearchTask is not None:
            search_cycle = int(self.SearchTask.Sector.ScanCycle)
            search_endpoint = str(self.SearchTask.Sector.ActiveEndpoint)

        return PointingState(
            TimestampSec=now,
            Mode=self.ActiveMode,
            AntennaAzimuthRelativeDeg=relative_az,
            AntennaElevationRelativeDeg=relative_el,
            PlatformHeadingTrueDeg=float(navigation.HeadingTrueDeg),
            BeamBearingTrueDeg=beam_true,
            BeamElevationTrueDeg=relative_el + float(navigation.PitchDeg),
            CommandedAzimuthRelativeDeg=self.LastCommandedRelativeDeg,
            CommandedBearingTrueDeg=self.LastCommandedTrueDeg,
            PanRateDegPerSec=rate,
            Ready=bool(ready),
            Reachable=bool(reachable),
            Valid=bool(ptz_valid and navigation.Valid),
            Source=source,
            ActiveTaskId=active_task_id,
            ActiveTrackId=active_track_id,
            SearchCycle=search_cycle,
            SearchActiveEndpoint=search_endpoint,
        )

    # ------------------------------------------------------------------
    # Search control
    # ------------------------------------------------------------------

    def _command_search_slew(
        self,
        task: SearchTask,
        navigation: PlatformAttitude,
    ) -> None:
        endpoint_relative = self._search_endpoint_relative(task, navigation)

        try:
            state = self.Ptz.Update()
            current_relative = float(state.AzimuthDeg)
        except Exception:
            current_relative = endpoint_relative

        delta = signed_angle_delta_deg(
            endpoint_relative,
            current_relative,
        )

        if not self.PtzWrapMode:
            delta = endpoint_relative - current_relative

        direction = 1.0 if delta >= 0.0 else -1.0
        rate = direction * float(task.Sector.ScanRateDegPerSec)

        self.LastCommandedRelativeDeg = endpoint_relative
        self.LastCommandedTrueDeg = self._search_endpoint_true(
            task,
            navigation,
        )

        self.Ptz.CommandSlew(rate, 0.0)

    @property
    def PtzWrapMode(self) -> bool:
        return bool(getattr(self.Ptz, "WrapMode", False))

    def _search_endpoint_relative(
        self,
        task: SearchTask,
        navigation: PlatformAttitude,
    ) -> float:
        endpoint = float(task.Sector.ActiveEndpointDeg)

        if task.Sector.Frame == AngleFrame.TRUE:
            relative = self.TrueToRelativeAzimuth(
                endpoint,
                navigation.HeadingTrueDeg,
            )
        else:
            relative = wrap360(endpoint)

        return self.ClampRelativeAzimuth(relative)

    def _search_endpoint_true(
        self,
        task: SearchTask,
        navigation: PlatformAttitude,
    ) -> float:
        endpoint = float(task.Sector.ActiveEndpointDeg)

        if task.Sector.Frame == AngleFrame.TRUE:
            return wrap360(endpoint)

        return self.RelativeToTrueBearing(
            endpoint,
            navigation.HeadingTrueDeg,
        )

    def _update_search_endpoint(
        self,
        task: SearchTask,
        navigation: PlatformAttitude,
        beam_true_deg: float,
    ) -> None:
        """Reverse search only when the measured beam reaches or crosses the
        active endpoint.

        The crossing decision is based on measured motion, not
        SearchSector.Direction.  SearchSector.Direction describes logical
        movement between START and STOP; it is not necessarily the physical
        sign of antenna motion when StartDeg is greater than StopDeg.
        """
        sector = task.Sector

        try:
            ptz_state = self.Ptz.GetState()
            relative_az = float(ptz_state.AzimuthDeg)
        except Exception:
            ptz_state = self.Ptz.Update()
            relative_az = float(ptz_state.AzimuthDeg)

        measured = (
            wrap360(beam_true_deg)
            if sector.Frame == AngleFrame.TRUE
            else (wrap360(relative_az) if self.PtzWrapMode else relative_az)
        )
        previous = sector.LastMeasuredAzimuthDeg
        sector.LastMeasuredAzimuthDeg = measured

        target = (
            wrap360(sector.ActiveEndpointDeg)
            if sector.Frame == AngleFrame.TRUE or self.PtzWrapMode
            else float(sector.ActiveEndpointDeg)
        )

        if sector.Frame == AngleFrame.TRUE or self.PtzWrapMode:
            error = signed_angle_delta_deg(target, measured)
        else:
            error = target - measured

        reached = abs(error) <= self.EndpointMarginDeg

        if not reached and previous is not None:
            if sector.Frame == AngleFrame.TRUE or self.PtzWrapMode:
                motion = signed_angle_delta_deg(measured, previous)
                previous_error = signed_angle_delta_deg(target, previous)
            else:
                motion = measured - previous
                previous_error = target - previous

            motion_epsilon = 0.01

            # The endpoint was crossed only when the measured antenna was
            # moving toward it and the target error changed sign.
            if motion > motion_epsilon:
                reached = previous_error > 0.0 and error <= 0.0
            elif motion < -motion_epsilon:
                reached = previous_error < 0.0 and error >= 0.0

        if reached:
            sector.Reverse()

            # Reset the crossing history for the new endpoint. Otherwise the
            # final sample from the previous leg can be interpreted as motion
            # across the newly selected endpoint.
            sector.LastMeasuredAzimuthDeg = measured
            self._command_search_slew(task, navigation)

            if self.Debug:
                print(
                    "PointingManager SEARCH reverse: "
                    f"target={sector.ActiveEndpoint} "
                    f"cycle={sector.ScanCycle}"
                )

        elif sector.Frame == AngleFrame.TRUE:
            # A true-fixed endpoint moves in platform coordinates as heading
            # changes. Refresh the commanded endpoint if it moved materially.
            new_relative = self._search_endpoint_relative(task, navigation)

            if (
                self.LastCommandedRelativeDeg is None
                or abs(
                    signed_angle_delta_deg(
                        new_relative,
                        self.LastCommandedRelativeDeg,
                    )
                ) > 0.25
            ):
                self._command_search_slew(task, navigation)
