"""
===============================================================================
Vanguard X Pointing Manager
PointingManager.py
===============================================================================

Purpose
-------
Provides the software-only pointing layer between radar tasks and the X6-60.

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

The X6-60 object must provide the public interface already used by Vanguard:
    CommandSlew(rate_deg_per_sec, tilt_rate_deg_per_sec=0.0)
    SetPanPositionNative(target_deg)
    Stop()
    Update() -> state with AzimuthDeg, ElevationDeg, Valid, PanRateDegPerSec
===============================================================================
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional
import math
import time

from RadarTasks import (
    AngleFrame,
    PointingMode,
    RadarTask,
    RadarTaskType,
    SearchPattern,
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
    elif delta < -180.0:
        delta += 360.0
    return delta


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
        x660,
        endpoint_margin_deg: float = 1.0,
        position_tolerance_deg: float = 1.0,
        settle_rate_threshold_deg_per_sec: float = 0.25,
        default_scan_rate_deg_per_sec: float = 14.0,
        scan_braking_enabled: bool = False,
        scan_deceleration_deg_per_sec2: float = 60.0,
        scan_command_latency_sec: float = 0.05,
        debug: bool = False,
    ):
        self.Positioner = x660

        self.EndpointMarginDeg = abs(float(endpoint_margin_deg))
        self.PositionToleranceDeg = abs(float(position_tolerance_deg))
        self.SettleRateThresholdDegPerSec = abs(
            float(settle_rate_threshold_deg_per_sec)
        )
        self.DefaultScanRateDegPerSec = abs(
            float(default_scan_rate_deg_per_sec)
        )
        self.ScanBrakingEnabled = bool(scan_braking_enabled)
        self.ScanDecelerationDegPerSec2 = float(
            scan_deceleration_deg_per_sec2
        )
        self.ScanCommandLatencySec = float(scan_command_latency_sec)
        self.Debug = bool(debug)

        if (
            not math.isfinite(self.ScanDecelerationDegPerSec2)
            or self.ScanDecelerationDegPerSec2 <= 0.0
        ):
            raise ValueError(
                "scan_deceleration_deg_per_sec2 must be greater than zero"
            )
        if (
            not math.isfinite(self.ScanCommandLatencySec)
            or self.ScanCommandLatencySec < 0.0
        ):
            raise ValueError("scan_command_latency_sec must not be negative")

        if not bool(getattr(self.Positioner, "UnlimitedAzimuth", False)):
            raise ValueError(
                "Vanguard X requires an unlimited-azimuth X6-60 controller"
            )

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
        # Every bearing is reachable on the unlimited multi-turn X6-60 axis.
        wrap360(relative_azimuth_deg)
        return True

    def ClampRelativeAzimuth(self, relative_azimuth_deg: float) -> float:
        # Kept as an interface-compatible normaliser.  There are no X6-60
        # azimuth end stops or software windows.
        return wrap360(relative_azimuth_deg)

    # ------------------------------------------------------------------
    # Task lifecycle
    # ------------------------------------------------------------------

    def ActivateTask(
        self,
        task: RadarTask,
        navigation: PlatformAttitude,
    ) -> None:
        """
        Activate a task and issue its initial X6-60 command.

        Search:
            starts/resumes continuous slew toward the active sector endpoint.

        Track:
            converts the target true bearing to a platform-relative X6-60 angle
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
                self.Positioner.Stop()

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
                state = self.Positioner.Update()
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

        self.Positioner.SetPanPositionNative(command_target)

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
        self.Positioner.Stop()

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

        Manual positioning is owned here so callers never command the X6-60
        directly. The target wraps through North on the unlimited X6-60 axis.
        Any active search or track task is cleared; the scheduler may reactivate
        search when the operator returns to SCAN mode.

        Returns the commanded platform-relative azimuth in degrees.
        """
        delta = float(delta_deg)

        try:
            state = self.Positioner.Update()
            current_relative = float(state.AzimuthDeg)
        except Exception:
            if self.LastCommandedRelativeDeg is None:
                raise
            current_relative = float(self.LastCommandedRelativeDeg)

        target_relative = wrap360(current_relative + delta)

        self.Positioner.Stop()
        self.Positioner.SetPanPositionNative(target_relative)

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
        self.Positioner.Stop()
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

        x660_state = self.Positioner.Update()

        relative_az = float(x660_state.AzimuthDeg)
        relative_el = float(getattr(x660_state, "ElevationDeg", 0.0))
        rate = float(getattr(x660_state, "PanRateDegPerSec", 0.0))
        x660_valid = bool(getattr(x660_state, "Valid", True))
        source = str(getattr(x660_state, "Source", "X6-60"))

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
            ready = x660_valid
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
                self.Positioner.SetPanPositionNative(command_target)

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
            ready = x660_valid and abs(rate) <= self.SettleRateThresholdDegPerSec

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
            search_endpoint = (
                self.SearchTask.Sector.Pattern.value
                if self.SearchTask.Sector.Pattern
                in (
                    SearchPattern.CONTINUOUS_CW,
                    SearchPattern.CONTINUOUS_CCW,
                )
                else str(self.SearchTask.Sector.ActiveEndpoint)
            )

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
            Valid=bool(x660_valid and navigation.Valid),
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
        if task.Sector.Pattern in (
            SearchPattern.CONTINUOUS_CW,
            SearchPattern.CONTINUOUS_CCW,
        ):
            self.LastCommandedRelativeDeg = None
            self.LastCommandedTrueDeg = None
            direction = (
                1.0
                if task.Sector.Pattern == SearchPattern.CONTINUOUS_CW
                else -1.0
            )
            self.Positioner.CommandSlew(
                direction * abs(float(task.Sector.ScanRateDegPerSec)),
                0.0,
            )
            return

        endpoint_relative = self._search_endpoint_relative(task, navigation)

        try:
            state = self.Positioner.Update()
            current_relative = float(state.AzimuthDeg)
        except Exception:
            current_relative = endpoint_relative

        delta = signed_angle_delta_deg(
            endpoint_relative,
            current_relative,
        )

        direction = 1.0 if delta >= 0.0 else -1.0
        rate = direction * float(task.Sector.ScanRateDegPerSec)

        self.LastCommandedRelativeDeg = endpoint_relative
        self.LastCommandedTrueDeg = self._search_endpoint_true(
            task,
            navigation,
        )

        self.Positioner.CommandSlew(rate, 0.0)

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
            x660_state = self.Positioner.GetState()
            relative_az = float(x660_state.AzimuthDeg)
        except Exception:
            x660_state = self.Positioner.Update()
            relative_az = float(x660_state.AzimuthDeg)

        measured = (
            wrap360(beam_true_deg)
            if sector.Frame == AngleFrame.TRUE
            else wrap360(relative_az)
        )
        previous = sector.LastMeasuredAzimuthDeg
        sector.LastMeasuredAzimuthDeg = measured

        if sector.Pattern in (
            SearchPattern.CONTINUOUS_CW,
            SearchPattern.CONTINUOUS_CCW,
        ):
            # Positive X6-60 azimuth is clockwise. Count one cycle at the
            # measured North crossing in the commanded direction and never
            # issue a reverse command there.
            if previous is not None:
                motion = signed_angle_delta_deg(measured, previous)
                crossed_cw = (
                    sector.Pattern == SearchPattern.CONTINUOUS_CW
                    and motion > 0.01
                    and measured < previous
                )
                crossed_ccw = (
                    sector.Pattern == SearchPattern.CONTINUOUS_CCW
                    and motion < -0.01
                    and measured > previous
                )
                if crossed_cw or crossed_ccw:
                    sector.ScanCycle += 1
            return

        target = wrap360(sector.ActiveEndpointDeg)
        error = signed_angle_delta_deg(target, measured)

        reached = abs(error) <= self.EndpointMarginDeg
        braking_advance_deg = 0.0

        # A speed command reversal does not stop the antenna instantly.  Begin
        # the reversal early enough for the X6-60 planner to decelerate through
        # zero at the operator-selected physical sector boundary.  Use measured
        # output-shaft speed so the advance automatically scales with scan rate.
        #
        #     advance = v^2 / (2a) + v * latency
        #
        # The direction check is essential: immediately after a reversal the
        # motor may still be decelerating in the old direction, away from the
        # newly active endpoint.
        measured_rate = float(
            getattr(x660_state, "PanRateDegPerSec", 0.0)
        )
        moving_toward_endpoint = (
            (measured_rate > 0.01 and error > 0.0)
            or (measured_rate < -0.01 and error < 0.0)
        )
        if self.ScanBrakingEnabled and moving_toward_endpoint:
            speed = abs(measured_rate)
            braking_advance_deg = (
                speed * speed
                / (2.0 * self.ScanDecelerationDegPerSec2)
                + speed * self.ScanCommandLatencySec
            )
            reached = reached or abs(error) <= braking_advance_deg

        if not reached and previous is not None:
            motion = signed_angle_delta_deg(measured, previous)
            previous_error = signed_angle_delta_deg(target, previous)

            motion_epsilon = 0.01

            # The endpoint was crossed only when the measured antenna was
            # moving toward it and the target error changed sign.
            if motion > motion_epsilon:
                reached = previous_error > 0.0 and error <= 0.0
            elif motion < -motion_epsilon:
                reached = previous_error < 0.0 and error >= 0.0

        if reached:
            reached_endpoint = str(sector.ActiveEndpoint)
            reached_target_deg = float(sector.ActiveEndpointDeg)
            sector.Reverse()

            # Reset the crossing history for the new endpoint. Otherwise the
            # final sample from the previous leg can be interpreted as motion
            # across the newly selected endpoint.
            sector.LastMeasuredAzimuthDeg = measured
            self._command_search_slew(task, navigation)

            if self.Debug:
                print(
                    "PointingManager SEARCH reverse: "
                    f"reached={reached_endpoint} "
                    f"target={reached_target_deg:.2f} deg "
                    f"advance={braking_advance_deg:.2f} deg "
                    f"rate={measured_rate:+.2f} deg/s "
                    f"next={sector.ActiveEndpoint} "
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
