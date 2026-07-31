"""Operator-directed track confirmation tasking for Vanguard X."""

from __future__ import annotations

from typing import Any, Dict, Optional
import time

from MissionProfile import TrackUpdatePolicy
from RadarTasks import RevisitRequest


def _field(value: Any, name: str, default=None):
    if isinstance(value, dict):
        return value.get(name, default)
    return getattr(value, name, default)


def _angle_distance_deg(left_deg: float, right_deg: float) -> float:
    return abs(
        ((float(left_deg) - float(right_deg) + 180.0) % 360.0) - 180.0
    )


def FindTaskingTrack(
    tracks,
    track_id: int,
    *,
    selected_source: str = "LEGACY",
    selected_range_m: Optional[float] = None,
    selected_azimuth_deg: Optional[float] = None,
    selected_was_tentative: Optional[bool] = None,
):
    tasking_tracks = []
    for track in list(tracks or []):
        status = str(_field(track, "Status", "")).upper()
        if status in ("", "DELETED"):
            continue
        tasking_tracks.append(track)

    if str(selected_source).upper() == "LEGACY":
        for track in tasking_tracks:
            if int(_field(track, "TrackId", -1)) == int(track_id):
                return track

    if selected_range_m is None or selected_azimuth_deg is None:
        return None
    best = None
    best_score = float("inf")
    for track in tasking_tracks:
        is_confirmed = bool(
            _field(track, "IsConfirmed", False)
        ) or str(_field(track, "Status", "")).upper() == "CONFIRMED"
        if (
            selected_was_tentative is not None
            and bool(selected_was_tentative) == is_confirmed
        ):
            # Earth- and legacy-tracker IDs are independent.  Preserve the
            # selected track's initiation class so an orange Earth track
            # cannot be resolved to a close confirmed legacy neighbour.
            continue
        range_error_m = abs(
            float(_field(track, "RangeM", 0.0))
            - float(selected_range_m)
        )
        angle_error_deg = _angle_distance_deg(
            float(_field(track, "AzimuthDeg", 0.0)),
            float(selected_azimuth_deg),
        )
        if range_error_m > 300.0 or angle_error_deg > 6.0:
            continue
        score = (range_error_m / 300.0) ** 2 + (
            angle_error_deg / 6.0
        ) ** 2
        if score < best_score:
            best = track
            best_score = score
    return best


def FindConfirmedTrack(*args, **kwargs):
    """Compatibility alias retained for Stage 12E-G callers and tests."""

    track = FindTaskingTrack(*args, **kwargs)
    if track is None:
        return None
    status = str(_field(track, "Status", "")).upper()
    confirmed = bool(
        _field(track, "IsConfirmed", False)
    ) or status == "CONFIRMED"
    return track if confirmed else None


def BuildTrackConfirmationRequest(
    track,
    policy: TrackUpdatePolicy,
    *,
    now_sec: Optional[float] = None,
    gate_half_width_deg: Optional[float] = None,
    retry: bool = False,
    selected_source: str = "LEGACY",
    selected_track_id: Optional[int] = None,
    selected_was_tentative: Optional[bool] = None,
) -> RevisitRequest:
    now = time.time() if now_sec is None else float(now_sec)
    gate_half = (
        float(policy.GateHalfWidthDeg)
        if gate_half_width_deg is None
        else float(gate_half_width_deg)
    )
    gate_half = min(
        max(gate_half, float(policy.MinimumGateDeg)),
        float(policy.MaximumGateDeg),
    )
    return RevisitRequest(
        TrackId=int(_field(track, "TrackId")),
        RequestedTimeSec=now,
        DeadlineTimeSec=now + float(policy.MaximumUpdateDurationSec),
        PredictedRangeM=float(_field(track, "RangeM", 0.0)),
        PredictedTrueBearingDeg=float(
            _field(track, "AzimuthDeg", 0.0)
        ) % 360.0,
        PredictedElevationDeg=0.0,
        Priority=1000 if not retry else 1100,
        RevisitIntervalSec=float(policy.RevisitIntervalSec),
        WaveformProfileId="TRACK_CONFIRM",
        Metadata={
            "TrackNoddyEnabled": True,
            "TrackGateHalfWidthDeg": gate_half,
            "TrackNodPasses": 3,
            "TrackSlewRateDegSec": float(policy.SlewRateDegSec),
            "TrackNodRateDegSec": float(policy.NodRateDegSec),
            "TrackPointingToleranceDeg": float(
                policy.PointingToleranceDeg
            ),
            "TrackSettleTimeSec": float(policy.SettleTimeSec),
            "TrackMaximumUpdateDurationSec": float(
                policy.MaximumUpdateDurationSec
            ),
            "TrackMissPolicy": str(policy.MissPolicy),
            "TrackConfirmationRetry": bool(retry),
            "TrackStatusAtRequest": str(
                _field(track, "Status", "UNKNOWN")
            ).upper(),
            "TrackWasTentativeAtRequest": not (
                bool(_field(track, "IsConfirmed", False))
                or str(
                    _field(track, "Status", "")
                ).upper() == "CONFIRMED"
            ),
            # Preserve the exact display selection as well as the
            # authoritative legacy tasking-track identity.  Earth and legacy
            # tracker identifiers are independent; the display-side track
            # must receive the same directed result immediately when the nod
            # finishes rather than waiting for a later search-pass boundary.
            "TrackSelectedSource": str(selected_source).upper(),
            "TrackSelectedTrackId": int(
                _field(track, "TrackId")
                if selected_track_id is None
                else selected_track_id
            ),
            "TrackSelectedWasTentative": (
                not (
                    bool(_field(track, "IsConfirmed", False))
                    or str(
                        _field(track, "Status", "")
                    ).upper() == "CONFIRMED"
                )
                if selected_was_tentative is None
                else bool(selected_was_tentative)
            ),
            "TrackWaveformId": str(policy.WaveformId),
            "TrackPrfHz": float(policy.PrfHz),
            "TrackPulsesPerCpi": int(policy.PulsesPerCpi),
            "TrackMaximumRangeM": float(policy.MaximumRangeM),
        },
    )


class TrackConfirmationController:
    """Turns one-shot UI requests into finite scheduler TRACK tasks."""

    def __init__(
        self,
        minimum_operator_gate_half_width_deg: float = 5.0,
    ):
        self.LastCommandId = 0
        self._plots_by_task_id = {}
        self.MinimumOperatorGateHalfWidthDeg = max(
            0.1,
            float(minimum_operator_gate_half_width_deg),
        )

    def _operator_gate_half_width(
        self,
        policy: TrackUpdatePolicy,
    ) -> float:
        return min(
            float(policy.MaximumGateDeg),
            max(
                float(policy.MinimumGateDeg),
                float(policy.GateHalfWidthDeg),
                self.MinimumOperatorGateHalfWidthDeg,
            ),
        )

    def HasPendingOperatorRequest(
        self,
        control_state: Optional[Dict[str, Any]],
    ) -> bool:
        """Return True before a new UI confirmation edge is consumed."""

        control = dict(control_state or {})
        command_id = int(control.get("TrackConfirmCommandId", 0))
        return command_id != self.LastCommandId

    def AccumulateTaskPlots(self, task_id: int, plots) -> None:
        """Retain evidence emitted during any of the three nod passes."""

        task_plots = self._plots_by_task_id.setdefault(int(task_id), [])
        task_plots.extend(list(plots or []))

    def FinalizeTaskPlots(self, task_id: int, plots=None):
        """Return all nod-pass evidence and release the finite-task buffer."""

        self.AccumulateTaskPlots(task_id, plots)
        return self._plots_by_task_id.pop(int(task_id), [])

    def DiscardTaskPlots(self, task_id: int) -> None:
        self._plots_by_task_id.pop(int(task_id), None)

    def ConsumeOperatorRequest(
        self,
        control_state: Optional[Dict[str, Any]],
        tracks,
        scheduler,
        policy: TrackUpdatePolicy,
        *,
        now_sec: Optional[float] = None,
    ) -> Dict[str, Any]:
        control = dict(control_state or {})
        command_id = int(control.get("TrackConfirmCommandId", 0))
        if command_id == self.LastCommandId:
            return {"Changed": False}
        self.LastCommandId = command_id
        track_id = int(control.get("TrackConfirmTrackId", 0))
        track = FindTaskingTrack(
            tracks,
            track_id,
            selected_source=str(
                control.get("TrackConfirmSource", "LEGACY")
            ),
            selected_range_m=control.get("TrackConfirmRangeM"),
            selected_azimuth_deg=control.get(
                "TrackConfirmAzimuthDeg"
            ),
            selected_was_tentative=control.get(
                "TrackConfirmWasTentative"
            ),
        )
        if track is None:
            return {
                "Changed": True,
                "Applied": False,
                "TrackId": track_id,
                "Message": (
                    f"T{track_id} is no longer available for tasking"
                ),
            }
        authoritative_track_id = int(_field(track, "TrackId"))
        status = str(_field(track, "Status", "")).upper()
        is_confirmed = bool(
            _field(track, "IsConfirmed", False)
        ) or status == "CONFIRMED"
        gate_half_width_deg = self._operator_gate_half_width(policy)
        request = BuildTrackConfirmationRequest(
            track,
            policy,
            now_sec=now_sec,
            gate_half_width_deg=gate_half_width_deg,
            selected_source=str(
                control.get("TrackConfirmSource", "LEGACY")
            ),
            selected_track_id=track_id,
            selected_was_tentative=control.get(
                "TrackConfirmWasTentative"
            ),
        )
        scheduler.SubmitRevisitRequest(request)
        return {
            "Changed": True,
            "Applied": True,
            "TrackId": authoritative_track_id,
            "TrackWasTentative": not is_confirmed,
            "Message": (
                f"T{authoritative_track_id} "
                f"{'confirmation' if is_confirmed else 'initiation check'} "
                "queued "
                f"(+/-{gate_half_width_deg:g} deg, 3 nod passes)"
            ),
        }

    def QueueRetryIfRequired(
        self,
        outcome: Dict[str, Any],
        tracks,
        scheduler,
        policy: TrackUpdatePolicy,
        *,
        now_sec: Optional[float] = None,
    ) -> Optional[Dict[str, Any]]:
        if bool(outcome.get("Hit", False)) or bool(
            outcome.get("Deleted", False)
        ):
            return None
        # A complete nod is exactly one opportunity in the normal tentative
        # 2-of-3 initiation rule.  It must not inherit the confirmed-track
        # wider-retry/delete policy.
        if bool(outcome.get("TrackWasTentative", False)):
            return None
        if not bool(outcome.get("CoverageValid", False)):
            return None
        if int(outcome.get("DirectedUpdateMisses", 0)) != 1:
            return None

        miss_policy = str(policy.MissPolicy).upper()
        if miss_policy not in (
            "RETRY",
            "WIDEN",
            "RETRY_WIDER_THEN_DELETE",
            "RETRY_WIDER_THEN_DEFER",
        ):
            return None

        track_id = int(outcome.get("TrackId", 0))
        track = FindConfirmedTrack(tracks, track_id)
        if track is None:
            return None
        widen = miss_policy != "RETRY"
        gate_half = self._operator_gate_half_width(policy)
        if widen:
            gate_half = min(
                float(policy.MaximumGateDeg),
                max(
                    float(policy.MinimumGateDeg),
                    2.0 * gate_half,
                ),
            )
        scheduler.SubmitRevisitRequest(
            BuildTrackConfirmationRequest(
                track,
                policy,
                now_sec=now_sec,
                gate_half_width_deg=gate_half,
                retry=True,
            )
        )
        return {
            "TrackId": track_id,
            "GateHalfWidthDeg": gate_half,
            "Message": (
                f"T{track_id} missed; retry queued "
                f"(+/-{gate_half:g} deg)"
            ),
        }
