"""Stage 9 operational validation for local and Ethernet PPI track products.

The validation drives the real Stage 8 display selector through a complete
ownship circle.  Each selected frame is delivered both directly, as in the
local Qt path, and through the actual bounded JSON snapshot/framing/client
adapter used by the separate Ethernet UI.

This is deliberately a display-only gate.  The diagnostics must continue to
declare the legacy tracker as the tasking and track-update authority.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from types import SimpleNamespace
from typing import Any, Dict, List

import numpy as np

from DisplayTrackSource import SelectDisplayTrackProducts
from RadarLinkProtocol import EncodeMessage, FrameDecoder, MakeMessage
from RadarRemoteDisplay import BuildDisplaySnapshot
from RunVanguardUiClient import ApplyServerMessage


@dataclass(frozen=True)
class OperationalDisplayValidationResult:
    Frames: int
    EarthFrames: int
    FallbackFrames: int
    MaximumRangeDifferenceM: float
    MaximumBearingDifferenceDeg: float
    MaximumRangeRateDifferenceMps: float
    TrackMetadataPreserved: bool
    DiagnosticsPreserved: bool
    LegacyTaskingPreserved: bool
    Passed: bool


class _CaptureDisplay:
    """Small stand-in for the Qt display public Update interface."""

    def __init__(self):
        self.Processed = None
        self.Detections = []
        self.Tracks = []
        self.Plots = []

    def Update(self, processed, detections, Tracks=None, Plots=None):
        self.Processed = processed
        self.Detections = list(detections or [])
        self.Tracks = list(Tracks or [])
        self.Plots = list(Plots or [])


def _pose(angle_rad: float, valid: bool = True):
    radius_m = 750.0
    speed_mps = 6.0
    east_m = radius_m * math.sin(angle_rad)
    north_m = radius_m * math.cos(angle_rad)
    velocity_east_mps = speed_mps * math.cos(angle_rad)
    velocity_north_mps = -speed_mps * math.sin(angle_rad)
    return SimpleNamespace(
        PositionValid=bool(valid),
        VelocityValid=bool(valid),
        PositionEnu=SimpleNamespace(east_m=east_m, north_m=north_m),
        VelocityEnu=SimpleNamespace(
            east_mps=velocity_east_mps,
            north_mps=velocity_north_mps,
        ),
    )


def _earth_track(
    track_id: int,
    east_m: float,
    north_m: float,
    velocity_east_mps: float,
    velocity_north_mps: float,
):
    return SimpleNamespace(
        TrackId=track_id,
        EastM=east_m,
        NorthM=north_m,
        VelocityEastMps=velocity_east_mps,
        VelocityNorthMps=velocity_north_mps,
        Status="CONFIRMED",
        IsConfirmed=True,
        Hits=72,
        Attempts=72,
        Misses=0,
        LastUpdateScan=72,
        SnrDb=18.0,
        AmplitudeDb=-42.0,
    )


def _legacy_track():
    return SimpleNamespace(
        TrackId=901,
        RangeM=3200.0,
        AzimuthDeg=215.0,
        RangeRateMps=0.0,
        AzimuthRateDps=0.0,
        Status="CONFIRMED",
        IsConfirmed=True,
        Hits=10,
        Misses=0,
        TrackSource="LEGACY",
    )


def _processed(selection):
    diagnostics = {
        "BoresightDeg": 0.0,
        "DisplayTrackSourceRequested": selection.RequestedSource,
        "DisplayTrackSourceApplied": selection.AppliedSource,
        "DisplayTrackSourceFallback": selection.FallbackUsed,
        "DisplayTrackSourceReason": selection.Reason,
        "DisplayTrackCount": len(selection.Tracks),
        "TaskingTrackSource": "LEGACY",
        "TrackUpdateSource": "LEGACY",
    }
    return SimpleNamespace(
        Diagnostics=diagnostics,
        RangeAxisM=np.asarray([0.0, 1000.0, 2000.0]),
        MagnitudeDb=np.asarray([-90.0, -40.0, -80.0]),
        DopplerAxisHz=np.asarray([0.0]),
    )


def _wire_delivery(config, processed, tracks, plots):
    snapshot = BuildDisplaySnapshot(
        config,
        processed,
        detections=[],
        tracks=tracks,
        plots=plots,
    )
    frame = EncodeMessage(MakeMessage("radar_snapshot", snapshot, sequence=1))
    messages = FrameDecoder().Feed(frame)
    if len(messages) != 1:
        raise AssertionError("one encoded snapshot must decode as one message")
    display = _CaptureDisplay()
    ApplyServerMessage(display, messages[0])
    return display


def _bearing_difference_deg(left: float, right: float) -> float:
    return abs((float(left) - float(right) + 180.0) % 360.0 - 180.0)


def RunOperationalDisplayValidation(
    frame_count: int = 72,
) -> OperationalDisplayValidationResult:
    if int(frame_count) < 4:
        raise ValueError("frame_count must be at least four")

    config: Dict[str, Any] = {
        "DisplayTrackSource": "EARTH",
        "EarthTrackerEnabled": True,
        "RadarLinkMaxRangeProfilePoints": 1500,
        "RangeProfileDopplerMode": "MAX",
    }
    earth_tracks = [
        _earth_track(41, 2200.0, 4100.0, 0.0, 0.0),
        _earth_track(52, -1800.0, 3300.0, 3.5, -1.25),
    ]
    legacy_tracks = [_legacy_track()]

    maximum_range_difference_m = 0.0
    maximum_bearing_difference_deg = 0.0
    maximum_range_rate_difference_mps = 0.0
    metadata_preserved = True
    diagnostics_preserved = True
    legacy_tasking_preserved = True
    earth_frames = 0

    for index in range(int(frame_count)):
        angle_rad = 2.0 * math.pi * index / float(frame_count)
        selection = SelectDisplayTrackProducts(
            config,
            legacy_tracks,
            [],
            earth_tracks,
            [],
            _pose(angle_rad),
        )
        if selection.AppliedSource != "EARTH":
            raise AssertionError("valid circular-route pose must select Earth tracks")
        earth_frames += 1
        processed = _processed(selection)

        local = _CaptureDisplay()
        local.Update(processed, [], Tracks=selection.Tracks, Plots=selection.Plots)
        remote = _wire_delivery(
            config,
            processed,
            selection.Tracks,
            selection.Plots,
        )
        if len(local.Tracks) != len(remote.Tracks):
            raise AssertionError("local and Ethernet track counts differ")

        for local_track, remote_track in zip(local.Tracks, remote.Tracks):
            maximum_range_difference_m = max(
                maximum_range_difference_m,
                abs(float(local_track.RangeM) - float(remote_track.RangeM)),
            )
            maximum_bearing_difference_deg = max(
                maximum_bearing_difference_deg,
                _bearing_difference_deg(
                    local_track.AzimuthDeg,
                    remote_track.AzimuthDeg,
                ),
            )
            maximum_range_rate_difference_mps = max(
                maximum_range_rate_difference_mps,
                abs(
                    float(local_track.RangeRateMps)
                    - float(remote_track.RangeRateMps)
                ),
            )
            metadata_preserved = metadata_preserved and all(
                hasattr(remote_track, name)
                for name in (
                    "TrackSource",
                    "EarthEastM",
                    "EarthNorthM",
                    "VelocityEastMps",
                    "VelocityNorthMps",
                )
            )
            metadata_preserved = (
                metadata_preserved
                and str(remote_track.TrackSource) == "EARTH"
                and int(remote_track.TrackId) == int(local_track.TrackId)
            )

        remote_diagnostics = dict(
            getattr(remote.Processed, "Diagnostics", {}) or {}
        )
        diagnostics_preserved = diagnostics_preserved and (
            remote_diagnostics == processed.Diagnostics
        )
        legacy_tasking_preserved = legacy_tasking_preserved and (
            remote_diagnostics.get("TaskingTrackSource") == "LEGACY"
            and remote_diagnostics.get("TrackUpdateSource") == "LEGACY"
        )

    fallback_selection = SelectDisplayTrackProducts(
        config,
        legacy_tracks,
        [],
        earth_tracks,
        [],
        _pose(0.0, valid=False),
    )
    fallback_processed = _processed(fallback_selection)
    fallback_remote = _wire_delivery(
        config,
        fallback_processed,
        fallback_selection.Tracks,
        fallback_selection.Plots,
    )
    fallback_diagnostics = dict(
        getattr(fallback_remote.Processed, "Diagnostics", {}) or {}
    )
    fallback_passed = (
        fallback_selection.AppliedSource == "LEGACY"
        and fallback_selection.FallbackUsed
        and fallback_selection.Reason == "INVALID_NAVIGATION_POSITION"
        and len(fallback_remote.Tracks) == 1
        and int(fallback_remote.Tracks[0].TrackId) == 901
        and fallback_diagnostics.get("DisplayTrackSourceApplied") == "LEGACY"
        and bool(fallback_diagnostics.get("DisplayTrackSourceFallback"))
    )

    passed = bool(
        earth_frames == int(frame_count)
        and fallback_passed
        and maximum_range_difference_m <= 1.0e-9
        and maximum_bearing_difference_deg <= 1.0e-9
        and maximum_range_rate_difference_mps <= 1.0e-9
        and metadata_preserved
        and diagnostics_preserved
        and legacy_tasking_preserved
    )
    return OperationalDisplayValidationResult(
        Frames=int(frame_count) + 1,
        EarthFrames=earth_frames,
        FallbackFrames=1 if fallback_passed else 0,
        MaximumRangeDifferenceM=maximum_range_difference_m,
        MaximumBearingDifferenceDeg=maximum_bearing_difference_deg,
        MaximumRangeRateDifferenceMps=maximum_range_rate_difference_mps,
        TrackMetadataPreserved=metadata_preserved,
        DiagnosticsPreserved=diagnostics_preserved,
        LegacyTaskingPreserved=legacy_tasking_preserved,
        Passed=passed,
    )


def Main() -> int:
    result = RunOperationalDisplayValidation()
    print("Vanguard X Stage 9 operational display validation")
    print(
        f"Frames: {result.Frames} "
        f"(Earth={result.EarthFrames}, fallback={result.FallbackFrames})"
    )
    print(
        "Local/Ethernet maximum differences: "
        f"range={result.MaximumRangeDifferenceM:.3e} m, "
        f"bearing={result.MaximumBearingDifferenceDeg:.3e} deg, "
        f"range-rate={result.MaximumRangeRateDifferenceMps:.3e} m/s"
    )
    print(
        "Track metadata: "
        f"{'PASS' if result.TrackMetadataPreserved else 'FAIL'}"
    )
    print(
        "Source diagnostics: "
        f"{'PASS' if result.DiagnosticsPreserved else 'FAIL'}"
    )
    print(
        "Legacy tasking authority: "
        f"{'PASS' if result.LegacyTaskingPreserved else 'FAIL'}"
    )
    print(f"RESULT: {'PASS' if result.Passed else 'FAIL'}")
    return 0 if result.Passed else 1


if __name__ == "__main__":
    raise SystemExit(Main())
