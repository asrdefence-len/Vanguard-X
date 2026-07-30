"""Select the Vanguard X tracker products shown on the operator display.

The legacy range/bearing tracker remains authoritative for mission tasking and
track-update control.  This module provides a deliberately narrower authority
transfer: when requested, Earth-referenced tracks and blobs are projected from
mission East/North into the current radar-relative polar coordinates expected
by the existing local and remote displays.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Iterable, List, Optional, Sequence


DISPLAY_TRACK_SOURCE_LEGACY = "LEGACY"
DISPLAY_TRACK_SOURCE_EARTH = "EARTH"
DISPLAY_TRACK_SOURCES = (
    DISPLAY_TRACK_SOURCE_LEGACY,
    DISPLAY_TRACK_SOURCE_EARTH,
)


def _field(value: Any, names: Sequence[str], default: Any = None) -> Any:
    if value is None:
        return default
    for name in names:
        if isinstance(value, dict) and name in value:
            return value[name]
        if hasattr(value, name):
            return getattr(value, name)
    return default


def _finite(value: Any) -> Optional[float]:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def NormaliseDisplayTrackSource(value: Any) -> str:
    """Return a validated display-track source name."""

    source = str(value).strip().upper()
    if source not in DISPLAY_TRACK_SOURCES:
        choices = ", ".join(name.lower() for name in DISPLAY_TRACK_SOURCES)
        raise ValueError(
            f"display track source must be one of: {choices}"
        )
    return source


@dataclass(frozen=True)
class DisplayTrack:
    """Display-compatible projection of one Earth-referenced track."""

    TrackId: int
    RangeM: float
    AzimuthDeg: float
    RangeRateMps: float
    AzimuthRateDps: float
    Status: str
    IsConfirmed: bool
    Hits: int
    Attempts: int
    Misses: int
    LastUpdateScan: int
    SnrDb: float
    AmplitudeDb: float
    EarthEastM: float
    EarthNorthM: float
    VelocityEastMps: float
    VelocityNorthMps: float
    TrackSource: str = DISPLAY_TRACK_SOURCE_EARTH
    TrackType: str = "SURFACE VESSEL"

    @property
    def TrackID(self):
        return self.TrackId

    @property
    def RangeRate(self):
        return self.RangeRateMps

    @property
    def VelocityMps(self):
        return self.RangeRateMps

    @property
    def AngleRateDps(self):
        return self.AzimuthRateDps

    @property
    def BearingRateDps(self):
        return self.AzimuthRateDps

    @property
    def HitCount(self):
        return self.Hits

    @property
    def MissedCount(self):
        return self.Misses

@dataclass(frozen=True)
class DisplayPlot:
    """Display-compatible projection of one Earth-referenced blob."""

    RangeM: float
    AzimuthDeg: float
    AmplitudeDb: float
    DopplerHz: float
    EarthEastM: float
    EarthNorthM: float
    TrackSource: str = DISPLAY_TRACK_SOURCE_EARTH


@dataclass(frozen=True)
class DisplayTrackSelection:
    RequestedSource: str
    AppliedSource: str
    Tracks: List[Any]
    Plots: List[Any]
    FallbackUsed: bool
    Reason: str


@dataclass(frozen=True)
class _RadarState:
    EastM: float
    NorthM: float
    VelocityEastMps: float
    VelocityNorthMps: float
    VelocityValid: bool


def _radar_state(navigation_pose: Any) -> Optional[_RadarState]:
    if not bool(_field(navigation_pose, ("PositionValid",), False)):
        return None

    position = _field(navigation_pose, ("PositionEnu",))
    east_m = _finite(_field(position, ("east_m", "EastM")))
    north_m = _finite(_field(position, ("north_m", "NorthM")))
    if east_m is None or north_m is None:
        return None

    velocity_valid = bool(
        _field(navigation_pose, ("VelocityValid",), False)
    )
    velocity = _field(navigation_pose, ("VelocityEnu",))
    velocity_east_mps = _finite(
        _field(velocity, ("east_mps", "VelocityEastMps"), 0.0)
    )
    velocity_north_mps = _finite(
        _field(velocity, ("north_mps", "VelocityNorthMps"), 0.0)
    )
    if velocity_east_mps is None or velocity_north_mps is None:
        velocity_valid = False
        velocity_east_mps = 0.0
        velocity_north_mps = 0.0

    return _RadarState(
        EastM=east_m,
        NorthM=north_m,
        VelocityEastMps=velocity_east_mps,
        VelocityNorthMps=velocity_north_mps,
        VelocityValid=velocity_valid,
    )


def _relative_polar(
    east_m: float,
    north_m: float,
    velocity_east_mps: float,
    velocity_north_mps: float,
    radar: _RadarState,
):
    delta_east_m = east_m - radar.EastM
    delta_north_m = north_m - radar.NorthM
    range_m = math.hypot(delta_east_m, delta_north_m)
    azimuth_deg = (
        math.degrees(math.atan2(delta_east_m, delta_north_m)) % 360.0
    )

    if range_m <= 1.0e-9 or not radar.VelocityValid:
        return range_m, azimuth_deg, 0.0, 0.0

    relative_east_mps = velocity_east_mps - radar.VelocityEastMps
    relative_north_mps = velocity_north_mps - radar.VelocityNorthMps
    range_rate_mps = (
        delta_east_m * relative_east_mps
        + delta_north_m * relative_north_mps
    ) / range_m
    bearing_rate_rad_per_sec = (
        delta_north_m * relative_east_mps
        - delta_east_m * relative_north_mps
    ) / (range_m * range_m)
    return (
        range_m,
        azimuth_deg,
        range_rate_mps,
        math.degrees(bearing_rate_rad_per_sec),
    )


def _earth_track_to_display(track: Any, radar: _RadarState) -> DisplayTrack:
    east_m = _finite(_field(track, ("EastM", "EarthEastM")))
    north_m = _finite(_field(track, ("NorthM", "EarthNorthM")))
    if east_m is None or north_m is None:
        raise ValueError("Earth track position is invalid")

    velocity_east_mps = _finite(
        _field(track, ("VelocityEastMps",), 0.0)
    )
    velocity_north_mps = _finite(
        _field(track, ("VelocityNorthMps",), 0.0)
    )
    if velocity_east_mps is None or velocity_north_mps is None:
        velocity_east_mps = 0.0
        velocity_north_mps = 0.0

    range_m, azimuth_deg, range_rate_mps, azimuth_rate_dps = (
        _relative_polar(
            east_m,
            north_m,
            velocity_east_mps,
            velocity_north_mps,
            radar,
        )
    )
    status = str(_field(track, ("Status",), "TENTATIVE")).upper()
    return DisplayTrack(
        TrackId=int(_field(track, ("TrackId", "TrackID"), -1)),
        RangeM=range_m,
        AzimuthDeg=azimuth_deg,
        RangeRateMps=range_rate_mps,
        AzimuthRateDps=azimuth_rate_dps,
        Status=status,
        IsConfirmed=bool(
            _field(track, ("IsConfirmed",), status == "CONFIRMED")
        ),
        Hits=int(_field(track, ("Hits", "HitCount"), 0)),
        Attempts=int(_field(track, ("Attempts", "AttemptCount"), 0)),
        Misses=int(_field(track, ("Misses", "MissedCount"), 0)),
        LastUpdateScan=int(
            _field(track, ("LastUpdateScan", "Age", "ScanAge"), 0)
        ),
        SnrDb=float(_field(track, ("SnrDb",), 0.0)),
        AmplitudeDb=float(_field(track, ("AmplitudeDb",), -120.0)),
        EarthEastM=east_m,
        EarthNorthM=north_m,
        VelocityEastMps=velocity_east_mps,
        VelocityNorthMps=velocity_north_mps,
    )


def _earth_plot_to_display(plot: Any, radar: _RadarState) -> DisplayPlot:
    east_m = _finite(_field(plot, ("EastM", "EarthEastM")))
    north_m = _finite(_field(plot, ("NorthM", "EarthNorthM")))
    if east_m is None or north_m is None:
        raise ValueError("Earth plot position is invalid")
    range_m, azimuth_deg, _, _ = _relative_polar(
        east_m,
        north_m,
        0.0,
        0.0,
        radar,
    )
    return DisplayPlot(
        RangeM=range_m,
        AzimuthDeg=azimuth_deg,
        AmplitudeDb=float(_field(plot, ("AmplitudeDb",), -120.0)),
        DopplerHz=float(_field(plot, ("DopplerHz",), 0.0)),
        EarthEastM=east_m,
        EarthNorthM=north_m,
    )


def _project_valid(
    values: Iterable[Any],
    projector,
    radar: _RadarState,
) -> List[Any]:
    projected = []
    for value in values or []:
        try:
            projected.append(projector(value, radar))
        except (TypeError, ValueError):
            continue
    return projected


def SelectDisplayTrackProducts(
    config: Any,
    legacy_tracks: Iterable[Any],
    legacy_plots: Iterable[Any],
    earth_tracks: Iterable[Any],
    earth_plots: Iterable[Any],
    navigation_pose: Any,
) -> DisplayTrackSelection:
    """Select display products without changing operational tracker authority."""

    requested = NormaliseDisplayTrackSource(
        _field(config, ("DisplayTrackSource",), DISPLAY_TRACK_SOURCE_LEGACY)
    )
    legacy_tracks = list(legacy_tracks or [])
    legacy_plots = list(legacy_plots or [])
    if requested == DISPLAY_TRACK_SOURCE_LEGACY:
        return DisplayTrackSelection(
            RequestedSource=requested,
            AppliedSource=DISPLAY_TRACK_SOURCE_LEGACY,
            Tracks=legacy_tracks,
            Plots=legacy_plots,
            FallbackUsed=False,
            Reason="REQUESTED_LEGACY",
        )

    if not bool(_field(config, ("EarthTrackerEnabled",), True)):
        return DisplayTrackSelection(
            RequestedSource=requested,
            AppliedSource=DISPLAY_TRACK_SOURCE_LEGACY,
            Tracks=legacy_tracks,
            Plots=legacy_plots,
            FallbackUsed=True,
            Reason="EARTH_TRACKER_DISABLED",
        )

    radar = _radar_state(navigation_pose)
    if radar is None:
        return DisplayTrackSelection(
            RequestedSource=requested,
            AppliedSource=DISPLAY_TRACK_SOURCE_LEGACY,
            Tracks=legacy_tracks,
            Plots=legacy_plots,
            FallbackUsed=True,
            Reason="INVALID_NAVIGATION_POSITION",
        )

    return DisplayTrackSelection(
        RequestedSource=requested,
        AppliedSource=DISPLAY_TRACK_SOURCE_EARTH,
        Tracks=_project_valid(
            earth_tracks,
            _earth_track_to_display,
            radar,
        ),
        Plots=_project_valid(
            earth_plots,
            _earth_plot_to_display,
            radar,
        ),
        FallbackUsed=False,
        Reason="OK",
    )
