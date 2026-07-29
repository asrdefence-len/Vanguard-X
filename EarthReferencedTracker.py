"""Non-authoritative mission-ENU tracker for Vanguard X.

This tracker runs in parallel with the established range/bearing tracker.  It
uses only valid Stage 5 Earth-referenced detection annotations and maintains
track position and velocity in fixed mission East/North coordinates.

The legacy tracker remains authoritative for display and tasking.  This module
exists to validate Earth-referenced association and filtering under ownship
motion before any authority is transferred.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

from CoordinateFrames import detection_to_enu_position


EARTH_TRACKER_VERSION = "earth-enu-2of3-alpha-beta-v1"


@dataclass
class _EarthPoint:
    EastM: float
    NorthM: float
    TimestampSec: Optional[float] = None
    AmplitudeDb: float = -120.0
    SnrDb: float = 0.0
    DopplerHz: float = 0.0


@dataclass
class EarthReferencedBlob:
    BlobId: int
    EastM: float
    NorthM: float
    TimestampSec: Optional[float] = None
    NumDetections: int = 1
    AmplitudeDb: float = -120.0
    SnrDb: float = 0.0
    DopplerHz: float = 0.0
    Status: str = "EARTH_BLOB"
    IsConfirmed: bool = False


@dataclass
class EarthReferencedTrack:
    TrackId: int
    EastM: float
    NorthM: float
    VelocityEastMps: float = 0.0
    VelocityNorthMps: float = 0.0
    PredictedEastM: float = 0.0
    PredictedNorthM: float = 0.0
    Status: str = "TENTATIVE"
    IsConfirmed: bool = False
    Hits: int = 1
    Attempts: int = 1
    Misses: int = 0
    LastUpdateScan: int = 0
    LastHitScan: int = 0
    LastUpdateTimestampSec: Optional[float] = None
    LastHitTimestampSec: Optional[float] = None
    SnrDb: float = 0.0
    AmplitudeDb: float = -120.0
    NumDetections: int = 1
    History: List[Tuple[int, Optional[float], float, float]] = field(
        default_factory=list
    )

    @property
    def TrackID(self):
        return self.TrackId

    @property
    def SpeedMps(self):
        return math.hypot(self.VelocityEastMps, self.VelocityNorthMps)

    @property
    def CourseTrueDeg(self):
        if self.SpeedMps <= 1e-12:
            return 0.0
        return (
            math.degrees(
                math.atan2(self.VelocityEastMps, self.VelocityNorthMps)
            )
            % 360.0
        )


@dataclass(frozen=True)
class ParallelTrackerComparison:
    Valid: bool
    Reason: str
    EarthConfirmedTrackCount: int
    LegacyConfirmedTrackCount: int
    MatchedTrackCount: int
    UnmatchedEarthTrackCount: int
    UnmatchedLegacyTrackCount: int
    MeanPositionSeparationM: Optional[float]
    RmsPositionSeparationM: Optional[float]
    MaximumPositionSeparationM: Optional[float]
    PositionSeparationsM: Tuple[float, ...]


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


def _distance(east_a: float, north_a: float, east_b: float, north_b: float):
    return math.hypot(east_a - east_b, north_a - north_b)


class EarthReferencedTracker:
    """Scan-pass Cartesian tracker operating in mission ENU coordinates."""

    def __init__(self, Config: Optional[Dict[str, Any]] = None):
        self.Config = Config or {}

        self.ClusterDistanceM = float(
            self.Config.get(
                "EarthClusterDistanceM",
                self.Config.get("ClusterRangeGapM", 200.0),
            )
        )
        self.MinBlobDetections = int(
            self.Config.get("EarthMinBlobDetections", 1)
        )
        self.InitiationGateM = float(
            self.Config.get(
                "EarthInitiationGateM",
                self.Config.get("InitiationRangeGateM", 300.0),
            )
        )
        self.AssociationGateM = float(
            self.Config.get(
                "EarthAssociationGateM",
                self.Config.get("AssociationRangeGateM", 300.0),
            )
        )
        self.InitiationWindow = int(
            self.Config.get("EarthInitiationWindow", 3)
        )
        self.InitiationRequiredHits = int(
            self.Config.get("EarthInitiationRequiredHits", 2)
        )
        self.DeleteConfirmedAfterMisses = int(
            self.Config.get(
                "EarthDeleteConfirmedAfterMisses",
                self.Config.get("DeleteAfterMissesConfirmed", 5),
            )
        )
        self.TrackDtSec = float(
            self.Config.get(
                "EarthTrackDtSec",
                self.Config.get("TrackDtSec", 1.0),
            )
        )
        self.TrackAlpha = float(
            self.Config.get("EarthTrackAlpha", 0.65)
        )
        self.TrackBeta = float(
            self.Config.get("EarthTrackBeta", 0.20)
        )
        self.TentativeAlpha = float(
            self.Config.get("EarthTentativeAlpha", 0.85)
        )
        self.MaxTrackHistory = int(
            self.Config.get("EarthMaxTrackHistory", 20)
        )

        self.Tracks: List[EarthReferencedTrack] = []
        self.CurrentScanId: Optional[int] = None
        self.CurrentScanPoints: List[_EarthPoint] = []
        self.LastCompletedBlobs: List[EarthReferencedBlob] = []
        self.NextTrackId = 1
        self.NextBlobId = 1

        self._inferred_scan_id = 0
        self._last_azimuth_deg: Optional[float] = None
        self._last_direction: Optional[int] = None
        self.LastDebug: Dict[str, Any] = {
            "TrackerVersion": EARTH_TRACKER_VERSION
        }

    def Update(self, Detections, Processed=None, ThisDwell=None):
        scan_id = self._scan_id(Processed, ThisDwell)
        if self.CurrentScanId is None:
            self.CurrentScanId = scan_id

        if scan_id != self.CurrentScanId:
            completed_blobs = self._cluster_points(self.CurrentScanPoints)
            self.LastCompletedBlobs = completed_blobs
            self._process_completed_scan(
                completed_blobs,
                self.CurrentScanId,
            )
            self.CurrentScanPoints = []
            self.CurrentScanId = scan_id

        detections = list(Detections) if Detections is not None else []
        accepted = 0
        for detection in detections:
            point = self._point_from_detection(detection)
            if point is not None:
                self.CurrentScanPoints.append(point)
                accepted += 1

        self.LastDebug = {
            "TrackerVersion": EARTH_TRACKER_VERSION,
            "CurrentScanId": self.CurrentScanId,
            "RawDetectionsThisDwell": len(detections),
            "ValidEarthDetectionsThisDwell": accepted,
            "RejectedEarthDetectionsThisDwell": len(detections) - accepted,
            "CurrentScanPoints": len(self.CurrentScanPoints),
            "LastCompletedBlobs": len(self.LastCompletedBlobs),
            "TentativeTracks": len(self.GetTentativeTracks()),
            "ConfirmedTracks": len(self.GetConfirmedTracks()),
            "TotalTracks": len(self.Tracks),
            "InitiationWindow": self.InitiationWindow,
            "InitiationRequiredHits": self.InitiationRequiredHits,
            "ClusterDistanceM": self.ClusterDistanceM,
            "InitiationGateM": self.InitiationGateM,
            "AssociationGateM": self.AssociationGateM,
            "TrackAlpha": self.TrackAlpha,
            "TrackBeta": self.TrackBeta,
            "TrackDtSec": self.TrackDtSec,
            "Authoritative": False,
        }
        return self.GetTracks(), list(self.LastCompletedBlobs)

    def GetTracks(self):
        return list(self.Tracks)

    def GetTentativeTracks(self):
        return [
            track
            for track in self.Tracks
            if track.Status != "CONFIRMED"
        ]

    def GetConfirmedTracks(self):
        return [
            track
            for track in self.Tracks
            if track.Status == "CONFIRMED"
        ]

    def GetDebugInfo(self):
        return dict(self.LastDebug)

    def _process_completed_scan(
        self,
        blobs: List[EarthReferencedBlob],
        scan_id: int,
    ):
        used_blob_ids = set()
        surviving: List[EarthReferencedTrack] = []

        confirmed = [
            track
            for track in self.Tracks
            if track.Status == "CONFIRMED"
        ]
        tentative = [
            track
            for track in self.Tracks
            if track.Status != "CONFIRMED"
        ]
        scan_timestamp_sec = self._completed_scan_timestamp(blobs)

        for track in confirmed:
            dt = self._track_dt(
                track,
                scan_id,
                scan_timestamp_sec,
            )
            predicted_east, predicted_north = self._predict(track, dt)
            track.PredictedEastM = predicted_east
            track.PredictedNorthM = predicted_north
            blob = self._best_blob_in_gate(
                predicted_east,
                predicted_north,
                blobs,
                used_blob_ids,
                self.AssociationGateM,
            )
            if blob is not None:
                used_blob_ids.add(blob.BlobId)
                self._alpha_beta_update(track, blob, scan_id, dt)
                track.Misses = 0
                surviving.append(track)
            else:
                self._coast(track, scan_id, dt)
                if track.Misses <= self.DeleteConfirmedAfterMisses:
                    surviving.append(track)

        for track in tentative:
            if track.LastUpdateScan == scan_id:
                surviving.append(track)
                continue

            blob = self._best_blob_in_gate(
                track.EastM,
                track.NorthM,
                blobs,
                used_blob_ids,
                self.InitiationGateM,
            )
            track.Attempts += 1
            track.LastUpdateScan = scan_id

            if blob is not None:
                used_blob_ids.add(blob.BlobId)
                track.Hits += 1
                track.LastHitScan = scan_id
                self._update_tentative(track, blob, scan_id)
            else:
                track.Misses += 1

            if track.Attempts >= self.InitiationWindow:
                if track.Hits >= self.InitiationRequiredHits:
                    track.Status = "CONFIRMED"
                    track.IsConfirmed = True
                    track.Misses = 0
                    self._estimate_velocity_from_history(track)
                    surviving.append(track)
            else:
                surviving.append(track)

        self.Tracks = surviving
        for blob in blobs:
            if blob.BlobId in used_blob_ids:
                continue
            if self._is_near_existing_track(blob):
                continue
            self._create_tentative(blob, scan_id)

    def _create_tentative(
        self,
        blob: EarthReferencedBlob,
        scan_id: int,
    ):
        track = EarthReferencedTrack(
            TrackId=self.NextTrackId,
            EastM=blob.EastM,
            NorthM=blob.NorthM,
            PredictedEastM=blob.EastM,
            PredictedNorthM=blob.NorthM,
            LastUpdateScan=scan_id,
            LastHitScan=scan_id,
            LastUpdateTimestampSec=blob.TimestampSec,
            LastHitTimestampSec=blob.TimestampSec,
            SnrDb=blob.SnrDb,
            AmplitudeDb=blob.AmplitudeDb,
            NumDetections=blob.NumDetections,
            History=[
                (
                    scan_id,
                    blob.TimestampSec,
                    blob.EastM,
                    blob.NorthM,
                )
            ],
        )
        self.NextTrackId += 1
        self.Tracks.append(track)

    def _update_tentative(
        self,
        track: EarthReferencedTrack,
        blob: EarthReferencedBlob,
        scan_id: int,
    ):
        alpha = self.TentativeAlpha
        track.EastM = alpha * blob.EastM + (1.0 - alpha) * track.EastM
        track.NorthM = (
            alpha * blob.NorthM + (1.0 - alpha) * track.NorthM
        )
        track.PredictedEastM = track.EastM
        track.PredictedNorthM = track.NorthM
        track.SnrDb = blob.SnrDb
        track.AmplitudeDb = blob.AmplitudeDb
        track.NumDetections = blob.NumDetections
        track.LastUpdateScan = scan_id
        track.LastUpdateTimestampSec = blob.TimestampSec
        track.LastHitTimestampSec = blob.TimestampSec
        self._append_history(track, scan_id, blob.TimestampSec)

    def _alpha_beta_update(
        self,
        track: EarthReferencedTrack,
        blob: EarthReferencedBlob,
        scan_id: int,
        dt: float,
    ):
        dt = max(float(dt), 1e-6)
        predicted_east, predicted_north = self._predict(track, dt)
        residual_east = blob.EastM - predicted_east
        residual_north = blob.NorthM - predicted_north

        track.EastM = predicted_east + self.TrackAlpha * residual_east
        track.NorthM = (
            predicted_north + self.TrackAlpha * residual_north
        )
        track.VelocityEastMps += (
            self.TrackBeta * residual_east / dt
        )
        track.VelocityNorthMps += (
            self.TrackBeta * residual_north / dt
        )
        track.PredictedEastM = predicted_east
        track.PredictedNorthM = predicted_north
        track.SnrDb = blob.SnrDb
        track.AmplitudeDb = blob.AmplitudeDb
        track.NumDetections = blob.NumDetections
        track.LastUpdateScan = scan_id
        track.LastHitScan = scan_id
        track.LastUpdateTimestampSec = blob.TimestampSec
        track.LastHitTimestampSec = blob.TimestampSec
        track.Hits += 1
        self._append_history(track, scan_id, blob.TimestampSec)

    def _coast(
        self,
        track: EarthReferencedTrack,
        scan_id: int,
        dt: float,
    ):
        predicted_east, predicted_north = self._predict(track, dt)
        track.EastM = predicted_east
        track.NorthM = predicted_north
        track.PredictedEastM = predicted_east
        track.PredictedNorthM = predicted_north
        track.Misses += 1
        track.LastUpdateScan = scan_id
        if track.LastUpdateTimestampSec is not None:
            track.LastUpdateTimestampSec += dt
        self._append_history(
            track,
            scan_id,
            track.LastUpdateTimestampSec,
        )

    def _predict(self, track: EarthReferencedTrack, dt: float):
        dt = max(float(dt), 0.0)
        return (
            track.EastM + track.VelocityEastMps * dt,
            track.NorthM + track.VelocityNorthMps * dt,
        )

    def _track_dt(
        self,
        track: EarthReferencedTrack,
        scan_id: int,
        timestamp_sec: Optional[float],
    ):
        if (
            timestamp_sec is not None
            and track.LastUpdateTimestampSec is not None
        ):
            timestamp_delta = (
                float(timestamp_sec) - float(track.LastUpdateTimestampSec)
            )
            if timestamp_delta > 1e-6:
                return timestamp_delta
        scan_delta = max(
            1,
            int(scan_id) - int(track.LastUpdateScan),
        )
        return max(1e-6, scan_delta * self.TrackDtSec)

    def _append_history(
        self,
        track: EarthReferencedTrack,
        scan_id: int,
        timestamp_sec: Optional[float],
    ):
        track.History.append(
            (
                scan_id,
                timestamp_sec,
                track.EastM,
                track.NorthM,
            )
        )
        if len(track.History) > self.MaxTrackHistory:
            track.History = track.History[-self.MaxTrackHistory:]

    def _estimate_velocity_from_history(
        self,
        track: EarthReferencedTrack,
    ):
        if len(track.History) < 2:
            return
        scan_2, time_2, east_2, north_2 = track.History[-1]
        for scan_1, time_1, east_1, north_1 in reversed(
            track.History[:-1]
        ):
            scan_delta = int(scan_2) - int(scan_1)
            if scan_delta <= 0:
                continue
            if time_2 is not None and time_1 is not None:
                timestamp_delta = float(time_2) - float(time_1)
            else:
                timestamp_delta = 0.0
            dt = (
                timestamp_delta
                if timestamp_delta > 1e-6
                else max(1e-6, scan_delta * self.TrackDtSec)
            )
            track.VelocityEastMps = (east_2 - east_1) / dt
            track.VelocityNorthMps = (north_2 - north_1) / dt
            return

    def _is_near_existing_track(self, blob: EarthReferencedBlob):
        gate_m = max(self.InitiationGateM, self.AssociationGateM)
        return any(
            _distance(
                blob.EastM,
                blob.NorthM,
                track.EastM,
                track.NorthM,
            )
            <= gate_m
            for track in self.Tracks
        )

    def _best_blob_in_gate(
        self,
        east_m: float,
        north_m: float,
        blobs: Iterable[EarthReferencedBlob],
        used_blob_ids,
        gate_m: float,
    ):
        best = None
        best_distance = float("inf")
        for blob in blobs:
            if blob.BlobId in used_blob_ids:
                continue
            distance_m = _distance(
                blob.EastM,
                blob.NorthM,
                east_m,
                north_m,
            )
            if distance_m <= gate_m and distance_m < best_distance:
                best = blob
                best_distance = distance_m
        return best

    def _cluster_points(
        self,
        points: List[_EarthPoint],
    ) -> List[EarthReferencedBlob]:
        if not points:
            return []

        used = [False] * len(points)
        blobs = []
        for index in range(len(points)):
            if used[index]:
                continue
            stack = [index]
            used[index] = True
            component = []
            while stack:
                point_index = stack.pop()
                component.append(point_index)
                point = points[point_index]
                for candidate_index, candidate in enumerate(points):
                    if used[candidate_index]:
                        continue
                    if (
                        _distance(
                            point.EastM,
                            point.NorthM,
                            candidate.EastM,
                            candidate.NorthM,
                        )
                        <= self.ClusterDistanceM
                    ):
                        used[candidate_index] = True
                        stack.append(candidate_index)

            if len(component) >= self.MinBlobDetections:
                blobs.append(
                    self._make_blob([points[i] for i in component])
                )
        return blobs

    def _make_blob(
        self,
        points: List[_EarthPoint],
    ) -> EarthReferencedBlob:
        east_values = np.array([point.EastM for point in points])
        north_values = np.array([point.NorthM for point in points])
        amplitudes = np.array([point.AmplitudeDb for point in points])
        snrs = np.array([point.SnrDb for point in points])
        dopplers = np.array([point.DopplerHz for point in points])

        blob = EarthReferencedBlob(
            BlobId=self.NextBlobId,
            EastM=float(np.median(east_values)),
            NorthM=float(np.median(north_values)),
            TimestampSec=self._median_optional(
                point.TimestampSec for point in points
            ),
            NumDetections=len(points),
            AmplitudeDb=float(np.max(amplitudes)),
            SnrDb=float(np.max(snrs)),
            DopplerHz=float(np.median(dopplers)),
        )
        self.NextBlobId += 1
        return blob

    def _point_from_detection(
        self,
        detection: Any,
    ) -> Optional[_EarthPoint]:
        if not bool(
            _field(
                detection,
                ["EarthReferenceValid", "earth_reference_valid"],
                False,
            )
        ):
            return None

        east_m = _finite(
            _field(detection, ["EarthEastM", "earth_east_m"])
        )
        north_m = _finite(
            _field(detection, ["EarthNorthM", "earth_north_m"])
        )
        if east_m is None or north_m is None:
            return None

        return _EarthPoint(
            EastM=east_m,
            NorthM=north_m,
            TimestampSec=_finite(
                _field(
                    detection,
                    [
                        "NavigationPoseTimestampSec",
                        "TimeStamp",
                        "TimestampSec",
                        "timestamp_sec",
                    ],
                )
            ),
            AmplitudeDb=float(
                _field(
                    detection,
                    ["AmplitudeDb", "amplitude_db"],
                    -120.0,
                )
            ),
            SnrDb=float(
                _field(detection, ["SnrDb", "SNRDb", "snr_db"], 0.0)
            ),
            DopplerHz=float(
                _field(
                    detection,
                    ["DopplerHz", "doppler_hz"],
                    0.0,
                )
            ),
        )

    def _completed_scan_timestamp(
        self,
        blobs: Iterable[EarthReferencedBlob],
    ) -> Optional[float]:
        return self._median_optional(
            blob.TimestampSec for blob in blobs
        )

    def _median_optional(
        self,
        values: Iterable[Optional[float]],
    ) -> Optional[float]:
        finite_values = [
            finite_value
            for value in values
            for finite_value in [_finite(value)]
            if finite_value is not None
        ]
        if not finite_values:
            return None
        return float(np.median(np.array(finite_values, dtype=float)))

    def _scan_id(self, Processed=None, ThisDwell=None):
        names = [
            "ScanCycle",
            "ScanId",
            "ScanID",
            "ScanNumber",
            "ScanPass",
            "SweepIndex",
            "SweepId",
        ]
        for value in (ThisDwell, Processed):
            scan_id = _field(value, names)
            if scan_id is not None:
                return int(float(scan_id))
            for container_name in (
                "Metadata",
                "metadata",
                "Diagnostics",
                "diagnostics",
            ):
                container = _field(value, [container_name])
                scan_id = _field(container, names)
                if scan_id is not None:
                    return int(float(scan_id))

        azimuth = self._azimuth(Processed, ThisDwell)
        if self._last_azimuth_deg is None:
            self._last_azimuth_deg = azimuth
            return self._inferred_scan_id

        delta = (
            (azimuth - self._last_azimuth_deg + 180.0) % 360.0
        ) - 180.0
        direction = 0
        if abs(delta) >= 0.05:
            direction = 1 if delta > 0.0 else -1
        if (
            direction
            and self._last_direction is not None
            and direction != self._last_direction
        ):
            self._inferred_scan_id += 1
        if direction:
            self._last_direction = direction
        self._last_azimuth_deg = azimuth
        return self._inferred_scan_id

    def _azimuth(self, Processed=None, ThisDwell=None):
        names = [
            "AzimuthDeg",
            "CurrentAzimuthDeg",
            "ScanAzimuthDeg",
            "BeamAzimuthDeg",
            "AngleDeg",
            "ScanAngleDeg",
            "AntennaAzimuthDeg",
        ]
        for value in (ThisDwell, Processed):
            azimuth = _field(value, names)
            if azimuth is not None:
                return float(azimuth)
            for container_name in (
                "Metadata",
                "metadata",
                "Diagnostics",
                "diagnostics",
            ):
                azimuth = _field(
                    _field(value, [container_name]),
                    names,
                )
                if azimuth is not None:
                    return float(azimuth)
        return 0.0


def BuildParallelTrackerComparison(
    EarthTracks: Iterable[Any],
    LegacyTracks: Iterable[Any],
    NavigationPose: Any,
) -> ParallelTrackerComparison:
    """Nearest-neighbour position comparison for non-authoritative validation.

    Legacy range/bearing tracks are projected from the current navigation pose
    into mission ENU, then paired one-to-one with the nearest Earth tracks.
    The result is diagnostic only: legacy and Earth track identifiers are not
    assumed to correspond.
    """

    earth_tracks = list(EarthTracks or [])
    legacy_tracks = list(LegacyTracks or [])
    position_valid = bool(
        NavigationPose is not None
        and _field(NavigationPose, ["PositionValid"], False)
    )
    if not position_valid:
        return ParallelTrackerComparison(
            Valid=False,
            Reason="INVALID_NAVIGATION_POSITION",
            EarthConfirmedTrackCount=len(earth_tracks),
            LegacyConfirmedTrackCount=len(legacy_tracks),
            MatchedTrackCount=0,
            UnmatchedEarthTrackCount=len(earth_tracks),
            UnmatchedLegacyTrackCount=len(legacy_tracks),
            MeanPositionSeparationM=None,
            RmsPositionSeparationM=None,
            MaximumPositionSeparationM=None,
            PositionSeparationsM=(),
        )

    radar_position = _field(NavigationPose, ["PositionEnu"])
    legacy_positions = []
    for track in legacy_tracks:
        range_m = _finite(
            _field(track, ["RangeM", "CentreRangeM", "CenterRangeM"])
        )
        bearing_deg = _finite(
            _field(
                track,
                [
                    "AzimuthDeg",
                    "BearingDeg",
                    "CentreAzimuthDeg",
                    "CenterAzimuthDeg",
                ],
            )
        )
        if range_m is None or range_m < 0.0 or bearing_deg is None:
            continue
        position = detection_to_enu_position(
            radar_position=radar_position,
            measured_range_m=range_m,
            true_bearing_deg=bearing_deg,
        )
        legacy_positions.append(
            (float(position.east_m), float(position.north_m))
        )

    earth_positions = []
    for track in earth_tracks:
        east_m = _finite(_field(track, ["EastM", "EarthEastM"]))
        north_m = _finite(_field(track, ["NorthM", "EarthNorthM"]))
        if east_m is not None and north_m is not None:
            earth_positions.append((east_m, north_m))

    candidates = []
    for earth_index, earth_position in enumerate(earth_positions):
        for legacy_index, legacy_position in enumerate(legacy_positions):
            candidates.append(
                (
                    _distance(*earth_position, *legacy_position),
                    earth_index,
                    legacy_index,
                )
            )

    matched_earth = set()
    matched_legacy = set()
    separations = []
    for separation, earth_index, legacy_index in sorted(candidates):
        if earth_index in matched_earth or legacy_index in matched_legacy:
            continue
        matched_earth.add(earth_index)
        matched_legacy.add(legacy_index)
        separations.append(separation)

    mean_separation = (
        sum(separations) / len(separations) if separations else None
    )
    rms_separation = (
        math.sqrt(
            sum(value * value for value in separations)
            / len(separations)
        )
        if separations
        else None
    )
    maximum_separation = max(separations) if separations else None

    return ParallelTrackerComparison(
        Valid=True,
        Reason="OK",
        EarthConfirmedTrackCount=len(earth_tracks),
        LegacyConfirmedTrackCount=len(legacy_tracks),
        MatchedTrackCount=len(separations),
        UnmatchedEarthTrackCount=(
            len(earth_tracks) - len(matched_earth)
        ),
        UnmatchedLegacyTrackCount=(
            len(legacy_tracks) - len(matched_legacy)
        ),
        MeanPositionSeparationM=mean_separation,
        RmsPositionSeparationM=rms_separation,
        MaximumPositionSeparationM=maximum_separation,
        PositionSeparationsM=tuple(separations),
    )
