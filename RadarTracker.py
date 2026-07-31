"""
===============================================================================
ASR Defence X-Band Radar Prototype
RadarTracker_AlphaBeta_v18.py
===============================================================================

Clean blob/median tracker with 2-of-3 initiation and alpha-beta tracking.

Design goal
-----------
Separate the problem into two simple stages:

1. Scan-pass blob formation
   - CFAR cells from one complete scan pass are accumulated.
   - At the end of the scan pass they are clustered in range/azimuth.
   - Each cluster becomes one median-centre blob.
   - This is the same idea that worked in RadarTracker_BlobAccumulator_v5, but
     memory is scan-pass based, not timed/persistent.

2. Dynamic track list with 2-of-3 initiation
   - A new blob starts one yellow TENTATIVE track with Attempts=1, Hits=1.
   - On each following completed scan pass, the track gets exactly one trial:
       * blob in gate     -> Attempts += 1, Hits += 1, update position
       * no blob in gate  -> Attempts += 1, Hits unchanged
   - After Attempts == 3:
       * Hits >= 2 -> CONFIRMED track, red
       * Hits < 2  -> delete tentative track
   - Confirmed tracks are maintained with prediction, gated association, alpha-beta update, coasting, and miss count.

Important
---------
There is NO timed plot persistence and NO display-side blob output by default.
The only normal display objects returned are tracks.

Expected use
------------
    from RadarTracker_Blob2of3Clean_v17 import RadarTracker

    Tracker = RadarTracker({
        "RangeBinM": 15.0,
        "MinBlobCells": 1,
        "ClusterRangeGapBins": 20,
        "ClusterAzimuthGapDeg": 4.0,
        "InitiationRangeGateBins": 20,
        "InitiationAzimuthGateDeg": 4.0,
        "InitiationWindow": 3,
        "InitiationRequiredHits": 2,
    })

    ThisDwell = {"AzimuthDeg": CurrentScanAngleDeg, "ScanCycle": ScanCycle}
    Tracks, _ = Tracker.Update(Detections, Processed, ThisDwell)

Best result requires ScanCycle to be supplied. If not supplied, the tracker will
try to infer scan-pass changes from beam direction reversals.
===============================================================================
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, List, Optional, Dict, Tuple
import math
import time
import numpy as np

TRACKER_VERSION = "blob-2of3-alpha-beta-scanpass-v20"


@dataclass
class _Point:
    RangeM: float
    AzimuthDeg: float
    AmplitudeDb: float = -120.0
    SnrDb: float = 0.0
    DopplerHz: float = 0.0
    RangeBin: Optional[int] = None
    AngleBin: Optional[int] = None
    DopplerBin: Optional[int] = None


@dataclass
class RadarBlob:
    BlobId: int
    RangeM: float
    AzimuthDeg: float
    NumCells: int = 1
    AmplitudeDb: float = -120.0
    SnrDb: float = 0.0
    DopplerHz: float = 0.0
    RangeBin: Optional[int] = None
    AngleBin: Optional[int] = None
    DopplerBin: Optional[int] = None
    Status: str = "BLOB"
    IsConfirmed: bool = False

    @property
    def CentreRangeM(self): return self.RangeM
    @property
    def CenterRangeM(self): return self.RangeM
    @property
    def CentreAzimuthDeg(self): return self.AzimuthDeg
    @property
    def CenterAzimuthDeg(self): return self.AzimuthDeg
    @property
    def BearingDeg(self): return self.AzimuthDeg
    @property
    def AngleDeg(self): return self.AzimuthDeg
    @property
    def SourceDetections(self): return []


@dataclass
class RadarTrack:
    TrackId: int
    RangeM: float
    AzimuthDeg: float
    RangeRateMps: float = 0.0
    AzimuthRateDps: float = 0.0
    PredictedRangeM: float = 0.0
    PredictedAzimuthDeg: float = 0.0
    Status: str = "TENTATIVE"
    IsConfirmed: bool = False
    Hits: int = 1
    Attempts: int = 1
    Misses: int = 0
    LastUpdateScan: int = 0
    LastHitScan: int = 0
    SnrDb: float = 0.0
    AmplitudeDb: float = -120.0
    NumCells: int = 1
    History: List[Tuple[int, float, float]] = field(default_factory=list)
    DirectedUpdateMisses: int = 0
    LastDirectedUpdateHit: Optional[bool] = None

    # Display compatibility aliases
    @property
    def TrackID(self): return self.TrackId
    @property
    def Id(self): return self.TrackId
    @property
    def ID(self): return self.TrackId
    @property
    def BearingDeg(self): return self.AzimuthDeg
    @property
    def AngleDeg(self): return self.AzimuthDeg
    @property
    def CentreRangeM(self): return self.RangeM
    @property
    def CenterRangeM(self): return self.RangeM
    @property
    def CentreAzimuthDeg(self): return self.AzimuthDeg
    @property
    def CenterAzimuthDeg(self): return self.AzimuthDeg
    @property
    def RangeRate(self): return self.RangeRateMps
    @property
    def RangeRateMPerSec(self): return self.RangeRateMps
    @property
    def BearingRateDegPerSec(self): return self.AzimuthRateDps
    @property
    def AngleRateDps(self): return self.AzimuthRateDps
    @property
    def SpeedMps(self): return abs(self.RangeRateMps)
    @property
    def HitCount(self): return self.Hits
    @property
    def AttemptCount(self): return self.Attempts
    @property
    def InitiationHits(self): return self.Hits
    @property
    def InitiationAttempts(self): return self.Attempts
    @property
    def InitiationWindow(self): return 3


class RadarTracker:
    def __init__(self, Config: Optional[Dict[str, Any]] = None):
        self.Config = Config or {}
        self.RangeBinM = float(self.Config.get("RangeBinM", self.Config.get("RangeResolutionM", 15.0)))

        # Blob clustering for ONE scan pass.
        self.ClusterRangeGapBins = int(self.Config.get("ClusterRangeGapBins", 20))
        self.ClusterRangeGapM = float(self.Config.get("ClusterRangeGapM", self.ClusterRangeGapBins * self.RangeBinM))
        self.ClusterAzimuthGapDeg = float(self.Config.get("ClusterAzimuthGapDeg", 4.0))
        self.MinBlobCells = int(self.Config.get("MinBlobCells", 1))
        self.CentreMethod = str(self.Config.get("BlobCentreMethod", "median")).lower()

        # Track gates.
        self.InitiationRangeGateBins = int(self.Config.get("InitiationRangeGateBins", 20))
        self.InitiationRangeGateM = float(self.Config.get("InitiationRangeGateM", self.InitiationRangeGateBins * self.RangeBinM))
        self.InitiationAzimuthGateDeg = float(self.Config.get("InitiationAzimuthGateDeg", 4.0))
        self.AssociationRangeGateBins = int(self.Config.get("AssociationRangeGateBins", self.InitiationRangeGateBins))
        self.AssociationRangeGateM = float(self.Config.get("AssociationRangeGateM", self.AssociationRangeGateBins * self.RangeBinM))
        self.AssociationAzimuthGateDeg = float(self.Config.get("AssociationAzimuthGateDeg", self.InitiationAzimuthGateDeg))

        # 2-of-3 initiation.
        self.InitiationWindow = int(self.Config.get("InitiationWindow", 3))
        self.InitiationRequiredHits = int(self.Config.get("InitiationRequiredHits", 2))
        self.DeleteConfirmedAfterMisses = int(self.Config.get("DeleteConfirmedAfterMisses", 5))
        self.DuplicateSuppressionEnabled = bool(
            self.Config.get("DuplicateTrackSuppressionEnabled", True)
        )
        self.DuplicateRangeGateM = float(
            self.Config.get("DuplicateTrackRangeGateM", 200.0)
        )
        self.DuplicateAzimuthGateDeg = float(
            self.Config.get("DuplicateTrackAzimuthGateDeg", 5.0)
        )
        self.DirectedDeleteAfterMisses = int(
            self.Config.get("DirectedTrackDeleteAfterMisses", 2)
        )

        # Confirmed track filter.  This is scan-to-scan alpha-beta tracking.
        # TrackDtSec is the assumed time between completed scan passes.  If the
        # main program later supplies a real scan period, set it in Config.
        self.TrackDtSec = float(self.Config.get("TrackDtSec", self.Config.get("ScanPeriodSec", 1.0)))
        self.TrackAlpha = float(self.Config.get("TrackAlpha", 0.65))
        self.TrackBeta = float(self.Config.get("TrackBeta", 0.20))
        self.MaxTrackHistory = int(self.Config.get("MaxTrackHistory", 20))

        self.ReturnBlobsForDebug = bool(self.Config.get("ReturnBlobsForDebug", False))

        self.Tracks: List[RadarTrack] = []
        self.CurrentScanId: Optional[int] = None
        self.CurrentScanPoints: List[_Point] = []
        self.LastCompletedBlobs: List[RadarBlob] = []

        self.NextTrackId = 1
        self.NextBlobId = 1

        # Fallback scan-pass inference if ScanCycle is not provided.
        self._inferred_scan_id = 0
        self._last_az: Optional[float] = None
        self._last_dir: Optional[int] = None

        self.LastDebug: Dict[str, Any] = {"TrackerVersion": TRACKER_VERSION}
        self.LastDuplicateTracksMerged = 0

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------
    def Update(self, Detections, Processed=None, ThisDwell=None):
        scan_id = self._scan_id(Processed, ThisDwell)

        if self.CurrentScanId is None:
            self.CurrentScanId = scan_id

        # New scan/pass: turn completed scan points into blobs and update tracks.
        if scan_id != self.CurrentScanId:
            completed_blobs = self._cluster_points(self.CurrentScanPoints)
            self.LastCompletedBlobs = completed_blobs
            self._process_completed_scan(completed_blobs, self.CurrentScanId)
            self.CurrentScanPoints = []
            self.CurrentScanId = scan_id

        # Add this dwell's detections to the CURRENT scan pass only.
        dets = list(Detections) if Detections is not None else []
        for d in dets:
            self.CurrentScanPoints.append(self._point_from_detection(d, Processed, ThisDwell))

        self.LastDebug = {
            "TrackerVersion": TRACKER_VERSION,
            "CurrentScanId": self.CurrentScanId,
            "RawDetectionsThisDwell": len(dets),
            "CurrentScanPoints": len(self.CurrentScanPoints),
            "LastCompletedBlobs": len(self.LastCompletedBlobs),
            "TentativeTracks": len([t for t in self.Tracks if t.Status != "CONFIRMED"]),
            "ConfirmedTracks": len([t for t in self.Tracks if t.Status == "CONFIRMED"]),
            "TotalTracks": len(self.Tracks),
            "InitiationWindow": self.InitiationWindow,
            "InitiationRequiredHits": self.InitiationRequiredHits,
            "ReturnBlobsForDebug": self.ReturnBlobsForDebug,
            "TrackAlpha": self.TrackAlpha,
            "TrackBeta": self.TrackBeta,
            "TrackDtSec": self.TrackDtSec,
            "DuplicateTracksMerged": self.LastDuplicateTracksMerged,
        }

        return self.GetTracks(), (self.LastCompletedBlobs if self.ReturnBlobsForDebug else [])

    def GetTracks(self):
        return list(self.Tracks)

    def GetTentativeTracks(self):
        return [t for t in self.Tracks if t.Status != "CONFIRMED"]

    def GetConfirmedTracks(self):
        return [t for t in self.Tracks if t.Status == "CONFIRMED"]

    def GetRecentPlots(self, CurrentTimeStamp=None):
        return []

    def GetRecentBlobs(self, CurrentTimeStamp=None):
        return self.LastCompletedBlobs if self.ReturnBlobsForDebug else []

    def GetDebugInfo(self):
        return dict(self.LastDebug)

    def DeleteTrack(self, track_id: int) -> bool:
        track_id = int(track_id)
        previous_count = len(self.Tracks)
        self.Tracks = [
            track for track in self.Tracks
            if int(track.TrackId) != track_id
        ]
        return len(self.Tracks) != previous_count

    def ApplyDirectedUpdate(
        self,
        track_id: int,
        measurements,
        *,
        coverage_valid: bool,
        miss_policy: str = "RETRY_WIDER_THEN_DELETE",
    ) -> Dict[str, Any]:
        """Apply one completed nod scan directly to its nominated track.

        Track confirmation is a finite, target-directed opportunity rather
        than a completed search pass.  Feeding it through the scan accumulator
        would delay the result and could seed a second track, so the nominated
        track is updated explicitly here.
        """

        track = next(
            (
                item for item in self.Tracks
                if int(item.TrackId) == int(track_id)
            ),
            None,
        )
        if track is None:
            return {
                "TrackId": int(track_id),
                "Outcome": "TRACK_NOT_FOUND",
                "Hit": False,
                "Deleted": False,
                "CoverageValid": bool(coverage_valid),
            }

        was_tentative = not (
            str(track.Status).upper() == "CONFIRMED"
            or bool(track.IsConfirmed)
        )

        blobs = []
        for measurement in list(measurements or []):
            point = self._point_from_detection(measurement)
            blobs.append(RadarBlob(
                BlobId=self.NextBlobId,
                RangeM=point.RangeM,
                AzimuthDeg=point.AzimuthDeg,
                NumCells=int(self._field(
                    measurement,
                    ["NumCells"],
                    1,
                )),
                AmplitudeDb=point.AmplitudeDb,
                SnrDb=point.SnrDb,
                DopplerHz=point.DopplerHz,
                RangeBin=point.RangeBin,
            ))
            self.NextBlobId += 1

        range_gate_m = (
            self.InitiationRangeGateM
            if was_tentative
            else self.AssociationRangeGateM
        )
        azimuth_gate_deg = (
            self.InitiationAzimuthGateDeg
            if was_tentative
            else self.AssociationAzimuthGateDeg
        )
        best = self._best_blob_in_gate(
            track.RangeM,
            track.AzimuthDeg,
            blobs,
            set(),
            range_gate_m,
            azimuth_gate_deg,
        )
        if was_tentative:
            return self._apply_directed_tentative_update(
                track,
                best,
                coverage_valid=coverage_valid,
            )

        if best is not None:
            update_scan = max(
                int(track.LastUpdateScan) + 1,
                int(self.CurrentScanId or 0),
            )
            self._alpha_beta_update_from_blob(
                track,
                best,
                update_scan,
                self.TrackDtSec,
            )
            track.Misses = 0
            track.DirectedUpdateMisses = 0
            track.LastDirectedUpdateHit = True
            merged = self._merge_duplicates_around_track(track)
            self.LastDuplicateTracksMerged += merged
            return {
                "TrackId": int(track_id),
                "Outcome": "CONFIRMED",
                "Hit": True,
                "Deleted": False,
                "CoverageValid": bool(coverage_valid),
                "RangeM": float(track.RangeM),
                "AzimuthDeg": float(track.AzimuthDeg),
                "DuplicatesMerged": int(merged),
            }

        if not coverage_valid:
            track.LastDirectedUpdateHit = None
            return {
                "TrackId": int(track_id),
                "Outcome": "INCOMPLETE_COVERAGE_COAST",
                "Hit": False,
                "Deleted": False,
                "CoverageValid": False,
            }

        track.LastDirectedUpdateHit = False
        track.DirectedUpdateMisses += 1
        track.Misses += 1
        policy = str(miss_policy).upper()
        delete_requested = policy in (
            "DELETE",
            "RETRY_WIDER_THEN_DELETE",
        )
        deleted = bool(
            delete_requested
            and (
                policy == "DELETE"
                or track.DirectedUpdateMisses
                >= self.DirectedDeleteAfterMisses
            )
        )
        if deleted:
            self.DeleteTrack(track.TrackId)
        return {
            "TrackId": int(track_id),
            "Outcome": (
                "DELETED_AFTER_CONFIRMED_MISSES"
                if deleted
                else "MISSED_COASTING"
            ),
            "Hit": False,
            "Deleted": deleted,
            "CoverageValid": True,
            "DirectedUpdateMisses": int(track.DirectedUpdateMisses),
        }

    def _apply_directed_tentative_update(
        self,
        track: RadarTrack,
        best: Optional[RadarBlob],
        *,
        coverage_valid: bool,
    ) -> Dict[str, Any]:
        """Apply one complete three-pass nod as one initiation opportunity."""

        common = {
            "TrackId": int(track.TrackId),
            "TrackWasTentative": True,
            "CoverageValid": bool(coverage_valid),
        }
        if not coverage_valid:
            track.LastDirectedUpdateHit = None
            return {
                **common,
                "Outcome": "TENTATIVE_INCOMPLETE_COVERAGE",
                "Hit": False,
                "Promoted": False,
                "Deleted": False,
                "Hits": int(track.Hits),
                "Attempts": int(track.Attempts),
                "InitiationWindow": int(self.InitiationWindow),
            }

        # Regardless of the three physical gate crossings, this finite task is
        # one tracker opportunity.
        update_scan = max(
            int(track.LastUpdateScan) + 1,
            int(self.CurrentScanId or 0),
        )
        track.Attempts += 1
        track.LastUpdateScan = update_scan
        track.LastDirectedUpdateHit = best is not None

        if best is not None:
            track.Hits += 1
            track.LastHitScan = update_scan
            track.Misses = 0
            self._update_tentative_from_blob(
                track,
                best,
                update_scan,
            )
        else:
            track.Misses += 1

        promoted = bool(
            track.Hits >= self.InitiationRequiredHits
        )
        deleted = bool(
            not promoted
            and track.Attempts >= self.InitiationWindow
        )
        merged = 0
        if promoted:
            track.Status = "CONFIRMED"
            track.IsConfirmed = True
            track.Misses = 0
            track.DirectedUpdateMisses = 0
            self._estimate_velocity_from_history(track)
            merged = self._merge_duplicates_around_track(track)
            self.LastDuplicateTracksMerged += merged
        elif deleted:
            self.DeleteTrack(track.TrackId)

        if promoted:
            outcome = "TENTATIVE_PROMOTED"
        elif deleted:
            outcome = "TENTATIVE_DELETED"
        elif best is not None:
            outcome = "TENTATIVE_REACQUIRED"
        else:
            outcome = "TENTATIVE_MISSED_RETAINED"

        return {
            **common,
            "Outcome": outcome,
            "Hit": best is not None,
            "Promoted": promoted,
            "Deleted": deleted,
            "Hits": int(track.Hits),
            "Attempts": int(track.Attempts),
            "InitiationWindow": int(self.InitiationWindow),
            "RangeM": float(track.RangeM),
            "AzimuthDeg": float(track.AzimuthDeg),
            "DuplicatesMerged": int(merged),
        }

    # ------------------------------------------------------------------
    # Scan-level tracking
    # ------------------------------------------------------------------
    def _process_completed_scan(self, blobs: List[RadarBlob], scan_id: int):
        """Update the track list once for one completed scan/pass.

        Confirmed tracks are handled first using prediction and alpha-beta
        update.  Any blobs not consumed by confirmed tracks then feed the
        2-of-3 tentative initiation logic.  The public return remains the same:
        GetTracks() returns the latest track states for the display.
        """
        used_blob_ids = set()
        surviving: List[RadarTrack] = []

        # --------------------------------------------------------------
        # 1) Confirmed tracks: predict -> gate -> associate -> update/coast
        # --------------------------------------------------------------
        confirmed = [t for t in self.Tracks if t.Status == "CONFIRMED"]
        tentative = [t for t in self.Tracks if t.Status != "CONFIRMED"]

        for trk in confirmed:
            dt = self._track_dt(trk, scan_id)
            pred_r, pred_a = self._predict_track_position(trk, dt)
            trk.PredictedRangeM = pred_r
            trk.PredictedAzimuthDeg = pred_a

            blob = self._best_blob_in_gate(
                pred_r, pred_a, blobs, used_blob_ids,
                self.AssociationRangeGateM, self.AssociationAzimuthGateDeg
            )

            if blob is not None:
                used_blob_ids.add(blob.BlobId)
                self._alpha_beta_update_from_blob(trk, blob, scan_id, dt)
                trk.Misses = 0
                surviving.append(trk)
            else:
                self._coast_confirmed_track(trk, scan_id, dt)
                if trk.Misses <= self.DeleteConfirmedAfterMisses:
                    surviving.append(trk)

        # --------------------------------------------------------------
        # 2) Tentative tracks: classic 2-of-3 initiation using remaining blobs
        # --------------------------------------------------------------
        for trk in tentative:
            blob = self._best_blob_in_gate(
                trk.RangeM, trk.AzimuthDeg, blobs, used_blob_ids,
                self.InitiationRangeGateM, self.InitiationAzimuthGateDeg
            )

            # Every completed scan after birth is one attempt.
            # Avoid double-counting the birth scan if it was just created from
            # that completed scan.
            if trk.LastUpdateScan == scan_id:
                surviving.append(trk)
                continue

            trk.Attempts += 1
            trk.LastUpdateScan = scan_id

            if blob is not None:
                used_blob_ids.add(blob.BlobId)
                trk.Hits += 1
                trk.LastHitScan = scan_id
                self._update_tentative_from_blob(trk, blob, scan_id)
            else:
                trk.Misses += 1

            if trk.Attempts >= self.InitiationWindow:
                if trk.Hits >= self.InitiationRequiredHits:
                    trk.Status = "CONFIRMED"
                    trk.IsConfirmed = True
                    trk.Misses = 0
                    self._estimate_velocity_from_history(trk)
                    surviving.append(trk)
                # else: delete tentative by not appending
            else:
                surviving.append(trk)

        self.Tracks = surviving

        # --------------------------------------------------------------
        # 3) Create new tentative tracks from unused blobs.
        # --------------------------------------------------------------
        for b in blobs:
            if b.BlobId in used_blob_ids:
                continue
            if self._is_near_existing_track(b):
                continue
            self._create_tentative(b, scan_id)

        self.LastDuplicateTracksMerged = self._merge_duplicate_tracks(scan_id)

    def _merge_duplicate_tracks(self, scan_id: int) -> int:
        if not self.DuplicateSuppressionEnabled:
            return 0
        merged = 0
        changed = True
        while changed:
            changed = False
            for left_index, left in enumerate(self.Tracks):
                for right in self.Tracks[left_index + 1:]:
                    if not self._tracks_are_duplicates(
                        left,
                        right,
                        scan_id,
                    ):
                        continue
                    keep, discard = self._preferred_duplicate_track(
                        left,
                        right,
                    )
                    self._merge_track_state(keep, discard)
                    self.Tracks = [
                        item for item in self.Tracks if item is not discard
                    ]
                    merged += 1
                    changed = True
                    break
                if changed:
                    break
        return merged

    def _merge_duplicates_around_track(self, nominated: RadarTrack) -> int:
        merged = 0
        for other in list(self.Tracks):
            if other is nominated:
                continue
            if (
                abs(other.RangeM - nominated.RangeM)
                > self.DuplicateRangeGateM
                or self._az_diff(
                    other.AzimuthDeg,
                    nominated.AzimuthDeg,
                ) > self.DuplicateAzimuthGateDeg
            ):
                continue
            self._merge_track_state(nominated, other)
            self.Tracks = [
                item for item in self.Tracks if item is not other
            ]
            merged += 1
        return merged

    def _tracks_are_duplicates(
        self,
        left: RadarTrack,
        right: RadarTrack,
        scan_id: int,
    ) -> bool:
        if (
            abs(left.RangeM - right.RangeM) > self.DuplicateRangeGateM
            or self._az_diff(
                left.AzimuthDeg,
                right.AzimuthDeg,
            ) > self.DuplicateAzimuthGateDeg
        ):
            return False
        # Two confirmed tracks independently updated by distinct plots in the
        # same pass may be two genuinely close vessels.  Preserve that case.
        if (
            left.Status == "CONFIRMED"
            and right.Status == "CONFIRMED"
            and left.LastHitScan == scan_id
            and right.LastHitScan == scan_id
        ):
            return False
        return True

    @staticmethod
    def _preferred_duplicate_track(left, right):
        def score(track):
            return (
                1 if track.Status == "CONFIRMED" else 0,
                int(track.Hits),
                -int(track.Misses),
                -int(track.TrackId),
            )
        return (left, right) if score(left) >= score(right) else (right, left)

    def _merge_track_state(self, keep: RadarTrack, discard: RadarTrack):
        total_hits = max(1, int(keep.Hits) + int(discard.Hits))
        discard_weight = int(discard.Hits) / float(total_hits)
        keep.RangeM = (
            (1.0 - discard_weight) * keep.RangeM
            + discard_weight * discard.RangeM
        )
        keep.AzimuthDeg = self._az_blend(
            keep.AzimuthDeg,
            discard.AzimuthDeg,
            discard_weight,
        )
        keep.Hits = max(int(keep.Hits), int(discard.Hits))
        keep.Attempts = max(int(keep.Attempts), int(discard.Attempts))
        keep.Misses = min(int(keep.Misses), int(discard.Misses))
        keep.SnrDb = max(float(keep.SnrDb), float(discard.SnrDb))
        keep.AmplitudeDb = max(
            float(keep.AmplitudeDb),
            float(discard.AmplitudeDb),
        )
        keep.NumCells = max(int(keep.NumCells), int(discard.NumCells))
        keep.History = (
            keep.History + discard.History
        )[-self.MaxTrackHistory:]

    def _create_tentative(self, blob: RadarBlob, scan_id: int):
        trk = RadarTrack(
            TrackId=self.NextTrackId,
            RangeM=blob.RangeM,
            AzimuthDeg=blob.AzimuthDeg,
            RangeRateMps=0.0,
            AzimuthRateDps=0.0,
            PredictedRangeM=blob.RangeM,
            PredictedAzimuthDeg=blob.AzimuthDeg,
            Status="TENTATIVE",
            IsConfirmed=False,
            Hits=1,
            Attempts=1,
            Misses=0,
            LastUpdateScan=scan_id,
            LastHitScan=scan_id,
            SnrDb=blob.SnrDb,
            AmplitudeDb=blob.AmplitudeDb,
            NumCells=blob.NumCells,
            History=[(scan_id, blob.RangeM, blob.AzimuthDeg)],
        )
        self.NextTrackId += 1
        self.Tracks.append(trk)

    def _update_tentative_from_blob(self, trk: RadarTrack, blob: RadarBlob, scan_id: int):
        """Update a tentative track during 2-of-3 initiation.

        Tentative tracks deliberately stay measurement-led.  Once they promote
        to CONFIRMED, velocity is estimated from the last two history points and
        then alpha-beta tracking takes over.
        """
        alpha = 0.85
        trk.RangeM = alpha * blob.RangeM + (1.0 - alpha) * trk.RangeM
        trk.AzimuthDeg = self._az_blend(trk.AzimuthDeg, blob.AzimuthDeg, alpha)
        trk.PredictedRangeM = trk.RangeM
        trk.PredictedAzimuthDeg = trk.AzimuthDeg
        trk.SnrDb = blob.SnrDb
        trk.AmplitudeDb = blob.AmplitudeDb
        trk.NumCells = blob.NumCells
        trk.LastUpdateScan = scan_id
        self._append_history(trk, scan_id)

    def _alpha_beta_update_from_blob(self, trk: RadarTrack, blob: RadarBlob, scan_id: int, dt: float):
        """Confirmed-track alpha-beta update in range and azimuth."""
        dt = max(float(dt), 1e-6)

        pred_r, pred_a = self._predict_track_position(trk, dt)
        dr = blob.RangeM - pred_r
        da = self._signed_az_delta(blob.AzimuthDeg, pred_a)

        trk.RangeM = pred_r + self.TrackAlpha * dr
        trk.AzimuthDeg = (pred_a + self.TrackAlpha * da) % 360.0
        trk.RangeRateMps = trk.RangeRateMps + self.TrackBeta * dr / dt
        trk.AzimuthRateDps = trk.AzimuthRateDps + self.TrackBeta * da / dt

        trk.PredictedRangeM = pred_r
        trk.PredictedAzimuthDeg = pred_a
        trk.SnrDb = blob.SnrDb
        trk.AmplitudeDb = blob.AmplitudeDb
        trk.NumCells = blob.NumCells
        trk.LastUpdateScan = scan_id
        trk.LastHitScan = scan_id
        trk.Hits += 1
        self._append_history(trk, scan_id)

    def _coast_confirmed_track(self, trk: RadarTrack, scan_id: int, dt: float):
        """Move a confirmed track using velocity when there is no blob hit."""
        pred_r, pred_a = self._predict_track_position(trk, dt)
        trk.RangeM = max(0.0, pred_r)
        trk.AzimuthDeg = pred_a
        trk.PredictedRangeM = pred_r
        trk.PredictedAzimuthDeg = pred_a
        trk.Misses += 1
        trk.LastUpdateScan = scan_id
        self._append_history(trk, scan_id)

    def _predict_track_position(self, trk: RadarTrack, dt: float):
        dt = max(float(dt), 0.0)
        pred_r = max(0.0, trk.RangeM + trk.RangeRateMps * dt)
        pred_a = (trk.AzimuthDeg + trk.AzimuthRateDps * dt) % 360.0
        return pred_r, pred_a

    def _track_dt(self, trk: RadarTrack, scan_id: int):
        # One alpha-beta update is applied once per completed scan pass.  If
        # scan ids skip, multiply by the configured scan period.
        dscan = max(1, int(scan_id) - int(trk.LastUpdateScan))
        return max(1e-6, dscan * self.TrackDtSec)

    def _append_history(self, trk: RadarTrack, scan_id: int):
        trk.History.append((scan_id, trk.RangeM, trk.AzimuthDeg))
        if len(trk.History) > self.MaxTrackHistory:
            trk.History = trk.History[-self.MaxTrackHistory:]

    def _estimate_velocity_from_history(self, trk: RadarTrack):
        # Use the last two distinct scan-history points to seed confirmed-track
        # velocity.  If unavailable, keep zero velocity.
        if len(trk.History) < 2:
            return
        s2, r2, a2 = trk.History[-1]
        for s1, r1, a1 in reversed(trk.History[:-1]):
            dscan = int(s2) - int(s1)
            if dscan <= 0:
                continue
            dt = max(1e-6, dscan * self.TrackDtSec)
            trk.RangeRateMps = (r2 - r1) / dt
            trk.AzimuthRateDps = self._signed_az_delta(a2, a1) / dt
            return

    def _is_near_existing_track(self, blob: RadarBlob):
        for trk in self.Tracks:
            gate_r = max(self.InitiationRangeGateM, self.AssociationRangeGateM)
            gate_a = max(self.InitiationAzimuthGateDeg, self.AssociationAzimuthGateDeg)
            if abs(blob.RangeM - trk.RangeM) <= gate_r and self._az_diff(blob.AzimuthDeg, trk.AzimuthDeg) <= gate_a:
                return True
        return False

    def _best_blob_in_gate(self, range_m, az_deg, blobs, used_ids, gate_r_m, gate_a_deg):
        best = None
        best_score = float("inf")
        for b in blobs:
            if b.BlobId in used_ids:
                continue
            dr = abs(b.RangeM - range_m)
            da = self._az_diff(b.AzimuthDeg, az_deg)
            if dr <= gate_r_m and da <= gate_a_deg:
                score = (dr / max(gate_r_m, 1e-9)) ** 2 + (da / max(gate_a_deg, 1e-9)) ** 2
                if score < best_score:
                    best_score = score
                    best = b
        return best

    # ------------------------------------------------------------------
    # Blob clustering over one completed scan pass
    # ------------------------------------------------------------------
    def _cluster_points(self, points: List[_Point]) -> List[RadarBlob]:
        if not points:
            return []
        n = len(points)
        used = [False] * n
        blobs: List[RadarBlob] = []

        for i in range(n):
            if used[i]:
                continue
            stack = [i]
            used[i] = True
            comp = []
            while stack:
                idx = stack.pop()
                comp.append(idx)
                p = points[idx]
                for j in range(n):
                    if used[j]:
                        continue
                    q = points[j]
                    if abs(q.RangeM - p.RangeM) <= self.ClusterRangeGapM and self._az_diff(q.AzimuthDeg, p.AzimuthDeg) <= self.ClusterAzimuthGapDeg:
                        used[j] = True
                        stack.append(j)

            if len(comp) < self.MinBlobCells:
                continue
            blobs.append(self._make_blob([points[k] for k in comp]))

        return blobs

    def _make_blob(self, pts: List[_Point]) -> RadarBlob:
        ranges = np.array([p.RangeM for p in pts], dtype=float)
        azs = np.array([p.AzimuthDeg for p in pts], dtype=float)
        amps = np.array([p.AmplitudeDb for p in pts], dtype=float)
        snrs = np.array([p.SnrDb for p in pts], dtype=float)
        dops = np.array([p.DopplerHz for p in pts], dtype=float)

        if self.CentreMethod == "mean":
            r = float(np.mean(ranges))
            az = self._circular_mean_deg(azs)
        else:
            r = float(np.median(ranges))
            az = self._circular_median_deg(azs)

        rb = int(round(r / max(self.RangeBinM, 1e-9)))
        b = RadarBlob(
            BlobId=self.NextBlobId,
            RangeM=r,
            AzimuthDeg=az,
            NumCells=len(pts),
            AmplitudeDb=float(np.max(amps)) if len(amps) else -120.0,
            SnrDb=float(np.max(snrs)) if len(snrs) else 0.0,
            DopplerHz=float(np.median(dops)) if len(dops) else 0.0,
            RangeBin=rb,
        )
        self.NextBlobId += 1
        return b

    # ------------------------------------------------------------------
    # Detection extraction
    # ------------------------------------------------------------------
    def _point_from_detection(self, d, Processed=None, ThisDwell=None):
        r = self._range_m(d)
        az = self._azimuth_deg(d, Processed, ThisDwell)
        rb = self._field(d, ["RangeBin", "range_bin", "rbin", "BinRange"], None)
        ab = self._field(d, ["AngleBin", "AzimuthBin", "angle_bin", "az_bin"], None)
        db = self._field(d, ["DopplerBin", "doppler_bin", "dbin", "BinDoppler"], None)
        return _Point(
            RangeM=float(r),
            AzimuthDeg=float(az) % 360.0,
            AmplitudeDb=float(self._field(d, ["AmplitudeDb", "amplitude_db", "PowerDb", "power_db", "MagDb", "mag_db"], -120.0)),
            SnrDb=float(self._field(d, ["SnrDb", "SNRDb", "snr_db", "SNR"], 10.0)),
            DopplerHz=float(self._field(d, ["DopplerHz", "doppler_hz", "Doppler", "doppler"], 0.0)),
            RangeBin=int(rb) if rb is not None else int(round(float(r) / max(self.RangeBinM, 1e-9))),
            AngleBin=int(ab) if ab is not None else None,
            DopplerBin=int(db) if db is not None else None,
        )

    def _range_m(self, d):
        r = self._field(d, ["RangeM", "range_m", "Range", "range"], None)
        if r is not None:
            return float(r)
        rb = self._field(d, ["RangeBin", "range_bin", "rbin", "BinRange"], 0)
        return float(rb) * self.RangeBinM

    def _azimuth_deg(self, d, Processed=None, ThisDwell=None):
        az = self._field(d, ["AzimuthDeg", "azimuth_deg", "BearingDeg", "bearing_deg", "AngleDeg", "angle_deg"], None)
        if az is not None:
            return float(az)
        for obj in (ThisDwell, Processed):
            az = self._field(obj, ["AzimuthDeg", "CurrentAzimuthDeg", "ScanAzimuthDeg", "BeamAzimuthDeg", "AngleDeg", "ScanAngleDeg", "AntennaAzimuthDeg"], None)
            if az is not None:
                return float(az)
            diag = self._field(obj, ["Diagnostics", "diagnostics"], None)
            az = self._field(diag, ["AzimuthDeg", "CurrentAzimuthDeg", "ScanAzimuthDeg", "BeamAzimuthDeg", "AngleDeg", "ScanAngleDeg", "AntennaAzimuthDeg"], None)
            if az is not None:
                return float(az)
        return 0.0

    # ------------------------------------------------------------------
    # Scan id extraction / inference
    # ------------------------------------------------------------------
    def _scan_id(self, Processed=None, ThisDwell=None):
        for obj in (ThisDwell, Processed):
            sid = self._field(obj, ["ScanCycle", "ScanId", "ScanID", "ScanNumber", "ScanPass", "SweepIndex", "SweepId"], None)
            if sid is not None:
                try:
                    return int(sid)
                except Exception:
                    return int(float(sid))
            diag = self._field(obj, ["Diagnostics", "diagnostics"], None)
            sid = self._field(diag, ["ScanCycle", "ScanId", "ScanID", "ScanNumber", "ScanPass", "SweepIndex", "SweepId"], None)
            if sid is not None:
                try:
                    return int(sid)
                except Exception:
                    return int(float(sid))

        az = self._azimuth_deg(None, Processed, ThisDwell)
        if self._last_az is None:
            self._last_az = az
            return self._inferred_scan_id

        delta = self._signed_az_delta(az, self._last_az)
        direction = 0
        if abs(delta) >= 0.05:
            direction = 1 if delta > 0 else -1

        if direction != 0 and self._last_dir is not None and direction != self._last_dir:
            self._inferred_scan_id += 1

        if direction != 0:
            self._last_dir = direction
        self._last_az = az
        return self._inferred_scan_id

    # ------------------------------------------------------------------
    # Generic helpers
    # ------------------------------------------------------------------
    def _field(self, obj, names, default=None):
        if obj is None:
            return default
        for n in names:
            if isinstance(obj, dict) and n in obj:
                return obj[n]
            if hasattr(obj, n):
                return getattr(obj, n)
        return default

    def _az_diff(self, a, b):
        return abs(((float(a) - float(b) + 180.0) % 360.0) - 180.0)

    def _signed_az_delta(self, a, b):
        return ((float(a) - float(b) + 180.0) % 360.0) - 180.0

    def _az_blend(self, old, new, alpha):
        # Blend shortest way around circle.
        delta = self._signed_az_delta(new, old)
        return (old + alpha * delta) % 360.0

    def _circular_mean_deg(self, degs):
        if len(degs) == 0:
            return 0.0
        rad = np.deg2rad(degs)
        return float(np.rad2deg(math.atan2(np.mean(np.sin(rad)), np.mean(np.cos(rad)))) % 360.0)

    def _circular_median_deg(self, degs):
        if len(degs) == 0:
            return 0.0
        # For small sector scans not crossing 0, normal median is best.
        degs = np.array(degs, dtype=float) % 360.0
        if np.ptp(degs) <= 180.0:
            return float(np.median(degs)) % 360.0
        # unwrap around circular mean for generality.
        centre = self._circular_mean_deg(degs)
        unwrapped = np.array([centre + self._signed_az_delta(d, centre) for d in degs])
        return float(np.median(unwrapped) % 360.0)


# Compatibility aliases used by some older code.
Track = RadarTrack
DetectionBlob = RadarBlob
DisplayBlob = RadarBlob
RadarPlot = RadarBlob


if __name__ == "__main__":
    # Tiny self-test: one false alarm dies after three completed scans; a repeated
    # blob confirms after the third completed scan.
    tr = RadarTracker({"RangeBinM": 15.0, "MinBlobCells": 1})
    def det(r, az):
        return {"RangeM": r, "AzimuthDeg": az, "SnrDb": 20.0}
    # scan 0 completed at transition to scan 1 -> create tentative 1/3
    tr.Update([det(1000, 10)], ThisDwell={"ScanCycle": 0, "AzimuthDeg": 10})
    tr.Update([], ThisDwell={"ScanCycle": 1, "AzimuthDeg": -60})
    assert len(tr.GetTentativeTracks()) == 1 and tr.GetTentativeTracks()[0].Hits == 1
    # scan 1 no hit -> attempts 2, still tentative
    tr.Update([], ThisDwell={"ScanCycle": 2, "AzimuthDeg": -60})
    assert len(tr.GetTentativeTracks()) == 1 and tr.GetTentativeTracks()[0].Attempts == 2
    # scan 2 no hit -> attempts 3, hits 1 -> deleted
    tr.Update([], ThisDwell={"ScanCycle": 3, "AzimuthDeg": -60})
    assert len(tr.GetTracks()) == 0
    print("Self-test passed", TRACKER_VERSION)
