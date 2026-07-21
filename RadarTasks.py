"""
===============================================================================
Vanguard X Radar Task Definitions
RadarTasks.py
===============================================================================

Purpose
-------
Defines the high-level task objects used by RadarScheduler.

A RadarTask describes WHAT the radar should do next. It does not control the
X6-60, Ettus, TRM, detector, tracker, or display directly.

Initial task types
------------------
    SEARCH
    TRACK
    CALIBRATION
    PASSIVE_LISTEN

The first implementation will actively use SEARCH and TRACK. The remaining task
types are included now so later scheduler work does not require another public
interface change.

Coordinate-frame principle
--------------------------
Search sectors and pointing requests explicitly declare whether angles are:

    TRUE       relative to true North
    PLATFORM   relative to the platform/body frame

This is important for future moving-platform operation.
===============================================================================
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, Optional
import time


class RadarTaskType(str, Enum):
    SEARCH = "SEARCH"
    TRACK = "TRACK"
    CALIBRATION = "CALIBRATION"
    PASSIVE_LISTEN = "PASSIVE_LISTEN"


class TaskStatus(str, Enum):
    QUEUED = "QUEUED"
    ACTIVE = "ACTIVE"
    PAUSED = "PAUSED"
    COMPLETE = "COMPLETE"
    CANCELLED = "CANCELLED"
    FAILED = "FAILED"


class PointingMode(str, Enum):
    CONTINUOUS_SCAN = "CONTINUOUS_SCAN"
    GOTO_AND_HOLD = "GOTO_AND_HOLD"
    HOLD_CURRENT = "HOLD_CURRENT"
    NO_POINTING = "NO_POINTING"


class AngleFrame(str, Enum):
    TRUE = "TRUE"
    PLATFORM = "PLATFORM"


class SearchPattern(str, Enum):
    """Azimuth movement used by a persistent search task."""

    SECTOR = "SECTOR"
    CONTINUOUS_CW = "CONTINUOUS_CW"


@dataclass
class PointingRequest:
    """
    Pointing intent attached to a radar task.

    Search normally uses CONTINUOUS_SCAN.
    Track normally uses GOTO_AND_HOLD.

    For a moving target, TargetTrueBearingDeg may be replaced by an updated
    prediction immediately before execution.
    """

    Mode: PointingMode
    Frame: AngleFrame

    TargetAzimuthDeg: Optional[float] = None
    TargetElevationDeg: float = 0.0

    PositionToleranceDeg: float = 1.0
    SettleTimeSec: float = 0.0

    Metadata: Dict[str, Any] = field(default_factory=dict)


@dataclass
class SearchSector:
    """
    Definition and persistent state of one search sector.

    StartDeg and StopDeg are interpreted according to Frame.

    Direction:
        +1 means scanning toward StopDeg
        -1 means scanning toward StartDeg
    """

    Frame: AngleFrame
    StartDeg: float
    StopDeg: float
    ScanRateDegPerSec: float

    Direction: int = 1
    ActiveEndpoint: str = "STOP"
    ScanCycle: int = 1

    LastMeasuredAzimuthDeg: Optional[float] = None
    InterruptedAzimuthDeg: Optional[float] = None
    Pattern: SearchPattern = SearchPattern.SECTOR

    def __post_init__(self) -> None:
        self.StartDeg = float(self.StartDeg)
        self.StopDeg = float(self.StopDeg)
        self.ScanRateDegPerSec = abs(float(self.ScanRateDegPerSec))
        self.Pattern = SearchPattern(self.Pattern)

        if self.ScanRateDegPerSec <= 0.0:
            raise ValueError("ScanRateDegPerSec must be greater than zero")

        if (
            self.Pattern == SearchPattern.CONTINUOUS_CW
            and self.Frame != AngleFrame.PLATFORM
        ):
            raise ValueError(
                "CONTINUOUS_CW search must use the PLATFORM angle frame"
            )

        if int(self.Direction) not in (-1, 1):
            raise ValueError("Direction must be +1 or -1")

        self.Direction = int(self.Direction)
        self.ActiveEndpoint = str(self.ActiveEndpoint).upper()

        if self.ActiveEndpoint not in ("START", "STOP"):
            raise ValueError("ActiveEndpoint must be START or STOP")

    @property
    def ActiveEndpointDeg(self) -> float:
        return self.StopDeg if self.ActiveEndpoint == "STOP" else self.StartDeg

    def Reverse(self) -> None:
        if self.Pattern == SearchPattern.CONTINUOUS_CW:
            raise RuntimeError("continuous-CW search does not reverse")

        if self.ActiveEndpoint == "STOP":
            self.ActiveEndpoint = "START"
            self.Direction = -1
        else:
            self.ActiveEndpoint = "STOP"
            self.Direction = 1

        self.ScanCycle += 1


@dataclass
class RadarTask:
    """
    Base high-level radar task.

    Priority:
        Larger numbers are more important.

    EarliestStartTimeSec:
        Task must not start before this time.

    DeadlineTimeSec:
        Optional execution deadline. An overdue task should generally outrank
        ordinary search work.

    WaveformProfileId:
        Refers to a scheduler/dwell-planner profile, not directly to waveform
        samples. The profile later expands into a DwellPlan.
    """

    TaskId: int
    TaskType: RadarTaskType
    Priority: int

    Pointing: PointingRequest
    WaveformProfileId: str

    CreatedTimeSec: float = field(default_factory=time.time)
    EarliestStartTimeSec: float = 0.0
    DeadlineTimeSec: Optional[float] = None

    Status: TaskStatus = TaskStatus.QUEUED
    Metadata: Dict[str, Any] = field(default_factory=dict)

    def IsReady(self, CurrentTimeSec: Optional[float] = None) -> bool:
        now = time.time() if CurrentTimeSec is None else float(CurrentTimeSec)
        return (
            self.Status in (TaskStatus.QUEUED, TaskStatus.PAUSED)
            and now >= float(self.EarliestStartTimeSec)
        )

    def IsOverdue(self, CurrentTimeSec: Optional[float] = None) -> bool:
        if self.DeadlineTimeSec is None:
            return False

        now = time.time() if CurrentTimeSec is None else float(CurrentTimeSec)
        return now > float(self.DeadlineTimeSec)

    def DeadlineSlackSec(self, CurrentTimeSec: Optional[float] = None) -> float:
        if self.DeadlineTimeSec is None:
            return float("inf")

        now = time.time() if CurrentTimeSec is None else float(CurrentTimeSec)
        return float(self.DeadlineTimeSec) - now


@dataclass
class SearchTask(RadarTask):
    Sector: SearchSector = None

    def __post_init__(self) -> None:
        if self.TaskType != RadarTaskType.SEARCH:
            raise ValueError("SearchTask TaskType must be SEARCH")

        if self.Sector is None:
            raise ValueError("SearchTask requires a SearchSector")

        if self.Pointing.Mode != PointingMode.CONTINUOUS_SCAN:
            raise ValueError(
                "SearchTask Pointing.Mode must be CONTINUOUS_SCAN"
            )


@dataclass
class TrackTask(RadarTask):
    TrackId: int = 0

    PredictedRangeM: Optional[float] = None
    PredictedTrueBearingDeg: Optional[float] = None
    PredictedElevationDeg: float = 0.0
    PredictionTimeSec: Optional[float] = None

    RevisitIntervalSec: float = 1.0

    def __post_init__(self) -> None:
        if self.TaskType != RadarTaskType.TRACK:
            raise ValueError("TrackTask TaskType must be TRACK")

        if int(self.TrackId) <= 0:
            raise ValueError("TrackTask requires TrackId > 0")

        if self.Pointing.Mode != PointingMode.GOTO_AND_HOLD:
            raise ValueError(
                "TrackTask Pointing.Mode must be GOTO_AND_HOLD"
            )

        self.TrackId = int(self.TrackId)
        self.RevisitIntervalSec = float(self.RevisitIntervalSec)

        if self.RevisitIntervalSec <= 0.0:
            raise ValueError("RevisitIntervalSec must be greater than zero")


@dataclass
class RevisitRequest:
    """
    A tracker request asking the scheduler to revisit one track.

    The scheduler converts this into a TrackTask.
    """

    TrackId: int
    RequestedTimeSec: float
    DeadlineTimeSec: float

    PredictedRangeM: float
    PredictedTrueBearingDeg: float
    PredictedElevationDeg: float = 0.0

    Priority: int = 100
    RevisitIntervalSec: float = 1.0
    WaveformProfileId: str = "TRACK_DEFAULT"

    Metadata: Dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        self.TrackId = int(self.TrackId)
        self.RequestedTimeSec = float(self.RequestedTimeSec)
        self.DeadlineTimeSec = float(self.DeadlineTimeSec)
        self.PredictedRangeM = float(self.PredictedRangeM)
        self.PredictedTrueBearingDeg = float(self.PredictedTrueBearingDeg)
        self.PredictedElevationDeg = float(self.PredictedElevationDeg)
        self.Priority = int(self.Priority)
        self.RevisitIntervalSec = float(self.RevisitIntervalSec)

        if self.TrackId <= 0:
            raise ValueError("TrackId must be greater than zero")

        if self.DeadlineTimeSec < self.RequestedTimeSec:
            raise ValueError(
                "DeadlineTimeSec cannot be earlier than RequestedTimeSec"
            )


def MakeSearchTask(
    TaskId: int,
    SectorStartDeg: float,
    SectorStopDeg: float,
    ScanRateDegPerSec: float,
    SectorFrame: AngleFrame = AngleFrame.PLATFORM,
    Pattern: SearchPattern = SearchPattern.SECTOR,
    WaveformProfileId: str = "SEARCH_DEFAULT",
    Priority: int = 10,
) -> SearchTask:
    """Convenience constructor for the persistent search task."""

    sector = SearchSector(
        Frame=SectorFrame,
        StartDeg=SectorStartDeg,
        StopDeg=SectorStopDeg,
        ScanRateDegPerSec=ScanRateDegPerSec,
        Pattern=Pattern,
    )

    pointing = PointingRequest(
        Mode=PointingMode.CONTINUOUS_SCAN,
        Frame=SectorFrame,
    )

    return SearchTask(
        TaskId=int(TaskId),
        TaskType=RadarTaskType.SEARCH,
        Priority=int(Priority),
        Pointing=pointing,
        WaveformProfileId=str(WaveformProfileId),
        Sector=sector,
    )


def MakeTrackTaskFromRevisit(
    TaskId: int,
    Request: RevisitRequest,
) -> TrackTask:
    """Convert one tracker RevisitRequest into a scheduler TrackTask."""

    pointing = PointingRequest(
        Mode=PointingMode.GOTO_AND_HOLD,
        Frame=AngleFrame.TRUE,
        TargetAzimuthDeg=Request.PredictedTrueBearingDeg,
        TargetElevationDeg=Request.PredictedElevationDeg,
    )

    return TrackTask(
        TaskId=int(TaskId),
        TaskType=RadarTaskType.TRACK,
        Priority=int(Request.Priority),
        Pointing=pointing,
        WaveformProfileId=str(Request.WaveformProfileId),
        EarliestStartTimeSec=float(Request.RequestedTimeSec),
        DeadlineTimeSec=float(Request.DeadlineTimeSec),
        TrackId=int(Request.TrackId),
        PredictedRangeM=float(Request.PredictedRangeM),
        PredictedTrueBearingDeg=float(Request.PredictedTrueBearingDeg),
        PredictedElevationDeg=float(Request.PredictedElevationDeg),
        PredictionTimeSec=float(Request.RequestedTimeSec),
        RevisitIntervalSec=float(Request.RevisitIntervalSec),
        Metadata=dict(Request.Metadata),
    )
