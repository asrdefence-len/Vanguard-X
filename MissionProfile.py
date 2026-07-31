"""Versioned Vanguard X mission profile data model.

The classes in this module are deliberately independent of Qt, the Ettus
source, and antenna hardware.  A Mission page edits a draft by constructing new
frozen dataclass instances.  Validation and loading then take deep snapshots so
later draft edits cannot alter the mission that has been accepted for
execution.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field, replace
from datetime import datetime, timezone
import json
import math
import re
from typing import Any, Dict, Optional, Tuple, Union
from uuid import uuid4


MISSION_SCHEMA_VERSION = 1
MISSION_DURATION_BASIS_ACTIVE_TASK_TIME = "ACTIVE_TASK_TIME"
MISSION_COMPLETION_POLICIES = ("STOP", "REPEAT")
SCAN_DIRECTIONS = ("CW", "CCW")
TRACK_MISS_POLICIES = (
    "RETRY_WIDER_THEN_DELETE",
    "RETRY_WIDER_THEN_DEFER",
    "DELETE",
    "COAST",
    "RETRY",
    "WIDEN",
    "DEFER",
)


def UtcNowIso() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


def NormalizeAngleDeg(angle_deg: float) -> float:
    value = math.fmod(float(angle_deg), 360.0)
    if value < 0.0:
        value += 360.0
    if math.isclose(value, 360.0):
        value = 0.0
    return value


def CircularSpanDeg(start_deg: float, stop_deg: float, direction: str) -> float:
    """Return the directed circular span from start to stop.

    Equal endpoints intentionally return zero rather than 360 degrees.  A
    Sector Scan with equal endpoints is invalid; a 360 Scan is represented by
    its own task type.
    """

    start = NormalizeAngleDeg(start_deg)
    stop = NormalizeAngleDeg(stop_deg)
    if math.isclose(start, stop, abs_tol=1.0e-9):
        return 0.0
    direction = str(direction).upper()
    if direction == "CW":
        return (stop - start) % 360.0
    if direction == "CCW":
        return (start - stop) % 360.0
    raise ValueError("direction must be CW or CCW")


@dataclass(frozen=True)
class Scan360Task:
    TaskId: str
    Name: str = "360 Scan"
    Enabled: bool = True
    DurationSec: Optional[float] = 600.0
    RepeatIntervalSec: Optional[float] = None
    WaveformId: str = "Frank10_20MHz"
    PrfHz: float = 2000.0
    PulsesPerCpi: int = 32
    MaximumRangeM: float = 15000.0
    Direction: str = "CW"
    RotationRpm: float = 24.0
    TaskType: str = field(default="SCAN_360", init=False)


@dataclass(frozen=True)
class SectorScanTask:
    TaskId: str
    Name: str = "Sector Scan"
    Enabled: bool = True
    DurationSec: Optional[float] = 1200.0
    RepeatIntervalSec: Optional[float] = None
    WaveformId: str = "Golay64_20MHz"
    PrfHz: float = 2000.0
    PulsesPerCpi: int = 32
    MaximumRangeM: float = 15000.0
    StartDeg: float = 40.0
    StopDeg: float = 90.0
    InitialDirection: str = "CW"
    ScanRateDegSec: float = 20.0
    EndpointMarginDeg: float = 1.0
    TaskType: str = field(default="SECTOR_SCAN", init=False)


PrimaryTask = Union[Scan360Task, SectorScanTask]


@dataclass(frozen=True)
class TrackUpdatePolicy:
    Enabled: bool = False
    Eligibility: str = "TAGGED_CONFIRMED"
    RevisitIntervalSec: float = 10.0
    WaveformId: str = "Golay64_20MHz"
    PrfHz: float = 2000.0
    PulsesPerCpi: int = 32
    MaximumRangeM: float = 15000.0
    GateMode: str = "FIXED"
    GateHalfWidthDeg: float = 5.0
    MinimumGateDeg: float = 1.0
    MaximumGateDeg: float = 8.0
    AngularStepDeg: float = 0.5
    DwellsPerAngle: int = 1
    SlewRateDegSec: float = 20.0
    NodRateDegSec: float = 5.0
    PointingToleranceDeg: float = 0.25
    SettleTimeSec: float = 0.25
    MaximumUpdateDurationSec: float = 20.0
    MissPolicy: str = "RETRY_WIDER_THEN_DELETE"


@dataclass(frozen=True)
class MissionProfile:
    SchemaVersion: int
    MissionId: str
    Name: str
    Revision: int
    Description: str
    DurationBasis: str
    CompletionPolicy: str
    PrimaryTasks: Tuple[PrimaryTask, ...]
    TrackUpdate: TrackUpdatePolicy
    ValidationSnapshot: Optional[Dict[str, Any]] = None
    CreatedUtc: str = ""
    ModifiedUtc: str = ""


def NewTaskId(prefix: str) -> str:
    return f"{prefix.lower()}-{uuid4().hex[:8]}"


def CreateDefaultMission() -> MissionProfile:
    now = UtcNowIso()
    return MissionProfile(
        SchemaVersion=MISSION_SCHEMA_VERSION,
        MissionId=f"mission-{uuid4().hex[:8]}",
        Name="Mission 1",
        Revision=1,
        Description=(
            "Draft example values only. Validate against the characterised "
            "antenna drive before mission execution."
        ),
        DurationBasis=MISSION_DURATION_BASIS_ACTIVE_TASK_TIME,
        CompletionPolicy="STOP",
        PrimaryTasks=(
            Scan360Task(
                TaskId=NewTaskId("scan360"),
                DurationSec=None,
            ),
            SectorScanTask(
                TaskId=NewTaskId("sector"),
                DurationSec=60.0,
                RepeatIntervalSec=300.0,
            ),
        ),
        TrackUpdate=TrackUpdatePolicy(),
        CreatedUtc=now,
        ModifiedUtc=now,
    )


def ResolveAvailableWaveformId(
    waveform_id: str,
    available_waveform_ids: Tuple[str, ...],
) -> str:
    """Return a waveform identifier that the active library can provide.

    Frank remains the preferred safe default when it is available.  Otherwise
    the first advertised waveform is used.  An empty library is left for the
    validator to report rather than inventing an identifier.
    """

    available = tuple(
        str(item) for item in available_waveform_ids if str(item)
    )
    requested = str(waveform_id)
    if requested in available:
        return requested

    # Migrate mission files created when the UI accidentally exposed the
    # internal A/B members instead of the logical complementary family.
    GolayMember = re.fullmatch(r"(Golay\d+)[AB](_\d+MHz)", requested)
    if GolayMember is not None:
        PairId = f"{GolayMember.group(1)}{GolayMember.group(2)}"
        if PairId in available:
            return PairId

    if "Frank10_20MHz" in available:
        return "Frank10_20MHz"
    if available:
        return available[0]
    return requested


def ReconcileMissionWaveforms(
    profile: MissionProfile,
    available_waveform_ids: Tuple[str, ...],
) -> MissionProfile:
    """Keep the mission model aligned with waveform selectors shown by Qt."""

    tasks = tuple(
        replace(
            task,
            WaveformId=ResolveAvailableWaveformId(
                task.WaveformId,
                available_waveform_ids,
            ),
        )
        for task in profile.PrimaryTasks
    )
    track_update = replace(
        profile.TrackUpdate,
        WaveformId=ResolveAvailableWaveformId(
            profile.TrackUpdate.WaveformId,
            available_waveform_ids,
        ),
    )
    if tasks == profile.PrimaryTasks and track_update == profile.TrackUpdate:
        return profile
    return TouchMission(
        profile,
        PrimaryTasks=tasks,
        TrackUpdate=track_update,
    )


def CloneMission(profile: MissionProfile) -> MissionProfile:
    """Return a detached immutable copy through the public schema."""

    return MissionProfileFromDict(MissionProfileToDict(profile))


def TouchMission(profile: MissionProfile, **changes: Any) -> MissionProfile:
    changes.setdefault("ModifiedUtc", UtcNowIso())
    changes.setdefault("ValidationSnapshot", None)
    return replace(profile, **changes)


def _TaskFromDict(data: Dict[str, Any]) -> PrimaryTask:
    task_type = str(data.get("TaskType", "")).upper()
    common = {
        "TaskId": str(data.get("TaskId", "")),
        "Name": str(data.get("Name", "")),
        "Enabled": bool(data.get("Enabled", True)),
        "DurationSec": (
            None if data.get("DurationSec") is None
            else float(data.get("DurationSec"))
        ),
        "RepeatIntervalSec": (
            None if data.get("RepeatIntervalSec") is None
            else float(data.get("RepeatIntervalSec"))
        ),
        "WaveformId": str(data.get("WaveformId", "")),
        "PrfHz": float(data.get("PrfHz", 0.0)),
        "PulsesPerCpi": int(data.get("PulsesPerCpi", 0)),
        "MaximumRangeM": float(data.get("MaximumRangeM", 0.0)),
    }
    if task_type == "SCAN_360":
        return Scan360Task(
            **common,
            Direction=str(data.get("Direction", "CW")).upper(),
            RotationRpm=float(data.get("RotationRpm", 0.0)),
        )
    if task_type == "SECTOR_SCAN":
        return SectorScanTask(
            **common,
            StartDeg=float(data.get("StartDeg", 0.0)),
            StopDeg=float(data.get("StopDeg", 0.0)),
            InitialDirection=str(data.get("InitialDirection", "CW")).upper(),
            ScanRateDegSec=float(data.get("ScanRateDegSec", 0.0)),
            EndpointMarginDeg=float(data.get("EndpointMarginDeg", 0.0)),
        )
    raise ValueError(f"unsupported mission task type {task_type!r}")


def MissionProfileToDict(profile: MissionProfile) -> Dict[str, Any]:
    result = asdict(profile)
    result["PrimaryTasks"] = [asdict(task) for task in profile.PrimaryTasks]
    result["TrackUpdate"] = asdict(profile.TrackUpdate)
    return result


def MissionProfileFromDict(data: Dict[str, Any]) -> MissionProfile:
    if not isinstance(data, dict):
        raise TypeError("mission profile must be a JSON object")
    schema_version = int(data.get("SchemaVersion", -1))
    if schema_version != MISSION_SCHEMA_VERSION:
        raise ValueError(
            f"unsupported mission schema {schema_version}; "
            f"expected {MISSION_SCHEMA_VERSION}"
        )
    track_data = dict(data.get("TrackUpdate", {}))
    track_update = TrackUpdatePolicy(
        Enabled=bool(track_data.get("Enabled", False)),
        Eligibility=str(track_data.get("Eligibility", "TAGGED_CONFIRMED")),
        RevisitIntervalSec=float(track_data.get("RevisitIntervalSec", 0.0)),
        WaveformId=str(track_data.get("WaveformId", "")),
        PrfHz=float(track_data.get("PrfHz", 0.0)),
        PulsesPerCpi=int(track_data.get("PulsesPerCpi", 0)),
        MaximumRangeM=float(track_data.get("MaximumRangeM", 0.0)),
        GateMode=str(track_data.get("GateMode", "FIXED")),
        GateHalfWidthDeg=float(track_data.get("GateHalfWidthDeg", 0.0)),
        MinimumGateDeg=float(track_data.get("MinimumGateDeg", 0.0)),
        MaximumGateDeg=float(track_data.get("MaximumGateDeg", 0.0)),
        AngularStepDeg=float(track_data.get("AngularStepDeg", 0.0)),
        DwellsPerAngle=int(track_data.get("DwellsPerAngle", 0)),
        SlewRateDegSec=float(track_data.get("SlewRateDegSec", 0.0)),
        NodRateDegSec=float(track_data.get("NodRateDegSec", 0.0)),
        PointingToleranceDeg=float(track_data.get("PointingToleranceDeg", 0.0)),
        SettleTimeSec=float(track_data.get("SettleTimeSec", 0.0)),
        MaximumUpdateDurationSec=float(
            track_data.get("MaximumUpdateDurationSec", 0.0)
        ),
        MissPolicy=str(track_data.get("MissPolicy", "DEFER")),
    )
    tasks = tuple(_TaskFromDict(dict(item)) for item in data.get("PrimaryTasks", []))
    return MissionProfile(
        SchemaVersion=schema_version,
        MissionId=str(data.get("MissionId", "")),
        Name=str(data.get("Name", "")),
        Revision=int(data.get("Revision", 0)),
        Description=str(data.get("Description", "")),
        DurationBasis=str(data.get(
            "DurationBasis",
            MISSION_DURATION_BASIS_ACTIVE_TASK_TIME,
        )),
        CompletionPolicy=str(data.get("CompletionPolicy", "STOP")).upper(),
        PrimaryTasks=tasks,
        TrackUpdate=track_update,
        ValidationSnapshot=data.get("ValidationSnapshot"),
        CreatedUtc=str(data.get("CreatedUtc", "")),
        ModifiedUtc=str(data.get("ModifiedUtc", "")),
    )


def MissionProfileToJson(profile: MissionProfile, indent: int = 2) -> str:
    return json.dumps(
        MissionProfileToDict(profile),
        indent=indent,
        sort_keys=True,
        allow_nan=False,
    )


def MissionProfileFromJson(text: str) -> MissionProfile:
    return MissionProfileFromDict(json.loads(text))
