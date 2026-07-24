"""GUI-independent static validation for Vanguard X mission profiles."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
import math
from typing import Any, Dict, Iterable, List, Optional, Tuple

from MissionProfile import (
    MISSION_COMPLETION_POLICIES,
    MISSION_DURATION_BASIS_ACTIVE_TASK_TIME,
    MISSION_SCHEMA_VERSION,
    SCAN_DIRECTIONS,
    TRACK_MISS_POLICIES,
    CircularSpanDeg,
    MissionProfile,
    Scan360Task,
    SectorScanTask,
)


@dataclass(frozen=True)
class MissionValidationIssue:
    Severity: str
    Code: str
    Field: str
    Message: str


@dataclass(frozen=True)
class MissionLimits:
    AvailableWaveformIds: Tuple[str, ...]
    MinimumPrfHz: float = 1000.0
    MaximumPrfHz: float = 4000.0
    MinimumPulsesPerCpi: int = 8
    MaximumPulsesPerCpi: int = 128
    MinimumRangeM: float = 1000.0
    MaximumRangeM: float = 15000.0
    DwellCadenceSec: float = 0.100
    UsableBeamwidthDeg: Optional[float] = None
    MaximumRotationRpm: Optional[float] = None
    MaximumScanRateDegSec: Optional[float] = None

    @staticmethod
    def FromConfig(config: Dict[str, Any]) -> "MissionLimits":
        def OptionalPositive(name: str) -> Optional[float]:
            value = config.get(name)
            if value is None:
                return None
            value = float(value)
            return value if value > 0.0 else None

        return MissionLimits(
            AvailableWaveformIds=tuple(str(item) for item in config.get(
                "AvailableWaveformIds",
                ("Frank10_20MHz", "Golay64_20MHz"),
            )),
            MinimumPrfHz=float(config.get("MinPrfHz", 1000.0)),
            MaximumPrfHz=float(config.get("MaxPrfHz", 4000.0)),
            MinimumPulsesPerCpi=int(config.get("MinPulsesPerCpi", 8)),
            MaximumPulsesPerCpi=int(config.get("MaxPulsesPerCpi", 128)),
            MinimumRangeM=float(config.get("MinSelectableRangeM", 1000.0)),
            MaximumRangeM=float(config.get("MaxSelectableRangeM", 15000.0)),
            DwellCadenceSec=float(config.get("DwellCadenceSec", 0.100)),
            UsableBeamwidthDeg=OptionalPositive("MissionUsableBeamwidthDeg"),
            MaximumRotationRpm=OptionalPositive("AntennaMaximumRotationRpm"),
            MaximumScanRateDegSec=OptionalPositive(
                "AntennaMaximumScanRateDegSec"
            ),
        )


@dataclass
class MissionValidationResult:
    Issues: List[MissionValidationIssue] = field(default_factory=list)
    TaskDerived: Dict[str, Dict[str, float]] = field(default_factory=dict)
    TrackUpdateDerived: Dict[str, float] = field(default_factory=dict)
    LimitsSnapshot: Dict[str, Any] = field(default_factory=dict)

    @property
    def IsValid(self) -> bool:
        return not any(issue.Severity == "ERROR" for issue in self.Issues)

    @property
    def ErrorCount(self) -> int:
        return sum(issue.Severity == "ERROR" for issue in self.Issues)

    @property
    def WarningCount(self) -> int:
        return sum(issue.Severity == "WARNING" for issue in self.Issues)

    def Snapshot(self) -> Dict[str, Any]:
        return {
            "IsValid": self.IsValid,
            "Issues": [asdict(issue) for issue in self.Issues],
            "TaskDerived": self.TaskDerived,
            "TrackUpdateDerived": self.TrackUpdateDerived,
            "Limits": self.LimitsSnapshot,
        }


class MissionValidator:
    def __init__(self, limits: MissionLimits):
        self.Limits = limits

    @staticmethod
    def FromConfig(config: Dict[str, Any]) -> "MissionValidator":
        return MissionValidator(MissionLimits.FromConfig(config))

    def Validate(self, profile: MissionProfile) -> MissionValidationResult:
        result = MissionValidationResult(
            LimitsSnapshot=asdict(self.Limits),
        )

        def Add(severity: str, code: str, field: str, message: str):
            result.Issues.append(MissionValidationIssue(
                severity.upper(), code, field, message,
            ))

        if profile.SchemaVersion != MISSION_SCHEMA_VERSION:
            Add(
                "ERROR", "SCHEMA_UNSUPPORTED", "SchemaVersion",
                f"Schema {profile.SchemaVersion} is unsupported.",
            )
        if not profile.MissionId.strip():
            Add("ERROR", "MISSION_ID_REQUIRED", "MissionId", "Mission ID is required.")
        if not profile.Name.strip():
            Add("ERROR", "MISSION_NAME_REQUIRED", "Name", "Mission name is required.")
        if profile.Revision < 1:
            Add("ERROR", "REVISION_INVALID", "Revision", "Revision must be at least 1.")
        if profile.DurationBasis != MISSION_DURATION_BASIS_ACTIVE_TASK_TIME:
            Add(
                "ERROR", "DURATION_BASIS_UNSUPPORTED", "DurationBasis",
                "Only ACTIVE_TASK_TIME is supported in this implementation.",
            )
        if profile.CompletionPolicy not in MISSION_COMPLETION_POLICIES:
            Add(
                "ERROR", "COMPLETION_POLICY_INVALID", "CompletionPolicy",
                "Completion policy must be STOP or REPEAT.",
            )

        enabled_tasks = [task for task in profile.PrimaryTasks if task.Enabled]
        if not enabled_tasks:
            Add(
                "ERROR", "NO_ENABLED_TASKS", "PrimaryTasks",
                "At least one primary scan task must be enabled.",
            )

        seen_ids = set()
        for index, task in enumerate(profile.PrimaryTasks):
            field_prefix = f"PrimaryTasks[{index}]"
            if not task.TaskId.strip():
                Add(
                    "ERROR", "TASK_ID_REQUIRED", f"{field_prefix}.TaskId",
                    "Every task requires an ID.",
                )
            elif task.TaskId in seen_ids:
                Add(
                    "ERROR", "TASK_ID_DUPLICATE", f"{field_prefix}.TaskId",
                    f"Task ID {task.TaskId!r} is duplicated.",
                )
            seen_ids.add(task.TaskId)
            self._ValidatePrimaryTask(task, field_prefix, result, Add)

        periodic_tasks = [
            task for task in enabled_tasks
            if task.RepeatIntervalSec is not None
        ]
        continuous_tasks = [
            task for task in enabled_tasks
            if task.DurationSec is None
        ]
        if periodic_tasks:
            if len(continuous_tasks) != 1:
                Add(
                    "ERROR", "PERIODIC_BASELINE_REQUIRED",
                    "PrimaryTasks",
                    "Periodic interrupts require exactly one continuous "
                    "baseline scan.",
                )
            elif enabled_tasks[0] is not continuous_tasks[0]:
                Add(
                    "ERROR", "PERIODIC_BASELINE_NOT_FIRST",
                    "PrimaryTasks",
                    "The continuous baseline scan must be the first enabled task.",
                )
            non_periodic_non_baseline = [
                task for task in enabled_tasks
                if task is not continuous_tasks[0]
                and task.RepeatIntervalSec is None
            ] if continuous_tasks else []
            if non_periodic_non_baseline:
                Add(
                    "ERROR", "PERIODIC_SEQUENCE_AMBIGUOUS",
                    "PrimaryTasks",
                    "When periodic interrupts are used, every enabled task "
                    "after the continuous baseline must have a repeat interval.",
                )
        else:
            for index, task in enumerate(enabled_tasks[:-1]):
                if task.DurationSec is None:
                    Add(
                        "ERROR", "CONTINUOUS_TASK_NOT_LAST",
                        f"PrimaryTasks[{index}].DurationSec",
                        "A continuous task must be last in an ordered mission, "
                        "or later tasks must be configured as periodic interrupts.",
                    )

        self._ValidateTrackUpdate(profile, result, Add)

        if self.Limits.UsableBeamwidthDeg is None:
            Add(
                "WARNING", "BEAMWIDTH_NOT_CHARACTERISED",
                "MissionUsableBeamwidthDeg",
                "Usable antenna beamwidth is not configured; beam coverage "
                "checks remain advisory. Simulation may proceed, but a "
                "characterised value is required before operational use.",
            )
        if (
            self.Limits.MaximumRotationRpm is None
            or self.Limits.MaximumScanRateDegSec is None
        ):
            Add(
                "WARNING", "ANTENNA_LIMITS_NOT_CHARACTERISED",
                "AntennaLimits",
                "Operational rotation and scan-rate limits are not fully "
                "configured. Simulation may proceed; mission motion values "
                "remain unapproved draft values.",
            )
        return result

    def _ValidateRadarCommon(
        self,
        task: Any,
        prefix: str,
        add,
    ) -> Optional[Dict[str, float]]:
        if hasattr(task, "Name") and not task.Name.strip():
            add("ERROR", "TASK_NAME_REQUIRED", f"{prefix}.Name", "Task name is required.")
        if (
            hasattr(task, "DurationSec")
            and task.DurationSec is not None
            and task.DurationSec <= 0.0
        ):
            add(
                "ERROR", "TASK_DURATION_INVALID", f"{prefix}.DurationSec",
                "Task duration must be positive or Continuous.",
            )
        if getattr(task, "RepeatIntervalSec", None) is not None:
            if task.RepeatIntervalSec <= 0.0:
                add(
                    "ERROR", "TASK_REPEAT_INTERVAL_INVALID",
                    f"{prefix}.RepeatIntervalSec",
                    "Repeat interval must be positive.",
                )
            if task.DurationSec is None:
                add(
                    "ERROR", "PERIODIC_TASK_CONTINUOUS",
                    f"{prefix}.DurationSec",
                    "A periodic interrupt must have a finite duration.",
                )
            elif task.RepeatIntervalSec <= task.DurationSec:
                add(
                    "ERROR", "TASK_REPEAT_INTERVAL_TOO_SHORT",
                    f"{prefix}.RepeatIntervalSec",
                    "Repeat interval must be longer than the task duration "
                    "so the baseline scan receives surveillance time.",
                )
        if task.WaveformId not in self.Limits.AvailableWaveformIds:
            add(
                "ERROR", "WAVEFORM_UNKNOWN", f"{prefix}.WaveformId",
                f"Waveform {task.WaveformId!r} is not available.",
            )
        if not self.Limits.MinimumPrfHz <= task.PrfHz <= self.Limits.MaximumPrfHz:
            add(
                "ERROR", "PRF_OUT_OF_RANGE", f"{prefix}.PrfHz",
                f"PRF must be {self.Limits.MinimumPrfHz:g}-"
                f"{self.Limits.MaximumPrfHz:g} Hz.",
            )
        if not (
            self.Limits.MinimumPulsesPerCpi
            <= task.PulsesPerCpi
            <= self.Limits.MaximumPulsesPerCpi
        ):
            add(
                "ERROR", "PULSES_OUT_OF_RANGE", f"{prefix}.PulsesPerCpi",
                f"Pulses/CPI must be {self.Limits.MinimumPulsesPerCpi}-"
                f"{self.Limits.MaximumPulsesPerCpi}.",
            )
        if not self.Limits.MinimumRangeM <= task.MaximumRangeM <= self.Limits.MaximumRangeM:
            add(
                "ERROR", "RANGE_OUT_OF_RANGE", f"{prefix}.MaximumRangeM",
                f"Maximum range must be {self.Limits.MinimumRangeM / 1000.0:g}-"
                f"{self.Limits.MaximumRangeM / 1000.0:g} km.",
            )
        if task.PrfHz <= 0.0 or task.PulsesPerCpi <= 0:
            return None
        pri_sec = 1.0 / task.PrfHz
        cpi_sec = task.PulsesPerCpi / task.PrfHz
        if cpi_sec > self.Limits.DwellCadenceSec:
            add(
                "ERROR", "CPI_EXCEEDS_DWELL_CADENCE", f"{prefix}.PulsesPerCpi",
                "CPI duration exceeds the configured dwell cadence.",
            )
        return {
            "PriUs": pri_sec * 1.0e6,
            "CpiMs": cpi_sec * 1.0e3,
            "CpiSec": cpi_sec,
        }

    def _ValidatePrimaryTask(self, task, prefix, result, add):
        derived = self._ValidateRadarCommon(task, prefix, add)
        if derived is None:
            return

        if isinstance(task, Scan360Task):
            if task.Direction not in SCAN_DIRECTIONS:
                add(
                    "ERROR", "DIRECTION_INVALID", f"{prefix}.Direction",
                    "Direction must be CW or CCW.",
                )
            if task.RotationRpm <= 0.0:
                add(
                    "ERROR", "ROTATION_RATE_INVALID", f"{prefix}.RotationRpm",
                    "Rotation speed must be positive.",
                )
                scan_rate = 0.0
            else:
                scan_rate = task.RotationRpm * 6.0
            if (
                self.Limits.MaximumRotationRpm is not None
                and task.RotationRpm > self.Limits.MaximumRotationRpm
            ):
                add(
                    "ERROR", "ROTATION_RATE_LIMIT", f"{prefix}.RotationRpm",
                    "Rotation speed exceeds the configured antenna limit.",
                )
            derived.update({
                "ScanRateDegSec": scan_rate,
                "RevolutionSec": (
                    60.0 / task.RotationRpm if task.RotationRpm > 0.0 else math.inf
                ),
            })
        elif isinstance(task, SectorScanTask):
            if task.InitialDirection not in SCAN_DIRECTIONS:
                add(
                    "ERROR", "DIRECTION_INVALID", f"{prefix}.InitialDirection",
                    "Initial direction must be CW or CCW.",
                )
                sector_width = 0.0
            else:
                sector_width = CircularSpanDeg(
                    task.StartDeg, task.StopDeg, task.InitialDirection,
                )
            if sector_width <= 0.0:
                add(
                    "ERROR", "SECTOR_ZERO_WIDTH", f"{prefix}.StopDeg",
                    "Sector boundaries must define a non-zero sector.",
                )
            if task.ScanRateDegSec <= 0.0:
                add(
                    "ERROR", "SECTOR_RATE_INVALID", f"{prefix}.ScanRateDegSec",
                    "Sector scan rate must be positive.",
                )
            if (
                self.Limits.MaximumScanRateDegSec is not None
                and task.ScanRateDegSec > self.Limits.MaximumScanRateDegSec
            ):
                add(
                    "ERROR", "SECTOR_RATE_LIMIT", f"{prefix}.ScanRateDegSec",
                    "Sector scan rate exceeds the configured antenna limit.",
                )
            if task.EndpointMarginDeg < 0.0:
                add(
                    "ERROR", "ENDPOINT_MARGIN_INVALID",
                    f"{prefix}.EndpointMarginDeg",
                    "Endpoint margin cannot be negative.",
                )
            usable_width = sector_width - 2.0 * max(0.0, task.EndpointMarginDeg)
            if usable_width <= 0.0:
                add(
                    "ERROR", "ENDPOINT_MARGIN_TOO_LARGE",
                    f"{prefix}.EndpointMarginDeg",
                    "Endpoint margins consume the complete sector.",
                )
            traverse_sec = (
                usable_width / task.ScanRateDegSec
                if usable_width > 0.0 and task.ScanRateDegSec > 0.0
                else math.inf
            )
            derived.update({
                "SectorWidthDeg": sector_width,
                "UsableSectorWidthDeg": max(0.0, usable_width),
                "ScanRateDegSec": max(0.0, task.ScanRateDegSec),
                "TraverseSec": traverse_sec,
                "ExpectedRevisitSec": 2.0 * traverse_sec,
            })
        else:
            add("ERROR", "TASK_TYPE_UNSUPPORTED", prefix, "Unsupported task type.")
            return

        scan_rate = float(derived.get("ScanRateDegSec", 0.0))
        cpi_travel = scan_rate * float(derived["CpiSec"])
        dwell_start_travel = scan_rate * self.Limits.DwellCadenceSec
        derived.update({
            "CpiTravelDeg": cpi_travel,
            "DwellStartTravelDeg": dwell_start_travel,
        })
        beamwidth = self.Limits.UsableBeamwidthDeg
        if beamwidth is not None and beamwidth > 0.0:
            derived["DwellsPerBeamwidth"] = (
                beamwidth / dwell_start_travel
                if dwell_start_travel > 0.0 else math.inf
            )
            if cpi_travel >= beamwidth:
                add(
                    "ERROR", "CPI_DOES_NOT_FIT_BEAM", prefix,
                    f"CPI travels {cpi_travel:.2f} deg, not less than the "
                    f"{beamwidth:.2f} deg usable beamwidth.",
                )
            elif cpi_travel > 0.5 * beamwidth:
                add(
                    "WARNING", "CPI_BEAM_MARGIN_LOW", prefix,
                    f"CPI consumes {100.0 * cpi_travel / beamwidth:.0f}% of "
                    "the configured usable beamwidth.",
                )
            if dwell_start_travel > beamwidth:
                add(
                    "ERROR", "AZIMUTH_COVERAGE_GAP", prefix,
                    f"Dwell starts are {dwell_start_travel:.2f} deg apart, "
                    f"exceeding the {beamwidth:.2f} deg usable beamwidth.",
                )
            elif dwell_start_travel > 0.75 * beamwidth:
                add(
                    "WARNING", "AZIMUTH_OVERLAP_LOW", prefix,
                    "Dwell-to-dwell angular overlap is marginal.",
                )
        result.TaskDerived[task.TaskId] = derived

    def _ValidateTrackUpdate(self, profile, result, add):
        policy = profile.TrackUpdate
        if not policy.Enabled:
            return
        prefix = "TrackUpdate"
        radar = self._ValidateRadarCommon(policy, prefix, add)
        if radar is None:
            return
        if policy.Eligibility != "TAGGED_CONFIRMED":
            add(
                "ERROR", "TRACK_ELIGIBILITY_UNSUPPORTED",
                f"{prefix}.Eligibility",
                "Only TAGGED_CONFIRMED tracks are currently supported.",
            )
        if policy.RevisitIntervalSec <= 0.0:
            add(
                "ERROR", "REVISIT_INVALID", f"{prefix}.RevisitIntervalSec",
                "Revisit interval must be positive.",
            )
        if policy.GateHalfWidthDeg <= 0.0:
            add(
                "ERROR", "TRACK_GATE_INVALID", f"{prefix}.GateHalfWidthDeg",
                "Gate half-width must be positive.",
            )
        if not 0.0 < policy.AngularStepDeg <= 2.0 * policy.GateHalfWidthDeg:
            add(
                "ERROR", "TRACK_STEP_INVALID", f"{prefix}.AngularStepDeg",
                "Angular step must be positive and no wider than the gate.",
            )
        if policy.DwellsPerAngle < 1:
            add(
                "ERROR", "TRACK_DWELLS_INVALID", f"{prefix}.DwellsPerAngle",
                "Dwells per angle must be at least one.",
            )
        if policy.SlewRateDegSec <= 0.0 or policy.NodRateDegSec <= 0.0:
            add(
                "ERROR", "TRACK_MOTION_RATE_INVALID", prefix,
                "Slew and nod rates must be positive.",
            )
        if policy.SettleTimeSec < 0.0:
            add(
                "ERROR", "TRACK_SETTLE_INVALID", f"{prefix}.SettleTimeSec",
                "Settle time cannot be negative.",
            )
        if policy.MissPolicy not in TRACK_MISS_POLICIES:
            add(
                "ERROR", "TRACK_MISS_POLICY_INVALID", f"{prefix}.MissPolicy",
                "Unsupported missed-track policy.",
            )

        sample_count = (
            int(math.ceil(
                2.0 * policy.GateHalfWidthDeg / policy.AngularStepDeg
            )) + 1
            if policy.AngularStepDeg > 0.0 else 0
        )
        acquisition_sec = (
            sample_count * policy.DwellsPerAngle * radar["CpiSec"]
        )
        nod_motion_sec = (
            2.0 * policy.GateHalfWidthDeg / policy.NodRateDegSec
            if policy.NodRateDegSec > 0.0 else math.inf
        )
        worst_slew_sec = (
            180.0 / policy.SlewRateDegSec
            if policy.SlewRateDegSec > 0.0 else math.inf
        )
        settle_total_sec = sample_count * max(0.0, policy.SettleTimeSec)
        estimated_total_sec = (
            worst_slew_sec + nod_motion_sec + acquisition_sec + settle_total_sec
        )
        effective_revisit_sec = max(
            policy.RevisitIntervalSec,
            estimated_total_sec,
        )
        result.TrackUpdateDerived = {
            **radar,
            "SampleCount": sample_count,
            "AcquisitionSec": acquisition_sec,
            "NodMotionSec": nod_motion_sec,
            "WorstCaseSlewSec": worst_slew_sec,
            "SettlingSec": settle_total_sec,
            "EstimatedInterruptionSec": estimated_total_sec,
            "RequestedRevisitSec": policy.RevisitIntervalSec,
            "EffectiveRevisitSec": effective_revisit_sec,
        }
        if estimated_total_sec > policy.MaximumUpdateDurationSec:
            add(
                "ERROR", "TRACK_UPDATE_TIMEOUT",
                f"{prefix}.MaximumUpdateDurationSec",
                f"Estimated worst-case interruption {estimated_total_sec:.1f} s "
                f"exceeds the {policy.MaximumUpdateDurationSec:.1f} s limit.",
            )
        elif estimated_total_sec > 0.75 * policy.MaximumUpdateDurationSec:
            add(
                "WARNING", "TRACK_UPDATE_MARGIN_LOW",
                f"{prefix}.MaximumUpdateDurationSec",
                "Estimated Track Update duration has less than 25% timeout margin.",
            )
