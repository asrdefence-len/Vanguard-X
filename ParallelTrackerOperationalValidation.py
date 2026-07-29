"""Operational circular-motion validation for Vanguard X parallel trackers.

This deterministic digital-twin harness exercises the same Stage 5 detection
annotation and Stage 6 tracker implementations used by the scheduler.  The
legacy range/bearing tracker remains authoritative; the mission-ENU tracker is
evaluated against known Earth-frame truth over a complete ownship orbit.

The harness intentionally includes:

* one stationary Earth target;
* one constant-velocity vessel;
* deterministic range/bearing measurement perturbations;
* missed scans that require coasting and reacquisition; and
* simultaneous operation of the legacy and Earth-referenced trackers.

It is a validation gate, not a source of display or tasking tracks.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import argparse
import json
import math
from types import SimpleNamespace
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

from CoordinateFrames import (
    EnuPosition,
    EnuVelocity,
    GeodeticPosition,
    detection_to_enu_position,
    measured_relative_radial_velocity_mps,
    target_relative_to_radar,
)
from EarthReferencedMeasurements import (
    AnnotateDetectionWithEarthReference,
)
from EarthReferencedTracker import EarthReferencedTracker
from NavigationState import CircularRouteNavigationSource
from RadarTracker import RadarTracker


VALIDATION_VERSION = "parallel-tracker-full-orbit-v1"


@dataclass(frozen=True)
class ValidationTarget:
    Name: str
    InitialEastM: float
    InitialNorthM: float
    VelocityEastMps: float
    VelocityNorthMps: float

    def position_at(self, elapsed_sec: float) -> EnuPosition:
        return EnuPosition(
            east_m=self.InitialEastM + self.VelocityEastMps * elapsed_sec,
            north_m=self.InitialNorthM + self.VelocityNorthMps * elapsed_sec,
            up_m=0.0,
        )

    @property
    def velocity(self) -> EnuVelocity:
        return EnuVelocity(
            east_mps=self.VelocityEastMps,
            north_mps=self.VelocityNorthMps,
            up_mps=0.0,
        )


@dataclass(frozen=True)
class OperationalValidationConfig:
    MissionOrigin: GeodeticPosition = field(
        default_factory=lambda: GeodeticPosition(-33.0, 151.0, 0.0)
    )
    OrbitCentreEastM: float = 0.0
    OrbitCentreNorthM: float = 0.0
    OrbitRadiusM: float = 500.0
    OwnshipSpeedMps: float = 10.0
    Clockwise: bool = True
    InitialRadialBearingDeg: float = 0.0
    ScansPerOrbit: int = 72
    StartTimestampSec: float = 1000.0
    RangeNoiseAmplitudeM: float = 3.0
    BearingNoiseAmplitudeDeg: float = 0.03
    DopplerNoiseAmplitudeMps: float = 0.05
    MissedScansByTarget: Tuple[Tuple[str, Tuple[int, ...]], ...] = (
        ("Stationary buoy", (30, 31)),
        ("Moving vessel", (48,)),
    )
    PositionRmsLimitM: float = 25.0
    PositionMaximumLimitM: float = 75.0
    VelocityRmsLimitMps: float = 1.5
    MinimumCoverageFraction: float = 0.90
    MaximumTrackIdSwitches: int = 0
    MaximumDuplicateConfirmedTracks: int = 0

    def __post_init__(self) -> None:
        if self.OrbitRadiusM <= 0.0:
            raise ValueError("OrbitRadiusM must be greater than zero")
        if self.OwnshipSpeedMps <= 0.0:
            raise ValueError("OwnshipSpeedMps must be greater than zero")
        if self.ScansPerOrbit < 12:
            raise ValueError("ScansPerOrbit must be at least 12")
        if not 0.0 < self.MinimumCoverageFraction <= 1.0:
            raise ValueError(
                "MinimumCoverageFraction must be in the interval (0, 1]"
            )
        for value, name in (
            (self.PositionRmsLimitM, "PositionRmsLimitM"),
            (self.PositionMaximumLimitM, "PositionMaximumLimitM"),
            (self.VelocityRmsLimitMps, "VelocityRmsLimitMps"),
        ):
            if value <= 0.0:
                raise ValueError(f"{name} must be greater than zero")

    @property
    def orbit_period_sec(self) -> float:
        return 2.0 * math.pi * self.OrbitRadiusM / self.OwnshipSpeedMps

    @property
    def scan_period_sec(self) -> float:
        return self.orbit_period_sec / self.ScansPerOrbit

    def missed_scans(self, target_name: str) -> Tuple[int, ...]:
        return dict(self.MissedScansByTarget).get(target_name, ())


@dataclass(frozen=True)
class TargetValidationResult:
    Name: str
    ExpectedEvaluatedScans: int
    EarthAssociatedScans: int
    LegacyAssociatedScans: int
    CoverageFraction: float
    EarthPositionRmsErrorM: Optional[float]
    EarthPositionMaximumErrorM: Optional[float]
    EarthVelocityRmsErrorMps: Optional[float]
    LegacyPositionRmsErrorM: Optional[float]
    LegacyPositionMaximumErrorM: Optional[float]
    EarthTrackIdSwitches: int
    EarthTrackIds: Tuple[int, ...]
    ReacquiredSameTrackAfterMisses: bool
    Passed: bool
    FailureReasons: Tuple[str, ...]


@dataclass(frozen=True)
class OperationalValidationReport:
    Version: str
    Passed: bool
    OrbitPeriodSec: float
    ScanPeriodSec: float
    ScansPerOrbit: int
    EvaluatedScans: int
    MaximumEarthConfirmedTracks: int
    MaximumLegacyConfirmedTracks: int
    MaximumDuplicateEarthConfirmedTracks: int
    EarthTrackerAuthoritative: bool
    LegacyTrackerAuthoritative: bool
    TargetResults: Tuple[TargetValidationResult, ...]
    FailureReasons: Tuple[str, ...]

    def as_dict(self) -> Dict[str, object]:
        return {
            "Version": self.Version,
            "Passed": self.Passed,
            "OrbitPeriodSec": self.OrbitPeriodSec,
            "ScanPeriodSec": self.ScanPeriodSec,
            "ScansPerOrbit": self.ScansPerOrbit,
            "EvaluatedScans": self.EvaluatedScans,
            "MaximumEarthConfirmedTracks": self.MaximumEarthConfirmedTracks,
            "MaximumLegacyConfirmedTracks": self.MaximumLegacyConfirmedTracks,
            "MaximumDuplicateEarthConfirmedTracks": (
                self.MaximumDuplicateEarthConfirmedTracks
            ),
            "EarthTrackerAuthoritative": self.EarthTrackerAuthoritative,
            "LegacyTrackerAuthoritative": self.LegacyTrackerAuthoritative,
            "TargetResults": [
                {
                    field_name: getattr(target, field_name)
                    for field_name in target.__dataclass_fields__
                }
                for target in self.TargetResults
            ],
            "FailureReasons": self.FailureReasons,
        }

    def to_json(self, indent: int = 2) -> str:
        return json.dumps(self.as_dict(), indent=indent)

    def format_text(self) -> str:
        lines = [
            "Vanguard X parallel-tracker operational validation",
            f"Version: {self.Version}",
            (
                f"Orbit: {self.OrbitPeriodSec:.3f} s, "
                f"{self.ScansPerOrbit} scans, "
                f"{self.ScanPeriodSec:.3f} s/scan"
            ),
            (
                "Authority: legacy=YES, "
                f"Earth={'YES' if self.EarthTrackerAuthoritative else 'NO'}"
            ),
        ]
        for result in self.TargetResults:
            earth_rms = _format_optional(result.EarthPositionRmsErrorM, "m")
            earth_max = _format_optional(
                result.EarthPositionMaximumErrorM,
                "m",
            )
            velocity_rms = _format_optional(
                result.EarthVelocityRmsErrorMps,
                "m/s",
            )
            legacy_rms = _format_optional(
                result.LegacyPositionRmsErrorM,
                "m",
            )
            lines.append(
                f"{result.Name}: "
                f"coverage={100.0 * result.CoverageFraction:.1f}%, "
                f"Earth RMS/max={earth_rms}/{earth_max}, "
                f"velocity RMS={velocity_rms}, "
                f"legacy RMS={legacy_rms}, "
                f"ID switches={result.EarthTrackIdSwitches}, "
                f"reacquisition={'PASS' if result.ReacquiredSameTrackAfterMisses else 'FAIL'}"
            )
        lines.append(
            "RESULT: PASS" if self.Passed else "RESULT: FAIL"
        )
        for reason in self.FailureReasons:
            lines.append(f"  - {reason}")
        return "\n".join(lines)


@dataclass
class _TargetAccumulator:
    target: ValidationTarget
    expected_scans: int = 0
    earth_position_errors_m: List[float] = field(default_factory=list)
    earth_velocity_errors_mps: List[float] = field(default_factory=list)
    legacy_position_errors_m: List[float] = field(default_factory=list)
    earth_track_ids: List[int] = field(default_factory=list)
    earth_track_id_by_scan: Dict[int, int] = field(default_factory=dict)
    legacy_track_ids: List[int] = field(default_factory=list)


def _format_optional(value: Optional[float], unit: str) -> str:
    return "n/a" if value is None else f"{value:.3f} {unit}"


def _rms(values: Sequence[float]) -> Optional[float]:
    if not values:
        return None
    return math.sqrt(sum(value * value for value in values) / len(values))


def _maximum(values: Sequence[float]) -> Optional[float]:
    return max(values) if values else None


def _distance(
    east_a: float,
    north_a: float,
    east_b: float,
    north_b: float,
) -> float:
    return math.hypot(east_a - east_b, north_a - north_b)


def _default_targets() -> Tuple[ValidationTarget, ...]:
    return (
        ValidationTarget(
            Name="Stationary buoy",
            InitialEastM=3500.0,
            InitialNorthM=750.0,
            VelocityEastMps=0.0,
            VelocityNorthMps=0.0,
        ),
        ValidationTarget(
            Name="Moving vessel",
            InitialEastM=-2500.0,
            InitialNorthM=3000.0,
            VelocityEastMps=4.0,
            VelocityNorthMps=-1.5,
        ),
    )


def _deterministic_noise(scan_id: int, target_index: int) -> Tuple[float, float, float]:
    phase = 0.71 * scan_id + 1.37 * target_index
    return (
        math.sin(phase),
        math.cos(0.83 * phase + 0.2),
        math.sin(1.11 * phase - 0.4),
    )


def _make_detection(
    target: ValidationTarget,
    target_index: int,
    elapsed_sec: float,
    scan_id: int,
    pose,
    config: OperationalValidationConfig,
):
    truth_position = target.position_at(elapsed_sec)
    relative = target_relative_to_radar(
        truth_position,
        pose.PositionEnu,
    )
    measured_velocity = measured_relative_radial_velocity_mps(
        target.velocity,
        pose.VelocityEnu,
        relative.true_bearing_deg,
    )
    range_noise, bearing_noise, doppler_noise = _deterministic_noise(
        scan_id,
        target_index,
    )
    detection = SimpleNamespace(
        DwellId=scan_id,
        RangeM=(
            relative.range_m
            + config.RangeNoiseAmplitudeM * range_noise
        ),
        AzimuthDeg=(
            relative.true_bearing_deg
            + config.BearingNoiseAmplitudeDeg * bearing_noise
        )
        % 360.0,
        VelocityMps=(
            measured_velocity
            + config.DopplerNoiseAmplitudeMps * doppler_noise
        ),
        DopplerHz=0.0,
        AmplitudeDb=-25.0 - target_index,
        SnrDb=20.0 - target_index,
        TimeStamp=pose.TimestampSec,
    )
    AnnotateDetectionWithEarthReference(detection, pose)
    return detection


def _one_to_one_nearest(
    truth_positions: Sequence[EnuPosition],
    candidate_positions: Sequence[Tuple[float, float, object]],
    maximum_distance_m: float,
) -> Dict[int, object]:
    candidates = []
    for truth_index, truth in enumerate(truth_positions):
        for candidate_index, (east_m, north_m, value) in enumerate(
            candidate_positions
        ):
            candidates.append(
                (
                    _distance(
                        truth.east_m,
                        truth.north_m,
                        east_m,
                        north_m,
                    ),
                    truth_index,
                    candidate_index,
                    value,
                )
            )
    assigned_truth = set()
    assigned_candidates = set()
    result = {}
    for distance_m, truth_index, candidate_index, value in sorted(
        candidates,
        key=lambda item: item[0],
    ):
        if distance_m > maximum_distance_m:
            continue
        if truth_index in assigned_truth or candidate_index in assigned_candidates:
            continue
        assigned_truth.add(truth_index)
        assigned_candidates.add(candidate_index)
        result[truth_index] = value
    return result


def _track_id(track) -> int:
    return int(getattr(track, "TrackId", getattr(track, "TrackID")))


def _track_id_switches(track_ids: Sequence[int]) -> int:
    if not track_ids:
        return 0
    return sum(
        1
        for previous, current in zip(track_ids, track_ids[1:])
        if previous != current
    )


def _reacquired_same_track_after_misses(
    accumulator: _TargetAccumulator,
    missed_scans: Iterable[int],
) -> bool:
    missed = sorted(set(int(scan) for scan in missed_scans))
    if not missed:
        return True
    first_miss = missed[0]
    last_miss = missed[-1]
    before = [
        (scan, track_id)
        for scan, track_id in accumulator.earth_track_id_by_scan.items()
        if scan < first_miss
    ]
    after = [
        (scan, track_id)
        for scan, track_id in accumulator.earth_track_id_by_scan.items()
        if scan > last_miss
    ]
    if not before or not after:
        return False
    return max(before)[1] == min(after)[1]


def _build_target_result(
    accumulator: _TargetAccumulator,
    config: OperationalValidationConfig,
) -> TargetValidationResult:
    earth_count = len(accumulator.earth_position_errors_m)
    coverage = (
        earth_count / accumulator.expected_scans
        if accumulator.expected_scans
        else 0.0
    )
    earth_rms = _rms(accumulator.earth_position_errors_m)
    earth_max = _maximum(accumulator.earth_position_errors_m)
    velocity_rms = _rms(accumulator.earth_velocity_errors_mps)
    legacy_rms = _rms(accumulator.legacy_position_errors_m)
    legacy_max = _maximum(accumulator.legacy_position_errors_m)
    switches = _track_id_switches(accumulator.earth_track_ids)
    reacquired = _reacquired_same_track_after_misses(
        accumulator,
        (
            scan_id
            for scan_id in config.missed_scans(accumulator.target.Name)
            if 0 <= scan_id < config.ScansPerOrbit
        ),
    )

    failures = []
    if coverage < config.MinimumCoverageFraction:
        failures.append(
            f"coverage {coverage:.3f} is below "
            f"{config.MinimumCoverageFraction:.3f}"
        )
    if earth_rms is None or earth_rms > config.PositionRmsLimitM:
        failures.append(
            f"Earth position RMS {earth_rms!r} exceeds "
            f"{config.PositionRmsLimitM:.3f} m"
        )
    if earth_max is None or earth_max > config.PositionMaximumLimitM:
        failures.append(
            f"Earth position maximum {earth_max!r} exceeds "
            f"{config.PositionMaximumLimitM:.3f} m"
        )
    if velocity_rms is None or velocity_rms > config.VelocityRmsLimitMps:
        failures.append(
            f"Earth velocity RMS {velocity_rms!r} exceeds "
            f"{config.VelocityRmsLimitMps:.3f} m/s"
        )
    if switches > config.MaximumTrackIdSwitches:
        failures.append(
            f"Earth track ID switches {switches} exceed "
            f"{config.MaximumTrackIdSwitches}"
        )
    if not reacquired:
        failures.append("track was not reacquired with the same ID")

    return TargetValidationResult(
        Name=accumulator.target.Name,
        ExpectedEvaluatedScans=accumulator.expected_scans,
        EarthAssociatedScans=earth_count,
        LegacyAssociatedScans=len(accumulator.legacy_position_errors_m),
        CoverageFraction=coverage,
        EarthPositionRmsErrorM=earth_rms,
        EarthPositionMaximumErrorM=earth_max,
        EarthVelocityRmsErrorMps=velocity_rms,
        LegacyPositionRmsErrorM=legacy_rms,
        LegacyPositionMaximumErrorM=legacy_max,
        EarthTrackIdSwitches=switches,
        EarthTrackIds=tuple(sorted(set(accumulator.earth_track_ids))),
        ReacquiredSameTrackAfterMisses=reacquired,
        Passed=not failures,
        FailureReasons=tuple(failures),
    )


def RunOperationalCircularTrackerValidation(
    Config: Optional[OperationalValidationConfig] = None,
    Targets: Optional[Sequence[ValidationTarget]] = None,
) -> OperationalValidationReport:
    """Run one deterministic complete-orbit parallel tracker evaluation."""

    config = Config or OperationalValidationConfig()
    targets = tuple(_default_targets() if Targets is None else Targets)
    if not targets:
        raise ValueError("at least one validation target is required")
    if len({target.Name for target in targets}) != len(targets):
        raise ValueError("validation target names must be unique")

    clock = [config.StartTimestampSec]
    navigation = CircularRouteNavigationSource(
        mission_origin=config.MissionOrigin,
        centre_east_m=config.OrbitCentreEastM,
        centre_north_m=config.OrbitCentreNorthM,
        radius_m=config.OrbitRadiusM,
        speed_mps=config.OwnshipSpeedMps,
        clockwise=config.Clockwise,
        initial_radial_bearing_deg=config.InitialRadialBearingDeg,
        time_source=lambda: clock[0],
    )

    tracker_config = {
        "RangeBinM": 15.0,
        "ClusterRangeGapM": 100.0,
        "ClusterAzimuthGapDeg": 1.0,
        "InitiationRangeGateM": 350.0,
        "InitiationAzimuthGateDeg": 8.0,
        "AssociationRangeGateM": 350.0,
        "AssociationAzimuthGateDeg": 8.0,
        "TrackDtSec": config.scan_period_sec,
        "EarthClusterDistanceM": 100.0,
        "EarthInitiationGateM": 350.0,
        "EarthAssociationGateM": 350.0,
        "EarthTrackDtSec": config.scan_period_sec,
        "DeleteConfirmedAfterMisses": 5,
        "EarthDeleteConfirmedAfterMisses": 5,
    }
    legacy_tracker = RadarTracker(tracker_config)
    earth_tracker = EarthReferencedTracker(tracker_config)
    accumulators = {
        target.Name: _TargetAccumulator(target=target)
        for target in targets
    }

    maximum_earth_tracks = 0
    maximum_legacy_tracks = 0
    maximum_duplicate_earth_tracks = 0
    confirmation_warmup_scans = 3

    previous_pose = None
    previous_elapsed_sec = None
    previous_scan_id = None

    for scan_id in range(config.ScansPerOrbit + 1):
        elapsed_sec = scan_id * config.scan_period_sec
        clock[0] = config.StartTimestampSec + elapsed_sec
        pose = navigation.get_pose()

        detections = []
        if scan_id < config.ScansPerOrbit:
            for target_index, target in enumerate(targets):
                if scan_id in config.missed_scans(target.Name):
                    continue
                detections.append(
                    _make_detection(
                        target,
                        target_index,
                        elapsed_sec,
                        scan_id,
                        pose,
                        config,
                    )
                )

        this_dwell = {
            "ScanCycle": scan_id,
            "AzimuthDeg": 0.0,
        }
        legacy_tracker.Update(
            detections,
            ThisDwell=this_dwell,
        )
        earth_tracker.Update(
            detections,
            ThisDwell=this_dwell,
        )

        earth_tracks = earth_tracker.GetConfirmedTracks()
        legacy_tracks = legacy_tracker.GetConfirmedTracks()
        maximum_earth_tracks = max(maximum_earth_tracks, len(earth_tracks))
        maximum_legacy_tracks = max(maximum_legacy_tracks, len(legacy_tracks))
        maximum_duplicate_earth_tracks = max(
            maximum_duplicate_earth_tracks,
            max(0, len(earth_tracks) - len(targets)),
        )

        # A scan transition processes the detections accumulated in the
        # preceding scan.  Evaluate that completed scan against its own pose
        # and truth epoch, rather than the new scan's pose.
        if (
            previous_scan_id is not None
            and previous_scan_id >= confirmation_warmup_scans - 1
        ):
            truth_positions = [
                target.position_at(previous_elapsed_sec)
                for target in targets
            ]
            earth_candidates = [
                (track.EastM, track.NorthM, track)
                for track in earth_tracks
            ]
            legacy_candidates = []
            for track in legacy_tracks:
                position = detection_to_enu_position(
                    radar_position=previous_pose.PositionEnu,
                    measured_range_m=track.RangeM,
                    true_bearing_deg=track.AzimuthDeg,
                )
                legacy_candidates.append(
                    (position.east_m, position.north_m, track)
                )

            earth_assignments = _one_to_one_nearest(
                truth_positions,
                earth_candidates,
                maximum_distance_m=1000.0,
            )
            legacy_assignments = _one_to_one_nearest(
                truth_positions,
                legacy_candidates,
                maximum_distance_m=2000.0,
            )

            for target_index, target in enumerate(targets):
                accumulator = accumulators[target.Name]
                accumulator.expected_scans += 1
                truth = truth_positions[target_index]

                earth_track = earth_assignments.get(target_index)
                if earth_track is not None:
                    accumulator.earth_position_errors_m.append(
                        _distance(
                            earth_track.EastM,
                            earth_track.NorthM,
                            truth.east_m,
                            truth.north_m,
                        )
                    )
                    accumulator.earth_velocity_errors_mps.append(
                        _distance(
                            earth_track.VelocityEastMps,
                            earth_track.VelocityNorthMps,
                            target.VelocityEastMps,
                            target.VelocityNorthMps,
                        )
                    )
                    track_id = _track_id(earth_track)
                    accumulator.earth_track_ids.append(track_id)
                    accumulator.earth_track_id_by_scan[
                        previous_scan_id
                    ] = track_id

                legacy_track = legacy_assignments.get(target_index)
                if legacy_track is not None:
                    legacy_position = detection_to_enu_position(
                        radar_position=previous_pose.PositionEnu,
                        measured_range_m=legacy_track.RangeM,
                        true_bearing_deg=legacy_track.AzimuthDeg,
                    )
                    accumulator.legacy_position_errors_m.append(
                        _distance(
                            legacy_position.east_m,
                            legacy_position.north_m,
                            truth.east_m,
                            truth.north_m,
                        )
                    )
                    accumulator.legacy_track_ids.append(
                        _track_id(legacy_track)
                    )

        previous_pose = pose
        previous_elapsed_sec = elapsed_sec
        previous_scan_id = scan_id

    target_results = tuple(
        _build_target_result(accumulators[target.Name], config)
        for target in targets
    )
    failures = []
    for target_result in target_results:
        failures.extend(
            f"{target_result.Name}: {reason}"
            for reason in target_result.FailureReasons
        )
    if (
        maximum_duplicate_earth_tracks
        > config.MaximumDuplicateConfirmedTracks
    ):
        failures.append(
            "maximum duplicate Earth confirmed tracks "
            f"{maximum_duplicate_earth_tracks} exceed "
            f"{config.MaximumDuplicateConfirmedTracks}"
        )

    return OperationalValidationReport(
        Version=VALIDATION_VERSION,
        Passed=not failures,
        OrbitPeriodSec=config.orbit_period_sec,
        ScanPeriodSec=config.scan_period_sec,
        ScansPerOrbit=config.ScansPerOrbit,
        EvaluatedScans=sum(
            result.ExpectedEvaluatedScans for result in target_results
        ),
        MaximumEarthConfirmedTracks=maximum_earth_tracks,
        MaximumLegacyConfirmedTracks=maximum_legacy_tracks,
        MaximumDuplicateEarthConfirmedTracks=(
            maximum_duplicate_earth_tracks
        ),
        EarthTrackerAuthoritative=False,
        LegacyTrackerAuthoritative=True,
        TargetResults=target_results,
        FailureReasons=tuple(failures),
    )


def Main(CommandLineArguments: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Run the Vanguard X full-orbit parallel-tracker validation"
        )
    )
    parser.add_argument(
        "--json",
        action="store_true",
        help="print the report as JSON",
    )
    parser.add_argument(
        "--scans-per-orbit",
        type=int,
        default=72,
        help="number of complete radar scans during one ownship orbit",
    )
    arguments = parser.parse_args(CommandLineArguments)
    report = RunOperationalCircularTrackerValidation(
        OperationalValidationConfig(
            ScansPerOrbit=arguments.scans_per_orbit,
        )
    )
    print(report.to_json() if arguments.json else report.format_text())
    return 0 if report.Passed else 1


if __name__ == "__main__":
    raise SystemExit(Main())
