"""Earth-reference Vanguard X radar detections without changing tracker state.

The current tracker remains range/bearing based.  This module adds a parallel,
non-authoritative measurement to each detection so Earth-frame geometry can be
validated before the tracker itself is migrated to East/North state.

Conventions
-----------
* Detection ``AzimuthDeg`` is true bearing, clockwise from true north.
* Detection ``VelocityMps`` is measured radar-relative radial velocity.
* Positive radial velocity is outward/receding.
* Mission coordinates are East-North-Up (ENU), in metres.
* Ownship velocity is added exactly once to recover the target's Earth-frame
  line-of-sight velocity component.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Iterable, List, Optional

from CoordinateFrames import (
    detection_to_enu_position,
    normalise_bearing_degrees,
    target_los_velocity_from_measured_mps,
)


@dataclass(frozen=True)
class EarthReferencedDetectionMeasurement:
    """Parallel Earth-frame interpretation of one legacy radar detection."""

    Valid: bool
    VelocityValid: bool
    Reason: str

    EastM: Optional[float]
    NorthM: Optional[float]
    UpM: Optional[float]

    TrueBearingDeg: float
    MeasuredRangeM: float
    MeasuredRelativeRadialVelocityMps: float
    TargetLineOfSightVelocityMps: Optional[float]

    RadarEastM: Optional[float]
    RadarNorthM: Optional[float]
    RadarUpM: Optional[float]
    NavigationTimestampSec: Optional[float]
    NavigationSequenceNumber: Optional[int]
    NavigationSource: str


def _field(value: Any, name: str, default: Any = None) -> Any:
    if isinstance(value, dict):
        return value.get(name, default)
    return getattr(value, name, default)


def _set_field(value: Any, name: str, field_value: Any) -> None:
    if isinstance(value, dict):
        value[name] = field_value
    else:
        setattr(value, name, field_value)


def _finite_float(value: Any, name: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def BuildEarthReferencedDetectionMeasurement(
    Detection: Any,
    NavigationPose: Any,
) -> EarthReferencedDetectionMeasurement:
    """Build one Earth-frame measurement from a detection and pose snapshot.

    An invalid navigation pose produces an explicitly invalid measurement
    instead of dropping the legacy detection or preventing tracker operation.
    Invalid radar measurement fields still raise ``ValueError`` because they
    indicate a detector/interface fault rather than unavailable navigation.
    """

    MeasuredRangeM = _finite_float(
        _field(Detection, "RangeM"),
        "Detection.RangeM",
    )
    if MeasuredRangeM < 0.0:
        raise ValueError("Detection.RangeM must not be negative")

    TrueBearingDeg = normalise_bearing_degrees(
        _finite_float(
            _field(Detection, "AzimuthDeg"),
            "Detection.AzimuthDeg",
        )
    )
    MeasuredRelativeRadialVelocityMps = _finite_float(
        _field(Detection, "VelocityMps"),
        "Detection.VelocityMps",
    )

    PositionValid = bool(
        NavigationPose is not None
        and _field(NavigationPose, "PositionValid", False)
    )
    HeadingValid = bool(
        NavigationPose is not None
        and _field(NavigationPose, "HeadingValid", False)
    )
    NavigationSource = str(
        _field(NavigationPose, "Source", "UNAVAILABLE")
        if NavigationPose is not None
        else "UNAVAILABLE"
    )
    NavigationTimestampSec = (
        _finite_float(
            _field(NavigationPose, "TimestampSec"),
            "NavigationPose.TimestampSec",
        )
        if NavigationPose is not None
        else None
    )
    SequenceValue = (
        _field(NavigationPose, "SequenceNumber")
        if NavigationPose is not None
        else None
    )
    NavigationSequenceNumber = (
        int(SequenceValue) if SequenceValue is not None else None
    )

    if not PositionValid or not HeadingValid:
        Missing = []
        if not PositionValid:
            Missing.append("position")
        if not HeadingValid:
            Missing.append("heading")
        return EarthReferencedDetectionMeasurement(
            Valid=False,
            VelocityValid=False,
            Reason="INVALID_NAVIGATION_" + "_AND_".join(Missing).upper(),
            EastM=None,
            NorthM=None,
            UpM=None,
            TrueBearingDeg=TrueBearingDeg,
            MeasuredRangeM=MeasuredRangeM,
            MeasuredRelativeRadialVelocityMps=(
                MeasuredRelativeRadialVelocityMps
            ),
            TargetLineOfSightVelocityMps=None,
            RadarEastM=None,
            RadarNorthM=None,
            RadarUpM=None,
            NavigationTimestampSec=NavigationTimestampSec,
            NavigationSequenceNumber=NavigationSequenceNumber,
            NavigationSource=NavigationSource,
        )

    RadarPosition = _field(NavigationPose, "PositionEnu")
    EarthPosition = detection_to_enu_position(
        radar_position=RadarPosition,
        measured_range_m=MeasuredRangeM,
        true_bearing_deg=TrueBearingDeg,
    )

    VelocityValid = bool(_field(NavigationPose, "VelocityValid", False))
    TargetLineOfSightVelocityMps = None
    if VelocityValid:
        TargetLineOfSightVelocityMps = (
            target_los_velocity_from_measured_mps(
                MeasuredRelativeRadialVelocityMps,
                _field(NavigationPose, "VelocityEnu"),
                TrueBearingDeg,
            )
        )

    return EarthReferencedDetectionMeasurement(
        Valid=True,
        VelocityValid=VelocityValid,
        Reason="OK" if VelocityValid else "INVALID_NAVIGATION_VELOCITY",
        EastM=float(EarthPosition.east_m),
        NorthM=float(EarthPosition.north_m),
        UpM=float(EarthPosition.up_m),
        TrueBearingDeg=TrueBearingDeg,
        MeasuredRangeM=MeasuredRangeM,
        MeasuredRelativeRadialVelocityMps=(
            MeasuredRelativeRadialVelocityMps
        ),
        TargetLineOfSightVelocityMps=TargetLineOfSightVelocityMps,
        RadarEastM=float(RadarPosition.east_m),
        RadarNorthM=float(RadarPosition.north_m),
        RadarUpM=float(RadarPosition.up_m),
        NavigationTimestampSec=NavigationTimestampSec,
        NavigationSequenceNumber=NavigationSequenceNumber,
        NavigationSource=NavigationSource,
    )


def AnnotateDetectionWithEarthReference(
    Detection: Any,
    NavigationPose: Any,
) -> EarthReferencedDetectionMeasurement:
    """Attach a parallel Earth-reference contract to one detection."""

    Measurement = BuildEarthReferencedDetectionMeasurement(
        Detection,
        NavigationPose,
    )
    _set_field(Detection, "EarthReference", Measurement)
    _set_field(Detection, "EarthReferenceValid", Measurement.Valid)
    _set_field(Detection, "EarthVelocityValid", Measurement.VelocityValid)
    _set_field(Detection, "EarthReferenceReason", Measurement.Reason)
    _set_field(Detection, "EarthEastM", Measurement.EastM)
    _set_field(Detection, "EarthNorthM", Measurement.NorthM)
    _set_field(Detection, "EarthUpM", Measurement.UpM)
    _set_field(
        Detection,
        "MeasuredRelativeRadialVelocityMps",
        Measurement.MeasuredRelativeRadialVelocityMps,
    )
    _set_field(
        Detection,
        "TargetLineOfSightVelocityMps",
        Measurement.TargetLineOfSightVelocityMps,
    )
    _set_field(
        Detection,
        "NavigationPoseTimestampSec",
        Measurement.NavigationTimestampSec,
    )
    _set_field(
        Detection,
        "NavigationPoseSequenceNumber",
        Measurement.NavigationSequenceNumber,
    )
    return Measurement


def AnnotateDetectionsWithEarthReference(
    Detections: Iterable[Any],
    NavigationPose: Any,
) -> List[EarthReferencedDetectionMeasurement]:
    """Annotate detections in order and return their parallel measurements."""

    return [
        AnnotateDetectionWithEarthReference(Detection, NavigationPose)
        for Detection in Detections
    ]
