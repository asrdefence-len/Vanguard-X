"""Scan-based angular consolidation for Vanguard X radar detections.

Range-Doppler CFAR remains a per-dwell operation.  This module consumes those
CFAR cells, clusters them into one range-Doppler component per dwell, associates
the components through one antenna crossing, and fits the built-in two-way
antenna pattern.  One tracker-quality plot is emitted when the crossing closes.

The first implementation deliberately estimates a centroid rather than target
width.  Genuine range extent is retained as plot metadata, but it is not used
to claim a physical vessel dimension.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

from TargetScenario import antenna_gain_power_sinc


ANGULAR_PROCESSOR_VERSION = "angular-pattern-crossing-v2"


def _field(value: Any, names: Sequence[str], default: Any = None) -> Any:
    if value is None:
        return default
    for name in names:
        if isinstance(value, dict) and name in value:
            return value[name]
        if hasattr(value, name):
            return getattr(value, name)
    return default


def _container_field(
    value: Any,
    container_names: Sequence[str],
    field_names: Sequence[str],
    default: Any = None,
) -> Any:
    direct = _field(value, field_names, None)
    if direct is not None:
        return direct
    for container_name in container_names:
        container = _field(value, (container_name,), None)
        result = _field(container, field_names, None)
        if result is not None:
            return result
    return default


def _finite(value: Any, default: float = 0.0) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return float(default)
    return result if math.isfinite(result) else float(default)


def _signed_angle_delta_deg(a_deg: float, b_deg: float) -> float:
    return ((float(a_deg) - float(b_deg) + 180.0) % 360.0) - 180.0


def _angle_distance_deg(a_deg: float, b_deg: float) -> float:
    return abs(_signed_angle_delta_deg(a_deg, b_deg))


def _circular_weighted_mean_deg(
    angles_deg: Iterable[float],
    weights: Iterable[float],
) -> float:
    angles = np.asarray(list(angles_deg), dtype=float)
    weights_array = np.asarray(list(weights), dtype=float)
    if angles.size == 0:
        return 0.0
    if (
        weights_array.size != angles.size
        or not np.any(np.isfinite(weights_array))
        or np.sum(np.maximum(weights_array, 0.0)) <= 0.0
    ):
        weights_array = np.ones(angles.size, dtype=float)
    weights_array = np.maximum(weights_array, 0.0)
    radians = np.deg2rad(angles)
    sine = float(np.sum(weights_array * np.sin(radians)))
    cosine = float(np.sum(weights_array * np.cos(radians)))
    return float(np.rad2deg(math.atan2(sine, cosine)) % 360.0)


@dataclass
class AngularDetectionPlot:
    """One consolidated measurement from one antenna crossing."""

    PlotId: int
    DwellId: int
    RangeM: float
    AzimuthDeg: float
    DopplerHz: float
    VelocityMps: float
    AmplitudeDb: float
    SnrDb: float
    TimeStamp: float
    RangeBin: Optional[int] = None
    DopplerBin: Optional[int] = None
    NumCells: int = 1
    NumAngularSamples: int = 1
    NumCfarHitDwells: int = 1
    RangeMinimumM: float = 0.0
    RangeMaximumM: float = 0.0
    RangeExtentM: float = 0.0
    AngularResponseSpanDeg: float = 0.0
    BearingUncertaintyDeg: float = 0.0
    PatternFitQuality: float = 0.0
    PatternModel: str = "BUILT_IN_TWO_WAY_SINC4"
    Status: str = "ANGULAR_PLOT"
    IsConfirmed: bool = False
    TrackSource: str = "ANGULAR_PATTERN"

    @property
    def BearingDeg(self):
        return self.AzimuthDeg

    @property
    def AngleDeg(self):
        return self.AzimuthDeg

    @property
    def CentreRangeM(self):
        return self.RangeM

    @property
    def CenterRangeM(self):
        return self.RangeM

    @property
    def CentreAzimuthDeg(self):
        return self.AzimuthDeg

    @property
    def CenterAzimuthDeg(self):
        return self.AzimuthDeg


@dataclass
class _DwellComponent:
    DwellId: int
    BeamBearingDeg: float
    RangeM: float
    DopplerHz: float
    VelocityMps: float
    PowerLinear: float
    AmplitudeDb: float
    SnrDb: float
    TimeStamp: float
    RangeMinimumM: float
    RangeMaximumM: float
    RangeBin: Optional[int]
    DopplerBin: Optional[int]
    NumCells: int


@dataclass
class _AngularSample:
    BeamBearingDeg: float
    PowerLinear: float
    Component: Optional[_DwellComponent]
    CfarHit: bool


@dataclass
class _Crossing:
    CrossingId: int
    ContextKey: Tuple[Any, ...]
    RangeM: float
    DopplerHz: float
    Samples: List[_AngularSample] = field(default_factory=list)
    MissedDwells: int = 0
    LastBeamBearingDeg: float = 0.0
    LastHitBeamBearingDeg: float = 0.0
    ProvisionalBearingDeg: Optional[float] = None
    ProvisionalFitQuality: float = 0.0


class AngularDetectionProcessor:
    """Incremental angular-pattern estimator operating over one beam crossing."""

    def __init__(self, Config: Optional[Dict[str, Any]] = None):
        self.Config = Config or {}
        self.Enabled = bool(
            self.Config.get("AngularDetectionProcessingEnabled", True)
        )
        self.BeamwidthDeg = float(self.Config.get("BeamwidthDeg", 5.0))
        self.SidelobeFloorDb = float(
            self.Config.get("SidelobeFloorDb", -50.0)
        )
        self.RangeBinM = float(
            self.Config.get(
                "RangeBinM",
                self.Config.get("RangeResolutionM", 15.0),
            )
        )
        self.ClusterRangeGapM = float(
            self.Config.get(
                "AngularPerDwellClusterRangeGapM",
                max(7.0 * self.RangeBinM, 100.0),
            )
        )
        self.ClusterDopplerGapHz = float(
            self.Config.get("AngularPerDwellClusterDopplerGapHz", 120.0)
        )
        self.AssociationRangeGateM = float(
            self.Config.get("AngularAssociationRangeGateM", 250.0)
        )
        self.AssociationDopplerGateHz = float(
            self.Config.get("AngularAssociationDopplerGateHz", 180.0)
        )
        self.MaximumAngularSampleGapDeg = float(
            self.Config.get(
                "AngularMaximumSampleGapDeg",
                max(4.0, 0.8 * self.BeamwidthDeg),
            )
        )
        self.CloseAfterMissedDwells = int(
            self.Config.get("AngularCloseAfterMissedDwells", 2)
        )
        self.CloseAfterMissedAngleDeg = float(
            self.Config.get(
                "AngularCloseAfterMissedAngleDeg",
                self.BeamwidthDeg,
            )
        )
        self.MaximumMissedDwells = int(
            self.Config.get("AngularMaximumMissedDwells", 12)
        )
        self.MinimumFitSamples = int(
            self.Config.get("AngularMinimumFitSamples", 3)
        )
        self.HypothesisStepDeg = float(
            self.Config.get("AngularHypothesisStepDeg", 0.1)
        )
        self.PowerRangeHalfWidthBins = int(
            self.Config.get("AngularPowerRangeHalfWidthBins", 32)
        )
        self.PowerDopplerHalfWidthBins = int(
            self.Config.get("AngularPowerDopplerHalfWidthBins", 1)
        )
        self.MaximumDisplayPlots = int(
            self.Config.get("AngularMaximumDisplayPlots", 200)
        )
        self.DisplayPersistenceSec = float(
            self.Config.get("AngularPlotPersistenceSec", 30.0)
        )

        if self.BeamwidthDeg <= 0.0:
            raise ValueError("BeamwidthDeg must be greater than zero")
        if self.HypothesisStepDeg <= 0.0:
            raise ValueError("AngularHypothesisStepDeg must be greater than zero")
        if self.CloseAfterMissedDwells < 1:
            raise ValueError("AngularCloseAfterMissedDwells must be at least one")
        if self.CloseAfterMissedAngleDeg <= 0.0:
            raise ValueError(
                "AngularCloseAfterMissedAngleDeg must be greater than zero"
            )
        if self.MaximumMissedDwells < self.CloseAfterMissedDwells:
            raise ValueError(
                "AngularMaximumMissedDwells must be at least "
                "AngularCloseAfterMissedDwells"
            )

        self.ActiveCrossings: List[_Crossing] = []
        self.DisplayPlots: List[AngularDetectionPlot] = []
        self.CurrentContextKey: Optional[Tuple[Any, ...]] = None
        self.NextCrossingId = 1
        self.NextPlotId = 1
        self.LastDebug: Dict[str, Any] = {
            "AngularProcessorVersion": ANGULAR_PROCESSOR_VERSION,
            "Enabled": self.Enabled,
        }

    def Update(self, Detections, Processed=None, ThisDwell=None):
        """Consume one dwell and return only plots completed by this update."""

        detections = list(Detections or [])
        if not self.Enabled:
            return detections

        beam_bearing_deg = self._beam_bearing_deg(Processed, ThisDwell)
        context_key = self._context_key(Processed, ThisDwell)
        completed: List[AngularDetectionPlot] = []

        if self.CurrentContextKey is None:
            self.CurrentContextKey = context_key
        elif context_key != self.CurrentContextKey:
            completed.extend(self._close_all_crossings())
            self.CurrentContextKey = context_key

        components = self._cluster_dwell_detections(
            detections,
            beam_bearing_deg,
            Processed,
        )
        matched_crossing_ids = set()
        unmatched_components = []

        for component in sorted(
            components,
            key=lambda item: item.PowerLinear,
            reverse=True,
        ):
            crossing = self._best_crossing_for_component(
                component,
                matched_crossing_ids,
            )
            if crossing is None:
                unmatched_components.append(component)
                continue
            self._append_component_sample(crossing, component, Processed)
            crossing.MissedDwells = 0
            matched_crossing_ids.add(crossing.CrossingId)

        still_active = []
        for crossing in self.ActiveCrossings:
            if crossing.CrossingId in matched_crossing_ids:
                still_active.append(crossing)
                continue

            self._append_non_cfar_sample(
                crossing,
                beam_bearing_deg,
                Processed,
            )
            crossing.MissedDwells += 1
            missed_angle_deg = _angle_distance_deg(
                beam_bearing_deg,
                crossing.LastHitBeamBearingDeg,
            )
            angular_exit_complete = (
                crossing.MissedDwells >= self.CloseAfterMissedDwells
                and missed_angle_deg >= self.CloseAfterMissedAngleDeg
            )
            safety_timeout = (
                crossing.MissedDwells >= self.MaximumMissedDwells
            )
            if angular_exit_complete or safety_timeout:
                completed.append(self._make_plot(crossing))
            else:
                still_active.append(crossing)
        self.ActiveCrossings = still_active

        for component in unmatched_components:
            self.ActiveCrossings.append(
                self._start_crossing(component, context_key, Processed)
            )

        self._publish_completed(completed)
        self._prune_display_plots(self._timestamp(Processed))
        self.LastDebug = {
            "AngularProcessorVersion": ANGULAR_PROCESSOR_VERSION,
            "Enabled": True,
            "RawDetectionsThisDwell": len(detections),
            "DwellComponentsThisDwell": len(components),
            "ActiveCrossings": len(self.ActiveCrossings),
            "CompletedPlotsThisDwell": len(completed),
            "DisplayedAngularPlots": len(self.DisplayPlots),
            "BeamBearingTrueDeg": beam_bearing_deg,
            "CloseAfterMissedAngleDeg": self.CloseAfterMissedAngleDeg,
            "PatternModel": "BUILT_IN_TWO_WAY_SINC4",
        }
        return completed

    def Flush(self):
        completed = self._close_all_crossings()
        self._publish_completed(completed)
        return completed

    def FinalizeCurrentContext(self):
        """Close the current finite scan task at its commanded endpoint.

        A track nod is intentionally narrower than the normal angular-exit
        distance, so waiting for a later search dwell to change context would
        delay the confirmation result.  The pointing state proves that all
        commanded nod passes reached both gate edges before this is called.
        """

        completed = self._close_all_crossings()
        self._publish_completed(completed)
        self.LastDebug = dict(self.LastDebug)
        self.LastDebug["ActiveCrossings"] = 0
        self.LastDebug["CompletedPlotsThisDwell"] = len(completed)
        self.LastDebug["DisplayedAngularPlots"] = len(self.DisplayPlots)
        self.LastDebug["FinalizedFiniteTask"] = True
        return completed

    def GetDisplayPlots(self):
        return list(self.DisplayPlots)

    def GetDebugInfo(self):
        return dict(self.LastDebug)

    def _cluster_dwell_detections(
        self,
        detections: List[Any],
        beam_bearing_deg: float,
        processed: Any,
    ) -> List[_DwellComponent]:
        if not detections:
            return []

        used = [False] * len(detections)
        components = []
        for index in range(len(detections)):
            if used[index]:
                continue
            stack = [index]
            used[index] = True
            indices = []
            while stack:
                detection_index = stack.pop()
                indices.append(detection_index)
                detection = detections[detection_index]
                detection_range_m = _finite(
                    _field(detection, ("RangeM",), 0.0)
                )
                detection_doppler_hz = _finite(
                    _field(detection, ("DopplerHz",), 0.0)
                )
                for candidate_index, candidate in enumerate(detections):
                    if used[candidate_index]:
                        continue
                    candidate_range_m = _finite(
                        _field(candidate, ("RangeM",), 0.0)
                    )
                    candidate_doppler_hz = _finite(
                        _field(candidate, ("DopplerHz",), 0.0)
                    )
                    if (
                        abs(candidate_range_m - detection_range_m)
                        <= self.ClusterRangeGapM
                        and abs(candidate_doppler_hz - detection_doppler_hz)
                        <= self.ClusterDopplerGapHz
                    ):
                        used[candidate_index] = True
                        stack.append(candidate_index)

            components.append(
                self._make_dwell_component(
                    [detections[item] for item in indices],
                    beam_bearing_deg,
                    processed,
                )
            )
        return components

    def _make_dwell_component(
        self,
        detections: List[Any],
        beam_bearing_deg: float,
        processed: Any,
    ) -> _DwellComponent:
        amplitudes_db = np.asarray(
            [
                _finite(_field(item, ("AmplitudeDb",), -300.0), -300.0)
                for item in detections
            ],
            dtype=float,
        )
        powers = np.power(10.0, amplitudes_db / 10.0)
        if not np.any(np.isfinite(powers)) or float(np.sum(powers)) <= 0.0:
            powers = np.ones(len(detections), dtype=float)
        ranges = np.asarray(
            [_finite(_field(item, ("RangeM",), 0.0)) for item in detections],
            dtype=float,
        )
        dopplers = np.asarray(
            [
                _finite(_field(item, ("DopplerHz",), 0.0))
                for item in detections
            ],
            dtype=float,
        )
        velocities = np.asarray(
            [
                _finite(_field(item, ("VelocityMps",), 0.0))
                for item in detections
            ],
            dtype=float,
        )
        weight_sum = max(float(np.sum(powers)), 1.0e-30)
        strongest = int(np.argmax(powers))
        range_bins = [
            _field(item, ("RangeBin",), None) for item in detections
        ]
        doppler_bins = [
            _field(item, ("DopplerBin",), None) for item in detections
        ]
        return _DwellComponent(
            DwellId=int(
                _field(
                    detections[strongest],
                    ("DwellId",),
                    _container_field(
                        processed,
                        ("Diagnostics",),
                        ("DwellId",),
                        0,
                    ),
                )
            ),
            BeamBearingDeg=float(beam_bearing_deg) % 360.0,
            RangeM=float(np.sum(powers * ranges) / weight_sum),
            DopplerHz=float(np.sum(powers * dopplers) / weight_sum),
            VelocityMps=float(np.sum(powers * velocities) / weight_sum),
            PowerLinear=weight_sum,
            AmplitudeDb=float(np.max(amplitudes_db)),
            SnrDb=max(
                _finite(_field(item, ("SnrDb", "SNRDb"), 0.0))
                for item in detections
            ),
            TimeStamp=float(
                _field(
                    detections[strongest],
                    ("TimeStamp",),
                    self._timestamp(processed),
                )
            ),
            RangeMinimumM=float(np.min(ranges)),
            RangeMaximumM=float(np.max(ranges)),
            RangeBin=(
                int(range_bins[strongest])
                if range_bins[strongest] is not None
                else None
            ),
            DopplerBin=(
                int(doppler_bins[strongest])
                if doppler_bins[strongest] is not None
                else None
            ),
            NumCells=len(detections),
        )

    def _start_crossing(
        self,
        component: _DwellComponent,
        context_key: Tuple[Any, ...],
        processed: Any,
    ) -> _Crossing:
        crossing = _Crossing(
            CrossingId=self.NextCrossingId,
            ContextKey=context_key,
            RangeM=component.RangeM,
            DopplerHz=component.DopplerHz,
            LastBeamBearingDeg=component.BeamBearingDeg,
            LastHitBeamBearingDeg=component.BeamBearingDeg,
        )
        self.NextCrossingId += 1
        self._append_component_sample(crossing, component, processed)
        return crossing

    def _best_crossing_for_component(
        self,
        component: _DwellComponent,
        excluded_ids,
    ) -> Optional[_Crossing]:
        best = None
        best_score = float("inf")
        for crossing in self.ActiveCrossings:
            if crossing.CrossingId in excluded_ids:
                continue
            range_error_m = abs(component.RangeM - crossing.RangeM)
            doppler_error_hz = abs(component.DopplerHz - crossing.DopplerHz)
            angular_gap_deg = _angle_distance_deg(
                component.BeamBearingDeg,
                crossing.LastBeamBearingDeg,
            )
            maximum_gap_deg = (
                self.MaximumAngularSampleGapDeg
                * (crossing.MissedDwells + 1)
            )
            if (
                range_error_m > self.AssociationRangeGateM
                or doppler_error_hz > self.AssociationDopplerGateHz
                or angular_gap_deg > maximum_gap_deg
            ):
                continue
            score = (
                range_error_m / max(self.AssociationRangeGateM, 1.0e-9)
                + doppler_error_hz
                / max(self.AssociationDopplerGateHz, 1.0e-9)
                + angular_gap_deg / max(maximum_gap_deg, 1.0e-9)
            )
            if score < best_score:
                best = crossing
                best_score = score
        return best

    def _append_component_sample(
        self,
        crossing: _Crossing,
        component: _DwellComponent,
        processed: Any,
    ) -> None:
        power = self._sample_processed_power(
            processed,
            component.RangeBin,
            component.DopplerBin,
        )
        if power is None:
            power = component.PowerLinear
        crossing.Samples.append(
            _AngularSample(
                BeamBearingDeg=component.BeamBearingDeg,
                PowerLinear=max(float(power), 1.0e-30),
                Component=component,
                CfarHit=True,
            )
        )
        crossing.LastBeamBearingDeg = component.BeamBearingDeg
        crossing.LastHitBeamBearingDeg = component.BeamBearingDeg
        hit_samples = [
            sample.Component
            for sample in crossing.Samples
            if sample.CfarHit and sample.Component is not None
        ]
        weights = [
            sample.PowerLinear for sample in crossing.Samples if sample.CfarHit
        ]
        total_weight = max(float(np.sum(weights)), 1.0e-30)
        crossing.RangeM = float(
            sum(
                weight * component_item.RangeM
                for weight, component_item in zip(weights, hit_samples)
            )
            / total_weight
        )
        crossing.DopplerHz = float(
            sum(
                weight * component_item.DopplerHz
                for weight, component_item in zip(weights, hit_samples)
            )
            / total_weight
        )
        self._update_provisional_fit(crossing)

    def _append_non_cfar_sample(
        self,
        crossing: _Crossing,
        beam_bearing_deg: float,
        processed: Any,
    ) -> None:
        if not crossing.Samples:
            return
        latest_component = next(
            (
                sample.Component
                for sample in reversed(crossing.Samples)
                if sample.Component is not None
            ),
            None,
        )
        power = None
        if latest_component is not None:
            power = self._sample_processed_power(
                processed,
                latest_component.RangeBin,
                latest_component.DopplerBin,
            )
        if power is None:
            power = min(sample.PowerLinear for sample in crossing.Samples)
        crossing.Samples.append(
            _AngularSample(
                BeamBearingDeg=float(beam_bearing_deg) % 360.0,
                PowerLinear=max(float(power), 1.0e-30),
                Component=None,
                CfarHit=False,
            )
        )
        crossing.LastBeamBearingDeg = float(beam_bearing_deg) % 360.0
        self._update_provisional_fit(crossing)

    def _sample_processed_power(
        self,
        processed: Any,
        range_bin: Optional[int],
        doppler_bin: Optional[int],
    ) -> Optional[float]:
        if processed is None or range_bin is None or doppler_bin is None:
            return None
        range_doppler_map = _field(processed, ("RangeDopplerMap",), None)
        if range_doppler_map is None:
            return None
        array = np.asarray(range_doppler_map)
        if array.ndim != 2 or array.size == 0:
            return None
        range_start = max(0, int(range_bin) - self.PowerRangeHalfWidthBins)
        range_stop = min(
            array.shape[1],
            int(range_bin) + self.PowerRangeHalfWidthBins + 1,
        )
        doppler_start = max(
            0,
            int(doppler_bin) - self.PowerDopplerHalfWidthBins,
        )
        doppler_stop = min(
            array.shape[0],
            int(doppler_bin) + self.PowerDopplerHalfWidthBins + 1,
        )
        if range_start >= range_stop or doppler_start >= doppler_stop:
            return None
        window = array[doppler_start:doppler_stop, range_start:range_stop]
        return float(np.sum(np.abs(window) ** 2))

    def _update_provisional_fit(self, crossing: _Crossing) -> None:
        bearing, quality, _ = self._fit_bearing(crossing.Samples)
        crossing.ProvisionalBearingDeg = bearing
        crossing.ProvisionalFitQuality = quality

    def _fit_bearing(
        self,
        samples: List[_AngularSample],
    ) -> Tuple[float, float, float]:
        if not samples:
            return 0.0, 0.0, self.BeamwidthDeg

        angles = np.asarray(
            [sample.BeamBearingDeg for sample in samples],
            dtype=float,
        )
        powers = np.asarray(
            [max(sample.PowerLinear, 1.0e-30) for sample in samples],
            dtype=float,
        )
        peak_index = int(np.argmax(powers))
        reference_deg = float(angles[peak_index])
        unwrapped = np.asarray(
            [
                reference_deg
                + _signed_angle_delta_deg(angle_deg, reference_deg)
                for angle_deg in angles
            ],
            dtype=float,
        )

        if len(samples) < self.MinimumFitSamples:
            bearing = _circular_weighted_mean_deg(angles, powers)
            spacing = self._sample_spacing_deg(unwrapped)
            return bearing, 0.0, max(0.5 * spacing, 0.25)

        margin_deg = self.BeamwidthDeg
        hypotheses = np.arange(
            float(np.min(unwrapped) - margin_deg),
            float(np.max(unwrapped) + margin_deg)
            + 0.5 * self.HypothesisStepDeg,
            self.HypothesisStepDeg,
        )
        observation_weights = np.asarray(
            [1.0 if sample.CfarHit else 0.5 for sample in samples],
            dtype=float,
        )
        best_hypothesis = reference_deg
        best_error = float("inf")
        errors = np.empty(hypotheses.size, dtype=float)

        for index, hypothesis_deg in enumerate(hypotheses):
            one_way = antenna_gain_power_sinc(
                unwrapped - hypothesis_deg,
                self.BeamwidthDeg,
                self.SidelobeFloorDb,
            )
            two_way_pattern = np.asarray(one_way, dtype=float) ** 2
            design = np.column_stack(
                (two_way_pattern, np.ones(two_way_pattern.size))
            )
            weighted_design = design * np.sqrt(observation_weights)[:, None]
            weighted_power = powers * np.sqrt(observation_weights)
            amplitude, background = np.linalg.lstsq(
                weighted_design,
                weighted_power,
                rcond=None,
            )[0]
            amplitude = max(float(amplitude), 0.0)
            background = max(float(background), 0.0)
            residual = powers - (
                amplitude * two_way_pattern + background
            )
            error = float(np.sum(observation_weights * residual * residual))
            errors[index] = error
            if error < best_error:
                best_error = error
                best_hypothesis = float(hypothesis_deg)

        weighted_mean = float(
            np.sum(observation_weights * powers)
            / max(np.sum(observation_weights), 1.0e-30)
        )
        total_variation = float(
            np.sum(
                observation_weights * (powers - weighted_mean) ** 2
            )
        )
        fit_quality = (
            max(0.0, min(1.0, 1.0 - best_error / total_variation))
            if total_variation > 1.0e-30
            else 0.0
        )
        spacing_deg = self._sample_spacing_deg(unwrapped)
        uncertainty_deg = max(
            0.25,
            0.25 * spacing_deg,
            self.HypothesisStepDeg,
        )
        if fit_quality < 0.5:
            uncertainty_deg = max(uncertainty_deg, 0.5 * spacing_deg)
        return best_hypothesis % 360.0, fit_quality, uncertainty_deg

    def _make_plot(self, crossing: _Crossing) -> AngularDetectionPlot:
        hit_samples = [
            sample
            for sample in crossing.Samples
            if sample.CfarHit and sample.Component is not None
        ]
        if not hit_samples:
            raise RuntimeError("cannot create angular plot without a CFAR seed")
        bearing_deg, fit_quality, uncertainty_deg = self._fit_bearing(
            crossing.Samples
        )
        hit_powers = np.asarray(
            [sample.PowerLinear for sample in hit_samples],
            dtype=float,
        )
        hit_components = [sample.Component for sample in hit_samples]
        power_sum = max(float(np.sum(hit_powers)), 1.0e-30)
        range_m = float(
            sum(
                power * component.RangeM
                for power, component in zip(hit_powers, hit_components)
            )
            / power_sum
        )
        doppler_hz = float(
            sum(
                power * component.DopplerHz
                for power, component in zip(hit_powers, hit_components)
            )
            / power_sum
        )
        velocity_mps = float(
            sum(
                power * component.VelocityMps
                for power, component in zip(hit_powers, hit_components)
            )
            / power_sum
        )
        range_minimum_m = min(
            component.RangeMinimumM for component in hit_components
        )
        range_maximum_m = max(
            component.RangeMaximumM for component in hit_components
        )
        hit_angles = np.asarray(
            [
                reference
                + _signed_angle_delta_deg(
                    sample.BeamBearingDeg,
                    reference,
                )
                for sample in hit_samples
                for reference in [hit_samples[0].BeamBearingDeg]
            ],
            dtype=float,
        )
        strongest_component = hit_components[int(np.argmax(hit_powers))]
        plot = AngularDetectionPlot(
            PlotId=self.NextPlotId,
            DwellId=strongest_component.DwellId,
            RangeM=range_m,
            AzimuthDeg=bearing_deg,
            DopplerHz=doppler_hz,
            VelocityMps=velocity_mps,
            AmplitudeDb=max(
                component.AmplitudeDb for component in hit_components
            ),
            SnrDb=max(component.SnrDb for component in hit_components),
            TimeStamp=float(
                np.median(
                    [component.TimeStamp for component in hit_components]
                )
            ),
            RangeBin=strongest_component.RangeBin,
            DopplerBin=strongest_component.DopplerBin,
            NumCells=sum(component.NumCells for component in hit_components),
            NumAngularSamples=len(crossing.Samples),
            NumCfarHitDwells=len(hit_samples),
            RangeMinimumM=range_minimum_m,
            RangeMaximumM=range_maximum_m,
            RangeExtentM=max(0.0, range_maximum_m - range_minimum_m),
            AngularResponseSpanDeg=(
                float(np.max(hit_angles) - np.min(hit_angles))
                if hit_angles.size > 1
                else 0.0
            ),
            BearingUncertaintyDeg=uncertainty_deg,
            PatternFitQuality=fit_quality,
        )
        self.NextPlotId += 1
        return plot

    def _close_all_crossings(self):
        completed = [
            self._make_plot(crossing)
            for crossing in self.ActiveCrossings
            if any(sample.CfarHit for sample in crossing.Samples)
        ]
        self.ActiveCrossings = []
        return completed

    def _publish_completed(
        self,
        completed: Iterable[AngularDetectionPlot],
    ) -> None:
        self.DisplayPlots.extend(completed)
        if len(self.DisplayPlots) > self.MaximumDisplayPlots:
            self.DisplayPlots = self.DisplayPlots[-self.MaximumDisplayPlots :]

    def _prune_display_plots(self, timestamp_sec: float) -> None:
        if self.DisplayPersistenceSec <= 0.0:
            self.DisplayPlots = []
            return
        cutoff = float(timestamp_sec) - self.DisplayPersistenceSec
        self.DisplayPlots = [
            plot for plot in self.DisplayPlots if plot.TimeStamp >= cutoff
        ]

    def _beam_bearing_deg(self, processed: Any, dwell: Any) -> float:
        value = _container_field(
            processed,
            ("Diagnostics",),
            (
                "BoresightDeg",
                "BeamAngleDeg",
                "BeamBearingTrueDeg",
                "AzimuthDeg",
            ),
            None,
        )
        if value is None:
            value = _container_field(
                dwell,
                ("Metadata",),
                (
                    "BoresightDeg",
                    "BeamAngleDeg",
                    "BeamBearingTrueDeg",
                    "AzimuthDeg",
                ),
                0.0,
            )
        return float(value) % 360.0

    def _context_key(self, processed: Any, dwell: Any) -> Tuple[Any, ...]:
        task_id = _container_field(
            dwell,
            ("Metadata",),
            ("TaskId", "ScheduledTaskId"),
            _container_field(
                processed,
                ("Diagnostics",),
                ("ScheduledTaskId", "TaskId"),
                None,
            ),
        )
        task_type = _container_field(
            dwell,
            ("Metadata",),
            ("TaskType", "ScheduledTaskType"),
            _container_field(
                processed,
                ("Diagnostics",),
                ("ScheduledTaskType", "TaskType"),
                "",
            ),
        )
        # A sector endpoint increments ScanCycle while the antenna is still
        # inside the response of a target near that endpoint. Treating the
        # cycle number as a hard boundary splits one physical crossing into a
        # leading-edge and trailing-edge plot. Keep the crossing alive across
        # a reversal of the same task; angular-exit logic closes it after the
        # beam has actually moved clear. A real task change still flushes all
        # active crossings.
        return (task_id, str(task_type))

    def _timestamp(self, processed: Any) -> float:
        return _finite(
            _field(processed, ("TimeStamp",), 0.0),
            0.0,
        )

    @staticmethod
    def _sample_spacing_deg(unwrapped_angles_deg: np.ndarray) -> float:
        unique = np.unique(np.round(unwrapped_angles_deg, decimals=9))
        if unique.size < 2:
            return 1.0
        differences = np.diff(np.sort(unique))
        positive = differences[differences > 1.0e-9]
        return float(np.median(positive)) if positive.size else 1.0
