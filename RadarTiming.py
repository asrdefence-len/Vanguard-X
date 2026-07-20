"""
===============================================================================
Vanguard X Radar Timing Model
RadarTiming.py
===============================================================================

This module contains the hardware-independent timing calculation used to turn
operator intent and waveform metadata into one validated radar timing solution.

The initial timing architecture uses:

    fixed SDR sample rate:       40 MS/s
    operator PRF range:          1 to 4 kHz
    default PRF:                 2 kHz
    default pulses per CPI:      32

The transport burst begins with eight zero samples (200 ns at 40 MS/s), then
the unchanged waveform-library samples establish the RF/range time origin.
Receive capture starts after the complete ATR transmit envelope and provisional
receiver-recovery interval. Capture continues until the complete coded return
from maximum instrumented range, plus the configured end margin, is available.

No hardware is commanded here.  The resulting immutable solution will later be
consumed by RadarExecutor, RadarPlans, SimulatedSource, EttusRadarSource and
RadarProcessor.
===============================================================================
"""

from dataclasses import asdict, dataclass
import math
from typing import Any, Dict, Mapping


SPEED_OF_LIGHT_MPS = 299_792_458.0
FIXED_SAMPLE_RATE_HZ = 40.0e6


@dataclass(frozen=True)
class RadarTimingSolution:
    """Complete validated timing solution for one uniform-PRI dwell profile."""

    WaveformId: str
    SampleRateHz: float
    ChipRateHz: float
    ChipCount: int
    SamplesPerChip: int
    TxWaveformSamples: int
    TxPulseDurationSec: float
    TxLeadingZeroSamples: int
    TxLeadingZeroDurationSec: float
    TxAtrEnvelopeSamples: int
    TxAtrEnvelopeDurationSec: float
    RfPulseStartDelaySec: float
    RfPulseEndDelaySec: float

    SelectedPrfHz: float
    PriSec: float
    OperatorMinPrfHz: float
    OperatorMaxPrfHz: float
    TimingLimitedMaxPrfHz: float
    EffectiveMaxPrfHz: float

    PulsesPerCpi: int
    CpiDurationSec: float
    PulseTrainSpanSec: float
    AcquisitionEndFromFirstPulseSec: float

    MaximumRangeM: float
    MaximumEchoLeadingEdgeDelaySec: float
    ReceiverRecoveryTimeSec: float
    RxEndMarginSec: float
    NextTxGuardTimeSec: float

    RxStartDelaySec: float
    RequiredRxEndDelaySec: float
    RequestedRxCaptureDurationSec: float
    NumRxSamples: int
    ActualRxCaptureDurationSec: float
    ActualRxEndDelaySec: float
    MinimumPriSec: float
    TimingMarginAboveMinimumPriSec: float

    MinimumFullEchoRangeM: float
    FirstRxSampleRangeOffsetM: float
    MaximumUnambiguousRangeM: float
    UnambiguousRadialVelocityMps: float
    VelocityBinSpacingMps: float
    AntennaMovementDuringCpiDeg: float
    TransmitDutyCycle: float
    ReceiveCaptureDutyCycle: float

    BytesPerPulseSc16: int
    BytesPerPulseFc32: int
    BytesPerCpiSc16: int
    BytesPerCpiFc32: int
    TimingSafe: bool

    def ToMetadata(self) -> Dict[str, Any]:
        """Return a serialisable copy suitable for DwellPlan diagnostics."""

        return asdict(self)


def _RequireFinite(Name, Value):
    Value = float(Value)
    if not math.isfinite(Value):
        raise ValueError(f"{Name} must be finite")
    return Value


def _RequirePositive(Name, Value):
    Value = _RequireFinite(Name, Value)
    if Value <= 0.0:
        raise ValueError(f"{Name} must be greater than zero")
    return Value


def _RequireNonNegative(Name, Value):
    Value = _RequireFinite(Name, Value)
    if Value < 0.0:
        raise ValueError(f"{Name} must not be negative")
    return Value


def _RequirePositiveInteger(Name, Value):
    if isinstance(Value, bool):
        raise ValueError(f"{Name} must be a positive integer")
    IntegerValue = int(Value)
    if IntegerValue <= 0 or float(Value) != float(IntegerValue):
        raise ValueError(f"{Name} must be a positive integer")
    return IntegerValue


def _RequireNonNegativeInteger(Name, Value):
    if isinstance(Value, bool):
        raise ValueError(f"{Name} must be a non-negative integer")
    IntegerValue = int(Value)
    if IntegerValue < 0 or float(Value) != float(IntegerValue):
        raise ValueError(f"{Name} must be a non-negative integer")
    return IntegerValue


def _MetadataValue(WaveformMetadata, Name):
    try:
        return WaveformMetadata[Name]
    except KeyError as Error:
        raise ValueError(
            f"Waveform metadata is missing required field: {Name}"
        ) from Error


def CalculateRadarTiming(
    WaveformMetadata: Mapping[str, Any],
    *,
    SelectedPrfHz: float = 2000.0,
    PulsesPerCpi: int = 32,
    MaximumRangeM: float = 15000.0,
    ReceiverRecoveryTimeSec: float = 1.0e-6,
    RxEndMarginSec: float = 2.0e-6,
    NextTxGuardTimeSec: float = 2.0e-6,
    OperatorMinPrfHz: float = 1000.0,
    OperatorMaxPrfHz: float = 4000.0,
    RfFrequencyHz: float = 9.4e9,
    AntennaScanRateDegPerSec: float = 90.0,
    TxLeadingZeroSamples: int = 8,
) -> RadarTimingSolution:
    """Calculate and validate one uniform-PRI Vanguard X timing solution."""

    WaveformId = str(_MetadataValue(WaveformMetadata, "WaveformId"))
    if not WaveformId:
        raise ValueError("WaveformId must not be empty")

    SampleRateHz = _RequirePositive(
        "SampleRateHz",
        _MetadataValue(WaveformMetadata, "SampleRateHz"),
    )
    if not math.isclose(
        SampleRateHz,
        FIXED_SAMPLE_RATE_HZ,
        rel_tol=0.0,
        abs_tol=1.0,
    ):
        raise ValueError(
            "The initial timing architecture requires a fixed 40 MS/s "
            f"sample rate; received {SampleRateHz / 1e6:g} MS/s"
        )

    ChipRateHz = _RequirePositive(
        "ChipRateHz",
        _MetadataValue(WaveformMetadata, "ChipRateHz"),
    )
    ChipCount = _RequirePositiveInteger(
        "ChipCount",
        _MetadataValue(WaveformMetadata, "ChipCount"),
    )
    SamplesPerChip = _RequirePositiveInteger(
        "SamplesPerChip",
        _MetadataValue(WaveformMetadata, "SamplesPerChip"),
    )
    TxWaveformSamples = _RequirePositiveInteger(
        "NumSamples",
        _MetadataValue(WaveformMetadata, "NumSamples"),
    )
    TxPulseDurationSec = _RequirePositive(
        "PulseDurationSec",
        _MetadataValue(WaveformMetadata, "PulseDurationSec"),
    )
    TxLeadingZeroSamples = _RequireNonNegativeInteger(
        "TxLeadingZeroSamples",
        TxLeadingZeroSamples,
    )

    ExpectedTxSamples = ChipCount * SamplesPerChip
    if TxWaveformSamples != ExpectedTxSamples:
        raise ValueError(
            f"Waveform {WaveformId} metadata is inconsistent: "
            f"NumSamples={TxWaveformSamples}, expected {ExpectedTxSamples}"
        )
    ExpectedPulseDurationSec = TxWaveformSamples / SampleRateHz
    if not math.isclose(
        TxPulseDurationSec,
        ExpectedPulseDurationSec,
        rel_tol=0.0,
        abs_tol=1e-15,
    ):
        raise ValueError(
            f"Waveform {WaveformId} pulse duration is inconsistent with its "
            "sample count and sample rate"
        )

    SelectedPrfHz = _RequirePositive("SelectedPrfHz", SelectedPrfHz)
    OperatorMinPrfHz = _RequirePositive(
        "OperatorMinPrfHz",
        OperatorMinPrfHz,
    )
    OperatorMaxPrfHz = _RequirePositive(
        "OperatorMaxPrfHz",
        OperatorMaxPrfHz,
    )
    if OperatorMinPrfHz > OperatorMaxPrfHz:
        raise ValueError(
            "OperatorMinPrfHz must not exceed OperatorMaxPrfHz"
        )
    if not OperatorMinPrfHz <= SelectedPrfHz <= OperatorMaxPrfHz:
        raise ValueError(
            f"Selected PRF {SelectedPrfHz:g} Hz is outside the operator "
            f"range {OperatorMinPrfHz:g}-{OperatorMaxPrfHz:g} Hz"
        )

    PulsesPerCpi = _RequirePositiveInteger("PulsesPerCpi", PulsesPerCpi)
    MaximumRangeM = _RequirePositive("MaximumRangeM", MaximumRangeM)
    ReceiverRecoveryTimeSec = _RequireNonNegative(
        "ReceiverRecoveryTimeSec",
        ReceiverRecoveryTimeSec,
    )
    RxEndMarginSec = _RequireNonNegative(
        "RxEndMarginSec",
        RxEndMarginSec,
    )
    NextTxGuardTimeSec = _RequireNonNegative(
        "NextTxGuardTimeSec",
        NextTxGuardTimeSec,
    )
    RfFrequencyHz = _RequirePositive("RfFrequencyHz", RfFrequencyHz)
    AntennaScanRateDegPerSec = _RequireFinite(
        "AntennaScanRateDegPerSec",
        AntennaScanRateDegPerSec,
    )

    PriSec = 1.0 / SelectedPrfHz
    TxLeadingZeroDurationSec = TxLeadingZeroSamples / SampleRateHz
    TxAtrEnvelopeSamples = TxLeadingZeroSamples + TxWaveformSamples
    TxAtrEnvelopeDurationSec = TxAtrEnvelopeSamples / SampleRateHz
    RfPulseStartDelaySec = TxLeadingZeroDurationSec
    RfPulseEndDelaySec = (
        RfPulseStartDelaySec + TxPulseDurationSec
    )
    MaximumEchoLeadingEdgeDelaySec = (
        2.0 * MaximumRangeM / SPEED_OF_LIGHT_MPS
    )
    RxStartDelaySec = (
        TxAtrEnvelopeDurationSec + ReceiverRecoveryTimeSec
    )
    MinimumFullEchoRangeM = (
        SPEED_OF_LIGHT_MPS
        * (RxStartDelaySec - RfPulseStartDelaySec)
        / 2.0
    )
    if MaximumRangeM <= MinimumFullEchoRangeM:
        raise ValueError(
            f"Maximum range {MaximumRangeM:g} m must exceed the minimum "
            f"full-echo range {MinimumFullEchoRangeM:.3f} m for waveform "
            f"{WaveformId}"
        )
    RequiredRxEndDelaySec = (
        RfPulseStartDelaySec
        + MaximumEchoLeadingEdgeDelaySec
        + TxPulseDurationSec
        + RxEndMarginSec
    )
    RequestedRxCaptureDurationSec = (
        RequiredRxEndDelaySec - RxStartDelaySec
    )
    if RequestedRxCaptureDurationSec <= 0.0:
        raise ValueError(
            "Maximum range and receive margins provide no positive receive "
            "capture interval after receiver recovery"
        )

    # Ceiling guarantees that the complete requested receive interval is kept.
    NumRxSamples = int(
        math.ceil(RequestedRxCaptureDurationSec * SampleRateHz)
    )
    ActualRxCaptureDurationSec = NumRxSamples / SampleRateHz
    ActualRxEndDelaySec = RxStartDelaySec + ActualRxCaptureDurationSec

    # Use the rounded-up actual capture end so sample quantisation cannot make
    # the timing-safety check optimistic by one sample interval.
    MinimumPriSec = ActualRxEndDelaySec + NextTxGuardTimeSec
    TimingLimitedMaxPrfHz = 1.0 / MinimumPriSec
    EffectiveMaxPrfHz = min(OperatorMaxPrfHz, TimingLimitedMaxPrfHz)

    if PriSec < MinimumPriSec:
        raise ValueError(
            f"Selected PRF {SelectedPrfHz:g} Hz is unsafe for waveform "
            f"{WaveformId} and maximum range {MaximumRangeM:g} m: derived "
            f"PRI is {PriSec * 1e6:.3f} us but at least "
            f"{MinimumPriSec * 1e6:.3f} us is required; maximum safe PRF "
            f"is {TimingLimitedMaxPrfHz:.3f} Hz"
        )

    CpiDurationSec = PulsesPerCpi * PriSec
    PulseTrainSpanSec = (PulsesPerCpi - 1) * PriSec
    AcquisitionEndFromFirstPulseSec = (
        PulseTrainSpanSec + ActualRxEndDelaySec
    )
    TimingMarginAboveMinimumPriSec = PriSec - MinimumPriSec

    WavelengthM = SPEED_OF_LIGHT_MPS / RfFrequencyHz
    FirstRxSampleRangeOffsetM = MinimumFullEchoRangeM
    MaximumUnambiguousRangeM = SPEED_OF_LIGHT_MPS * PriSec / 2.0
    UnambiguousRadialVelocityMps = WavelengthM * SelectedPrfHz / 4.0
    VelocityBinSpacingMps = WavelengthM / (2.0 * CpiDurationSec)
    AntennaMovementDuringCpiDeg = (
        abs(AntennaScanRateDegPerSec) * CpiDurationSec
    )

    BytesPerPulseSc16 = NumRxSamples * 4
    BytesPerPulseFc32 = NumRxSamples * 8
    BytesPerCpiSc16 = BytesPerPulseSc16 * PulsesPerCpi
    BytesPerCpiFc32 = BytesPerPulseFc32 * PulsesPerCpi

    return RadarTimingSolution(
        WaveformId=WaveformId,
        SampleRateHz=SampleRateHz,
        ChipRateHz=ChipRateHz,
        ChipCount=ChipCount,
        SamplesPerChip=SamplesPerChip,
        TxWaveformSamples=TxWaveformSamples,
        TxPulseDurationSec=TxPulseDurationSec,
        TxLeadingZeroSamples=TxLeadingZeroSamples,
        TxLeadingZeroDurationSec=TxLeadingZeroDurationSec,
        TxAtrEnvelopeSamples=TxAtrEnvelopeSamples,
        TxAtrEnvelopeDurationSec=TxAtrEnvelopeDurationSec,
        RfPulseStartDelaySec=RfPulseStartDelaySec,
        RfPulseEndDelaySec=RfPulseEndDelaySec,
        SelectedPrfHz=SelectedPrfHz,
        PriSec=PriSec,
        OperatorMinPrfHz=OperatorMinPrfHz,
        OperatorMaxPrfHz=OperatorMaxPrfHz,
        TimingLimitedMaxPrfHz=TimingLimitedMaxPrfHz,
        EffectiveMaxPrfHz=EffectiveMaxPrfHz,
        PulsesPerCpi=PulsesPerCpi,
        CpiDurationSec=CpiDurationSec,
        PulseTrainSpanSec=PulseTrainSpanSec,
        AcquisitionEndFromFirstPulseSec=AcquisitionEndFromFirstPulseSec,
        MaximumRangeM=MaximumRangeM,
        MaximumEchoLeadingEdgeDelaySec=MaximumEchoLeadingEdgeDelaySec,
        ReceiverRecoveryTimeSec=ReceiverRecoveryTimeSec,
        RxEndMarginSec=RxEndMarginSec,
        NextTxGuardTimeSec=NextTxGuardTimeSec,
        RxStartDelaySec=RxStartDelaySec,
        RequiredRxEndDelaySec=RequiredRxEndDelaySec,
        RequestedRxCaptureDurationSec=RequestedRxCaptureDurationSec,
        NumRxSamples=NumRxSamples,
        ActualRxCaptureDurationSec=ActualRxCaptureDurationSec,
        ActualRxEndDelaySec=ActualRxEndDelaySec,
        MinimumPriSec=MinimumPriSec,
        TimingMarginAboveMinimumPriSec=TimingMarginAboveMinimumPriSec,
        MinimumFullEchoRangeM=MinimumFullEchoRangeM,
        FirstRxSampleRangeOffsetM=FirstRxSampleRangeOffsetM,
        MaximumUnambiguousRangeM=MaximumUnambiguousRangeM,
        UnambiguousRadialVelocityMps=UnambiguousRadialVelocityMps,
        VelocityBinSpacingMps=VelocityBinSpacingMps,
        AntennaMovementDuringCpiDeg=AntennaMovementDuringCpiDeg,
        TransmitDutyCycle=TxAtrEnvelopeDurationSec / PriSec,
        ReceiveCaptureDutyCycle=ActualRxCaptureDurationSec / PriSec,
        BytesPerPulseSc16=BytesPerPulseSc16,
        BytesPerPulseFc32=BytesPerPulseFc32,
        BytesPerCpiSc16=BytesPerCpiSc16,
        BytesPerCpiFc32=BytesPerCpiFc32,
        TimingSafe=True,
    )
