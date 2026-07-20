"""
===============================================================================
Vanguard X Radar Pulse and Dwell Plans
RadarPlans.py
===============================================================================

This module introduces pulse-by-pulse radar scheduling while preserving the
current fixed-waveform, fixed-PRI behaviour.

The first supported processing mode is UNIFORM_PRI_FFT.  Later processing modes
can add Golay complementary pairs, staggered or jittered PRI, frequency agility,
and passive receive-only pulses without changing the source/processor API.
===============================================================================
"""

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional


@dataclass(frozen=True)
class PulsePlan:
    """Definition of one transmitted pulse and its receive window."""

    PulseIndex: int
    WaveformId: str
    PriSec: float
    TxEnabled: bool = True
    FrequencyOffsetHz: float = 0.0
    PhaseOffsetRad: float = 0.0
    AmplitudeScale: float = 1.0
    RxStartDelaySec: float = 0.0
    NumRxSamples: Optional[int] = None
    GroupId: Optional[int] = None
    GroupRole: Optional[str] = None


@dataclass(frozen=True)
class ProcessingPlan:
    """Instructions telling RadarProcessor how to interpret a dwell."""

    Mode: str = "UNIFORM_PRI_FFT"
    ProcessorId: str = "STANDARD_RANGE_DOPPLER"
    NominalPriSec: Optional[float] = None
    CombineGroupsBeforeDoppler: bool = False
    DopplerCompensationEnabled: bool = False


@dataclass
class DwellPlan:
    """Complete pulse-by-pulse description of one radar dwell."""

    DwellId: int
    TaskId: int
    TaskType: str
    SampleRate: float
    NumSamples: int
    PulsePlans: List[PulsePlan]
    Processing: ProcessingPlan
    AzimuthDeg: float = 0.0
    ElevationDeg: float = 0.0
    RxAttenuationDb: float = 0.0
    TxAttenuationDb: float = 31.5
    Metadata: Dict[str, Any] = field(default_factory=dict)

    @property
    def NumPulses(self) -> int:
        return len(self.PulsePlans)

    # Compatibility properties used by older modules during the transition.
    @property
    def WaveformName(self) -> str:
        if not self.PulsePlans:
            raise ValueError("DwellPlan contains no pulses")
        return self.PulsePlans[0].WaveformId

    @property
    def PRI(self) -> float:
        if self.Processing.NominalPriSec is not None:
            return float(self.Processing.NominalPriSec)
        if not self.PulsePlans:
            raise ValueError("DwellPlan contains no pulses")
        return float(self.PulsePlans[0].PriSec)

    @property
    def RxStartDelaySec(self) -> float:
        """Return the uniform receive-window delay used by this dwell."""

        if not self.PulsePlans:
            raise ValueError("DwellPlan contains no pulses")
        delays = [float(pulse.RxStartDelaySec) for pulse in self.PulsePlans]
        if any(abs(delay - delays[0]) > 1e-12 for delay in delays[1:]):
            raise ValueError("DwellPlan has non-uniform receive start delays")
        return delays[0]


def make_uniform_dwell_plan(
    dwell_id: int,
    task_id: int,
    task_type: str,
    waveform_id: str,
    sample_rate: float,
    num_samples: int,
    num_pulses: int,
    pri_sec: float,
    azimuth_deg: float = 0.0,
    elevation_deg: float = 0.0,
    rx_attenuation_db: float = 0.0,
    tx_attenuation_db: float = 31.5,
    metadata: Optional[Dict[str, Any]] = None,
    rx_start_delay_sec: float = 0.0,
) -> DwellPlan:
    """Create a fixed-waveform, fixed-PRI dwell matching legacy behaviour."""

    if num_pulses <= 0:
        raise ValueError("num_pulses must be greater than zero")
    if pri_sec <= 0.0:
        raise ValueError("pri_sec must be greater than zero")
    if sample_rate <= 0.0:
        raise ValueError("sample_rate must be greater than zero")
    if num_samples <= 0:
        raise ValueError("num_samples must be greater than zero")
    if rx_start_delay_sec < 0.0:
        raise ValueError("rx_start_delay_sec must not be negative")

    pulses = [
        PulsePlan(
            PulseIndex=index,
            WaveformId=str(waveform_id),
            PriSec=float(pri_sec),
            TxEnabled=True,
            RxStartDelaySec=float(rx_start_delay_sec),
            NumRxSamples=int(num_samples),
        )
        for index in range(int(num_pulses))
    ]

    processing = ProcessingPlan(
        Mode="UNIFORM_PRI_FFT",
        ProcessorId="STANDARD_RANGE_DOPPLER",
        NominalPriSec=float(pri_sec),
    )

    return DwellPlan(
        DwellId=int(dwell_id),
        TaskId=int(task_id),
        TaskType=str(task_type).upper(),
        SampleRate=float(sample_rate),
        NumSamples=int(num_samples),
        PulsePlans=pulses,
        Processing=processing,
        AzimuthDeg=float(azimuth_deg),
        ElevationDeg=float(elevation_deg),
        RxAttenuationDb=float(rx_attenuation_db),
        TxAttenuationDb=float(tx_attenuation_db),
        Metadata=dict(metadata or {}),
    )


def make_golay_dwell_plan(
    dwell_id: int,
    task_id: int,
    task_type: str,
    waveform_a_id: str,
    waveform_b_id: str,
    sample_rate: float,
    num_samples: int,
    num_pulses: int,
    pri_sec: float,
    azimuth_deg: float = 0.0,
    elevation_deg: float = 0.0,
    rx_attenuation_db: float = 0.0,
    tx_attenuation_db: float = 31.5,
    metadata: Optional[Dict[str, Any]] = None,
    rx_start_delay_sec: float = 0.0,
) -> DwellPlan:
    """Create a strict A/B Golay dwell for complementary processing."""

    if num_pulses <= 0 or num_pulses % 2 != 0:
        raise ValueError("Golay dwell requires a positive even pulse count")
    if pri_sec <= 0.0:
        raise ValueError("pri_sec must be greater than zero")
    if sample_rate <= 0.0:
        raise ValueError("sample_rate must be greater than zero")
    if num_samples <= 0:
        raise ValueError("num_samples must be greater than zero")
    if rx_start_delay_sec < 0.0:
        raise ValueError("rx_start_delay_sec must not be negative")
    if not waveform_a_id or not waveform_b_id:
        raise ValueError("Golay A and B waveform ids must not be empty")
    if waveform_a_id == waveform_b_id:
        raise ValueError("Golay A and B waveform ids must be different")

    pulses = []
    for pulse_index in range(int(num_pulses)):
        pair_index = pulse_index // 2
        pair_role = "A" if pulse_index % 2 == 0 else "B"
        pulses.append(PulsePlan(
            PulseIndex=pulse_index,
            WaveformId=(
                str(waveform_a_id)
                if pair_role == "A"
                else str(waveform_b_id)
            ),
            PriSec=float(pri_sec),
            TxEnabled=True,
            RxStartDelaySec=float(rx_start_delay_sec),
            NumRxSamples=int(num_samples),
            GroupId=pair_index,
            GroupRole=pair_role,
        ))

    processing = ProcessingPlan(
        Mode="GOLAY_COMPLEMENTARY",
        ProcessorId="GOLAY_COMPLEMENTARY_RANGE_DOPPLER",
        NominalPriSec=float(pri_sec),
        CombineGroupsBeforeDoppler=True,
        DopplerCompensationEnabled=False,
    )

    return DwellPlan(
        DwellId=int(dwell_id),
        TaskId=int(task_id),
        TaskType=str(task_type).upper(),
        SampleRate=float(sample_rate),
        NumSamples=int(num_samples),
        PulsePlans=pulses,
        Processing=processing,
        AzimuthDeg=float(azimuth_deg),
        ElevationDeg=float(elevation_deg),
        RxAttenuationDb=float(rx_attenuation_db),
        TxAttenuationDb=float(tx_attenuation_db),
        Metadata=dict(metadata or {}),
    )
