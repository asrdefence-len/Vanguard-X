"""
===============================================================================
ASR Defence X-Band Radar Prototype
DataTypes.py
===============================================================================

Foreword
--------
This file defines the main data structures used by the radar software.

These structures keep the interfaces between modules clean. Instead of passing
many loose variables between functions, the radar passes objects such as:

    Dwell
    RawDwellData
    ProcessedDwellData
    Detection

This makes it easier to later replace the simulated source with the Ettus source,
or replace a simple detector with CFAR, without changing the whole program.

Doppler processing note
-----------------------
The first version of the radar used a single pulse and produced one range
profile.

This version supports a coherent processing interval, or CPI, containing multiple
pulses. This allows Doppler processing.

The raw IQ data is now expected to be a 2D array:

    NumPulses x NumSamples

where:

    NumPulses  = slow-time dimension, used for Doppler
    NumSamples = fast-time dimension, used for range

===============================================================================
"""

from dataclasses import dataclass, field
import numpy as np


@dataclass
class Dwell:
    """
    Defines one radar dwell.

    A dwell is one radar action. It tells the source what waveform to use and
    how many pulses/samples to generate.

    DwellId:
        Unique dwell number.

    WaveformName:
        Name of waveform selected from the waveform library.

    SampleRate:
        Fast-time sampling rate in samples per second.

    NumSamples:
        Number of fast-time samples per pulse.

    NumPulses:
        Number of pulses in the coherent processing interval.

    PRI:
        Pulse repetition interval in seconds.
    """

    DwellId: int
    WaveformName: str
    SampleRate: float
    NumSamples: int
    NumPulses: int
    PRI: float


@dataclass
class RawDwellData:
    """
    Raw IQ data returned by a radar source.

    Both the simulated source and the future Ettus source should return this
    same type of object.

    IQ is now a 2D array:

        IQ[PulseIndex, SampleIndex]

    Shape:

        NumPulses x NumSamples

    PulseRxStartDelaySec stores the delay from each PRI origin to the first
    captured receive sample. RadarProcessor uses the corresponding DwellPlan
    value to restore the absolute range origin after pulse compression.
    """

    DwellId: int
    IQ: np.ndarray
    SampleRate: float
    PRI: float
    TimeStamp: float

    # Pulse-aware metadata. These fields preserve compatibility with the
    # original fixed-PRI interface while allowing future waveform/PRI agility.
    PulseTimesSec: np.ndarray = None
    PulsePriSec: np.ndarray = None
    PulseWaveformIds: list = None
    PulseValid: np.ndarray = None
    Diagnostics: dict = field(default_factory=dict)
    PulseRxStartDelaySec: np.ndarray = None


@dataclass
class ProcessedDwellData:
    """
    Processed radar data.

    RangeCompressed:
        2D pulse-compressed data:

            NumPulses x NumRangeBins

    RangeDopplerMap:
        2D range-Doppler data:

            NumDopplerBins x NumRangeBins

    MagnitudeDb:
        Magnitude of the range-Doppler map in dB.

    RangeAxisM:
        Range axis in metres.

    DopplerAxisHz:
        Doppler frequency axis in Hz.

    VelocityAxisMps:
        Radial velocity axis in metres per second.

    TimeStamp:
        Time associated with the dwell.
    """

    DwellId: int
    RangeCompressed: np.ndarray
    RangeDopplerMap: np.ndarray
    MagnitudeDb: np.ndarray
    RangeAxisM: np.ndarray
    DopplerAxisHz: np.ndarray
    VelocityAxisMps: np.ndarray
    TimeStamp: float


@dataclass
class Detection:
    """
    One radar detection.

    The detector converts processed radar data into one or more Detection
    objects.

    AzimuthDeg is the beam boresight angle associated with this detection.
    This is needed for sector display and tracking.
    """

    DwellId: int
    RangeBin: int
    DopplerBin: int
    RangeM: float
    DopplerHz: float
    VelocityMps: float
    AzimuthDeg: float
    AmplitudeDb: float
    TimeStamp: float
