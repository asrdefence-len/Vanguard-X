"""
===============================================================================
ASR Defence X-Band Radar Prototype
RadarProcessor_FFT.py
===============================================================================

Foreword
--------
This file contains the radar signal processing module.

Current processing behaviour
----------------------------
    - FFT-based matched filtering / pulse compression for all pulses in a dwell
    - Doppler FFT across pulses
    - range-Doppler map
    - range axis in metres
    - Doppler axis in Hz
    - velocity axis in m/s

Important radar note
--------------------
Range comes from fast-time delay within each pulse.

Doppler comes from phase change across pulses in a coherent processing interval.

Implementation note
-------------------
This version replaces the per-pulse np.convolve() loop with batched FFT pulse
compression along fast time. It keeps the external RadarProcessor class and
Process() interface the same as the original file.
===============================================================================
"""

import numpy as np
from DataTypes import ProcessedDwellData


class RadarProcessor:
    """
    Radar signal processor.
    """

    def __init__(self, Config, TheWaveformLibrary):
        self.Config = Config
        self.TheWaveformLibrary = TheWaveformLibrary

        # Small caches avoid rebuilding windows, axes and FFT filters every dwell.
        self._FilterCache = {}
        self._DopplerWindowCache = {}
        self._AxisCache = {}

    @staticmethod
    def _NextPowerOfTwo(Value):
        """
        Return the next power of two >= Value.
        """
        return 1 << (int(Value) - 1).bit_length()

    def _GetMatchedFilterFft(self, WaveformName, TxWaveform, NumSamples):
        """
        Build/cache the FFT of the matched filter for linear convolution.
        """
        CodeLength = len(TxWaveform)
        FullLength = NumSamples + CodeLength - 1
        Nfft = self._NextPowerOfTwo(FullLength)

        CacheKey = (WaveformName, NumSamples, CodeLength, Nfft)
        Cached = self._FilterCache.get(CacheKey)
        if Cached is not None:
            return Cached

        MatchedFilter = np.conj(TxWaveform[::-1]).astype(np.complex64)

        FilterPadded = np.zeros(Nfft, dtype=np.complex64)
        FilterPadded[:CodeLength] = MatchedFilter
        MatchedFilterFft = np.fft.fft(FilterPadded).astype(np.complex64)

        # np.convolve(..., mode="same") is the centred part of the full linear
        # convolution. For NumSamples >= CodeLength this is the matching slice.
        SameStart = (CodeLength - 1) // 2
        SameEnd = SameStart + NumSamples

        Cached = (MatchedFilterFft, Nfft, SameStart, SameEnd)
        self._FilterCache[CacheKey] = Cached
        return Cached

    def _PulseCompressFft(self, RawIq, WaveformName, TxWaveform):
        """
        Pulse-compress all pulses using a batched FFT along fast time.

        RawIq shape:
            NumPulses x NumSamples
        """
        NumPulses, NumSamples = RawIq.shape

        MatchedFilterFft, Nfft, SameStart, SameEnd = self._GetMatchedFilterFft(
            WaveformName,
            TxWaveform,
            NumSamples
        )

        # Zero pad raw IQ for linear convolution.
        RawPadded = np.zeros((NumPulses, Nfft), dtype=np.complex64)
        RawPadded[:, :NumSamples] = RawIq.astype(np.complex64, copy=False)

        RawFft = np.fft.fft(RawPadded, axis=1)
        FullCompressed = np.fft.ifft(RawFft * MatchedFilterFft[np.newaxis, :], axis=1)

        return FullCompressed[:, SameStart:SameEnd].astype(np.complex64, copy=False)

    def _GetDopplerWindow(self, NumPulses):
        Cached = self._DopplerWindowCache.get(NumPulses)
        if Cached is not None:
            return Cached

        DopplerWindow = np.hanning(NumPulses).astype(np.float32)
        self._DopplerWindowCache[NumPulses] = DopplerWindow
        return DopplerWindow

    def _GetAxes(self, NumPulses, NumSamples, SampleRate, PRI):
        RfFrequency = self.Config["RfFrequency"]
        CacheKey = (NumPulses, NumSamples, float(SampleRate), float(PRI), float(RfFrequency))
        Cached = self._AxisCache.get(CacheKey)
        if Cached is not None:
            return Cached

        SpeedOfLight = 299792458.0
        WavelengthM = SpeedOfLight / RfFrequency

        SampleNumbers = np.arange(NumSamples)
        RangeAxisM = SampleNumbers * SpeedOfLight / (2.0 * SampleRate)

        DopplerAxisHz = np.fft.fftshift(
            np.fft.fftfreq(NumPulses, d=PRI)
        )

        VelocityAxisMps = DopplerAxisHz * WavelengthM / 2.0

        Cached = (RangeAxisM, DopplerAxisHz, VelocityAxisMps)
        self._AxisCache[CacheKey] = Cached
        return Cached

    def Process(self, Raw, ThisDwell):
        """Dispatch processing according to the dwell ProcessingPlan."""

        Processing = getattr(ThisDwell, "Processing", None)
        if Processing is None:
            return self._ProcessUniformPri(Raw, ThisDwell)

        Mode = str(Processing.Mode).upper()

        if Mode == "UNIFORM_PRI_FFT":
            return self._ProcessUniformPri(Raw, ThisDwell)

        if Mode == "GOLAY_COMPLEMENTARY":
            return self._ProcessGolay(Raw, ThisDwell)

        if Mode in ("NONUNIFORM_DOPPLER", "STAGGERED_PRI"):
            return self._ProcessNonuniformDoppler(Raw, ThisDwell)

        raise ValueError(f"Unsupported processing mode: {Processing.Mode}")

    def _ValidateUniformPriPlan(self, ThisDwell):
        """Validate that a planned dwell is suitable for the FFT fast path."""

        if not hasattr(ThisDwell, "PulsePlans"):
            return str(ThisDwell.WaveformName), float(ThisDwell.PRI)

        if len(ThisDwell.PulsePlans) == 0:
            raise ValueError("DwellPlan contains no pulses")

        WaveformIds = [pulse.WaveformId for pulse in ThisDwell.PulsePlans]
        PriValues = np.asarray(
            [pulse.PriSec for pulse in ThisDwell.PulsePlans],
            dtype=np.float64,
        )
        TxEnabled = [bool(pulse.TxEnabled) for pulse in ThisDwell.PulsePlans]

        if len(set(WaveformIds)) != 1:
            raise ValueError(
                "UNIFORM_PRI_FFT requires the same waveform on every pulse"
            )
        if not np.allclose(PriValues, PriValues[0], rtol=0.0, atol=1e-12):
            raise ValueError("UNIFORM_PRI_FFT requires a constant PRI")
        if not all(TxEnabled):
            raise ValueError(
                "UNIFORM_PRI_FFT currently requires every pulse to transmit"
            )

        NominalPri = getattr(ThisDwell.Processing, "NominalPriSec", None)
        if NominalPri is None:
            NominalPri = float(PriValues[0])

        return str(WaveformIds[0]), float(NominalPri)

    def _ProcessUniformPri(self, Raw, ThisDwell):
        """Existing fixed-waveform, fixed-PRI pulse compression and Doppler FFT."""

        WaveformName, PRI = self._ValidateUniformPriPlan(ThisDwell)
        TxWaveform = self.TheWaveformLibrary.Get(WaveformName)
        CodeLength = len(TxWaveform)

        NumPulses = Raw.IQ.shape[0]
        NumSamples = Raw.IQ.shape[1]

        RangeCompressed = self._PulseCompressFft(
            Raw.IQ,
            WaveformName,
            TxWaveform,
        )

        DopplerWindow = self._GetDopplerWindow(NumPulses)
        Windowed = RangeCompressed * DopplerWindow[:, np.newaxis]

        RangeDopplerMap = np.fft.fftshift(
            np.fft.fft(Windowed, axis=0),
            axes=0,
        )
        MagnitudeDb = 20.0 * np.log10(np.abs(RangeDopplerMap) + 1e-12)

        RangeAxisM, DopplerAxisHz, VelocityAxisMps = self._GetAxes(
            NumPulses,
            NumSamples,
            Raw.SampleRate,
            PRI,
        )

        PeakDopplerBin, PeakRangeBin = np.unravel_index(
            np.argmax(np.abs(RangeDopplerMap)),
            RangeDopplerMap.shape,
        )

        PeakRangeM = float(RangeAxisM[PeakRangeBin])
        PeakDopplerHz = float(DopplerAxisHz[PeakDopplerBin])
        PeakVelocityMps = float(VelocityAxisMps[PeakDopplerBin])
        IdealProcessingGainDb = 10.0 * np.log10(CodeLength)

        Diagnostics = {
            "ProcessingMode": "UNIFORM_PRI_FFT",
            "ProcessorId": getattr(
                getattr(ThisDwell, "Processing", None),
                "ProcessorId",
                "STANDARD_RANGE_DOPPLER",
            ),
            "WaveformId": WaveformName,
            "NominalPriSec": PRI,
            "CodeLength": CodeLength,
            "IdealProcessingGainDb": IdealProcessingGainDb,
            "PeakRangeBin": int(PeakRangeBin),
            "PeakDopplerBin": int(PeakDopplerBin),
            "PeakRangeM": PeakRangeM,
            "PeakDopplerHz": PeakDopplerHz,
            "PeakVelocityMps": PeakVelocityMps,
            "TaskId": getattr(ThisDwell, "TaskId", None),
            "TaskType": getattr(ThisDwell, "TaskType", None),
        }

        RawDiagnostics = getattr(Raw, "Diagnostics", None)
        if RawDiagnostics:
            Diagnostics.update(RawDiagnostics)

        Processed = ProcessedDwellData(
            DwellId=Raw.DwellId,
            RangeCompressed=RangeCompressed,
            RangeDopplerMap=RangeDopplerMap,
            MagnitudeDb=MagnitudeDb,
            RangeAxisM=RangeAxisM,
            DopplerAxisHz=DopplerAxisHz,
            VelocityAxisMps=VelocityAxisMps,
            TimeStamp=Raw.TimeStamp,
        )
        Processed.Diagnostics = Diagnostics
        return Processed

    def _ProcessGolay(self, Raw, ThisDwell):
        raise NotImplementedError(
            "Golay complementary processing is defined architecturally but "
            "has not yet been implemented"
        )

    def _ProcessNonuniformDoppler(self, Raw, ThisDwell):
        raise NotImplementedError(
            "Non-uniform/staggered PRI Doppler processing has not yet been "
            "implemented"
        )
