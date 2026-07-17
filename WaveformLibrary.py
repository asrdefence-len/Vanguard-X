"""
===============================================================================
ASR Defence X-Band Radar Prototype
WaveformLibrary.py
===============================================================================

The waveform library owns the authoritative sampled waveform definitions used
by timed transmission, simulation and matched filtering.

Initial waveform architecture
-----------------------------
The initial Ettus hardware path uses a fixed 40 MS/s complex sample rate.
Waveform bandwidth and pulse duration are selected through chip rate:

    10 Mchip/s -> 4 samples per chip
    20 Mchip/s -> 2 samples per chip

Waveform identifiers explicitly include nominal chip rate:

    Barker13_10MHz
    Barker13_20MHz
    Frank10_10MHz
    Frank10_20MHz

For an N-chip constant-amplitude phase code, ideal pulse-compression processing
gain is approximately 10 log10(N) dB.  It is calculated from chip count, not
from the expanded sampled-waveform length.
===============================================================================
"""

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class WaveformDefinition:
    """Immutable metadata and samples for one phase-coded waveform."""

    WaveformId: str
    CodeFamily: str
    Chips: np.ndarray
    ChipCount: int
    ChipRateHz: float
    SampleRateHz: float
    SamplesPerChip: int
    Samples: np.ndarray
    NumSamples: int
    PulseDurationSec: float


class WaveformLibrary:
    """Create and store authoritative sampled waveform definitions."""

    FIXED_SAMPLE_RATE_HZ = 40.0e6

    def __init__(self, Config):
        self.Config = Config
        self.SampleRateHz = float(
            Config.get("EttusSampleRateHz", self.FIXED_SAMPLE_RATE_HZ)
        )

        if not np.isclose(
            self.SampleRateHz,
            self.FIXED_SAMPLE_RATE_HZ,
            rtol=0.0,
            atol=1.0,
        ):
            raise ValueError(
                "The initial waveform architecture requires a fixed "
                f"{self.FIXED_SAMPLE_RATE_HZ / 1e6:.0f} MS/s sample rate; "
                f"received {self.SampleRateHz / 1e6:g} MS/s"
            )

        # Waveforms preserves the established Get() interface. Definitions
        # carries the physical metadata needed by planning and diagnostics.
        self.Waveforms = {}
        self.Definitions = {}

    def LoadDefaultWaveforms(self):
        """Load the initial 10 and 20 Mchip/s Barker and Frank catalogue."""

        self.Waveforms.clear()
        self.Definitions.clear()

        Barker13 = self.MakeBarker13()
        Frank10 = self.MakeFrankCode(10)

        self.RegisterPhaseCode(
            WaveformId="Barker13_10MHz",
            CodeFamily="BARKER",
            Chips=Barker13,
            ChipRateHz=10.0e6,
        )
        self.RegisterPhaseCode(
            WaveformId="Barker13_20MHz",
            CodeFamily="BARKER",
            Chips=Barker13,
            ChipRateHz=20.0e6,
        )
        self.RegisterPhaseCode(
            WaveformId="Frank10_10MHz",
            CodeFamily="FRANK",
            Chips=Frank10,
            ChipRateHz=10.0e6,
        )
        self.RegisterPhaseCode(
            WaveformId="Frank10_20MHz",
            CodeFamily="FRANK",
            Chips=Frank10,
            ChipRateHz=20.0e6,
        )

    def RegisterPhaseCode(
        self,
        WaveformId,
        CodeFamily,
        Chips,
        ChipRateHz,
    ):
        """Validate, sample and register one constant-amplitude phase code."""

        WaveformId = str(WaveformId)
        CodeFamily = str(CodeFamily).upper()
        ChipRateHz = float(ChipRateHz)

        if not WaveformId:
            raise ValueError("WaveformId must not be empty")
        if WaveformId in self.Definitions:
            raise ValueError(f"Waveform already registered: {WaveformId}")
        if not CodeFamily:
            raise ValueError("CodeFamily must not be empty")
        if ChipRateHz <= 0.0:
            raise ValueError("ChipRateHz must be greater than zero")

        Chips = np.asarray(Chips, dtype=np.complex64)
        if Chips.ndim != 1 or len(Chips) == 0:
            raise ValueError("Chips must be a non-empty one-dimensional array")
        if not np.all(np.isfinite(Chips)):
            raise ValueError("Chips must contain only finite values")
        if not np.allclose(np.abs(Chips), 1.0, rtol=0.0, atol=1e-6):
            raise ValueError(
                "Phase-code chips must have constant unit magnitude"
            )

        SamplesPerChipExact = self.SampleRateHz / ChipRateHz
        SamplesPerChip = int(round(SamplesPerChipExact))
        if SamplesPerChip <= 0 or not np.isclose(
            SamplesPerChipExact,
            SamplesPerChip,
            rtol=0.0,
            atol=1e-12,
        ):
            raise ValueError(
                f"Chip rate {ChipRateHz:g} Hz does not divide the fixed "
                f"sample rate {self.SampleRateHz:g} Hz into an integer "
                "number of samples per chip"
            )

        ExpectedSuffix = self._ChipRateSuffix(ChipRateHz)
        if not WaveformId.endswith(ExpectedSuffix):
            raise ValueError(
                f"WaveformId '{WaveformId}' must end with "
                f"'{ExpectedSuffix}' to identify its nominal chip rate"
            )

        # Own independent read-only arrays so callers cannot alter the
        # definition used later by transmission or matched filtering.
        StoredChips = Chips.copy()
        StoredChips.setflags(write=False)
        Samples = np.repeat(StoredChips, SamplesPerChip).astype(
            np.complex64,
            copy=False,
        )
        Samples.setflags(write=False)

        ChipCount = int(len(StoredChips))
        NumSamples = int(len(Samples))
        PulseDurationSec = ChipCount / ChipRateHz

        Definition = WaveformDefinition(
            WaveformId=WaveformId,
            CodeFamily=CodeFamily,
            Chips=StoredChips,
            ChipCount=ChipCount,
            ChipRateHz=ChipRateHz,
            SampleRateHz=self.SampleRateHz,
            SamplesPerChip=SamplesPerChip,
            Samples=Samples,
            NumSamples=NumSamples,
            PulseDurationSec=PulseDurationSec,
        )

        self.Definitions[WaveformId] = Definition
        self.Waveforms[WaveformId] = Samples
        return Definition

    @staticmethod
    def _ChipRateSuffix(ChipRateHz):
        RateMHz = float(ChipRateHz) / 1.0e6
        if not RateMHz.is_integer():
            raise ValueError(
                "Initial WaveformId naming requires a whole-MHz chip rate"
            )
        return f"_{int(RateMHz)}MHz"

    @staticmethod
    def MakeBarker13():
        """Return the 13-chip Barker binary phase code."""

        return np.array(
            [1, 1, 1, 1, 1, -1, -1, 1, 1, -1, 1, -1, 1],
            dtype=np.complex64,
        )

    @staticmethod
    def MakeFrankCode(Order):
        """Return an order-M Frank code containing M squared phase chips."""

        Order = int(Order)
        if Order <= 0:
            raise ValueError("Frank-code order must be greater than zero")

        Chips = []
        for RowIndex in range(Order):
            for ColumnIndex in range(Order):
                Phase = 2.0 * np.pi * RowIndex * ColumnIndex / Order
                Chips.append(np.exp(1j * Phase))

        return np.asarray(Chips, dtype=np.complex64)

    def Get(self, WaveformName):
        """Return the authoritative sampled complex waveform."""

        return self.GetDefinition(WaveformName).Samples

    def GetDefinition(self, WaveformName):
        """Return the complete immutable waveform definition."""

        try:
            return self.Definitions[str(WaveformName)]
        except KeyError as Error:
            raise ValueError(f"Waveform not found: {WaveformName}") from Error

    def GetMetadata(self, WaveformName):
        """Return physical waveform metadata without copying sample arrays."""

        Definition = self.GetDefinition(WaveformName)
        return {
            "WaveformId": Definition.WaveformId,
            "CodeFamily": Definition.CodeFamily,
            "ChipCount": Definition.ChipCount,
            "ChipRateHz": Definition.ChipRateHz,
            "SampleRateHz": Definition.SampleRateHz,
            "SamplesPerChip": Definition.SamplesPerChip,
            "NumSamples": Definition.NumSamples,
            "PulseDurationSec": Definition.PulseDurationSec,
        }

    def GetChips(self, WaveformName):
        """Return the original read-only phase-code chip sequence."""

        return self.GetDefinition(WaveformName).Chips

    def GetCodeLength(self, WaveformName):
        """Return chip count, not expanded sampled-waveform length."""

        return self.GetDefinition(WaveformName).ChipCount

    def GetSampleCount(self, WaveformName):
        """Return the number of complex samples in the transmit waveform."""

        return self.GetDefinition(WaveformName).NumSamples

    def GetPulseDurationSec(self, WaveformName):
        """Return sampled transmit pulse duration in seconds."""

        return self.GetDefinition(WaveformName).PulseDurationSec

    def GetIdealProcessingGainDb(self, WaveformName):
        """Return ideal phase-code gain calculated from chip count."""

        return 10.0 * np.log10(self.GetCodeLength(WaveformName))

    def ListWaveforms(self):
        """Return registered waveform identifiers in catalogue order."""

        return list(self.Definitions.keys())
