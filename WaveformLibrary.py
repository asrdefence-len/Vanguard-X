"""
===============================================================================
ASR Defence X-Band Radar Prototype
WaveformLibrary.py
===============================================================================

Foreword
--------
This file contains the radar waveform library.

The waveform library creates and stores transmit waveforms used by the radar.
The rest of the system asks for a waveform by name.

Important radar modelling note
------------------------------
For pulse compression processing gain, the waveform chips should have constant
chip amplitude.

That means:
    - Barker13 chips have magnitude 1
    - Frank10 chips have magnitude 1
    - a 100-chip Frank code has more total energy than a 13-chip Barker code

This models the radar case where peak transmit power is held approximately
constant and a longer coded pulse transmits more total energy.

For an N-chip phase code, the ideal processing gain is approximately:

    10 log10(N) dB

Current waveforms
-----------------
    - Barker13
    - Frank10
===============================================================================
"""

import numpy as np


class WaveformLibrary:
    """
    Stores radar waveforms.

    Each waveform is stored as a complex NumPy array.
    """

    def __init__(self, Config):
        self.Config = Config
        self.Waveforms = {}

    def LoadDefaultWaveforms(self):
        """
        Load default radar waveforms.
        """

        self.Waveforms["Barker13"] = self.MakeBarker13()
        self.Waveforms["Frank10"] = self.MakeFrankCode(10)

    def MakeBarker13(self):
        """
        Create a Barker 13 binary phase code.

        Each chip has magnitude 1.
        """

        Barker13 = np.array(
            [1, 1, 1, 1, 1, -1, -1, 1, 1, -1, 1, -1, 1],
            dtype=np.complex64
        )

        return Barker13

    def MakeFrankCode(self, Order):
        """
        Create a Frank polyphase code.

        A Frank code of order M has M x M = M^2 chips.

        For Order = 10:
            number of chips = 100

        Each chip has magnitude 1.
        """

        Chips = []

        for RowIndex in range(Order):
            for ColumnIndex in range(Order):
                Phase = 2.0 * np.pi * RowIndex * ColumnIndex / Order
                Chip = np.exp(1j * Phase)
                Chips.append(Chip)

        FrankCode = np.array(Chips, dtype=np.complex64)

        return FrankCode

    def Get(self, WaveformName):
        """
        Return a waveform by name.
        """

        if WaveformName not in self.Waveforms:
            raise ValueError(f"Waveform not found: {WaveformName}")

        return self.Waveforms[WaveformName]

    def GetCodeLength(self, WaveformName):
        """
        Return number of chips in the waveform.
        """

        return len(self.Get(WaveformName))

    def GetIdealProcessingGainDb(self, WaveformName):
        """
        Return ideal processing gain in dB for an N-chip phase code.
        """

        CodeLength = self.GetCodeLength(WaveformName)

        return 10.0 * np.log10(CodeLength)

    def ListWaveforms(self):
        """
        Return available waveform names.
        """

        return list(self.Waveforms.keys())