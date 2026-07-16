"""
===============================================================================
ASR Defence X-Band Radar Prototype
SimpleDetector.py
===============================================================================

Foreword
--------
This file contains the first simple radar detector.

Current detection behaviour
---------------------------
Current version:
    - searches the 2D range-Doppler map
    - finds the strongest cell
    - compares it with a fixed threshold
    - outputs one detection if the peak exceeds the threshold

Future versions:
    - CA-CFAR
    - GO-CFAR
    - OS-CFAR
    - 2D range-Doppler CFAR
    - clutter-map assisted detection
===============================================================================
"""

import numpy as np
from DataTypes import Detection


class SimpleDetector:
    """
    Simple fixed-threshold detector for a range-Doppler map.
    """

    def __init__(self, Config):
        self.Config = Config
        self.ThresholdDb = Config["ThresholdDb"]

    def Detect(self, Processed, ThisDwell):
        """
        Detect targets in the 2D range-Doppler map.

        Processed.MagnitudeDb has shape:

            NumDopplerBins x NumRangeBins
        """

        Detections = []

        PeakDopplerBin, PeakRangeBin = np.unravel_index(
            np.argmax(Processed.MagnitudeDb),
            Processed.MagnitudeDb.shape
        )

        PeakAmplitudeDb = float(Processed.MagnitudeDb[PeakDopplerBin, PeakRangeBin])
        PeakRangeM = float(Processed.RangeAxisM[PeakRangeBin])
        PeakDopplerHz = float(Processed.DopplerAxisHz[PeakDopplerBin])
        PeakVelocityMps = float(Processed.VelocityAxisMps[PeakDopplerBin])

        if PeakAmplitudeDb >= self.ThresholdDb:
            ThisDetection = Detection(
                DwellId=Processed.DwellId,
                RangeBin=int(PeakRangeBin),
                DopplerBin=int(PeakDopplerBin),
                RangeM=PeakRangeM,
                DopplerHz=PeakDopplerHz,
                VelocityMps=PeakVelocityMps,
                AmplitudeDb=PeakAmplitudeDb,
                TimeStamp=Processed.TimeStamp
            )

            Detections.append(ThisDetection)

        return Detections