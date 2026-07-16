"""
===============================================================================
ASR Defence X-Band Radar Prototype
CfarDetector_Fast.py
===============================================================================

Fast 2D CA-CFAR detector for range-Doppler maps.

This version keeps the same public interface as CfarDetector.py:

    Detector = CfarDetector(Config)
    Detections = Detector.Detect(Processed, ThisDwell)

Main speed change
-----------------
The original detector looped over every range-Doppler cell and rebuilt the CFAR
window/mask each time. That is simple, but slow.

This version uses summed-area/integral-image processing so the training-cell
sum for every valid cell is computed using array operations. This avoids the
large Python loop over every cell-under-test.

No SciPy or Numba dependency is required.
===============================================================================
"""

import numpy as np
from DataTypes import Detection


class CfarDetector:
    """
    Fast 2D CA-CFAR detector for range-Doppler maps.
    """

    def __init__(self, Config):
        self.Config = Config

        # Number of training cells on each side of the cell under test.
        self.TrainingCellsRange = Config.get("TrainingCellsRange", 12)
        self.TrainingCellsDoppler = Config.get("TrainingCellsDoppler", 4)

        # Number of guard cells on each side of the cell under test.
        self.GuardCellsRange = Config.get("GuardCellsRange", 4)
        self.GuardCellsDoppler = Config.get("GuardCellsDoppler", 1)

        # Threshold offset in dB above the local noise estimate.
        self.CfarThresholdDb = Config.get("CfarThresholdDb", 12.0)

        # Optional maximum number of detections to return.
        self.MaxDetections = Config.get("MaxDetections", 20)

        # Optional range limits for detections.
        self.MinRangeM = Config.get("MinRangeM", 0.0)
        self.MaxRangeM = Config.get("MaxRangeM", 15000.0)

        # Optional Doppler-bin limits for CFAR. Leave unset to use all valid bins.
        # These can be useful if the display/processing only needs part of the
        # Doppler spectrum.
        self.MinDopplerBin = Config.get("MinDopplerBin", None)
        self.MaxDopplerBin = Config.get("MaxDopplerBin", None)

        self.ThresholdScale = np.float32(10.0 ** (self.CfarThresholdDb / 10.0))

        self.TrainR = int(self.TrainingCellsRange)
        self.TrainD = int(self.TrainingCellsDoppler)
        self.GuardR = int(self.GuardCellsRange)
        self.GuardD = int(self.GuardCellsDoppler)

        self.HalfWindowR = self.TrainR + self.GuardR
        self.HalfWindowD = self.TrainD + self.GuardD

        full_rows = 2 * self.HalfWindowD + 1
        full_cols = 2 * self.HalfWindowR + 1
        guard_rows = 2 * self.GuardD + 1
        guard_cols = 2 * self.GuardR + 1

        self.NumTrainingCells = (full_rows * full_cols) - (guard_rows * guard_cols)

        if self.NumTrainingCells <= 0:
            raise ValueError("CFAR has zero training cells. Increase TrainingCellsRange/Doppler.")

    @staticmethod
    def _rect_sum_from_integral(Integral, row0, row1, col0, col1):
        """
        Return rectangular sums using a zero-padded integral image.

        row0, row1, col0, col1 are arrays or scalars using half-open intervals:
            rows row0:row1, columns col0:col1
        """
        return (
            Integral[row1, col1]
            - Integral[row0, col1]
            - Integral[row1, col0]
            + Integral[row0, col0]
        )

    def Detect(self, Processed, ThisDwell=None):
        """
        Run fast 2D CA-CFAR on the range-Doppler map.

        Processed.RangeDopplerMap is complex. Power is formed as:

            Power = abs(RangeDopplerMap)^2

        Returns a list of Detection objects sorted strongest first.
        """

        # float32 is faster and lighter than float64 on embedded systems.
        PowerMap = (np.abs(Processed.RangeDopplerMap).astype(np.float32) ** 2).astype(np.float32)

        NumDopplerBins, NumRangeBins = PowerMap.shape

        # Not enough cells to form a full CFAR stencil.
        if (
            NumDopplerBins <= 2 * self.HalfWindowD
            or NumRangeBins <= 2 * self.HalfWindowR
        ):
            return []

        # Valid CUT indices excluding the stencil edges.
        DopplerIdx = np.arange(
            self.HalfWindowD,
            NumDopplerBins - self.HalfWindowD,
            dtype=np.int32,
        )
        RangeIdx = np.arange(
            self.HalfWindowR,
            NumRangeBins - self.HalfWindowR,
            dtype=np.int32,
        )

        # Optional Doppler-bin restriction.
        if self.MinDopplerBin is not None:
            DopplerIdx = DopplerIdx[DopplerIdx >= int(self.MinDopplerBin)]
        if self.MaxDopplerBin is not None:
            DopplerIdx = DopplerIdx[DopplerIdx <= int(self.MaxDopplerBin)]

        if DopplerIdx.size == 0 or RangeIdx.size == 0:
            return []

        # Optional range restriction before CFAR output selection.
        RangeAxisValid = np.asarray(Processed.RangeAxisM)[RangeIdx]
        RangeMask = (RangeAxisValid >= self.MinRangeM) & (RangeAxisValid <= self.MaxRangeM)
        RangeIdx = RangeIdx[RangeMask]

        if RangeIdx.size == 0:
            return []

        # Zero-padded integral image. Shape is one larger in both dimensions.
        Integral = np.pad(PowerMap, ((1, 0), (1, 0)), mode="constant")
        Integral = Integral.cumsum(axis=0).cumsum(axis=1)

        D = DopplerIdx[:, None]
        R = RangeIdx[None, :]

        # Full CFAR window sum around each CUT.
        FullSum = self._rect_sum_from_integral(
            Integral,
            D - self.HalfWindowD,
            D + self.HalfWindowD + 1,
            R - self.HalfWindowR,
            R + self.HalfWindowR + 1,
        )

        # Guard region includes the CUT and is excluded from the noise estimate.
        GuardSum = self._rect_sum_from_integral(
            Integral,
            D - self.GuardD,
            D + self.GuardD + 1,
            R - self.GuardR,
            R + self.GuardR + 1,
        )

        NoiseEstimate = (FullSum - GuardSum) / np.float32(self.NumTrainingCells)
        Threshold = NoiseEstimate * self.ThresholdScale

        Cells = PowerMap[DopplerIdx[:, None], RangeIdx[None, :]]
        DetectionMask = Cells > Threshold

        if not np.any(DetectionMask):
            return []

        DopplerHitLocal, RangeHitLocal = np.nonzero(DetectionMask)

        DopplerHits = DopplerIdx[DopplerHitLocal]
        RangeHits = RangeIdx[RangeHitLocal]
        CellPowers = Cells[DetectionMask]
        NoiseHits = NoiseEstimate[DetectionMask]
        ThresholdHits = Threshold[DetectionMask]

        # Keep only the strongest hits before creating Python Detection objects.
        if CellPowers.size > self.MaxDetections:
            Strongest = np.argpartition(CellPowers, -self.MaxDetections)[-self.MaxDetections:]
            SortOrder = Strongest[np.argsort(CellPowers[Strongest])[::-1]]
        else:
            SortOrder = np.argsort(CellPowers)[::-1]

        Detections = []
        BoresightDeg = float(Processed.Diagnostics.get("BoresightDeg", 0.0))

        for k in SortOrder:
            DopplerIndex = int(DopplerHits[k])
            RangeIndex = int(RangeHits[k])
            CellPower = float(CellPowers[k])
            NoisePower = float(NoiseHits[k])
            ThresholdPower = float(ThresholdHits[k])

            ThisDetection = Detection(
                DwellId=Processed.DwellId,
                RangeBin=RangeIndex,
                DopplerBin=DopplerIndex,
                RangeM=float(Processed.RangeAxisM[RangeIndex]),
                DopplerHz=float(Processed.DopplerAxisHz[DopplerIndex]),
                VelocityMps=float(Processed.VelocityAxisMps[DopplerIndex]),
                AzimuthDeg=BoresightDeg,
                AmplitudeDb=float(10.0 * np.log10(CellPower + 1e-30)),
                TimeStamp=Processed.TimeStamp,
            )

            ThisDetection.NoiseEstimateDb = float(10.0 * np.log10(NoisePower + 1e-30))
            ThisDetection.ThresholdDb = float(10.0 * np.log10(ThresholdPower + 1e-30))

            Detections.append(ThisDetection)

        return Detections
