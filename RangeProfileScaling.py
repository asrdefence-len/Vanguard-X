"""UI-independent range-profile scaling calculations."""

import numpy as np


def CalculateNoiseReferencedRangeProfileLimits(
    NoiseFloorDb,
    PeakDb,
    NoiseMarginDb=5.0,
    MinimumSpanAboveNoiseDb=20.0,
    PeakHeadroomDb=3.0,
):
    """Return lower/upper display limits referenced to noise and peak."""
    Values = np.asarray(
        [
            NoiseFloorDb,
            PeakDb,
            NoiseMarginDb,
            MinimumSpanAboveNoiseDb,
            PeakHeadroomDb,
        ],
        dtype=float,
    )
    if not np.all(np.isfinite(Values)):
        raise ValueError("Range-profile scale inputs must be finite")
    if NoiseMarginDb < 0.0:
        raise ValueError("Noise margin must not be negative")
    if MinimumSpanAboveNoiseDb <= 0.0:
        raise ValueError("Minimum noise span must be positive")
    if PeakHeadroomDb < 0.0:
        raise ValueError("Peak headroom must not be negative")

    DisplayMinDb = float(NoiseFloorDb) - float(NoiseMarginDb)
    DisplayMaxDb = max(
        float(NoiseFloorDb) + float(MinimumSpanAboveNoiseDb),
        float(PeakDb) + float(PeakHeadroomDb),
    )
    return DisplayMinDb, DisplayMaxDb
