"""UI-independent range-profile scaling calculations."""

import numpy as np


def SelectRangeProfileDb(
    MagnitudeDb,
    DopplerAxisHz=None,
    Mode="MAX",
):
    """Select a range cut from a range-Doppler magnitude array."""

    MagnitudeDb = np.asarray(MagnitudeDb)
    if MagnitudeDb.ndim != 2 or MagnitudeDb.shape[0] == 0:
        raise ValueError("MagnitudeDb must be a non-empty 2D array")

    Mode = str(Mode).upper()
    if Mode == "MAX":
        return np.max(MagnitudeDb, axis=0)
    if Mode != "ZERO_DOPPLER":
        raise ValueError("Range profile Doppler mode must be MAX or ZERO_DOPPLER")
    if DopplerAxisHz is None:
        raise ValueError("ZERO_DOPPLER mode requires DopplerAxisHz")

    DopplerAxisHz = np.asarray(DopplerAxisHz, dtype=float).reshape(-1)
    if DopplerAxisHz.size != MagnitudeDb.shape[0]:
        raise ValueError("Doppler axis length does not match MagnitudeDb")
    if not np.all(np.isfinite(DopplerAxisHz)):
        raise ValueError("DopplerAxisHz must contain only finite values")

    ZeroDopplerBin = int(np.argmin(np.abs(DopplerAxisHz)))
    return MagnitudeDb[ZeroDopplerBin].copy()


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
