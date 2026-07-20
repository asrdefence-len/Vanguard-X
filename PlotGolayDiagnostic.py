"""Plot one-shot Vanguard X Golay A/B diagnostic data from an HDF5 log."""

from __future__ import annotations

import argparse
from pathlib import Path

import h5py
import matplotlib.pyplot as plt
import numpy as np


def _db(Value, Reference):
    return 20.0 * np.log10(
        np.maximum(np.abs(Value) / max(float(Reference), 1.0e-30), 1.0e-15)
    )


def _attribute(Group, Name, Default=np.nan):
    Value = Group.attrs.get(Name, Default)
    if isinstance(Value, bytes):
        return Value.decode("utf-8")
    return Value


def CalculateGolayDiagnostic(Data):
    """Calculate zero-Doppler A, B and complex-sum profiles and metrics."""

    CompressedA = np.asarray(Data["compressed_a"], dtype=np.complex64)
    CompressedB = np.asarray(Data["compressed_b"], dtype=np.complex64)
    RawA = np.asarray(Data["raw_a"], dtype=np.complex64)
    RawB = np.asarray(Data["raw_b"], dtype=np.complex64)
    RangeM = np.asarray(Data["range_m"], dtype=np.float64).reshape(-1)
    if (
        CompressedA.ndim != 2
        or CompressedA.shape != CompressedB.shape
        or RawA.shape != RawB.shape
        or RawA.shape != CompressedA.shape
        or RangeM.size != CompressedA.shape[1]
    ):
        raise ValueError("Golay diagnostic datasets have inconsistent shapes")

    NumPairs = CompressedA.shape[0]
    Window = np.hanning(NumPairs) if NumPairs > 1 else np.ones(1)
    ProfileA = np.sum(CompressedA * Window[:, np.newaxis], axis=0)
    ProfileB = np.sum(CompressedB * Window[:, np.newaxis], axis=0)
    ProfileSum = ProfileA + ProfileB

    PeakIndex = int(np.argmax(np.abs(ProfileSum)))
    PeakAIndex = int(np.argmax(np.abs(ProfileA)))
    PeakBIndex = int(np.argmax(np.abs(ProfileB)))
    Reference = float(np.abs(ProfileSum[PeakIndex]))
    GainDifferenceDb = float(
        20.0
        * np.log10(
            max(float(np.abs(ProfileB[PeakIndex])), 1.0e-30)
            / max(float(np.abs(ProfileA[PeakIndex])), 1.0e-30)
        )
    )
    PhaseDifferenceDeg = float(np.degrees(np.angle(
        ProfileB[PeakIndex] * np.conj(ProfileA[PeakIndex])
    )))

    SamplesPerChip = int(_attribute(Data, "samples_per_chip", 1))
    ExclusionBins = max(1, SamplesPerChip - 1)
    SidelobeMask = np.ones(ProfileSum.size, dtype=bool)
    SidelobeMask[
        max(0, PeakIndex - ExclusionBins):
        min(ProfileSum.size, PeakIndex + ExclusionBins + 1)
    ] = False
    SumDb = _db(ProfileSum, Reference)
    PeakSidelobeDb = float(np.max(SumDb[SidelobeMask]))

    Raw = np.concatenate((RawA.reshape(-1), RawB.reshape(-1)))
    MaximumComplexMagnitude = float(np.max(np.abs(Raw)))
    MaximumComponentMagnitude = float(max(
        np.max(np.abs(Raw.real)),
        np.max(np.abs(Raw.imag)),
    ))
    ClippedComponentCount = int(np.count_nonzero(
        (np.abs(Raw.real) >= 0.999) | (np.abs(Raw.imag) >= 0.999)
    ))

    return {
        "RangeM": RangeM,
        "ProfileA": ProfileA,
        "ProfileB": ProfileB,
        "ProfileSum": ProfileSum,
        "ProfileADb": _db(ProfileA, Reference),
        "ProfileBDb": _db(ProfileB, Reference),
        "ProfileSumDb": SumDb,
        "PeakIndex": PeakIndex,
        "PeakRangeM": float(RangeM[PeakIndex]),
        "PeakRangeDifferenceSamples": PeakBIndex - PeakAIndex,
        "GainDifferenceDb": GainDifferenceDb,
        "PhaseDifferenceDeg": PhaseDifferenceDeg,
        "PeakSidelobeDb": PeakSidelobeDb,
        "MaximumComplexMagnitude": MaximumComplexMagnitude,
        "MaximumComponentMagnitude": MaximumComponentMagnitude,
        "ClippedComponentCount": ClippedComponentCount,
        "NumPairs": NumPairs,
    }


def Main():
    Parser = argparse.ArgumentParser(
        description="Plot one saved Vanguard X Golay A/B diagnostic dwell",
    )
    Parser.add_argument("filename", help="HDF5 file written by the Save button")
    Parser.add_argument(
        "--span-m",
        type=float,
        default=500.0,
        help="range span shown either side of the configured target",
    )
    Parser.add_argument("--output", default=None, help="output PNG filename")
    Parser.add_argument("--no-show", action="store_true")
    Arguments = Parser.parse_args()

    InputPath = Path(Arguments.filename).expanduser().resolve()
    if not InputPath.is_file():
        raise FileNotFoundError(InputPath)

    with h5py.File(InputPath, "r") as File:
        if "golay_diagnostic" not in File:
            raise RuntimeError(
                "No /golay_diagnostic group found; enable Save and wait for "
                "the fixed target to become active"
            )
        Group = File["golay_diagnostic"]
        Result = CalculateGolayDiagnostic(Group)
        Attributes = dict(Group.attrs)

    print("Vanguard X one-shot Golay diagnostic")
    print(f"  file:                         {InputPath}")
    print(f"  dwell:                        {Attributes.get('dwell_id', -1)}")
    print(f"  complementary pairs:          {Result['NumPairs']}")
    print(f"  peak range:                   {Result['PeakRangeM']:.3f} m")
    print(
        "  B/A peak gain difference:     "
        f"{Result['GainDifferenceDb']:+.3f} dB"
    )
    print(
        "  B-A peak phase difference:    "
        f"{Result['PhaseDifferenceDeg']:+.3f} deg"
    )
    print(
        "  B-A peak range difference:    "
        f"{Result['PeakRangeDifferenceSamples']:+d} samples"
    )
    print(
        "  A+B peak sidelobe:            "
        f"{Result['PeakSidelobeDb']:.3f} dB"
    )
    print(
        "  raw maximum complex magnitude:"
        f" {Result['MaximumComplexMagnitude']:.6f}"
    )
    print(
        "  raw maximum I/Q component:    "
        f"{Result['MaximumComponentMagnitude']:.6f}"
    )
    print(
        "  possible clipped components:  "
        f"{Result['ClippedComponentCount']}"
    )

    Figure, Axis = plt.subplots(figsize=(12, 6))
    RangeKm = Result["RangeM"] / 1000.0
    Axis.plot(RangeKm, Result["ProfileADb"], label="MF A", linewidth=1.0)
    Axis.plot(RangeKm, Result["ProfileBDb"], label="MF B", linewidth=1.0)
    Axis.plot(
        RangeKm,
        Result["ProfileSumDb"],
        label="Complex A+B",
        linewidth=1.5,
        color="black",
    )
    Axis.grid(True, alpha=0.3)
    Axis.set_xlabel("Range (km)")
    Axis.set_ylabel("Magnitude relative to A+B peak (dB)")
    Axis.set_title("Vanguard X Golay zero-Doppler A/B diagnostic")
    Axis.set_ylim(-80.0, 5.0)
    Axis.legend()

    TargetRangeM = float(Attributes.get("target_range_m", np.nan))
    if np.isfinite(TargetRangeM) and Arguments.span_m > 0.0:
        Axis.set_xlim(
            (TargetRangeM - Arguments.span_m) / 1000.0,
            (TargetRangeM + Arguments.span_m) / 1000.0,
        )

    Figure.tight_layout()
    OutputPath = (
        Path(Arguments.output).expanduser().resolve()
        if Arguments.output
        else InputPath.with_name(InputPath.stem + "_golay_diagnostic.png")
    )
    Figure.savefig(OutputPath, dpi=160)
    print(f"  plot:                         {OutputPath}")
    if not Arguments.no_show:
        plt.show()


if __name__ == "__main__":
    Main()
