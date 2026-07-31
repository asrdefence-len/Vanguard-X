"""Operational simulation gate for Vanguard X Golay complementary processing."""

from __future__ import annotations

from dataclasses import replace

import numpy as np

from RadarPlans import make_golay_dwell_plan
from RadarProcessor import RadarProcessor
from SimulatedSource import SimulatedSource
from WaveformLibrary import WaveformLibrary


RF_FREQUENCY_HZ = 9.4e9
PHYSICAL_PRI_SEC = 500.0e-6
PHYSICAL_PULSE_COUNT = 32
TARGET_DOPPLER_HZ = 313.55
TARGET_RANGE_M = 3000.0


def _PslrDb(Profile, SamplesPerChip):
    Magnitude = np.abs(np.asarray(Profile))
    PeakIndex = int(np.argmax(Magnitude))
    ExclusionBins = max(1, int(SamplesPerChip) - 1)
    Mask = np.ones(Magnitude.size, dtype=bool)
    Mask[
        max(0, PeakIndex - ExclusionBins):
        min(Magnitude.size, PeakIndex + ExclusionBins + 1)
    ] = False
    return float(
        20.0
        * np.log10(
            np.max(Magnitude[Mask])
            / max(float(Magnitude[PeakIndex]), 1.0e-30)
        )
    )


def _PeakRangePslr(Processed, SamplesPerChip):
    PeakDopplerBin, PeakRangeBin = np.unravel_index(
        np.argmax(np.abs(Processed.RangeDopplerMap)),
        Processed.RangeDopplerMap.shape,
    )
    PslrDb = _PslrDb(
        Processed.RangeDopplerMap[PeakDopplerBin],
        SamplesPerChip,
    )
    return PeakDopplerBin, PeakRangeBin, PslrDb


def RunValidation():
    Config = {
        "EttusSampleRateHz": 40.0e6,
        "RfFrequency": RF_FREQUENCY_HZ,
        "TransmitPowerW": 50.0,
        "AntennaGainDb": 23.0,
        "SystemLossDb": 6.0,
        "NoisePowerW": 0.0,
        "SceneReturns": [{
            "name": "GolayMovingTarget",
            "range_m": TARGET_RANGE_M,
            "doppler_hz": TARGET_DOPPLER_HZ,
            "rcs": 1000.0,
            "two_way_gain_power": 1.0,
        }],
    }
    Library = WaveformLibrary(Config)
    Library.LoadDefaultWaveforms()
    SamplesPerChip = int(
        Library.GetMetadata("Golay64A_20MHz")["SamplesPerChip"]
    )

    ChipsA = Library.GetChips("Golay64A_20MHz")
    ChipsB = Library.GetChips("Golay64B_20MHz")
    ComplementaryResponse = (
        np.correlate(ChipsA, ChipsA, mode="full")
        + np.correlate(ChipsB, ChipsB, mode="full")
    )
    Centre = ComplementaryResponse.size // 2
    CodeResidual = float(np.max(np.abs(
        np.delete(ComplementaryResponse, Centre)
    )))

    SamplesA = Library.Get("Golay64A_20MHz")
    AOnlyResponse = np.convolve(
        SamplesA,
        np.conj(SamplesA[::-1]),
        mode="full",
    )
    AOnlyPslrDb = _PslrDb(AOnlyResponse, SamplesPerChip)

    Plan = make_golay_dwell_plan(
        dwell_id=1,
        task_id=1,
        task_type="SEARCH",
        waveform_a_id="Golay64A_20MHz",
        waveform_b_id="Golay64B_20MHz",
        sample_rate=40.0e6,
        num_samples=2048,
        num_pulses=PHYSICAL_PULSE_COUNT,
        pri_sec=PHYSICAL_PRI_SEC,
        rx_start_delay_sec=4.4e-6,
    )
    Raw = SimulatedSource(Config, Library).ExecuteDwell(Plan)
    Processor = RadarProcessor(Config, Library)
    Corrected = Processor.Process(Raw, Plan)
    CorrectedDopplerBin, CorrectedRangeBin, CorrectedPslrDb = (
        _PeakRangePslr(Corrected, SamplesPerChip)
    )

    LegacyPlan = replace(
        Plan,
        Processing=replace(
            Plan.Processing,
            CombineGroupsBeforeDoppler=True,
            DopplerCompensationEnabled=False,
        ),
    )
    Legacy = Processor.Process(Raw, LegacyPlan)
    _, _, LegacyPslrDb = _PeakRangePslr(Legacy, SamplesPerChip)

    OperationalIds = Library.ListOperationalWaveforms()
    Alternating = all(
        WaveformId
        == (
            "Golay64A_20MHz"
            if PulseIndex % 2 == 0
            else "Golay64B_20MHz"
        )
        for PulseIndex, WaveformId in enumerate(Raw.PulseWaveformIds)
    )
    DopplerBinSpacingHz = 1.0 / (
        PHYSICAL_PULSE_COUNT * PHYSICAL_PRI_SEC
    )
    DetectedDopplerHz = float(
        Corrected.DopplerAxisHz[CorrectedDopplerBin]
    )
    DetectedRangeM = float(Corrected.RangeAxisM[CorrectedRangeBin])
    DopplerErrorHz = abs(DetectedDopplerHz - TARGET_DOPPLER_HZ)

    Checks = {
        "code_pair_exact": CodeResidual <= 1.0e-6,
        "operator_selects_pair": (
            "Golay64_20MHz" in OperationalIds
            and "Golay64A_20MHz" not in OperationalIds
            and "Golay64B_20MHz" not in OperationalIds
        ),
        "simulator_alternates_a_b": Alternating,
        "pair_rate_doppler_bins": (
            Corrected.RangeDopplerMap.shape[0]
            == PHYSICAL_PULSE_COUNT // 2
        ),
        "doppler_within_half_bin": (
            DopplerErrorHz <= DopplerBinSpacingHz / 2.0
        ),
        "corrected_pslr_below_minus_50_db": CorrectedPslrDb < -50.0,
        "correction_improves_pslr_by_20_db": (
            CorrectedPslrDb < LegacyPslrDb - 20.0
        ),
    }
    Passed = all(Checks.values())
    return {
        "Passed": Passed,
        "Checks": Checks,
        "CodeResidual": CodeResidual,
        "AOnlyPslrDb": AOnlyPslrDb,
        "LegacyPslrDb": LegacyPslrDb,
        "CorrectedPslrDb": CorrectedPslrDb,
        "DetectedDopplerHz": DetectedDopplerHz,
        "DopplerErrorHz": DopplerErrorHz,
        "DetectedRangeM": DetectedRangeM,
        "PulseWaveformIds": Raw.PulseWaveformIds,
        "OperationalWaveformIds": OperationalIds,
    }


def Main():
    Result = RunValidation()
    print("Vanguard X Golay simulation operational validation")
    print(
        "  pulse sequence:       "
        + " ".join(
            WaveformId.replace("Golay64", "").replace("_20MHz", "")
            for WaveformId in Result["PulseWaveformIds"][:8]
        )
        + " ..."
    )
    print(
        "  operator catalogue:   "
        + ", ".join(Result["OperationalWaveformIds"])
    )
    print(f"  code residual:        {Result['CodeResidual']:.3e}")
    print(f"  A-only PSLR:          {Result['AOnlyPslrDb']:.2f} dB")
    print(f"  legacy pair PSLR:     {Result['LegacyPslrDb']:.2f} dB")
    print(f"  corrected pair PSLR:  {Result['CorrectedPslrDb']:.2f} dB")
    print(
        "  Doppler:              "
        f"injected={TARGET_DOPPLER_HZ:.2f} Hz, "
        f"detected={Result['DetectedDopplerHz']:.2f} Hz, "
        f"error={Result['DopplerErrorHz']:.2f} Hz"
    )
    print(
        f"  detected range:       {Result['DetectedRangeM']:.3f} m"
    )
    for Name, Passed in Result["Checks"].items():
        print(f"  {Name}: {'PASS' if Passed else 'FAIL'}")
    print(f"RESULT: {'PASS' if Result['Passed'] else 'FAIL'}")
    return 0 if Result["Passed"] else 1


if __name__ == "__main__":
    raise SystemExit(Main())
