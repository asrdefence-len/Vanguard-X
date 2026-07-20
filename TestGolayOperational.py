"""Operational Golay A/B planning and complementary processing regressions."""

from dataclasses import replace
from types import SimpleNamespace
import unittest

import numpy as np

from DataTypes import RawDwellData
from PointingManager import PointingState
from RadarExecutor import RadarExecutor
from RadarPlans import make_golay_dwell_plan
from RadarProcessor import RadarProcessor
from RadarTasks import (
    AngleFrame,
    PointingMode,
    PointingRequest,
    RadarTaskType,
    SearchSector,
    SearchTask,
)
from WaveformLibrary import WaveformLibrary


def MakeLibrary():
    Library = WaveformLibrary({"EttusSampleRateHz": 40.0e6})
    Library.LoadDefaultWaveforms()
    return Library


def MakePlan(NumPulses=8):
    return make_golay_dwell_plan(
        dwell_id=7,
        task_id=11,
        task_type="SEARCH",
        waveform_a_id="Golay64A_20MHz",
        waveform_b_id="Golay64B_20MHz",
        sample_rate=40.0e6,
        num_samples=512,
        num_pulses=NumPulses,
        pri_sec=250.0e-6,
        rx_start_delay_sec=4.2e-6,
    )


def MakeStationaryEcho(Library, Plan, EchoSample=80):
    IQ = np.zeros((Plan.NumPulses, Plan.NumSamples), dtype=np.complex64)
    for pulse in Plan.PulsePlans:
        waveform = Library.Get(pulse.WaveformId)
        IQ[
            pulse.PulseIndex,
            EchoSample:EchoSample + len(waveform),
        ] = waveform
    return RawDwellData(
        DwellId=Plan.DwellId,
        IQ=IQ,
        SampleRate=Plan.SampleRate,
        PRI=Plan.PRI,
        TimeStamp=1.0,
        PulseTimesSec=np.arange(Plan.NumPulses) * Plan.PRI,
        PulsePriSec=np.full(Plan.NumPulses, Plan.PRI),
        PulseWaveformIds=[p.WaveformId for p in Plan.PulsePlans],
        PulseValid=np.ones(Plan.NumPulses, dtype=bool),
        PulseRxStartDelaySec=np.full(Plan.NumPulses, 4.2e-6),
    )


class TestGolayOperational(unittest.TestCase):
    def test_catalogue_contains_a_declared_complementary_pair(self):
        Library = MakeLibrary()
        A = Library.GetMetadata("Golay64A_20MHz")
        B = Library.GetMetadata("Golay64B_20MHz")

        self.assertEqual(A["PairId"], "Golay64_20MHz")
        self.assertEqual(A["PairRole"], "A")
        self.assertEqual(B["PairRole"], "B")
        self.assertEqual(A["ComplementaryWaveformId"], B["WaveformId"])
        self.assertEqual(B["ComplementaryWaveformId"], A["WaveformId"])
        self.assertEqual(A["ChipCount"], 64)
        self.assertEqual(A["SamplesPerChip"], 2)
        self.assertEqual(A["NumSamples"], 128)
        self.assertAlmostEqual(A["PulseDurationSec"], 3.2e-6, places=15)

        ChipsA = Library.GetChips(A["WaveformId"])
        ChipsB = Library.GetChips(B["WaveformId"])
        Sum = np.correlate(ChipsA, ChipsA, mode="full")
        Sum += np.correlate(ChipsB, ChipsB, mode="full")
        Centre = len(Sum) // 2
        self.assertEqual(Sum[Centre], 128)
        self.assertTrue(np.allclose(np.delete(Sum, Centre), 0.0))

    def test_plan_is_strict_a_then_b_at_four_khz(self):
        Plan = MakePlan(64)

        self.assertEqual(Plan.Processing.Mode, "GOLAY_COMPLEMENTARY")
        self.assertTrue(Plan.Processing.CombineGroupsBeforeDoppler)
        self.assertFalse(Plan.Processing.DopplerCompensationEnabled)
        self.assertEqual(Plan.NumPulses, 64)
        self.assertAlmostEqual(Plan.PRI, 250.0e-6, places=15)
        self.assertAlmostEqual(Plan.NumPulses * Plan.PRI, 16.0e-3, places=15)
        for index, pulse in enumerate(Plan.PulsePlans):
            self.assertEqual(pulse.GroupId, index // 2)
            self.assertEqual(pulse.GroupRole, "A" if index % 2 == 0 else "B")

    def test_executor_selects_same_operational_pair_for_search(self):
        Library = MakeLibrary()
        Config = {
            "RfFrequency": 9.4e9,
            "SearchWaveformId": "Golay64_20MHz",
            "SelectedPrfHz": 4000.0,
            "SelectedPulsesPerCpi": 64,
            "InstrumentedMaxRangeM": 15000.0,
            "MinPrfHz": 1000.0,
            "MaxPrfHz": 4000.0,
            "ReceiverRecoveryTimeSec": 1.0e-6,
            "RxEndMarginSec": 2.0e-6,
            "NextTxGuardTimeSec": 2.0e-6,
            "PTZScanSlewRateDegPerSec": 14.0,
        }
        Executor = RadarExecutor(
            source=SimpleNamespace(TheWaveformLibrary=Library),
            pointing_manager=None,
            config=Config,
            waveform_library=Library,
        )
        Sector = SearchSector(AngleFrame.TRUE, 10.0, 20.0, 14.0)
        Task = SearchTask(
            TaskId=3,
            TaskType=RadarTaskType.SEARCH,
            Priority=1,
            Pointing=PointingRequest(
                Mode=PointingMode.CONTINUOUS_SCAN,
                Frame=AngleFrame.TRUE,
            ),
            WaveformProfileId="SEARCH_DEFAULT",
            Sector=Sector,
        )
        Pointing = PointingState(
            TimestampSec=1.0,
            Mode=PointingMode.CONTINUOUS_SCAN,
            AntennaAzimuthRelativeDeg=15.0,
            AntennaElevationRelativeDeg=0.0,
            PlatformHeadingTrueDeg=0.0,
            BeamBearingTrueDeg=15.0,
            BeamElevationTrueDeg=0.0,
            CommandedAzimuthRelativeDeg=20.0,
            CommandedBearingTrueDeg=20.0,
            PanRateDegPerSec=14.0,
            Ready=True,
            Reachable=True,
            Valid=True,
            Source="TEST",
        )

        Profile = Executor.GetExecutionProfile(Task)
        Plan = Executor.BuildDwellPlan(Task, Pointing, Profile)

        self.assertEqual(Profile.WaveformId, "Golay64_20MHz")
        self.assertEqual(Profile.NumPulses, 64)
        self.assertAlmostEqual(Profile.PriSec, 250.0e-6)
        self.assertAlmostEqual(Profile.RxStartDelaySec, 4.4e-6)
        self.assertEqual(Plan.Processing.Mode, "GOLAY_COMPLEMENTARY")
        self.assertEqual(Plan.PulsePlans[0].WaveformId, "Golay64A_20MHz")
        self.assertEqual(Plan.PulsePlans[1].WaveformId, "Golay64B_20MHz")

    def test_complementary_sum_precedes_pair_rate_doppler(self):
        Library = MakeLibrary()
        Plan = MakePlan()
        Raw = MakeStationaryEcho(Library, Plan)
        Processor = RadarProcessor({"RfFrequency": 9.4e9}, Library)

        Result = Processor.Process(Raw, Plan)

        self.assertEqual(Result.RangeCompressed.shape, (4, 512))
        self.assertEqual(Result.RangeDopplerMap.shape, (4, 512))
        self.assertEqual(Result.Diagnostics["DopplerCompensationMode"], "NONE")
        self.assertEqual(Result.Diagnostics["ComplementaryPairCount"], 4)
        self.assertAlmostEqual(Result.Diagnostics["PairPriSec"], 500.0e-6)
        self.assertEqual(Result.Diagnostics["PeakRangeBin"], 80)
        self.assertEqual(Result.Diagnostics["PeakDopplerHz"], 0.0)

        Profile = Result.RangeCompressed[0]
        Peak = abs(Profile[80])
        ChipAlignedSidelobes = np.delete(abs(Profile[::2]), 40)
        self.assertGreater(Peak, 127.9)
        self.assertLess(np.max(ChipAlignedSidelobes), 1.0e-3)

    def test_missing_or_reordered_pair_is_rejected(self):
        Library = MakeLibrary()
        Plan = MakePlan()
        Raw = MakeStationaryEcho(Library, Plan)
        Processor = RadarProcessor({"RfFrequency": 9.4e9}, Library)

        Plan.PulsePlans[1] = replace(
            Plan.PulsePlans[1],
            WaveformId="Golay64A_20MHz",
        )
        with self.assertRaisesRegex(ValueError, "A waveform then B"):
            Processor.Process(Raw, Plan)

        with self.assertRaisesRegex(ValueError, "positive even"):
            MakePlan(7)


if __name__ == "__main__":
    unittest.main()
