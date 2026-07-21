"""Regression tests for dwell-boundary operator timing application."""

from types import SimpleNamespace
import unittest

from RadarExecutor import RadarExecutor
from RadarTasks import RadarTaskType
from RadarTimingControls import ApplyTimingControlState
from WaveformLibrary import WaveformLibrary


def MakeConfig():
    return {
        "EttusSampleRateHz": 40.0e6,
        "RfFrequency": 9.4e9,
        "SearchWaveformId": "Frank10_20MHz",
        "TrackWaveformId": "Barker13_20MHz",
        "MinPrfHz": 1000.0,
        "MaxPrfHz": 4000.0,
        "SelectedPrfHz": 2000.0,
        "SelectedPulsesPerCpi": 32,
        "InstrumentedMaxRangeM": 15000.0,
        "ReceiverRecoveryTimeSec": 1.0e-6,
        "RxEndMarginSec": 2.0e-6,
        "NextTxGuardTimeSec": 2.0e-6,
        "X660ScanSlewRateDegPerSec": 14.0,
    }


class TestRadarTimingControls(unittest.TestCase):
    def setUp(self):
        self.Config = MakeConfig()
        self.Library = WaveformLibrary(self.Config)
        self.Library.LoadDefaultWaveforms()
        self.Executor = RadarExecutor(
            source=SimpleNamespace(TheWaveformLibrary=self.Library),
            pointing_manager=None,
            config=self.Config,
            waveform_library=self.Library,
        )
        self.SearchTask = SimpleNamespace(TaskType=RadarTaskType.SEARCH)

    def Apply(
        self,
        Revision,
        Waveform,
        PrfHz,
        Pulses,
        LastRevision,
        MaximumRangeM=15000.0,
    ):
        return ApplyTimingControlState(
            Config=self.Config,
            ControlState={
                "TimingSelectionRevision": Revision,
                "SelectedWaveformId": Waveform,
                "SelectedPrfHz": PrfHz,
                "SelectedPulsesPerCpi": Pulses,
                "SelectedMaximumRangeM": MaximumRangeM,
            },
            Executor=self.Executor,
            SearchTask=self.SearchTask,
            LastAppliedRevision=LastRevision,
        )

    def test_valid_selection_is_applied_to_validated_search_profile(self):
        Result = self.Apply(
            Revision=1,
            Waveform="Frank10_10MHz",
            PrfHz=2500.0,
            Pulses=64,
            LastRevision=0,
            MaximumRangeM=10000.0,
        )

        self.assertTrue(Result.Changed)
        self.assertTrue(Result.Applied)
        self.assertEqual(Result.Profile.WaveformId, "Frank10_10MHz")
        self.assertAlmostEqual(Result.Profile.PriSec, 400.0e-6, places=15)
        self.assertEqual(Result.Profile.NumPulses, 64)
        self.assertEqual(Result.Profile.NumSamples, 2709)
        self.assertAlmostEqual(
            Result.Profile.Timing.CpiDurationSec,
            25.6e-3,
            places=15,
        )
        self.assertAlmostEqual(
            Result.Profile.RxStartDelaySec,
            11.2e-6,
            places=15,
        )
        self.assertEqual(self.Config["SelectedPrfHz"], 2500.0)
        self.assertEqual(self.Config["SearchPulsesPerCpi"], 64)
        self.assertEqual(self.Config["InstrumentedMaxRangeM"], 10000.0)

    def test_same_revision_is_not_applied_twice(self):
        Result = self.Apply(
            Revision=3,
            Waveform="Frank10_20MHz",
            PrfHz=2000.0,
            Pulses=32,
            LastRevision=3,
        )

        self.assertFalse(Result.Changed)
        self.assertFalse(Result.Applied)

    def test_invalid_selection_is_rejected_and_config_is_rolled_back(self):
        Before = dict(self.Config)
        Result = self.Apply(
            Revision=2,
            Waveform="Frank10_20MHz",
            PrfHz=5000.0,
            Pulses=32,
            LastRevision=1,
        )

        self.assertTrue(Result.Changed)
        self.assertFalse(Result.Applied)
        self.assertIn("outside the operator range", Result.Message)
        self.assertEqual(self.Config, Before)

    def test_unknown_waveform_is_rejected_without_crashing_main(self):
        Result = self.Apply(
            Revision=4,
            Waveform="UnknownWaveform",
            PrfHz=2000.0,
            Pulses=32,
            LastRevision=3,
        )

        self.assertTrue(Result.Changed)
        self.assertFalse(Result.Applied)
        self.assertIn("Waveform not found", Result.Message)

    def test_unsafe_prf_and_maximum_range_combination_is_rolled_back(self):
        Before = dict(self.Config)
        Result = self.Apply(
            Revision=5,
            Waveform="Frank10_20MHz",
            PrfHz=4000.0,
            Pulses=32,
            MaximumRangeM=50000.0,
            LastRevision=4,
        )

        self.assertTrue(Result.Changed)
        self.assertFalse(Result.Applied)
        self.assertIn("maximum safe PRF", Result.Message)
        self.assertEqual(self.Config, Before)

    def test_range_below_waveform_blind_range_is_rejected(self):
        Result = self.Apply(
            Revision=6,
            Waveform="Frank10_10MHz",
            PrfHz=2000.0,
            Pulses=32,
            MaximumRangeM=1000.0,
            LastRevision=5,
        )

        self.assertTrue(Result.Changed)
        self.assertFalse(Result.Applied)
        self.assertIn("minimum full-echo range", Result.Message)


if __name__ == "__main__":
    unittest.main(verbosity=2)
