"""Numerical regression tests for the Vanguard X radar timing model."""

import unittest

from RadarTiming import (
    CalculateRadarTiming,
    SPEED_OF_LIGHT_MPS,
)
from WaveformLibrary import WaveformLibrary


class TestRadarTiming(unittest.TestCase):
    def setUp(self):
        self.Library = WaveformLibrary(
            {"EttusSampleRateHz": 40.0e6}
        )
        self.Library.LoadDefaultWaveforms()

    def Calculate(self, WaveformId="Frank10_20MHz", **Overrides):
        return CalculateRadarTiming(
            self.Library.GetMetadata(WaveformId),
            **Overrides,
        )

    def test_default_frank10_20mhz_timing_solution(self):
        Timing = self.Calculate()

        self.assertEqual(Timing.WaveformId, "Frank10_20MHz")
        self.assertEqual(Timing.SampleRateHz, 40.0e6)
        self.assertEqual(Timing.TxWaveformSamples, 200)
        self.assertAlmostEqual(Timing.TxPulseDurationSec, 5.0e-6, places=15)

        self.assertEqual(Timing.SelectedPrfHz, 2000.0)
        self.assertAlmostEqual(Timing.PriSec, 500.0e-6, places=15)
        self.assertEqual(Timing.PulsesPerCpi, 32)
        self.assertAlmostEqual(Timing.CpiDurationSec, 16.0e-3, places=15)
        self.assertAlmostEqual(Timing.PulseTrainSpanSec, 15.5e-3, places=15)

        self.assertAlmostEqual(Timing.RxStartDelaySec, 6.0e-6, places=15)
        self.assertEqual(Timing.NumRxSamples, 4043)
        self.assertAlmostEqual(
            Timing.ActualRxCaptureDurationSec,
            101.075e-6,
            places=15,
        )
        self.assertAlmostEqual(
            Timing.ActualRxEndDelaySec,
            107.075e-6,
            places=15,
        )
        self.assertAlmostEqual(
            Timing.MinimumPriSec,
            109.075e-6,
            places=15,
        )
        self.assertAlmostEqual(
            Timing.TimingLimitedMaxPrfHz,
            9168.003667201467,
            places=9,
        )
        self.assertEqual(Timing.EffectiveMaxPrfHz, 4000.0)
        self.assertTrue(Timing.TimingSafe)

    def test_receive_sample_ceiling_captures_complete_requested_window(self):
        Timing = self.Calculate()
        SampleIntervalSec = 1.0 / Timing.SampleRateHz

        self.assertGreaterEqual(
            Timing.ActualRxEndDelaySec,
            Timing.RequiredRxEndDelaySec,
        )
        self.assertLess(
            Timing.ActualRxEndDelaySec - Timing.RequiredRxEndDelaySec,
            SampleIntervalSec,
        )
        self.assertAlmostEqual(
            Timing.MinimumPriSec,
            Timing.ActualRxEndDelaySec + Timing.NextTxGuardTimeSec,
            places=15,
        )

    def test_default_operational_consequences(self):
        Timing = self.Calculate()

        self.assertAlmostEqual(
            Timing.MaximumEchoLeadingEdgeDelaySec,
            2.0 * 15000.0 / SPEED_OF_LIGHT_MPS,
            places=15,
        )
        self.assertAlmostEqual(
            Timing.MinimumFullEchoRangeM,
            899.377374,
            places=6,
        )
        self.assertEqual(
            Timing.FirstRxSampleRangeOffsetM,
            Timing.MinimumFullEchoRangeM,
        )
        self.assertAlmostEqual(
            Timing.MaximumUnambiguousRangeM,
            74948.1145,
            places=4,
        )
        self.assertAlmostEqual(
            Timing.UnambiguousRadialVelocityMps,
            15.94640734042553,
            places=12,
        )
        self.assertAlmostEqual(
            Timing.VelocityBinSpacingMps,
            0.9966504587765956,
            places=12,
        )
        self.assertAlmostEqual(
            Timing.AntennaMovementDuringCpiDeg,
            1.44,
            places=12,
        )

    def test_receive_buffer_sizes_are_explicit(self):
        Timing = self.Calculate()

        self.assertEqual(Timing.BytesPerPulseSc16, 4043 * 4)
        self.assertEqual(Timing.BytesPerPulseFc32, 4043 * 8)
        self.assertEqual(Timing.BytesPerCpiSc16, 4043 * 4 * 32)
        self.assertEqual(Timing.BytesPerCpiFc32, 4043 * 8 * 32)

    def test_pulse_duration_changes_minimum_range_and_safe_prf(self):
        Barker = self.Calculate("Barker13_20MHz")
        Frank = self.Calculate("Frank10_20MHz")

        # Capture duration/sample count is range-driven and the waveform pulse
        # duration cancels between RX start and required RX end.
        self.assertEqual(Barker.NumRxSamples, Frank.NumRxSamples)
        self.assertEqual(
            Barker.ActualRxCaptureDurationSec,
            Frank.ActualRxCaptureDurationSec,
        )

        self.assertAlmostEqual(
            Barker.MinimumFullEchoRangeM,
            247.32877785,
            places=8,
        )
        self.assertAlmostEqual(
            Frank.MinimumFullEchoRangeM,
            899.377374,
            places=6,
        )
        self.assertLess(Frank.TimingLimitedMaxPrfHz, Barker.TimingLimitedMaxPrfHz)

    def test_four_khz_is_safe_for_current_15_km_profile(self):
        Timing = self.Calculate(SelectedPrfHz=4000.0)

        self.assertAlmostEqual(Timing.PriSec, 250.0e-6, places=15)
        self.assertGreater(Timing.PriSec, Timing.MinimumPriSec)
        self.assertAlmostEqual(Timing.CpiDurationSec, 8.0e-3, places=15)
        self.assertAlmostEqual(
            Timing.AntennaMovementDuringCpiDeg,
            0.72,
            places=12,
        )

    def test_one_khz_is_safe_and_has_longer_cpi(self):
        Timing = self.Calculate(SelectedPrfHz=1000.0)

        self.assertAlmostEqual(Timing.PriSec, 1.0e-3, places=15)
        self.assertAlmostEqual(Timing.CpiDurationSec, 32.0e-3, places=15)
        self.assertAlmostEqual(
            Timing.AntennaMovementDuringCpiDeg,
            2.88,
            places=12,
        )

    def test_prf_outside_operator_range_is_rejected(self):
        for PrfHz in (999.0, 4001.0):
            with self.subTest(PrfHz=PrfHz):
                with self.assertRaisesRegex(ValueError, "operator range"):
                    self.Calculate(SelectedPrfHz=PrfHz)

    def test_prf_unsafe_for_maximum_range_is_rejected(self):
        with self.assertRaisesRegex(
            ValueError,
            "unsafe.*maximum safe PRF",
        ):
            self.Calculate(
                SelectedPrfHz=4000.0,
                MaximumRangeM=50000.0,
            )

    def test_maximum_range_must_exceed_waveform_blind_range(self):
        with self.assertRaisesRegex(
            ValueError,
            "must exceed the minimum full-echo range",
        ):
            CalculateRadarTiming(
                self.Library.GetMetadata("Frank10_10MHz"),
                MaximumRangeM=1000.0,
            )

    def test_invalid_pulse_count_and_negative_margin_are_rejected(self):
        with self.assertRaisesRegex(ValueError, "positive integer"):
            self.Calculate(PulsesPerCpi=0)
        with self.assertRaisesRegex(ValueError, "must not be negative"):
            self.Calculate(ReceiverRecoveryTimeSec=-1.0e-6)

    def test_inconsistent_waveform_metadata_is_rejected(self):
        Metadata = self.Library.GetMetadata("Frank10_20MHz")
        Metadata["NumSamples"] = 199

        with self.assertRaisesRegex(ValueError, "metadata is inconsistent"):
            CalculateRadarTiming(Metadata)

    def test_solution_metadata_is_serialisable_and_complete(self):
        Timing = self.Calculate()
        Metadata = Timing.ToMetadata()

        self.assertEqual(Metadata["WaveformId"], "Frank10_20MHz")
        self.assertEqual(Metadata["NumRxSamples"], 4043)
        self.assertEqual(Metadata["TimingSafe"], True)
        self.assertNotIn("Chips", Metadata)
        self.assertNotIn("Samples", Metadata)


if __name__ == "__main__":
    unittest.main(verbosity=2)
